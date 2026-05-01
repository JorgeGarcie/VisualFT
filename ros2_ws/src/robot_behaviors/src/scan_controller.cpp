/**
 * @file scan_controller.cpp
 * @brief Automated phantom scanning state machine using ArmCommander.
 *
 * Uses ArmCommander for ALL robot interaction -- no direct RDK calls, no
 * quaternion reordering, no manual mode switching.
 *
 * State machine: HOMING -> ZEROING_FT -> DESCENDING -> SCANNING -> RETURNING -> DONE
 *
 * For each scan in the session config:
 *   1. Home the robot (MoveL to home pose)
 *   2. Zero the FT sensor
 *   3. Descend until contact force threshold is reached
 *   4. Scan: zigzag passes with orientation sweeps, impedance control
 *   5. Return to home pose
 *   6. Start/stop MCAP bag recording per scan
 *
 * Config is loaded from two YAML files:
 *   - robot.yaml: robot connection, safety, workspace bounds
 *   - scan.yaml: session name, scan parameters, motion/impedance, recording
 *
 * Usage:
 *     scan_controller <scan_config_path> <robot_config_path>
 *
 * Ctrl+C to stop cleanly. Robot returns to IDLE.
 */

#include "arm_commander/arm_commander.hpp"
#include "arm_commander/config.hpp"
#include "robot_behaviors/scan_config.hpp"

#include <Eigen/Geometry>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <filesystem>
#include <iostream>
#include <string>
#include <sys/types.h>
#include <sys/wait.h>
#include <thread>
#include <unistd.h>
#include <vector>

namespace {

// ── Signal handling ─────────────────────────────────────────────────────────

std::atomic<arm_commander::ArmCommander*> g_commander{nullptr};

void SignalHandler(int signum)
{
    (void)signum;
    auto* cmd = g_commander.load(std::memory_order_acquire);
    if (cmd) {
        cmd->request_stop();
    }
}

// No default — robot config path must be provided as CLI arg.

// ── State machine ───────────────────────────────────────────────────────────

// ── Pass definition ─────────────────────────────────────────────────────────

struct ScanPass {
    double rz_offset;  // deg
    double rx_offset;  // deg
    bool forward;
    double x_offset;   // metres
};

// ── Euler to quaternion ─────────────────────────────────────────────────────

/**
 * Convert Euler XYZ degrees to RDK quaternion [qw, qx, qy, qz].
 * Matches the Python euler_to_rdk_quat() which uses scipy extrinsic XYZ.
 * Eigen uses intrinsic convention, so we use ZYX intrinsic = XYZ extrinsic.
 */
std::array<double, 4> euler_xyz_to_quat(double rx_deg, double ry_deg, double rz_deg)
{
    double rx = rx_deg * M_PI / 180.0;
    double ry = ry_deg * M_PI / 180.0;
    double rz = rz_deg * M_PI / 180.0;

    // Eigen eulerAngles(2,1,0) = intrinsic ZYX = extrinsic XYZ
    Eigen::Quaterniond q =
        Eigen::AngleAxisd(rz, Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(ry, Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(rx, Eigen::Vector3d::UnitX());
    q.normalize();

    return {q.w(), q.x(), q.y(), q.z()};
}

// ── Inclusive range ─────────────────────────────────────────────────────────

std::vector<double> arange_inclusive(double start, double end, double step)
{
    if (step <= 0.0)
        return {start};
    std::vector<double> vals;
    double v = start;
    while (v <= end + 1e-9) {
        vals.push_back(std::round(v * 1e6) / 1e6);
        v += step;
    }
    return vals;
}

// ── Build passes from scan params ───────────────────────────────────────────

std::vector<ScanPass> build_passes(const arm_commander::ScanParams& p)
{
    auto rz_values = arange_inclusive(p.rz_start, p.rz_end, p.rz_step);
    auto rx_values = arange_inclusive(p.rx_start, p.rx_end, p.rx_step);
    const auto& x_offsets = p.x_offsets_mm;

    if (x_offsets.empty()) {
        throw std::runtime_error("x_offsets_mm must not be empty in scan config");
    }

    std::vector<ScanPass> passes;
    for (size_t i = 0; i < rz_values.size(); ++i) {
        double x_m = x_offsets[i % x_offsets.size()] / 1000.0;
        for (const auto& rx : rx_values) {
            passes.push_back({rz_values[i], rx, true, x_m});   // forward
            passes.push_back({rz_values[i], rx, false, x_m});  // backward
        }
    }
    return passes;
}

// ── MCAP bag recording ──────────────────────────────────────────────────────

class BagRecorder {
public:
    void start(const std::string& bag_dir, const std::vector<std::string>& topics)
    {
        stop();  // stop any existing recording

        // Create parent directory
        std::filesystem::create_directories(
            std::filesystem::path(bag_dir).parent_path());

        pid_ = fork();
        if (pid_ == 0) {
            // Child process
            // Build argv
            std::vector<std::string> args = {"ros2", "bag", "record",
                "--output", bag_dir, "--storage", "mcap",
                "--max-cache-size", "524288000"};
            for (const auto& topic : topics) {
                args.push_back(topic);
            }

            // Convert to char*[]
            std::vector<char*> argv;
            for (auto& a : args) {
                argv.push_back(a.data());
            }
            argv.push_back(nullptr);

            // Redirect stdout/stderr to /dev/null
            freopen("/dev/null", "w", stdout);
            freopen("/dev/null", "w", stderr);

            execvp("ros2", argv.data());
            // If exec fails, exit child
            _exit(1);
        } else if (pid_ > 0) {
            spdlog::info("Recording -> {} (pid={})", bag_dir, pid_);
        } else {
            spdlog::error("Failed to fork bag recorder: {}", strerror(errno));
            throw std::runtime_error("Failed to spawn bag recorder");
        }
    }

    void stop()
    {
        if (pid_ <= 0)
            return;

        // Send SIGINT (same as Ctrl+C) for clean bag closure
        kill(pid_, SIGINT);

        // Wait up to 10 seconds
        for (int i = 0; i < 100; ++i) {
            int status = 0;
            pid_t result = waitpid(pid_, &status, WNOHANG);
            if (result != 0) {
                spdlog::info("Recording stopped");
                pid_ = -1;
                return;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        // Force kill if still alive
        spdlog::warn("Bag recorder did not exit cleanly, sending SIGTERM");
        kill(pid_, SIGTERM);
        waitpid(pid_, nullptr, 0);
        pid_ = -1;
    }

    ~BagRecorder() { stop(); }

private:
    pid_t pid_ = -1;
};

// ── Main scan loop ──────────────────────────────────────────────────────────

/**
 * Run a single scan: HOMING -> ZEROING_FT -> DESCENDING -> SCANNING -> RETURNING.
 * Returns false if interrupted by stop request.
 */
bool run_scan(arm_commander::ArmCommander& commander,
    const arm_commander::ScanEntry& scan_entry,
    const arm_commander::ScanMotionConfig& motion,
    const arm_commander::RecordConfig& record_config,
    const std::array<double, 6>& max_contact_wrench,
    const std::string& session_name,
    int scan_idx, int total_scans)
{
    const auto& params = scan_entry.params;

    spdlog::info("{}", std::string(60, '='));
    spdlog::info("  Scan {}/{}: {}", scan_idx + 1, total_scans, scan_entry.name);
    spdlog::info("{}", std::string(60, '='));

    // Parse home pose: Elements [mm,mm,mm,deg,deg,deg] -> metres + degrees
    double home_x = params.home_pose[0] / 1000.0;
    double home_y = params.home_pose[1] / 1000.0;
    double home_z = params.home_pose[2] / 1000.0;
    double home_rx = params.home_pose[3];
    double home_ry = params.home_pose[4];
    double home_rz = params.home_pose[5];

    // Build home pose as [x,y,z,qw,qx,qy,qz]
    auto home_q = euler_xyz_to_quat(home_rx, home_ry, home_rz);
    std::array<double, 7> home_pose = {
        home_x, home_y, home_z,
        home_q[0], home_q[1], home_q[2], home_q[3]};

    // Build passes
    auto passes = build_passes(params);
    spdlog::info("  {} passes", passes.size());

    // Start MCAP recording
    BagRecorder recorder;
    if (record_config.enabled) {
        std::string bag_dir = record_config.output_dir + "/" + session_name + "/" +
            std::to_string(scan_idx) + "_" + scan_entry.name;
        recorder.start(bag_dir, record_config.topics);
    }

    // ── HOMING ──────────────────────────────────────────────────────
    spdlog::info("State: HOMING");
    commander.move_to(home_pose, motion.movel_vel);
    if (commander.stop_was_requested())
        return false;

    // ── ZEROING_FT ──────────────────────────────────────────────────
    spdlog::info("State: ZEROING_FT");
    commander.zero_ft();
    if (commander.stop_was_requested())
        return false;

    // ── DESCENDING ──────────────────────────────────────────────────
    spdlog::info("State: DESCENDING");

    // Use Contact primitive: descend in -Z until contact_force threshold
    commander.contact({0.0, 0.0, -1.0}, params.search_velocity, params.contact_force);
    if (commander.stop_was_requested()) {
        spdlog::warn("Descent interrupted before contact");
        return false;
    }

    auto state = commander.get_state();
    double contact_z = state.tcp_pose[2];
    spdlog::info("Contact at Z={:.4f}m", contact_z);

    // ── SCANNING ────────────────────────────────────────────────────
    spdlog::info("State: SCANNING");

    if (passes.empty()) {
        spdlog::info("No scan passes -- skipping to RETURNING");
    } else {
        // Configure impedance for scanning
        // Switch to streaming mode first (Contact primitive leaves us in NRT_PRIMITIVE)
        // Hold at contact position while we configure impedance
        std::array<double, 7> hold_pose = state.tcp_pose;
        commander.stream_cartesian(hold_pose, {0, 0, 0, 0, 0, 0},
            motion.max_linear_vel, motion.max_angular_vel,
            motion.max_linear_acc, motion.max_angular_acc);

        // Now configure impedance (requires NRT_CARTESIAN_MOTION_FORCE)
        double K = motion.xy_stiffness;
        double Kz = motion.z_stiffness;
        double Kr = motion.rot_stiffness;
        double Z = motion.damping_ratio;
        commander.set_impedance({K, K, Kz, Kr, Kr, Kr}, {Z, Z, Z, Z, Z, Z});
        commander.set_force_control_axis({false, false, false, false, false, false});
        commander.set_max_contact_wrench(max_contact_wrench);

        // Settle time for impedance settings
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        for (size_t pass_idx = 0; pass_idx < passes.size(); ++pass_idx) {
            if (commander.stop_was_requested())
                return false;

            const auto& pass = passes[pass_idx];

            // Compute target orientation
            double rx = home_rx + pass.rx_offset;
            double ry = home_ry;
            double rz = home_rz + pass.rz_offset;
            auto q = euler_xyz_to_quat(rx, ry, rz);

            // Compute Y target
            double target_y = pass.forward ? (home_y + params.y_scan_range) : home_y;
            double start_y = pass.forward ? home_y : (home_y + params.y_scan_range);

            std::string direction = pass.forward ? "fwd" : "bwd";
            spdlog::info("Pass {}/{}: rz{}_rx{}_x{:.0f}mm_{}, target_y={:.4f}",
                pass_idx + 1, passes.size(),
                pass.rz_offset, pass.rx_offset, pass.x_offset * 1000.0,
                direction, target_y);

            // Build scan target pose
            std::array<double, 7> scan_target = {
                home_x + pass.x_offset, target_y, contact_z,
                q[0], q[1], q[2], q[3]};
            std::array<double, 6> scan_wrench = {0, 0, 0, 0, 0, 0};

            // Stream until Y target reached
            while (!commander.stop_was_requested()) {
                state = commander.get_state();
                double current_y = state.tcp_pose[1];

                // Sinusoidal X oscillation based on Y progress
                if (motion.x_period > 0.0 && motion.x_amplitude > 0.0) {
                    double y_progress = std::abs(current_y - start_y);
                    double x_osc = motion.x_amplitude *
                        std::sin(2.0 * M_PI * y_progress / motion.x_period);
                    scan_target[0] = home_x + x_osc;
                }

                commander.stream_cartesian(scan_target, scan_wrench,
                    params.scan_speed, motion.max_angular_vel,
                    motion.max_linear_acc, motion.max_angular_acc);

                constexpr double kPassCompleteTolerance = 0.002;  // metres
                if (std::abs(current_y - target_y) < kPassCompleteTolerance) {
                    spdlog::info("Pass {}/{} done", pass_idx + 1, passes.size());
                    break;
                }

                std::this_thread::sleep_for(std::chrono::milliseconds(20));
            }

            // Settle pause when config changes between passes
            if (pass_idx + 1 < passes.size()) {
                const auto& next = passes[pass_idx + 1];
                constexpr double kEps = 1e-9;
                if (std::abs(next.rz_offset - pass.rz_offset) > kEps ||
                    std::abs(next.rx_offset - pass.rx_offset) > kEps ||
                    std::abs(next.x_offset - pass.x_offset) > kEps) {
                    spdlog::info("Config change -> rz={}, rx={}, x={:.0f}mm (settling 0.5s)",
                        next.rz_offset, next.rx_offset, next.x_offset * 1000.0);
                    std::this_thread::sleep_for(std::chrono::milliseconds(500));
                }
            }
        }

        spdlog::info("All passes complete!");
    }

    // ── RETURNING ───────────────────────────────────────────────────
    spdlog::info("State: RETURNING");
    commander.move_to(home_pose, motion.movel_vel);
    if (commander.stop_was_requested())
        return false;

    spdlog::info("Returned to home");

    // Stop recording
    recorder.stop();

    return true;
}

}  // namespace

int main(int argc, char* argv[])
{
    // Parse args
    if (argc < 3 || std::string(argv[1]) == "-h" || std::string(argv[1]) == "--help") {
        std::cout << "Usage: scan_controller <scan_config_path> <robot_config_path>"
                  << std::endl;
        std::cout << "  scan_config_path:  path to scan session YAML" << std::endl;
        std::cout << "  robot_config_path: path to robot.yaml" << std::endl;
        return (argc < 3) ? 1 : 0;
    }

    std::string scan_config_path = argv[1];
    std::string robot_config_path = argv[2];

    // Install signal handlers
    std::signal(SIGINT, SignalHandler);
    std::signal(SIGTERM, SignalHandler);

    try {
        // Load configs
        spdlog::info("Loading robot config from {}", robot_config_path);
        auto robot_config = arm_commander::load_config(robot_config_path);

        spdlog::info("Loading scan config from {}", scan_config_path);
        auto scan_config = arm_commander::load_scan_config(scan_config_path);

        spdlog::info("Session '{}': {} scan(s)",
            scan_config.session_name, scan_config.scans.size());

        // Create ArmCommander
        arm_commander::ArmCommander commander(robot_config);
        g_commander.store(&commander, std::memory_order_release);

        // Connect
        commander.connect();

        // Run each scan in the session
        int total = static_cast<int>(scan_config.scans.size());
        for (int i = 0; i < total; ++i) {
            if (commander.stop_was_requested()) {
                spdlog::info("Stop requested -- aborting remaining scans");
                break;
            }

            bool ok = run_scan(commander, scan_config.scans[i], scan_config.motion,
                scan_config.record, robot_config.safety.max_contact_wrench,
                scan_config.session_name, i, total);
            if (!ok) {
                spdlog::warn("Scan {} interrupted", i + 1);
                break;
            }
        }

        spdlog::info("All scans complete!");
        commander.shutdown();

    } catch (const std::exception& e) {
        spdlog::error("Fatal: {}", e.what());
        auto* cmd = g_commander.load(std::memory_order_acquire);
        if (cmd) {
            try {
                cmd->shutdown();
            } catch (const std::exception& e2) {
                spdlog::error("Shutdown failed: {}", e2.what());
            }
        }
        g_commander.store(nullptr, std::memory_order_release);
        return 1;
    }

    g_commander.store(nullptr, std::memory_order_release);
    spdlog::info("Done");
    return 0;
}
