/**
 * @file teleop.cpp
 * @brief VR hand-tracking teleop for Flexiv Rizon4.
 *
 * Subscribes to transformed hand frame via ZMQ (raw bytes, no pickle),
 * retargets to robot workspace with clutching + 1-Euro filter + deadzone,
 * and streams Cartesian impedance commands via ArmCommander.
 *
 * Data flow:
 *   Quest APK -> leapft OculusVRHandDetector (port 8087->8088)
 *   -> leapft TransformHandPositionCoords (port 8088->8089)
 *   -> THIS (subscribes port 8089 via cppzmq)
 *
 * Usage:
 *     teleop <teleop_config_path> <robot_config_path>
 *
 * Ctrl+C to stop. Robot switches to IDLE cleanly.
 */

#include "arm_commander/arm_commander.hpp"
#include "arm_commander/config.hpp"
#include "arm_commander/teleop_config.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <spdlog/spdlog.h>
#include <zmq.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstring>
#include <iostream>
#include <optional>
#include <string>
#include <thread>

namespace {

// ── Signal handling ─────────────────────────────────────────────────────────

arm_commander::ArmCommander* g_commander = nullptr;

void SignalHandler(int signum)
{
    (void)signum;
    spdlog::info("Caught signal, shutting down...");
    if (g_commander) {
        g_commander->request_stop();
    }
}

// No default — robot config path must be provided as CLI arg.

// ── 1-Euro Filter ───────────────────────────────────────────────────────────

class OneEuroFilter {
public:
    OneEuroFilter(double freq, double min_cutoff, double beta, double d_cutoff = 1.0)
        : freq_(freq)
        , min_cutoff_(min_cutoff)
        , beta_(beta)
        , d_cutoff_(d_cutoff)
        , initialized_(false)
    {
    }

    void reset()
    {
        initialized_ = false;
    }

    Eigen::Vector3d filter(const Eigen::Vector3d& x, double t)
    {
        if (!initialized_) {
            x_prev_ = x;
            dx_prev_ = Eigen::Vector3d::Zero();
            t_prev_ = t;
            initialized_ = true;
            return x;
        }

        double dt = std::max(t - t_prev_, 1e-6);
        t_prev_ = t;

        // Derivative estimate (low-pass filtered)
        Eigen::Vector3d dx = (x - x_prev_) / dt;
        double a_d = alpha(d_cutoff_, dt);
        Eigen::Vector3d dx_hat = a_d * dx + (1.0 - a_d) * dx_prev_;
        dx_prev_ = dx_hat;

        // Adaptive cutoff per element, then filter
        Eigen::Vector3d x_hat;
        for (int i = 0; i < 3; ++i) {
            double cutoff = min_cutoff_ + beta_ * std::abs(dx_hat(i));
            double a = alpha(cutoff, dt);
            x_hat(i) = a * x(i) + (1.0 - a) * x_prev_(i);
        }
        x_prev_ = x_hat;
        return x_hat;
    }

private:
    static double alpha(double cutoff, double dt)
    {
        double tau = 1.0 / (2.0 * M_PI * cutoff);
        return 1.0 / (1.0 + tau / dt);
    }

    double freq_, min_cutoff_, beta_, d_cutoff_;
    bool initialized_;
    Eigen::Vector3d x_prev_, dx_prev_;
    double t_prev_ = 0.0;
};

// ── Dropout Handler ─────────────────────────────────────────────────────────

class DropoutHandler {
public:
    DropoutHandler(double timeout_ms, double max_extrapolate_ms, double decay_rate)
        : timeout_ms_(timeout_ms)
        , max_extrapolate_ms_(max_extrapolate_ms)
        , decay_rate_(decay_rate)
    {
    }

    /**
     * Update with new pose (or nullopt if no data).
     * Returns (output_pose, is_dropout).
     */
    std::pair<Eigen::Vector3d, bool> update(
        const std::optional<Eigen::Vector3d>& pose, double t)
    {
        if (pose.has_value()) {
            if (has_prev_) {
                double dt = std::max(t - prev_t_, 1e-6);
                last_velocity_ = (pose.value() - prev_pose_) / dt;
            }
            prev_pose_ = pose.value();
            prev_t_ = t;
            has_prev_ = true;
            last_valid_pose_ = pose.value();
            last_valid_t_ = t;
            has_valid_ = true;
            in_dropout_ = false;
            return {pose.value(), false};
        }

        // No reading
        if (!has_valid_) {
            return {Eigen::Vector3d::Zero(), true};
        }

        double dt_since_valid_ms = (t - last_valid_t_) * 1000.0;
        in_dropout_ = dt_since_valid_ms > timeout_ms_;

        if (!in_dropout_) {
            return {last_valid_pose_, false};
        }

        // Dropout — extrapolate with decaying velocity
        if (has_prev_ && dt_since_valid_ms < max_extrapolate_ms_) {
            double dt_s = t - last_valid_t_;
            double decay = std::exp(-decay_rate_ * dt_s);
            Eigen::Vector3d extrapolated =
                last_valid_pose_ + last_velocity_ * dt_s * decay;
            return {extrapolated, true};
        }

        // Beyond extrapolation — hold last pose
        return {last_valid_pose_, true};
    }

private:
    double timeout_ms_, max_extrapolate_ms_, decay_rate_;
    Eigen::Vector3d last_valid_pose_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d last_velocity_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d prev_pose_ = Eigen::Vector3d::Zero();
    double last_valid_t_ = 0.0;
    double prev_t_ = 0.0;
    bool has_valid_ = false;
    bool has_prev_ = false;
    bool in_dropout_ = false;
};

// ── Task-Space Retargeter ───────────────────────────────────────────────────

class TaskSpaceRetargeter {
public:
    explicit TaskSpaceRetargeter(const arm_commander::RetargetingConfig& cfg,
        const arm_commander::TeleopSafetyConfig& safety,
        const arm_commander::OneEuroFilterConfig& filter_cfg)
        : cfg_(cfg)
        , safety_(safety)
        , pos_filter_(filter_cfg.freq, filter_cfg.min_cutoff_pos,
              filter_cfg.beta_pos, filter_cfg.d_cutoff)
        , rot_filter_(filter_cfg.freq, filter_cfg.min_cutoff_rot,
              filter_cfg.beta_rot, filter_cfg.d_cutoff)
    {
        // Per-axis scale = robot_workspace / vr_workspace
        scale_ = cfg_.robot_workspace / cfg_.vr_workspace;
    }

    /**
     * Handle clutching — grip true = engaged, false = decoupled.
     * On engage: record VR offset + robot anchor, start hold period.
     */
    void update_clutch(bool grip, const Eigen::Vector3d& vr_pos,
        const Eigen::Matrix3d& vr_rot,
        const Eigen::Vector3d& current_robot_pos,
        const Eigen::Quaterniond& current_robot_quat)
    {
        if (grip && !clutch_engaged_) {
            // Engage
            clutch_engaged_ = true;
            vr_offset_ = vr_pos;
            robot_anchor_ = current_robot_pos;
            robot_anchor_quat_ = current_robot_quat;
            rot_offset_ = Eigen::Quaterniond(vr_rot);
            hold_frames_ = cfg_.hold_duration;

            // Reset filters
            pos_filter_.reset();
            rot_filter_.reset();

        } else if (!grip) {
            clutch_engaged_ = false;
        }
    }

    /**
     * Map VR pose -> robot EE pose.
     * Returns (position, quaternion). Quaternion is [x,y,z,w] (Eigen default).
     */
    std::pair<Eigen::Vector3d, Eigen::Quaterniond> retarget(
        const Eigen::Vector3d& vr_pos, const Eigen::Matrix3d& vr_rot, double t)
    {
        if (!clutch_engaged_) {
            return {cfg_.robot_home, Eigen::Quaterniond::Identity()};
        }

        // Post-engage hold
        if (hold_frames_ > 0) {
            --hold_frames_;
            vr_offset_ = vr_pos;
            rot_offset_ = Eigen::Quaterniond(vr_rot);
            return {robot_anchor_, robot_anchor_quat_};
        }

        // ── Position ──

        // Delta in VR frame
        Eigen::Vector3d delta_vr = vr_pos - vr_offset_;

        // Deadzone
        double dist = delta_vr.norm();
        if (dist < cfg_.deadzone_radius) {
            delta_vr = Eigen::Vector3d::Zero();
        } else {
            // Smooth deadzone: subtract deadzone radius in direction of motion
            delta_vr *= (1.0 - cfg_.deadzone_radius / dist);
        }

        // Rotate to robot frame
        Eigen::Vector3d delta_robot = cfg_.vr_to_robot_rot * delta_vr;

        // Scale
        delta_robot = delta_robot.cwiseProduct(scale_);

        // Add to anchor
        Eigen::Vector3d target_pos = robot_anchor_ + delta_robot;

        // Safety clamp
        target_pos = target_pos.cwiseMax(safety_.pos_min).cwiseMin(safety_.pos_max);

        // Filter position
        target_pos = pos_filter_.filter(target_pos, t);

        // ── Orientation ──

        // Delta rotation = current_vr * inverse(vr_at_engage)
        Eigen::Quaterniond vr_rot_q(vr_rot);
        Eigen::Quaterniond delta_rot = vr_rot_q * rot_offset_.inverse();

        // Scale rotation (reduce magnitude via rotvec)
        Eigen::AngleAxisd aa(delta_rot);
        Eigen::Vector3d rotvec = aa.axis() * aa.angle();
        rotvec *= cfg_.orientation_scale;

        // Filter orientation (as rotvec)
        rotvec = rot_filter_.filter(rotvec, t);

        // Apply scaled delta to robot anchor orientation
        double angle = rotvec.norm();
        Eigen::Quaterniond scaled_delta = Eigen::Quaterniond::Identity();
        if (angle > 1e-9) {
            scaled_delta = Eigen::Quaterniond(Eigen::AngleAxisd(angle, rotvec / angle));
        }

        Eigen::Quaterniond target_quat = robot_anchor_quat_ * scaled_delta;
        target_quat.normalize();

        return {target_pos, target_quat};
    }

    bool is_engaged() const { return clutch_engaged_; }

private:
    arm_commander::RetargetingConfig cfg_;
    arm_commander::TeleopSafetyConfig safety_;
    OneEuroFilter pos_filter_;
    OneEuroFilter rot_filter_;
    Eigen::Vector3d scale_;

    // Clutch state
    bool clutch_engaged_ = false;
    Eigen::Vector3d vr_offset_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d robot_anchor_ = Eigen::Vector3d::Zero();
    Eigen::Quaterniond robot_anchor_quat_ = Eigen::Quaterniond::Identity();
    Eigen::Quaterniond rot_offset_ = Eigen::Quaterniond::Identity();
    int hold_frames_ = 0;
};

// ── ZMQ Raw Bytes Receiver ──────────────────────────────────────────────────

/**
 * Parse a hand frame from raw bytes.
 * Wire format: topic prefix "transformed_hand_frame " + 12 doubles (96 bytes).
 * Layout: [origin(3), x_axis(3), y_axis(3), z_axis(3)]
 *
 * Returns true if parse succeeded.
 */
bool parse_hand_frame(const zmq::message_t& msg,
    Eigen::Vector3d& origin, Eigen::Matrix3d& rotation)
{
    const std::string topic_prefix = "transformed_hand_frame ";
    const size_t prefix_len = topic_prefix.size();
    const size_t payload_size = 12 * sizeof(double);  // 96 bytes

    if (msg.size() < prefix_len + payload_size) {
        return false;
    }

    // Verify topic prefix
    if (std::memcmp(msg.data(), topic_prefix.data(), prefix_len) != 0) {
        return false;
    }

    // Parse 12 doubles from raw bytes
    double data[12];
    std::memcpy(data, static_cast<const char*>(msg.data()) + prefix_len, payload_size);

    origin = Eigen::Vector3d(data[0], data[1], data[2]);

    // x_axis, y_axis, z_axis form columns of rotation matrix
    Eigen::Vector3d x_axis(data[3], data[4], data[5]);
    Eigen::Vector3d y_axis(data[6], data[7], data[8]);
    Eigen::Vector3d z_axis(data[9], data[10], data[11]);

    rotation.col(0) = x_axis;
    rotation.col(1) = y_axis;
    rotation.col(2) = z_axis;

    // Ensure valid rotation matrix (det > 0)
    if (rotation.determinant() < 0) {
        rotation.col(0) = -rotation.col(0);
    }

    return true;
}

/**
 * Parse pause state from raw bytes.
 * Wire format: topic prefix "pause " + 1 byte (bool).
 */
bool parse_pause(const zmq::message_t& msg, bool& paused)
{
    const std::string topic_prefix = "pause ";
    const size_t prefix_len = topic_prefix.size();

    if (msg.size() < prefix_len + 1) {
        return false;
    }

    if (std::memcmp(msg.data(), topic_prefix.data(), prefix_len) != 0) {
        return false;
    }

    // Single byte: 0 = STOP (not engaged), 1 = CONT (engaged)
    uint8_t val = 0;
    std::memcpy(&val, static_cast<const char*>(msg.data()) + prefix_len, 1);
    paused = (val != 0);
    return true;
}

// ── Velocity-limited position ───────────────────────────────────────────────

Eigen::Vector3d velocity_limit(const Eigen::Vector3d& target,
    const Eigen::Vector3d& last_cmd, double dt, double max_vel)
{
    Eigen::Vector3d delta = target - last_cmd;
    double dist = delta.norm();
    if (dist < 1e-9) return target;
    double max_step = max_vel * dt;
    if (dist > max_step) {
        return last_cmd + delta * (max_step / dist);
    }
    return target;
}

// ── Helpers ─────────────────────────────────────────────────────────────────

/** Convert Eigen quaternion (x,y,z,w) to RDK pose [x,y,z, qw,qx,qy,qz]. */
std::array<double, 7> to_rdk_pose(const Eigen::Vector3d& pos,
    const Eigen::Quaterniond& quat)
{
    return {pos.x(), pos.y(), pos.z(),
        quat.w(), quat.x(), quat.y(), quat.z()};
}

/** Extract position from RDK pose [x,y,z,qw,qx,qy,qz]. */
Eigen::Vector3d rdk_pos(const std::array<double, 7>& pose)
{
    return {pose[0], pose[1], pose[2]};
}

/** Extract quaternion from RDK pose [x,y,z,qw,qx,qy,qz] -> Eigen (w,x,y,z). */
Eigen::Quaterniond rdk_quat(const std::array<double, 7>& pose)
{
    return Eigen::Quaterniond(pose[3], pose[4], pose[5], pose[6]);
}

double force_norm(const std::array<double, 6>& wrench)
{
    return std::sqrt(wrench[0] * wrench[0] + wrench[1] * wrench[1] +
                     wrench[2] * wrench[2]);
}

double now_seconds()
{
    using clock = std::chrono::steady_clock;
    static auto t0 = clock::now();
    return std::chrono::duration<double>(clock::now() - t0).count();
}

}  // namespace

int main(int argc, char* argv[])
{
    // ── Parse args ──────────────────────────────────────────────────────────
    if (argc < 3 || std::string(argv[1]) == "-h" || std::string(argv[1]) == "--help") {
        std::cout << "Usage: teleop <teleop_config_path> <robot_config_path>"
                  << std::endl;
        std::cout << "  teleop_config_path: path to teleop.yaml" << std::endl;
        std::cout << "  robot_config_path:  path to robot.yaml" << std::endl;
        return (argc < 3) ? 1 : 0;
    }

    std::string teleop_config_path = argv[1];
    std::string robot_config_path = argv[2];

    // ── Signal handlers ─────────────────────────────────────────────────────
    std::signal(SIGINT, SignalHandler);
    std::signal(SIGTERM, SignalHandler);

    try {
        // ── Load configs ────────────────────────────────────────────────────
        spdlog::info("Loading robot config from {}", robot_config_path);
        auto robot_config = arm_commander::load_config(robot_config_path);

        spdlog::info("Loading teleop config from {}", teleop_config_path);
        auto teleop_cfg = arm_commander::load_teleop_config(teleop_config_path);

        // ── Create ArmCommander ─────────────────────────────────────────────
        arm_commander::ArmCommander commander(robot_config);
        g_commander = &commander;

        // ── Connect + init ──────────────────────────────────────────────────
        commander.connect();
        commander.home();
        commander.zero_ft();

        // Send one stream_cartesian to switch to NRT_CARTESIAN_MOTION_FORCE,
        // then configure impedance (requires active streaming mode).
        {
            auto init_state = commander.get_state();
            commander.stream_cartesian(init_state.tcp_pose, {0, 0, 0, 0, 0, 0});
        }
        commander.set_impedance(
            teleop_cfg.impedance.stiffness, teleop_cfg.impedance.damping);
        commander.set_force_control_axis({false, false, false, false, false, false});

        // ── Set up ZMQ subscribers ──────────────────────────────────────────
        zmq::context_t zmq_ctx(1);

        zmq::socket_t frame_sub(zmq_ctx, zmq::socket_type::sub);
        frame_sub.set(zmq::sockopt::conflate, 1);
        frame_sub.set(zmq::sockopt::rcvtimeo, 0);  // non-blocking
        std::string frame_addr = "tcp://" + teleop_cfg.zmq.host + ":" +
                                 std::to_string(teleop_cfg.zmq.keypoint_port);
        frame_sub.connect(frame_addr);
        frame_sub.set(zmq::sockopt::subscribe, "transformed_hand_frame");
        spdlog::info("ZMQ frame subscriber: {}", frame_addr);

        zmq::socket_t pause_sub(zmq_ctx, zmq::socket_type::sub);
        pause_sub.set(zmq::sockopt::conflate, 1);
        pause_sub.set(zmq::sockopt::rcvtimeo, 0);
        std::string pause_addr = "tcp://" + teleop_cfg.zmq.host + ":" +
                                 std::to_string(teleop_cfg.zmq.pause_port);
        pause_sub.connect(pause_addr);
        pause_sub.set(zmq::sockopt::subscribe, "pause");
        spdlog::info("ZMQ pause subscriber: {}", pause_addr);

        // ── Create retargeter + filters ─────────────────────────────────────
        TaskSpaceRetargeter retargeter(
            teleop_cfg.retargeting, teleop_cfg.safety, teleop_cfg.filter);
        DropoutHandler dropout(
            teleop_cfg.dropout.timeout_ms,
            teleop_cfg.dropout.max_extrapolate_ms,
            teleop_cfg.dropout.decay_rate);

        // ── Teleop state ────────────────────────────────────────────────────
        bool engaged = false;
        bool prev_engaged_set = false;
        Eigen::Vector3d last_vr_pos = Eigen::Vector3d::Zero();
        Eigen::Matrix3d last_vr_rot = Eigen::Matrix3d::Identity();
        bool has_vr_data = false;
        Eigen::Vector3d last_cmd_pos = Eigen::Vector3d::Zero();
        bool has_last_cmd = false;

        double dt = 1.0 / teleop_cfg.loop_rate_hz;
        int loop_count = 0;
        double t0 = now_seconds();

        spdlog::info("=== VR Teleop Active ({} Hz) ===", teleop_cfg.loop_rate_hz);
        spdlog::info("  Quest APK button to engage/disengage teleop");
        spdlog::info("  Ctrl+C to stop");

        // ── Main loop ───────────────────────────────────────────────────────
        while (!commander.stop_was_requested()) {
            double t_loop_start = now_seconds();
            double t = t_loop_start - t0;

            // ── Poll pause/engage state ─────────────────────────────────────
            {
                zmq::message_t msg;
                auto result = pause_sub.recv(msg, zmq::recv_flags::dontwait);
                if (result.has_value()) {
                    bool new_state = false;
                    if (parse_pause(msg, new_state)) {
                        if (!prev_engaged_set) {
                            prev_engaged_set = true;
                            engaged = false;  // first message — stay disengaged
                        } else {
                            engaged = new_state;
                        }
                    }
                }
            }

            // ── Poll hand frame ─────────────────────────────────────────────
            bool vr_valid = false;
            {
                zmq::message_t msg;
                auto result = frame_sub.recv(msg, zmq::recv_flags::dontwait);
                if (result.has_value()) {
                    Eigen::Vector3d origin;
                    Eigen::Matrix3d rotation;
                    if (parse_hand_frame(msg, origin, rotation)) {
                        if (!origin.isZero(1e-6)) {
                            last_vr_pos = origin;
                            last_vr_rot = rotation;
                            has_vr_data = true;
                            vr_valid = true;
                        }
                    }
                }
                if (!vr_valid && has_vr_data) {
                    vr_valid = true;  // use cached data
                }
            }

            // ── Dropout handling ────────────────────────────────────────────
            std::optional<Eigen::Vector3d> vr_pos_in;
            if (vr_valid) {
                vr_pos_in = last_vr_pos;
            }
            auto [vr_pos_safe, is_dropout] = dropout.update(vr_pos_in, t);

            if (is_dropout && loop_count % 90 == 0) {
                spdlog::warn("[{:.1f}s] VR DROPOUT -- holding last pose", t);
            }

            // ── Get robot state ─────────────────────────────────────────────
            auto robot_state = commander.get_state();
            Eigen::Vector3d current_pos = rdk_pos(robot_state.tcp_pose);
            Eigen::Quaterniond current_quat = rdk_quat(robot_state.tcp_pose);

            // ── Force safety check ──────────────────────────────────────────
            double fmag = force_norm(robot_state.wrench_in_world);
            if (fmag > teleop_cfg.safety.force_threshold) {
                spdlog::error("SAFETY STOP: force {:.1f}N > threshold {:.1f}N",
                    fmag, teleop_cfg.safety.force_threshold);
                commander.stop();
                break;
            }

            // ── Clutch ──────────────────────────────────────────────────────
            retargeter.update_clutch(
                engaged, vr_pos_safe, last_vr_rot, current_pos, current_quat);

            // ── Retarget ────────────────────────────────────────────────────
            auto [target_pos, target_quat] =
                retargeter.retarget(vr_pos_safe, last_vr_rot, t);

            // ── Velocity limit ──────────────────────────────────────────────
            if (has_last_cmd) {
                target_pos = velocity_limit(
                    target_pos, last_cmd_pos, dt,
                    teleop_cfg.safety.max_linear_vel);
            }

            // ── Send to robot ───────────────────────────────────────────────
            // When engaged: send retargeted pose.
            // When not engaged: resend current pose (heartbeat, holds position).
            std::array<double, 7> cmd_pose;
            if (retargeter.is_engaged()) {
                cmd_pose = to_rdk_pose(target_pos, target_quat);
            } else {
                cmd_pose = to_rdk_pose(current_pos, current_quat);
            }

            commander.stream_cartesian(cmd_pose, {0, 0, 0, 0, 0, 0},
                teleop_cfg.safety.max_linear_vel,
                teleop_cfg.safety.max_angular_vel);

            last_cmd_pos = retargeter.is_engaged() ? target_pos : current_pos;
            has_last_cmd = true;

            // ── Status (every 2s) ───────────────────────────────────────────
            int status_interval = static_cast<int>(teleop_cfg.loop_rate_hz * 2);
            if (loop_count > 0 && loop_count % status_interval == 0) {
                double avg_dt = (now_seconds() - t0) / loop_count;
                spdlog::info("[{:.1f}s] pos=[{:.3f},{:.3f},{:.3f}] engaged={} "
                             "force={:.1f}N rate={:.0f}Hz",
                    t, target_pos.x(), target_pos.y(), target_pos.z(),
                    retargeter.is_engaged(), fmag, 1.0 / avg_dt);
            }

            // ── Rate limit ──────────────────────────────────────────────────
            ++loop_count;
            double elapsed = now_seconds() - t_loop_start;
            if (elapsed < dt) {
                long sleep_us = static_cast<long>((dt - elapsed) * 1e6);
                std::this_thread::sleep_for(std::chrono::microseconds(sleep_us));
            }
        }

        // ── Shutdown ────────────────────────────────────────────────────────
        spdlog::info("Stopping robot...");
        commander.shutdown();

        frame_sub.close();
        pause_sub.close();
        zmq_ctx.close();

    } catch (const std::exception& e) {
        spdlog::error(e.what());
        if (g_commander) {
            try {
                g_commander->shutdown();
            } catch (...) {
            }
        }
        return 1;
    }

    g_commander = nullptr;
    spdlog::info("Teleop shutdown complete.");
    return 0;
}
