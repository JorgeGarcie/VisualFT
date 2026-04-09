/**
 * @file massage.cpp
 * @brief Force-controlled surface contact: hold constant pressing force on Z.
 *
 * Uses unified motion-force control: Z axis is force-controlled (maintains
 * constant pressing force), XY axes are motion-controlled (hold position).
 * If the surface moves, the robot adapts its Z position to maintain force.
 *
 * Usage:
 *     massage <robot_config>
 *
 * Ctrl+C to stop. Robot switches to IDLE cleanly.
 */

#include "arm_commander/arm_commander.hpp"
#include "arm_commander/config.hpp"

#include <flexiv/rdk/data.hpp>
#include <spdlog/spdlog.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <iostream>
#include <limits>
#include <string>
#include <thread>

namespace {

constexpr double PRESSING_FORCE = 5.0;  // N
constexpr double LOOP_RATE_HZ = 50.0;

std::atomic<arm_commander::ArmCommander*> g_commander{nullptr};

}  // namespace

void SignalHandler(int signum)
{
    (void)signum;
    auto* cmd = g_commander.load(std::memory_order_acquire);
    if (cmd) {
        cmd->request_stop();
    }
}

int main(int argc, char* argv[])
{
    if (argc < 2 || std::string(argv[1]) == "-h" || std::string(argv[1]) == "--help") {
        std::cout << "Usage: massage <robot_config>" << std::endl;
        std::cout << "  robot_config: path to robot.yaml" << std::endl;
        return (argc < 2) ? 1 : 0;
    }
    std::string config_path = argv[1];

    std::signal(SIGINT, SignalHandler);
    std::signal(SIGTERM, SignalHandler);

    try {
        spdlog::info("Loading config from {}", config_path);
        auto config = arm_commander::load_config(config_path);

        arm_commander::ArmCommander commander(config);
        g_commander.store(&commander, std::memory_order_release);

        // Setup: connect, home, zero FT, descend to contact
        commander.connect();
        commander.home();
        commander.zero_ft();
        commander.contact();

        // Record contact pose as the hold position for motion-controlled axes
        auto contact_state = commander.get_state();
        auto hold_pose = contact_state.tcp_pose;
        spdlog::info("Contact at Z={:.4f}m, holding XY position", hold_pose[2]);

        // Enter NRT Cartesian motion-force mode with an initial command
        commander.stream_cartesian(hold_pose, {0, 0, 0, 0, 0, 0});

        // Configure force control: Z = force, everything else = motion
        commander.set_force_control_frame(flexiv::rdk::CoordType::WORLD);
        commander.set_force_control_axis({false, false, true, false, false, false});

        // Disable max contact wrench limit — force is explicitly controlled on Z
        constexpr double INF = std::numeric_limits<double>::infinity();
        commander.set_max_contact_wrench({INF, INF, INF, INF, INF, INF});

        // Stiff XY + rotation, Z handled by force controller (stiffness irrelevant)
        commander.set_impedance(
            {2000.0, 2000.0, 0.0, 80.0, 80.0, 80.0},
            {0.7, 0.7, 0.7, 0.7, 0.7, 0.7});

        spdlog::info("Force control active: {:.1f}N on Z — press Ctrl+C to stop", PRESSING_FORCE);

        // Streaming loop: hold position + apply pressing force
        const auto period = std::chrono::duration<double>(1.0 / LOOP_RATE_HZ);
        while (!commander.stop_was_requested()) {
            auto loop_start = std::chrono::steady_clock::now();

            if (commander.has_fault()) {
                spdlog::error("Robot fault detected — stopping");
                break;
            }

            // Stream: hold XY at contact pose, push Z with constant force
            commander.stream_cartesian(
                hold_pose, {0.0, 0.0, PRESSING_FORCE, 0.0, 0.0, 0.0});

            // Sleep for remainder of period
            auto elapsed = std::chrono::steady_clock::now() - loop_start;
            if (elapsed < period) {
                std::this_thread::sleep_for(period - elapsed);
            }
        }

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
