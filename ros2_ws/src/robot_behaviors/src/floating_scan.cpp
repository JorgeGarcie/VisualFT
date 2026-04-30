/**
 * @file floating_scan.cpp
 * @brief Hand-guided scanning: home → zero FT → Cartesian floating.
 *
 * Uses NRT Cartesian impedance with zero stiffness for free-floating
 * in Cartesian space (operator pushes TCP, it moves in straight lines).
 *
 * Usage:
 *     floating_scan <config_path>
 *
 * Ctrl+C to stop. Robot switches to IDLE cleanly.
 */

#include "arm_commander/arm_commander.hpp"
#include "arm_commander/config.hpp"

#include <spdlog/spdlog.h>

#include <atomic>
#include <csignal>
#include <iostream>
#include <string>

namespace {

/** Atomic pointer for safe signal handler access. */
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
    // Parse args
    if (argc < 2 || std::string(argv[1]) == "-h" || std::string(argv[1]) == "--help") {
        std::cout << "Usage: floating_scan <config_path>" << std::endl;
        std::cout << "  config_path: path to robot.yaml" << std::endl;
        return (argc < 2) ? 1 : 0;
    }
    std::string config_path = argv[1];

    // Install signal handlers
    std::signal(SIGINT, SignalHandler);
    std::signal(SIGTERM, SignalHandler);

    try {
        // Load config
        spdlog::info("Loading config from {}", config_path);
        auto config = arm_commander::load_config(config_path);

        // Create ArmCommander
        arm_commander::ArmCommander commander(config);
        g_commander.store(&commander, std::memory_order_release);

        // Sequence: connect → home → zero FT → Cartesian float
        commander.connect();
        commander.home();
        commander.zero_ft();

        spdlog::info("Press Ctrl+C to stop");
        // TODO: tune damping on hardware (J1-J3 shoulder/elbow, J4-J7 wrist)
        commander.float_joints({10.0, 10.0, 8.0, 4.0, 4.0, 4.0, 4.0});

        // float_cartesian blocks until stop requested
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
