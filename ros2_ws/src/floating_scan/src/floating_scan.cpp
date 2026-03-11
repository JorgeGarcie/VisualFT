/**
 * @file floating_scan.cpp
 * @brief Hand-guided scanning: home → zero FT → RT joint floating.
 *
 * First consumer of ArmCommander. Config from robot.yaml.
 *
 * Usage:
 *     floating_scan <config_path>
 *
 * Ctrl+C to stop. Robot switches to IDLE cleanly.
 */

#include "arm_commander/arm_commander.hpp"
#include "arm_commander/config.hpp"

#include <spdlog/spdlog.h>

#include <csignal>
#include <iostream>
#include <string>

namespace {

/** Pointer for signal handler access */
arm_commander::ArmCommander* g_commander = nullptr;

/**
 * Joint velocity damping gains.
 * Joints 1-3 (shoulder/elbow): low damping → free translation.
 * Joints 4-7 (wrist): high damping → resist rotation.
 * Tune on hardware.
 */
const std::vector<double> kFloatingDamping = {10.0, 10.0, 5.0, 20.0, 20.0, 20.0, 20.0};

}  // namespace

void SignalHandler(int signum)
{
    (void)signum;
    spdlog::info("Caught signal, shutting down...");
    if (g_commander) {
        g_commander->request_stop();
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
        g_commander = &commander;

        // Sequence: connect → home → zero FT → float
        commander.connect();
        commander.home();
        commander.zero_ft();

        spdlog::info("Press Ctrl+C to stop");
        commander.float_joints(kFloatingDamping);

        // float_joints blocks until stop requested
        commander.shutdown();

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
    spdlog::info("Done");
    return 0;
}
