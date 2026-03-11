/**
 * @file scan_config.hpp
 * @brief Scan session configuration loaded from YAML.
 *
 * Mirrors the Python scan_node session config structure:
 * - Session has a name, defaults, and a list of scans
 * - Each scan has per-scan parameter overrides
 * - Parameters merge: hardcoded defaults < session defaults < per-scan overrides
 */

#ifndef ARM_COMMANDER_SCAN_CONFIG_HPP_
#define ARM_COMMANDER_SCAN_CONFIG_HPP_

#include <array>
#include <string>
#include <vector>

namespace arm_commander {

/**
 * Parameters for a single scan pass configuration.
 * home_pose is in [mm, mm, mm, deg, deg, deg] (Elements format).
 */
struct ScanParams {
    std::array<double, 6> home_pose = {512.0, 260.0, 45.0, 0.05, -179.44, 0.0};
    double y_scan_range = 0.03;      // m
    double scan_speed = 0.04;        // m/s
    double contact_force = 20.0;     // N threshold for contact detection
    double search_velocity = 0.02;   // m/s descent speed
    double rz_start = 0.0;          // deg
    double rz_end = 175.0;          // deg
    double rz_step = 5.0;           // deg
    double rx_start = 0.0;          // deg
    double rx_end = 0.0;            // deg
    double rx_step = 5.0;           // deg
    std::vector<double> x_offsets_mm = {0.0, 1.0, 2.0, 1.0, -1.0, 0.0};
};

/** A single scan entry in a session. */
struct ScanEntry {
    std::string name;
    ScanParams params;  // merged: defaults < per-scan overrides
};

/** Impedance and motion parameters (global, not per-scan). */
struct ScanMotionConfig {
    double movel_vel = 0.05;       // m/s for MoveL primitives
    double max_linear_vel = 0.05;  // m/s streaming cap
    double max_angular_vel = 0.3;  // rad/s
    double max_linear_acc = 0.3;   // m/s^2
    double max_angular_acc = 0.5;  // rad/s^2

    // Impedance for scanning phase
    double z_stiffness = 5000.0;   // N/m
    double xy_stiffness = 5000.0;  // N/m
    double rot_stiffness = 1500.0; // Nm/rad
    double damping_ratio = 0.8;    // [0.3-0.8]

    // Sinusoidal X oscillation during Y sweep
    double x_amplitude = 0.0;     // m (0 = disabled)
    double x_period = 0.008;      // m (full oscillation per Y distance)
};

/** Topics to record in MCAP bags. */
struct RecordConfig {
    bool enabled = true;
    std::string output_dir;  // base output directory
    std::vector<std::string> topics = {
        "/scan/state",
        "/rdk/tcp_pose",
        "/rdk/wrench",
        "/coinft/wrench",
        "/image_raw",
    };
};

/** Full scan session configuration. */
struct ScanSessionConfig {
    std::string session_name;
    ScanMotionConfig motion;
    RecordConfig record;
    std::vector<ScanEntry> scans;
};

/** Load scan session config from a YAML file. */
ScanSessionConfig load_scan_config(const std::string& path);

}  // namespace arm_commander

#endif  // ARM_COMMANDER_SCAN_CONFIG_HPP_
