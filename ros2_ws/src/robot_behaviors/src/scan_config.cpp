/**
 * @file scan_config.cpp
 * @brief Load ScanSessionConfig from YAML.
 *
 * Merge order: hardcoded defaults < session "defaults" < per-scan overrides.
 * This mirrors the Python scan_node's _get_scan_params() logic.
 */

#include "robot_behaviors/scan_config.hpp"

#include <yaml-cpp/yaml.h>

#include <chrono>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <stdexcept>

namespace arm_commander {

namespace {

std::string timestamp_name(const std::string& prefix)
{
    auto now = std::chrono::system_clock::now();
    auto t = std::chrono::system_clock::to_time_t(now);
    std::tm tm{};
    localtime_r(&t, &tm);
    std::ostringstream ss;
    ss << prefix << std::put_time(&tm, "%Y%m%d_%H%M%S");
    return ss.str();
}

/** Apply YAML node overrides to a ScanParams struct. */
void apply_overrides(ScanParams& params, const YAML::Node& node)
{
    if (!node || !node.IsMap())
        return;

    if (node["home_pose"] && node["home_pose"].IsSequence() && node["home_pose"].size() == 6) {
        for (size_t i = 0; i < 6; ++i) {
            params.home_pose[i] = node["home_pose"][i].as<double>();
        }
    }
    if (node["y_scan_range"])
        params.y_scan_range = node["y_scan_range"].as<double>();
    if (node["scan_speed"])
        params.scan_speed = node["scan_speed"].as<double>();
    if (node["contact_force"])
        params.contact_force = node["contact_force"].as<double>();
    if (node["search_velocity"])
        params.search_velocity = node["search_velocity"].as<double>();
    if (node["rz_start"])
        params.rz_start = node["rz_start"].as<double>();
    if (node["rz_end"])
        params.rz_end = node["rz_end"].as<double>();
    if (node["rz_step"])
        params.rz_step = node["rz_step"].as<double>();
    if (node["rx_start"])
        params.rx_start = node["rx_start"].as<double>();
    if (node["rx_end"])
        params.rx_end = node["rx_end"].as<double>();
    if (node["rx_step"])
        params.rx_step = node["rx_step"].as<double>();

    if (node["x_offsets_mm"] && node["x_offsets_mm"].IsSequence()) {
        params.x_offsets_mm.clear();
        for (size_t i = 0; i < node["x_offsets_mm"].size(); ++i) {
            params.x_offsets_mm.push_back(node["x_offsets_mm"][i].as<double>());
        }
    }
}

}  // namespace

ScanSessionConfig load_scan_config(const std::string& path)
{
    YAML::Node root;
    try {
        root = YAML::LoadFile(path);
    } catch (const YAML::Exception& e) {
        throw std::runtime_error("Failed to load scan config: " + std::string(e.what()));
    }

    ScanSessionConfig config;

    // Session name
    config.session_name =
        root["session_name"].as<std::string>(timestamp_name("session_"));

    // Motion config (global)
    if (auto m = root["motion"]) {
        config.motion.movel_vel = m["movel_vel"].as<double>(config.motion.movel_vel);
        config.motion.max_linear_vel =
            m["max_linear_vel"].as<double>(config.motion.max_linear_vel);
        config.motion.max_angular_vel =
            m["max_angular_vel"].as<double>(config.motion.max_angular_vel);
        config.motion.max_linear_acc =
            m["max_linear_acc"].as<double>(config.motion.max_linear_acc);
        config.motion.max_angular_acc =
            m["max_angular_acc"].as<double>(config.motion.max_angular_acc);
        config.motion.z_stiffness = m["z_stiffness"].as<double>(config.motion.z_stiffness);
        config.motion.xy_stiffness = m["xy_stiffness"].as<double>(config.motion.xy_stiffness);
        config.motion.rot_stiffness = m["rot_stiffness"].as<double>(config.motion.rot_stiffness);
        config.motion.damping_ratio = m["damping_ratio"].as<double>(config.motion.damping_ratio);
        config.motion.x_amplitude = m["x_amplitude"].as<double>(config.motion.x_amplitude);
        config.motion.x_period = m["x_period"].as<double>(config.motion.x_period);
    }

    // Record config
    if (auto r = root["record"]) {
        config.record.enabled = r["enabled"].as<bool>(config.record.enabled);
        config.record.output_dir = r["output_dir"].as<std::string>(config.record.output_dir);
        if (r["topics"] && r["topics"].IsSequence()) {
            config.record.topics.clear();
            for (size_t i = 0; i < r["topics"].size(); ++i) {
                config.record.topics.push_back(r["topics"][i].as<std::string>());
            }
        }
    }

    // Default output dir if not set
    if (config.record.output_dir.empty()) {
        const char* home = std::getenv("HOME");
        if (!home) {
            throw std::runtime_error("$HOME not set and record.output_dir not provided");
        }
        config.record.output_dir = std::string(home) + "/VisualFT/data";
    }

    // Build scan entries
    ScanParams session_defaults;
    if (root["defaults"]) {
        apply_overrides(session_defaults, root["defaults"]);
    }

    if (!root["scans"] || !root["scans"].IsSequence() || root["scans"].size() == 0) {
        throw std::runtime_error("scan config must have at least one entry in 'scans'");
    }

    for (size_t i = 0; i < root["scans"].size(); ++i) {
        auto scan_node = root["scans"][i];
        ScanEntry entry;
        entry.name = scan_node["name"].as<std::string>(
            "scan_" + std::to_string(i));

        // Start from session defaults, then apply per-scan overrides
        entry.params = session_defaults;
        apply_overrides(entry.params, scan_node);

        // Validate per-scan params
        const auto& p = entry.params;
        if (p.scan_speed <= 0.0)
            throw std::runtime_error("scan_speed must be positive (scan: " + entry.name + ")");
        if (p.contact_force <= 0.0)
            throw std::runtime_error("contact_force must be positive (scan: " + entry.name + ")");
        if (p.search_velocity <= 0.0)
            throw std::runtime_error("search_velocity must be positive (scan: " + entry.name + ")");
        if (p.rz_step <= 0.0)
            throw std::runtime_error("rz_step must be positive (scan: " + entry.name + ")");
        if (p.y_scan_range <= 0.0)
            throw std::runtime_error("y_scan_range must be positive (scan: " + entry.name + ")");
        if (p.x_offsets_mm.empty())
            throw std::runtime_error("x_offsets_mm must not be empty (scan: " + entry.name + ")");

        config.scans.push_back(std::move(entry));
    }

    // Validate motion config
    if (config.motion.max_linear_vel <= 0.0)
        throw std::runtime_error("motion.max_linear_vel must be positive");
    if (config.motion.movel_vel <= 0.0)
        throw std::runtime_error("motion.movel_vel must be positive");

    return config;
}

}  // namespace arm_commander
