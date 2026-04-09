/**
 * @file scan_config.cpp
 * @brief Load ScanSessionConfig from YAML.
 *
 * Merge order: hardcoded defaults < session "defaults" < per-scan overrides.
 * This mirrors the Python scan_node's _get_scan_params() logic.
 */

#include "arm_commander/scan_config.hpp"

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
        config.record.output_dir = std::string(home ? home : ".") + "/VisualFT/data";
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

        config.scans.push_back(std::move(entry));
    }

    return config;
}

}  // namespace arm_commander
