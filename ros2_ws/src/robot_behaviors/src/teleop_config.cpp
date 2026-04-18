/**
 * @file teleop_config.cpp
 * @brief YAML loader for TeleopConfig.
 */

#include "robot_behaviors/teleop_config.hpp"

#include <yaml-cpp/yaml.h>
#include <spdlog/spdlog.h>

#include <stdexcept>

namespace arm_commander {

namespace {

template <typename T>
T get_or(const YAML::Node& node, const std::string& key, const T& fallback)
{
    if (node[key]) {
        return node[key].as<T>();
    }
    return fallback;
}

Eigen::Vector3d read_vec3(const YAML::Node& node)
{
    if (!node || !node.IsSequence() || node.size() != 3) {
        throw std::runtime_error("Expected a 3-element sequence for Vector3d");
    }
    return Eigen::Vector3d(
        node[0].as<double>(), node[1].as<double>(), node[2].as<double>());
}

Eigen::Matrix3d read_mat3(const YAML::Node& node)
{
    if (!node || !node.IsSequence() || node.size() != 9) {
        throw std::runtime_error("Expected a 9-element sequence for Matrix3d (row-major)");
    }
    Eigen::Matrix3d m;
    for (int i = 0; i < 9; ++i) {
        m(i / 3, i % 3) = node[i].as<double>();
    }
    return m;
}

std::array<double, 6> read_arr6(const YAML::Node& node,
    const std::array<double, 6>& fallback)
{
    if (!node || !node.IsSequence() || node.size() != 6) {
        return fallback;
    }
    std::array<double, 6> arr;
    for (int i = 0; i < 6; ++i) {
        arr[i] = node[i].as<double>();
    }
    return arr;
}

}  // namespace

TeleopConfig load_teleop_config(const std::string& path)
{
    YAML::Node root;
    try {
        root = YAML::LoadFile(path);
    } catch (const std::exception& e) {
        throw std::runtime_error("Failed to load teleop config '" + path + "': " + e.what());
    }

    TeleopConfig cfg;

    // ZMQ
    if (auto zmq = root["zmq"]) {
        cfg.zmq.host = get_or<std::string>(zmq, "host", cfg.zmq.host);
        cfg.zmq.keypoint_port = get_or<int>(zmq, "keypoint_port", cfg.zmq.keypoint_port);
        cfg.zmq.pause_port = get_or<int>(zmq, "pause_port", cfg.zmq.pause_port);
    }

    // Retargeting
    if (auto rt = root["retargeting"]) {
        cfg.retargeting.vr_workspace =
            get_or<double>(rt, "vr_workspace", cfg.retargeting.vr_workspace);
        if (rt["robot_workspace"]) {
            cfg.retargeting.robot_workspace = read_vec3(rt["robot_workspace"]);
        }
        if (rt["vr_to_robot_rot"]) {
            cfg.retargeting.vr_to_robot_rot = read_mat3(rt["vr_to_robot_rot"]);
        }
        if (rt["robot_home"]) {
            cfg.retargeting.robot_home = read_vec3(rt["robot_home"]);
        }
        cfg.retargeting.deadzone_radius =
            get_or<double>(rt, "deadzone_radius", cfg.retargeting.deadzone_radius);
        cfg.retargeting.orientation_scale =
            get_or<double>(rt, "orientation_scale", cfg.retargeting.orientation_scale);
        cfg.retargeting.hold_duration =
            get_or<int>(rt, "hold_duration", cfg.retargeting.hold_duration);
    }

    // Safety
    if (auto s = root["safety"]) {
        if (s["pos_min"]) cfg.safety.pos_min = read_vec3(s["pos_min"]);
        if (s["pos_max"]) cfg.safety.pos_max = read_vec3(s["pos_max"]);
        cfg.safety.max_linear_vel =
            get_or<double>(s, "max_linear_vel", cfg.safety.max_linear_vel);
        cfg.safety.max_angular_vel =
            get_or<double>(s, "max_angular_vel", cfg.safety.max_angular_vel);
        cfg.safety.force_threshold =
            get_or<double>(s, "force_threshold", cfg.safety.force_threshold);
    }

    // Impedance
    if (auto imp = root["impedance"]) {
        cfg.impedance.stiffness = read_arr6(imp["stiffness"], cfg.impedance.stiffness);
        cfg.impedance.damping = read_arr6(imp["damping"], cfg.impedance.damping);
    }

    // Filter
    if (auto f = root["filter"]) {
        cfg.filter.freq = get_or<double>(f, "freq", cfg.filter.freq);
        cfg.filter.min_cutoff_pos = get_or<double>(f, "min_cutoff_pos", cfg.filter.min_cutoff_pos);
        cfg.filter.beta_pos = get_or<double>(f, "beta_pos", cfg.filter.beta_pos);
        cfg.filter.min_cutoff_rot = get_or<double>(f, "min_cutoff_rot", cfg.filter.min_cutoff_rot);
        cfg.filter.beta_rot = get_or<double>(f, "beta_rot", cfg.filter.beta_rot);
        cfg.filter.d_cutoff = get_or<double>(f, "d_cutoff", cfg.filter.d_cutoff);
    }

    // Dropout
    if (auto d = root["dropout"]) {
        cfg.dropout.timeout_ms = get_or<double>(d, "timeout_ms", cfg.dropout.timeout_ms);
        cfg.dropout.max_extrapolate_ms =
            get_or<double>(d, "max_extrapolate_ms", cfg.dropout.max_extrapolate_ms);
        cfg.dropout.decay_rate = get_or<double>(d, "decay_rate", cfg.dropout.decay_rate);
    }

    // Z-approach mode
    if (auto za = root["z_approach"]) {
        cfg.z_approach.enabled = get_or<bool>(za, "enabled", cfg.z_approach.enabled);
        if (za["xy_pos"] && za["xy_pos"].IsSequence() && za["xy_pos"].size() == 2) {
            cfg.z_approach.xy_pos = Eigen::Vector2d(
                za["xy_pos"][0].as<double>(), za["xy_pos"][1].as<double>());
        }
        cfg.z_approach.z_stiffness =
            get_or<double>(za, "z_stiffness", cfg.z_approach.z_stiffness);
        cfg.z_approach.z_damping =
            get_or<double>(za, "z_damping", cfg.z_approach.z_damping);
    }

    // Loop rate
    cfg.loop_rate_hz = get_or<double>(root, "loop_rate_hz", cfg.loop_rate_hz);

    spdlog::info("Teleop config loaded from {}", path);
    spdlog::info("  ZMQ: {}:{} (frames), {} (pause)",
        cfg.zmq.host, cfg.zmq.keypoint_port, cfg.zmq.pause_port);
    spdlog::info("  Loop rate: {} Hz", cfg.loop_rate_hz);
    spdlog::info("  Mode: {}", cfg.z_approach.enabled ? "z-approach" : "full 6DOF");
    spdlog::info("  Orientation scale: {}", cfg.retargeting.orientation_scale);
    spdlog::info("  Deadzone: {} m", cfg.retargeting.deadzone_radius);
    spdlog::info("  Force threshold: {} N", cfg.safety.force_threshold);
    if (cfg.z_approach.enabled) {
        spdlog::info("  Z-approach XY: [{:.3f}, {:.3f}]",
            cfg.z_approach.xy_pos.x(), cfg.z_approach.xy_pos.y());
        spdlog::info("  Z-approach stiffness: {} N/m, damping: {}",
            cfg.z_approach.z_stiffness, cfg.z_approach.z_damping);
    }

    return cfg;
}

}  // namespace arm_commander
