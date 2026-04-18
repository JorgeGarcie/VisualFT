/**
 * @file config.cpp
 * @brief Load RobotConfig from YAML.
 */

#include "arm_commander/config.hpp"

#include <yaml-cpp/yaml.h>

#include <stdexcept>

namespace arm_commander {

RobotConfig load_config(const std::string& path)
{
    YAML::Node root;
    try {
        root = YAML::LoadFile(path);
    } catch (const YAML::Exception& e) {
        throw std::runtime_error("Failed to load config: " + std::string(e.what()));
    }

    RobotConfig config;

    if (auto robot = root["robot"]) {
        config.serial_number = robot["serial_number"].as<std::string>(config.serial_number);
        config.control_rate_hz = robot["control_rate_hz"].as<double>(config.control_rate_hz);
        config.max_linear_vel = robot["max_linear_vel"].as<double>(config.max_linear_vel);
        config.max_angular_vel = robot["max_angular_vel"].as<double>(config.max_angular_vel);
        config.max_linear_acc = robot["max_linear_acc"].as<double>(config.max_linear_acc);
        config.max_angular_acc = robot["max_angular_acc"].as<double>(config.max_angular_acc);
    }

    if (auto safety = root["safety"]) {
        if (auto ws = safety["workspace"]) {
            auto x = ws["x"];
            auto y = ws["y"];
            auto z = ws["z"];
            if (x && x.IsSequence() && x.size() == 2) {
                config.safety.workspace.x_min = x[0].as<double>();
                config.safety.workspace.x_max = x[1].as<double>();
            }
            if (y && y.IsSequence() && y.size() == 2) {
                config.safety.workspace.y_min = y[0].as<double>();
                config.safety.workspace.y_max = y[1].as<double>();
            }
            if (z && z.IsSequence() && z.size() == 2) {
                config.safety.workspace.z_min = z[0].as<double>();
                config.safety.workspace.z_max = z[1].as<double>();
            }
        }
        config.safety.max_force_z = safety["max_force_z"].as<double>(config.safety.max_force_z);
        config.safety.heartbeat_timeout =
            safety["heartbeat_timeout"].as<double>(config.safety.heartbeat_timeout);

        if (auto mcw = safety["max_contact_wrench"]) {
            if (mcw.IsSequence() && mcw.size() == 6) {
                for (size_t i = 0; i < 6; ++i) {
                    config.safety.max_contact_wrench[i] = mcw[i].as<double>();
                }
            }
        }
    }

    // ── Validate ──────────────────────────────────────────────────────────
    if (config.max_linear_vel <= 0.0)
        throw std::runtime_error("robot.max_linear_vel must be positive");
    if (config.max_angular_vel <= 0.0)
        throw std::runtime_error("robot.max_angular_vel must be positive");
    if (config.max_linear_acc <= 0.0)
        throw std::runtime_error("robot.max_linear_acc must be positive");
    if (config.max_angular_acc <= 0.0)
        throw std::runtime_error("robot.max_angular_acc must be positive");
    if (config.control_rate_hz <= 0.0)
        throw std::runtime_error("robot.control_rate_hz must be positive");

    auto& ws = config.safety.workspace;
    if (ws.x_min >= ws.x_max)
        throw std::runtime_error("safety.workspace: x_min must be < x_max");
    if (ws.y_min >= ws.y_max)
        throw std::runtime_error("safety.workspace: y_min must be < y_max");
    if (ws.z_min >= ws.z_max)
        throw std::runtime_error("safety.workspace: z_min must be < z_max");

    if (config.safety.max_force_z <= 0.0)
        throw std::runtime_error("safety.max_force_z must be positive");

    return config;
}

}  // namespace arm_commander
