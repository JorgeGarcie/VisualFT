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

    return config;
}

}  // namespace arm_commander
