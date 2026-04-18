/**
 * @file config.hpp
 * @brief Robot and safety configuration loaded from YAML.
 */

#ifndef ARM_COMMANDER_CONFIG_HPP_
#define ARM_COMMANDER_CONFIG_HPP_

#include <array>
#include <string>

namespace arm_commander {

struct WorkspaceBounds {
    double x_min = 0.30, x_max = 0.735;
    double y_min = -0.25, y_max = 0.430;
    double z_min = -0.02, z_max = 0.55;

    bool contains(double x, double y, double z) const
    {
        return x >= x_min && x <= x_max && y >= y_min && y <= y_max && z >= z_min && z <= z_max;
    }
};

struct SafetyConfig {
    WorkspaceBounds workspace;
    double max_force_z = 25.0;
    std::array<double, 6> max_contact_wrench = {50.0, 50.0, 50.0, 10.0, 10.0, 10.0};
    double heartbeat_timeout = 2.0;
};

struct RobotConfig {
    std::string serial_number = "Rizon4-062174";
    double control_rate_hz = 50.0;
    double max_linear_vel = 0.05;
    double max_angular_vel = 0.5;
    double max_linear_acc = 0.5;
    double max_angular_acc = 1.0;
    SafetyConfig safety;
};

/** Load RobotConfig from a YAML file. */
RobotConfig load_config(const std::string& path);

}  // namespace arm_commander

#endif  // ARM_COMMANDER_CONFIG_HPP_
