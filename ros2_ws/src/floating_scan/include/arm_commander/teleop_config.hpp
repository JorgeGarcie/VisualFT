/**
 * @file teleop_config.hpp
 * @brief VR teleop configuration structs, loaded from YAML.
 *
 * All retargeting, safety, impedance, and filter params for the
 * VR hand-tracking teleop pipeline.
 */

#ifndef ARM_COMMANDER_TELEOP_CONFIG_HPP_
#define ARM_COMMANDER_TELEOP_CONFIG_HPP_

#include <Eigen/Core>

#include <array>
#include <string>

namespace arm_commander {

struct ZmqConfig {
    std::string host = "localhost";
    int keypoint_port = 8089;
    int pause_port = 8102;
};

struct RetargetingConfig {
    // VR workspace range (m) — how far the hand moves in VR
    double vr_workspace = 1.0;

    // Robot workspace scaling per axis (m)
    Eigen::Vector3d robot_workspace{0.7, 0.7, 0.7};

    // VR → robot rotation matrix (column-major storage, row-major semantics)
    // Default: hand tracking axis swap [delta_y, -delta_x, delta_z]
    Eigen::Matrix3d vr_to_robot_rot = (Eigen::Matrix3d() <<
        0,  1, 0,   // robot X <- VR Y
       -1,  0, 0,   // robot Y <- -VR X
        0,  0, 1    // robot Z <- VR Z
    ).finished();

    // Robot home position (centre of workspace, metres)
    Eigen::Vector3d robot_home{0.5, 0.0, 0.4};

    // Deadzone radius (m) — VR deltas within this are zeroed
    double deadzone_radius = 0.005;

    // Orientation scaling factor (reduce wrist rotation magnitude)
    double orientation_scale = 0.3;

    // Post-engage hold frames (freeze after clutch engage to settle anchor)
    int hold_duration = 10;
};

struct TeleopSafetyConfig {
    // Workspace bounds (metres, robot base frame)
    Eigen::Vector3d pos_min{0.35, -0.20, 0.08};
    Eigen::Vector3d pos_max{0.65,  0.20, 0.50};

    // Velocity limits
    double max_linear_vel = 0.10;   // m/s
    double max_angular_vel = 0.3;   // rad/s

    // Force threshold — auto-stop if exceeded
    double force_threshold = 25.0;  // N
};

struct ZApproachConfig {
    bool enabled = false;                    // false = full 6DOF, true = z-approach
    Eigen::Vector2d xy_pos{0.512, 0.260};   // locked XY position (metres)
    double z_stiffness = 500.0;              // N/m — compliant Z for gentle contact
    double z_damping = 0.8;                  // damping ratio for Z axis
};

struct TeleopImpedanceConfig {
    // Stiffness [Kx, Ky, Kz, Krx, Kry, Krz] (N/m, Nm/rad)
    std::array<double, 6> stiffness = {2000, 2000, 1500, 80, 80, 80};
    // Damping ratio per axis [0.3 - 0.8]
    std::array<double, 6> damping = {0.7, 0.7, 0.7, 0.7, 0.7, 0.7};
};

struct OneEuroFilterConfig {
    double freq = 90.0;
    double min_cutoff_pos = 2.0;
    double beta_pos = 0.05;
    double min_cutoff_rot = 1.5;
    double beta_rot = 0.03;
    double d_cutoff = 1.0;
};

struct DropoutConfig {
    double timeout_ms = 50.0;
    double max_extrapolate_ms = 200.0;
    double decay_rate = 5.0;
};

struct TeleopConfig {
    ZmqConfig zmq;
    RetargetingConfig retargeting;
    TeleopSafetyConfig safety;
    TeleopImpedanceConfig impedance;
    OneEuroFilterConfig filter;
    DropoutConfig dropout;
    ZApproachConfig z_approach;
    double loop_rate_hz = 90.0;
};

/** Load TeleopConfig from a YAML file. */
TeleopConfig load_teleop_config(const std::string& path);

}  // namespace arm_commander

#endif  // ARM_COMMANDER_TELEOP_CONFIG_HPP_
