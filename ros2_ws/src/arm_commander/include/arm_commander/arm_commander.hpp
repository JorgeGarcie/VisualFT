/**
 * @file arm_commander.hpp
 * @brief Single owner of the Flexiv RDK connection.
 *
 * All RDK access is mutex-protected. Mode switching is internal.
 * Callers use high-level methods: home(), zero_ft(), contact(), etc.
 */

#ifndef ARM_COMMANDER_ARM_COMMANDER_HPP_
#define ARM_COMMANDER_ARM_COMMANDER_HPP_

#include <flexiv/rdk/data.hpp>
#include <flexiv/rdk/robot.hpp>
#include <flexiv/rdk/scheduler.hpp>
#include <flexiv/rdk/tool.hpp>

#include <array>
#include <atomic>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "arm_commander/config.hpp"

namespace arm_commander {

struct RobotState {
    std::array<double, 7> tcp_pose;         // [x,y,z,qw,qx,qy,qz]
    std::array<double, 6> wrench;           // [fx,fy,fz,tx,ty,tz] in TCP frame
    std::array<double, 6> wrench_in_world;  // [fx,fy,fz,tx,ty,tz] in world frame
    std::vector<double> joint_positions;
    std::vector<double> joint_velocities;
    bool operational = false;
    bool fault = false;
};

class ArmCommander {
public:
    explicit ArmCommander(const RobotConfig& config);
    ~ArmCommander();

    // ── Lifecycle ──────────────────────────────────────────────────
    void connect();
    void shutdown();

    // ── Blocking primitives ────────────────────────────────────────
    void home();
    void zero_ft();
    void contact(const std::vector<double>& direction = {0.0, 0.0, -1.0},
        double velocity = 0.02, double max_force = 20.0);
    void move_to(const std::array<double, 7>& pose, double velocity = 0.05);

    // ── Tool management ────────────────────────────────────────────
    void set_tool(const std::string& name, double mass,
        const std::array<double, 3>& com, const std::array<double, 6>& inertia,
        const std::array<double, 7>& tcp_location = {});

    // ── Streaming control ──────────────────────────────────────────
    void stream_cartesian(const std::array<double, 7>& pose,
        const std::array<double, 6>& wrench = {},
        double max_linear_vel = 0.0, double max_angular_vel = 0.0,
        double max_linear_acc = 0.0, double max_angular_acc = 0.0);
    void set_impedance(const std::array<double, 6>& stiffness,
        const std::array<double, 6>& damping);
    void set_max_contact_wrench(const std::array<double, 6>& wrench);
    void set_force_control_axis(const std::array<bool, 6>& axes);
    void set_force_control_frame(flexiv::rdk::CoordType frame);

    // ── RT joint torque floating ───────────────────────────────────
    /** Enter RT_JOINT_TORQUE floating mode. Blocks until stop() or shutdown(). */
    void float_joints(const std::vector<double>& damping_gains);

    // ── Cartesian floating ──────────────────────────────────────────
    /** Enter NRT Cartesian floating (zero stiffness impedance). Blocks until stop. */
    void float_cartesian(double loop_rate_hz = 90.0);

    // ── State ──────────────────────────────────────────────────────
    RobotState get_state();
    bool has_fault();
    bool is_healthy();
    void stop();

    // ── For external use (e.g. signal handlers) ────────────────────
    /** Thread-safe flag to request stop from signal handler. */
    void request_stop() { stop_requested_ = true; }
    bool stop_was_requested() const { return stop_requested_.load(); }

private:
    void ensure_connected();
    void check_workspace(double x, double y, double z);
    void switch_mode(flexiv::rdk::Mode target);
    void wait_busy();
    void wait_primitive(const std::string& state_key);

    RobotConfig config_;
    std::unique_ptr<flexiv::rdk::Robot> robot_;
    std::unique_ptr<flexiv::rdk::Tool> tool_;
    std::mutex mutex_;
    flexiv::rdk::Mode current_mode_ = flexiv::rdk::Mode::UNKNOWN;
    bool connected_ = false;
    bool stopped_ = false;
    std::atomic<bool> stop_requested_{false};
};

}  // namespace arm_commander

#endif  // ARM_COMMANDER_ARM_COMMANDER_HPP_
