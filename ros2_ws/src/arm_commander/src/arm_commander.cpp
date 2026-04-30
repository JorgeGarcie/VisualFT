/**
 * @file arm_commander.cpp
 * @brief ArmCommander implementation — single RDK owner.
 */

#include "arm_commander/arm_commander.hpp"

#include <Eigen/Geometry>
#include <spdlog/spdlog.h>

#include <chrono>
#include <cmath>
#include <stdexcept>
#include <thread>

namespace arm_commander {

ArmCommander::ArmCommander(const RobotConfig& config) : config_(config) {}

ArmCommander::~ArmCommander()
{
    try {
        shutdown();
    } catch (const std::exception& e) {
        spdlog::error("Exception in ~ArmCommander: {}", e.what());
    }
}

// ── Lifecycle ──────────────────────────────────────────────────────────────

void ArmCommander::connect()
{
    std::lock_guard<std::mutex> lock(mutex_);

    robot_ = std::make_unique<flexiv::rdk::Robot>(config_.serial_number);

    if (robot_->fault()) {
        spdlog::warn("Fault detected — clearing...");
        if (!robot_->ClearFault()) {
            throw std::runtime_error("Cannot clear robot fault");
        }
        std::this_thread::sleep_for(std::chrono::seconds(2));
        if (robot_->fault()) {
            throw std::runtime_error("Cannot clear fault — check E-stop and teach pendant");
        }
        spdlog::info("Fault cleared");
    }

    spdlog::info("Enabling robot...");
    robot_->Enable();

    auto start = std::chrono::steady_clock::now();
    while (!robot_->operational()) {
        if (std::chrono::steady_clock::now() - start > std::chrono::seconds(10)) {
            throw std::runtime_error("Timeout waiting for robot to become operational");
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    tool_ = std::make_unique<flexiv::rdk::Tool>(*robot_);
    current_mode_ = flexiv::rdk::Mode::UNKNOWN;
    connected_ = true;
    stopped_ = false;
    stop_requested_ = false;
    spdlog::info("Robot operational: {}", config_.serial_number);
}

void ArmCommander::shutdown()
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (robot_) {
        try {
            robot_->Stop();
        } catch (const std::exception& e) {
            spdlog::warn("Error during shutdown Stop(): {}", e.what());
        }
    }
    connected_ = false;
    stopped_ = true;
    spdlog::info("Robot shutdown complete");
}

// ── Blocking primitives ────────────────────────────────────────────────────

void ArmCommander::home()
{
    ensure_connected();
    {
        std::lock_guard<std::mutex> lock(mutex_);
        switch_mode(flexiv::rdk::Mode::NRT_PLAN_EXECUTION);
        robot_->ExecutePlan("PLAN-Home");
    }
    wait_busy();
    spdlog::info("Home position reached");
}

void ArmCommander::zero_ft()
{
    ensure_connected();
    {
        std::lock_guard<std::mutex> lock(mutex_);
        switch_mode(flexiv::rdk::Mode::NRT_PRIMITIVE_EXECUTION);
        robot_->ExecutePrimitive("ZeroFTSensor",
            std::map<std::string, flexiv::rdk::FlexivDataTypes>());
    }
    wait_primitive("terminated");
    spdlog::info("FT sensor zeroed");
}

void ArmCommander::contact(
    const std::vector<double>& direction, double velocity, double max_force)
{
    ensure_connected();
    {
        std::lock_guard<std::mutex> lock(mutex_);
        switch_mode(flexiv::rdk::Mode::NRT_PRIMITIVE_EXECUTION);
        std::map<std::string, flexiv::rdk::FlexivDataTypes> params;
        params["contactDir"] = std::vector<double>(direction.begin(), direction.end());
        params["contactVel"] = velocity;
        params["maxContactForce"] = max_force;
        params["enableFineContact"] = 1;
        robot_->ExecutePrimitive("Contact", params);
    }
    wait_primitive("terminated");
    spdlog::info("Contact detected (max_force={:.1f}N)", max_force);
}

void ArmCommander::move_to(const std::array<double, 7>& pose, double velocity)
{
    ensure_connected();
    check_workspace(pose[0], pose[1], pose[2]);

    // Convert quaternion [qw,qx,qy,qz] to Euler ZYX degrees for MoveL Coord
    Eigen::Quaterniond q(pose[3], pose[4], pose[5], pose[6]);  // w,x,y,z
    Eigen::Vector3d euler = q.toRotationMatrix().eulerAngles(2, 1, 0);  // ZYX
    double rx_deg = euler[2] * 180.0 / M_PI;
    double ry_deg = euler[1] * 180.0 / M_PI;
    double rz_deg = euler[0] * 180.0 / M_PI;

    {
        std::lock_guard<std::mutex> lock(mutex_);
        switch_mode(flexiv::rdk::Mode::NRT_PRIMITIVE_EXECUTION);

        std::map<std::string, flexiv::rdk::FlexivDataTypes> params;
        params["target"] = flexiv::rdk::Coord(
            {pose[0], pose[1], pose[2]}, {rx_deg, ry_deg, rz_deg},
            {"WORLD", "WORLD_ORIGIN"});
        params["vel"] = velocity;
        robot_->ExecutePrimitive("MoveL", params);
    }
    wait_primitive("reachedTarget");
    spdlog::info("MoveL complete");
}

// ── Tool management ────────────────────────────────────────────────────────

// TODO: register CoinFT as a tool (mass / CoM / inertia in robot.yaml) so gravity
// compensation works in floating mode and Cartesian impedance control.
void ArmCommander::set_tool(const std::string& name, double mass,
    const std::array<double, 3>& com, const std::array<double, 6>& inertia,
    const std::array<double, 7>& tcp_location)
{
    ensure_connected();
    std::lock_guard<std::mutex> lock(mutex_);
    switch_mode(flexiv::rdk::Mode::IDLE);

    flexiv::rdk::ToolParams params;
    params.mass = mass;
    params.CoM = com;
    params.inertia = inertia;
    if (tcp_location != std::array<double, 7>{}) {
        params.tcp_location = tcp_location;
    }

    if (tool_->exist(name)) {
        tool_->Switch("Flange");
        tool_->Remove(name);
    }

    tool_->Add(name, params);
    tool_->Switch(name);
    spdlog::info("Tool active: {} (mass={:.2f}kg)", name, mass);
}

// ── Streaming control ──────────────────────────────────────────────────────

void ArmCommander::stream_cartesian(
    const std::array<double, 7>& pose, const std::array<double, 6>& wrench,
    double max_linear_vel, double max_angular_vel,
    double max_linear_acc, double max_angular_acc)
{
    ensure_connected();
    check_workspace(pose[0], pose[1], pose[2]);

    // Use caller overrides if provided (> 0), else fall back to config
    double lin_vel = max_linear_vel > 0.0 ? max_linear_vel : config_.max_linear_vel;
    double ang_vel = max_angular_vel > 0.0 ? max_angular_vel : config_.max_angular_vel;
    double lin_acc = max_linear_acc > 0.0 ? max_linear_acc : config_.max_linear_acc;
    double ang_acc = max_angular_acc > 0.0 ? max_angular_acc : config_.max_angular_acc;

    std::lock_guard<std::mutex> lock(mutex_);
    switch_mode(flexiv::rdk::Mode::NRT_CARTESIAN_MOTION_FORCE);
    robot_->SendCartesianMotionForce(pose, wrench, lin_vel, ang_vel, lin_acc, ang_acc);
}

void ArmCommander::set_impedance(
    const std::array<double, 6>& stiffness, const std::array<double, 6>& damping)
{
    ensure_connected();
    std::lock_guard<std::mutex> lock(mutex_);
    robot_->SetCartesianImpedance(stiffness, damping);
    spdlog::info("Impedance set");
}

void ArmCommander::set_max_contact_wrench(const std::array<double, 6>& wrench)
{
    ensure_connected();
    std::lock_guard<std::mutex> lock(mutex_);
    robot_->SetMaxContactWrench(wrench);
}

void ArmCommander::set_force_control_axis(const std::array<bool, 6>& axes)
{
    ensure_connected();
    std::lock_guard<std::mutex> lock(mutex_);
    robot_->SetForceControlAxis(axes);
}

void ArmCommander::set_force_control_frame(flexiv::rdk::CoordType frame)
{
    ensure_connected();
    std::lock_guard<std::mutex> lock(mutex_);
    robot_->SetForceControlFrame(frame);
}

// ── RT joint torque floating ───────────────────────────────────────────────

void ArmCommander::float_joints(const std::vector<double>& damping_gains)
{
    ensure_connected();

    // Capture robot pointer for the periodic task (no lock needed inside
    // scheduler — RDK streaming calls are designed for single-thread RT use)
    auto* robot_ptr = robot_.get();
    const auto& gains = damping_gains;
    std::atomic<bool> sched_stop{false};

    auto periodic_task = [robot_ptr, &gains, &sched_stop, this]() {
        try {
            if (robot_ptr->fault()) {
                spdlog::error("Fault during floating");
                sched_stop = true;
                return;
            }

            std::vector<double> torque(robot_ptr->info().DoF, 0.0);
            for (size_t i = 0; i < torque.size() && i < gains.size(); ++i) {
                torque[i] = -gains[i] * robot_ptr->states().dtheta[i];
            }

            robot_ptr->StreamJointTorque(torque, true, true);
        } catch (const std::exception& e) {
            spdlog::error("Float periodic: {}", e.what());
            sched_stop = true;
        }
    };

    // Set up scheduler before switching mode to minimize gap between
    // entering RT_JOINT_TORQUE and first torque command
    flexiv::rdk::Scheduler scheduler;
    scheduler.AddTask(periodic_task, "floating", 1, scheduler.max_priority());

    {
        std::lock_guard<std::mutex> lock(mutex_);
        switch_mode(flexiv::rdk::Mode::RT_JOINT_TORQUE);
    }

    scheduler.Start();
    spdlog::info("Floating mode active — move the robot by hand");

    // Block until stop requested
    while (!sched_stop && !stop_requested_) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    scheduler.Stop();
    spdlog::info("Floating stopped");
}

// ── Cartesian floating ─────────────────────────────────────────────────────

void ArmCommander::float_cartesian(double loop_rate_hz)
{
    (void)loop_rate_hz;  // RT scheduler runs at 1kHz internally
    ensure_connected();

    // Send one NRT command to enter Cartesian mode so we can configure impedance
    {
        auto state = get_state();
        std::lock_guard<std::mutex> lock(mutex_);
        switch_mode(flexiv::rdk::Mode::NRT_CARTESIAN_MOTION_FORCE);
        robot_->SendCartesianMotionForce(state.tcp_pose, {0, 0, 0, 0, 0, 0});
    }

    // Zero stiffness = free-floating on all Cartesian axes
    set_force_control_axis({false, false, false, false, false, false});
    set_impedance({0, 0, 0, 0, 0, 0}, {0.3, 0.3, 0.3, 0.3, 0.3, 0.3});

    // Now switch to RT mode for true free-floating (NRT motion generator fights pushes)
    auto* robot_ptr = robot_.get();
    std::atomic<bool> sched_stop{false};

    auto periodic_task = [robot_ptr, &sched_stop, this]() {
        try {
            if (robot_ptr->fault()) {
                spdlog::error("Fault during Cartesian floating");
                sched_stop = true;
                return;
            }

            // Stream current pose back as target — with K=0 the position
            // target is irrelevant but keeps the RT stream alive
            auto tcp = robot_ptr->states().tcp_pose;
            robot_ptr->StreamCartesianMotionForce(tcp, {0, 0, 0, 0, 0, 0});
        } catch (const std::exception& e) {
            spdlog::error("Cartesian float periodic: {}", e.what());
            sched_stop = true;
        }
    };

    flexiv::rdk::Scheduler scheduler;
    scheduler.AddTask(periodic_task, "cartesian_float", 1, scheduler.max_priority());

    {
        std::lock_guard<std::mutex> lock(mutex_);
        switch_mode(flexiv::rdk::Mode::RT_CARTESIAN_MOTION_FORCE);
    }

    scheduler.Start();
    spdlog::info("Cartesian floating active (RT) — move the robot by hand");

    while (!sched_stop && !stop_requested_) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    scheduler.Stop();
    spdlog::info("Cartesian floating stopped");
}

// ── State ──────────────────────────────────────────────────────────────────

RobotState ArmCommander::get_state()
{
    ensure_connected();
    std::lock_guard<std::mutex> lock(mutex_);
    RobotState state;
    auto s = robot_->states();
    for (size_t i = 0; i < 7 && i < s.tcp_pose.size(); ++i) {
        state.tcp_pose[i] = s.tcp_pose[i];
    }
    for (size_t i = 0; i < 6 && i < s.ext_wrench_in_tcp.size(); ++i) {
        state.wrench[i] = s.ext_wrench_in_tcp[i];
    }
    for (size_t i = 0; i < 6 && i < s.ext_wrench_in_world.size(); ++i) {
        state.wrench_in_world[i] = s.ext_wrench_in_world[i];
    }
    state.joint_positions = std::vector<double>(s.q.begin(), s.q.end());
    state.joint_velocities = std::vector<double>(s.dtheta.begin(), s.dtheta.end());
    state.operational = robot_->operational();
    state.fault = robot_->fault();
    return state;
}

bool ArmCommander::has_fault()
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (!robot_)
        return false;
    return robot_->fault();
}

bool ArmCommander::is_healthy()
{
    if (!connected_ || !robot_)
        return false;
    std::lock_guard<std::mutex> lock(mutex_);
    return robot_->operational() && !robot_->fault() && !stopped_;
}

void ArmCommander::stop()
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (robot_) {
        robot_->Stop();
    }
    stopped_ = true;
    stop_requested_ = true;
    spdlog::warn("Robot stopped");
}

// ── Internal ───────────────────────────────────────────────────────────────

void ArmCommander::ensure_connected()
{
    if (!connected_ || !robot_) {
        throw std::runtime_error("Not connected. Call connect() first.");
    }
    if (stopped_) {
        throw std::runtime_error("Robot stopped. Call connect() to re-initialize.");
    }
}

void ArmCommander::check_workspace(double x, double y, double z)
{
    if (!config_.safety.workspace.contains(x, y, z)) {
        stop();
        throw std::runtime_error("Pose outside workspace bounds");
    }
}

void ArmCommander::switch_mode(flexiv::rdk::Mode target)
{
    // Must be called with mutex_ held
    if (current_mode_ == target)
        return;
    robot_->SwitchMode(target);
    current_mode_ = target;
}

void ArmCommander::wait_busy()
{
    while (!stop_requested_) {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (!robot_->busy())
                return;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    throw std::runtime_error("Interrupted by stop request");
}

void ArmCommander::wait_primitive(const std::string& state_key)
{
    // Most primitives don't exit by themselves — busy() stays true.
    // Must poll primitive_states() for the expected state key.
    while (!stop_requested_) {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            auto states = robot_->primitive_states();
            auto it = states.find(state_key);
            if (it != states.end()) {
                if (auto* val = std::get_if<int>(&it->second)) {
                    if (*val)
                        return;
                }
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    throw std::runtime_error("Interrupted by stop request");
}

}  // namespace arm_commander
