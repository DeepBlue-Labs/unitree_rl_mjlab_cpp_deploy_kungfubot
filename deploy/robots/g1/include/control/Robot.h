#pragma once

#include "control/Policy.h"
#include <array>
#include <cstdint>

namespace g1control {
struct Command;

// Defaults match unified_deploy/site.json. The caller owns the 50 Hz deadline
// check and the independent 500 Hz command-age watchdog, using these values.
struct RobotOptions {
    double max_state_age_s = .02;
    double watchdog_ms = 100.;
    double max_loop_lateness_s = .04;
    double stop_temperature_c = 65.;
    double damp_s = 1.;
    double startup_timeout_s = 5.;
    double quiet_seconds = .5;
    // The original Python runner sends its q/kp/kd unchanged. Projection is an
    // explicit optional transport behavior, never the migration default.
    bool project_effort_limits = false;
};

struct RobotSnapshot {
    Sensors sensors;
    std::array<double, 35> q{}, dq{}, tau{};
    std::array<std::array<double, 2>, 35> temperature{};
    std::array<std::uint32_t, 35> motor_errors{};
    std::array<std::uint8_t, 40> remote{};
    Eigen::Vector3d acceleration = Eigen::Vector3d::Zero(), rpy = Eigen::Vector3d::Zero();
    std::uint32_t tick = 0;
    std::uint8_t mode_machine = 0, mode_pr = 0;
    double received_monotonic_s = 0., tick_changed_monotonic_s = 0., age_s = 0.;
    std::uint64_t received_count = 0, crc_errors = 0;
    bool tick_moving = false, valid = false;
    std::string error;
};

struct RobotStatus {
    std::uint64_t published_commands = 0, publish_failures = 0, consecutive_failures = 0;
    std::uint64_t received_lowcmd_count = 0, late_publish_gaps = 0;
    double last_publish_monotonic_s = 0., max_publish_gap_s = 0., max_publish_write_s = 0.;
    bool watchdog_latched = false, publisher_created = false;
    std::string watchdog_reason;
};

// SDK motor order is the G1 29-DOF order. This class never selects a policy.
// Construction receives state only; a low-level command is sent only by write.
class Robot {
public:
    Robot(const std::string& network, int domain, bool readonly = false,
          const RobotOptions& options = {});
    ~Robot() noexcept;
    Robot(const Robot&) = delete;
    Robot& operator=(const Robot&) = delete;
    Robot(Robot&&) = delete;
    Robot& operator=(Robot&&) = delete;

    Sensors read() const;
    RobotSnapshot readSnapshot() const;
    // Returns the latest diagnostic snapshot even when its validity check fails.
    RobotSnapshot snapshot() const;
    RobotStatus status() const;
    const RobotOptions& options() const;
    bool available() const;
    void requireQuiet(double seconds = -1.) const;
    void latchWatchdog(const std::string& reason = "runner command watchdog expired");
    void write(const Command& command);
    std::string remoteCommand();
    bool operatorStopRequested() const;

    // An independent process entry point for the robot's built-in StandUp RPC.
    // It cannot be combined with a Robot instance in the same process.
    static void officialStand(const std::string& network, int domain);

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
}  // namespace g1control
