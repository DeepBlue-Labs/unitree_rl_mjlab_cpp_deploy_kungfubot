#include "control/Robot.h"
#include "control/Controller.h"

#include <unitree/idl/hg/LowCmd_.hpp>
#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/robot/g1/loco/g1_loco_client.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstring>
#include <deque>
#include <limits>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <thread>

namespace g1control {
namespace {
using LowState = unitree_hg::msg::dds_::LowState_;
using LowCmd = unitree_hg::msg::dds_::LowCmd_;
using Clock = std::chrono::steady_clock;
double seconds(Clock::time_point time) {
    return std::chrono::duration<double>(time.time_since_epoch()).count();
}
void validateOptions(const RobotOptions& options) {
    for (double value : {options.max_state_age_s, options.watchdog_ms, options.max_loop_lateness_s,
                         options.stop_temperature_c, options.damp_s, options.startup_timeout_s,
                         options.quiet_seconds})
        if (!std::isfinite(value) || value <= 0.)
            throw std::invalid_argument("robot timing and temperature options must be finite and positive");
}

// Same polynomial/word order as unitree/dds_wrapper/common/crc.h, without
// reading the DDS object through an aliased uint32_t pointer.
template<class Message> std::uint32_t crc(const Message& message) {
    static_assert(sizeof(Message)%4 == 0, "unexpected DDS message layout");
    std::uint32_t result = 0xffffffff;
    const auto* bytes = reinterpret_cast<const unsigned char*>(&message);
    for (std::size_t index = 0; index < sizeof(Message)/4-1; ++index) {
        std::uint32_t word;
        std::memcpy(&word, bytes+4*index, sizeof(word));
        for (std::uint32_t mask = 0x80000000; mask; mask >>= 1) {
            const bool high = (result & 0x80000000) != 0;
            result <<= 1;
            if (high) result ^= 0x04c11db7;
            if (word & mask) result ^= 0x04c11db7;
        }
    }
    return result;
}

struct Remote {
    std::uint16_t buttons = 0;
    double lx = 0, ly = 0, rx = 0;
};
Remote decodeRemote(const std::array<std::uint8_t, 40>& bytes) {
    Remote result;
    result.buttons = static_cast<std::uint16_t>(bytes[2]) |
        static_cast<std::uint16_t>(bytes[3]) << 8;
    auto axis = [&](std::size_t offset) {
        float value;
        std::memcpy(&value, bytes.data()+offset, sizeof(value));
        if (!std::isfinite(value) || std::abs(value) > 1.05f)
            throw std::runtime_error("invalid wireless remote axis");
        return std::clamp(static_cast<double>(value), -1., 1.);
    };
    result.lx = axis(4); result.rx = axis(8); result.ly = axis(20);
    return result;
}
Sensors decodeState(const LowState& state, const RobotOptions& options = {}, bool physical = true) {
    if (physical && crc(state) != state.crc()) throw std::runtime_error("G1 lowstate CRC mismatch");
    if (state.mode_machine() != 5) throw std::runtime_error("requires full-waist G1 mode_machine=5");
    if (state.mode_pr() != 0) throw std::runtime_error("requires G1 pitch/roll motor mode_pr=0");
    Sensors result;
    const auto& quaternion = state.imu_state().quaternion();
    result.quat = Eigen::Quaterniond(quaternion[0], quaternion[1], quaternion[2], quaternion[3]);
    for (int i = 0; i < 3; ++i) result.gyro[i] = state.imu_state().gyroscope()[i];
    for (int i = 0; i < 29; ++i) {
        const auto& motor = state.motor_state()[i];
        if (physical && motor.motorstate() != 0)
            throw std::runtime_error("motor error in G1 joint " + std::to_string(i));
        for (auto temperature : motor.temperature())
            if (physical && temperature >= options.stop_temperature_c)
                throw std::runtime_error("motor temperature reached " + std::to_string(options.stop_temperature_c)
                                         + " C in G1 joint " + std::to_string(i));
        result.q[i] = motor.q(); result.dq[i] = motor.dq();
    }
    if (!result.q.allFinite() || !result.dq.allFinite() || !result.gyro.allFinite() ||
        !result.quat.coeffs().allFinite() || std::abs(result.quat.norm()-1.) > .05)
        throw std::runtime_error("non-finite G1 state or invalid IMU quaternion");
    const auto remote = decodeRemote(state.wireless_remote());
    result.velocity_command = {remote.ly, -remote.lx, -remote.rx};
    return result;
}

bool isDamping(const Command& command) {
    return command.kp.isZero(0.) && (command.kd.array() == 3.).all();
}

LowCmd encodeCommand(const Command& command, std::uint8_t mode_machine, const Sensors* measured,
                     bool project_effort = false) {
    const bool damping = isDamping(command);
    if (!command.q.allFinite() || !command.kp.allFinite() || !command.kd.allFinite() ||
        !command.effort.allFinite() || (command.kp.array() < 0.).any() ||
        (command.kd.array() < 0.).any() || (command.effort.array() <= 0.).any() ||
        (command.q.array().abs() > std::numeric_limits<float>::max()).any() ||
        (command.kp.array() > std::numeric_limits<float>::max()).any() ||
        (command.kd.array() > std::numeric_limits<float>::max()).any())
        throw std::invalid_argument("invalid G1 motor command");
    if (!measured && !damping)
        throw std::runtime_error("a fresh measured state is required for position control");
    if (measured && (!measured->q.allFinite() || !measured->dq.allFinite()))
        throw std::runtime_error("non-finite state in G1 torque limit projection");
    LowCmd result{};
    result.mode_pr() = 0;
    result.mode_machine() = mode_machine;
    for (int i = 0; i < 29; ++i) {
        auto& motor = result.motor_cmd()[i];
        motor.mode() = 1;
        double target = command.q[i];
        // Preserve the policy's target in Command. Only the outgoing motor
        // setpoint is adjusted to enforce its training effort limit at the
        // current measured position and velocity. Pure damping is unchanged.
        if (project_effort && measured && command.kp[i] > 0.) {
            const double requested = command.kp[i]*(target-measured->q[i])
                - command.kd[i]*measured->dq[i];
            if (!std::isfinite(requested)) throw std::runtime_error("non-finite G1 predicted PD torque");
            const double limited = std::clamp(requested, -command.effort[i], command.effort[i]);
            if (limited != requested)
                target = measured->q[i] + (limited+command.kd[i]*measured->dq[i])/command.kp[i];
        }
        if (!std::isfinite(target) || std::abs(target) > std::numeric_limits<float>::max())
            throw std::runtime_error("G1 torque limit produced an invalid motor setpoint");
        motor.q() = static_cast<float>(target);
        motor.kp() = static_cast<float>(command.kp[i]);
        motor.kd() = static_cast<float>(command.kd[i]);
        motor.dq() = 0; motor.tau() = 0;
    }
    result.crc() = crc(result);
    return result;
}

void initializeTransport(const std::string& network, int domain, bool official) {
    if (network.empty()) throw std::invalid_argument("an explicit robot network interface is required");
    if (domain < 0 || domain > 232) throw std::invalid_argument("DDS domain must be in [0,232]");
    static std::mutex mutex;
    static bool used = false;
    std::lock_guard<std::mutex> lock(mutex);
    if (used) throw std::runtime_error("use a separate process for each low-level session or official stand");
    used = true;
    (void)official;
    unitree::robot::ChannelFactory::Instance()->Init(domain, network);
}

void requireQuietLowcmd(double quiet_seconds = 1.) {
    auto count = std::make_shared<std::atomic<std::uint64_t>>(0);
    unitree::robot::ChannelSubscriber<LowCmd> listener("rt/lowcmd");
    listener.InitChannel([count](const void*) { ++*count; }, 1);
    // Allow discovery and require a full one-second receive-only observation.
    std::this_thread::sleep_for(std::chrono::duration<double>(quiet_seconds));
    listener.CloseChannel();
    if (count->load() != 0)
        throw std::runtime_error("another rt/lowcmd controller is active; release it before starting this process");
}

std::string pressedCommand(std::uint16_t current, std::uint16_t previous) {
    const auto edge = [&](std::uint16_t chord) {
        return (current & chord) == chord && (previous & chord) != chord;
    };
    constexpr std::uint16_t rb = 1u<<0, rt = 1u<<4, lt = 1u<<5,
                            a = 1u<<8, b = 1u<<9, up = 1u<<12;
    if (edge(lt|b)) return "passive";
    if (edge(lt|up)) return "stand";
    if (edge(rt|a)) return "rl Velocity";
    if (edge(rb|a)) return "policy dance";
    return {};
}

struct ReceiveState {
    RobotOptions options;
    bool physical = true;
    std::mutex mutex;
    std::condition_variable changed;
    Sensors sensors;
    Clock::time_point received{};
    Clock::time_point advanced{};
    std::uint32_t tick = 0;
    std::uint8_t machine = 0;
    std::uint16_t buttons = 0;
    bool seen = false, valid = false;
    std::string error = "no G1 lowstate received";
    std::deque<std::string> remote_commands;
    RobotSnapshot telemetry;

    explicit ReceiveState(const RobotOptions& settings = {}, bool real = true)
        : options(settings), physical(real) {}

    bool fresh() const {
        const auto now = Clock::now();
        return seen && valid && std::chrono::duration<double>(now-received).count() <= options.max_state_age_s;
    }
    void requireFresh() const {
        if (!seen || !valid) throw std::runtime_error(error);
        if (!fresh()) throw std::runtime_error("G1 lowstate exceeds configured maximum age "
                                             + std::to_string(options.max_state_age_s) + " s");
    }
    void accept(const LowState& state) noexcept {
        std::lock_guard<std::mutex> lock(mutex);
        try {
            // Match the Python bridge: bad CRC frames increment the diagnostic
            // counter but never refresh the age of the last accepted packet.
            if (physical && crc(state) != state.crc()) {
                ++telemetry.crc_errors;
                changed.notify_all();
                return;
            }
            const auto now = Clock::now();
            if (telemetry.received_count == 0 || tick != state.tick()) {
                if (telemetry.received_count != 0) telemetry.tick_moving = true;
                advanced = now;
            }
            received = now; tick = state.tick();
            ++telemetry.received_count;
            telemetry.tick = tick;
            telemetry.mode_machine = state.mode_machine(); telemetry.mode_pr = state.mode_pr();
            telemetry.received_monotonic_s = seconds(now);
            telemetry.tick_changed_monotonic_s = seconds(advanced);
            telemetry.remote = state.wireless_remote();
            for (int i = 0; i < 35; ++i) {
                const auto& motor = state.motor_state()[i];
                telemetry.q[i] = motor.q(); telemetry.dq[i] = motor.dq(); telemetry.tau[i] = motor.tau_est();
                telemetry.motor_errors[i] = motor.motorstate();
                for (int t = 0; t < 2; ++t) telemetry.temperature[i][t] = motor.temperature()[t];
            }
            for (int i = 0; i < 3; ++i) {
                telemetry.acceleration[i] = state.imu_state().accelerometer()[i];
                telemetry.rpy[i] = state.imu_state().rpy()[i];
                telemetry.sensors.gyro[i] = state.imu_state().gyroscope()[i];
            }
            const auto& quat = state.imu_state().quaternion();
            telemetry.sensors.quat = Eigen::Quaterniond(quat[0], quat[1], quat[2], quat[3]);
            for (int i = 0; i < 29; ++i) {
                telemetry.sensors.q[i] = telemetry.q[i];
                telemetry.sensors.dq[i] = telemetry.dq[i];
            }
            const Sensors next = decodeState(state, options, physical);
            const auto remote = decodeRemote(state.wireless_remote());
            if (seen) {
                const auto command = pressedCommand(remote.buttons, buttons);
                if (!command.empty()) {
                    if (command == "passive") remote_commands.clear();
                    if (remote_commands.size() == 16) remote_commands.pop_front();
                    remote_commands.push_back(command);
                }
            }
            sensors = next; telemetry.sensors = next; machine = state.mode_machine();
            buttons = remote.buttons; received = now; seen = true; valid = true; error.clear();
            changed.notify_all();
        } catch (const std::exception& exception) {
            valid = false; error = exception.what(); remote_commands.clear();
            changed.notify_all();
        }
    }
};

// State failure permits one bounded damping interval after a validated G1
// state has established its machine type. Repeated writes never extend it.
struct DampingWindow {
    bool active = false;
    Clock::time_point deadline{};
    void reset() { active = false; }
    bool allow(bool previously_valid, Clock::time_point now, double duration_s = 1.) {
        if (!previously_valid) return false;
        if (!active) {
            active = true;
            deadline = now+std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(duration_s));
        }
        return now <= deadline;
    }
};
}  // namespace

struct Robot::Impl {
    bool readonly;
    RobotOptions options;
    std::shared_ptr<ReceiveState> state;
    std::shared_ptr<std::atomic<std::uint64_t>> lowcmd_count = std::make_shared<std::atomic<std::uint64_t>>(0);
    mutable std::mutex writer_mutex;
    RobotStatus status;
    DampingWindow damping_window;
    std::unique_ptr<unitree::robot::ChannelSubscriber<LowState>> subscriber;
    std::unique_ptr<unitree::robot::ChannelSubscriber<LowCmd>> watcher;
    std::unique_ptr<unitree::robot::ChannelPublisher<LowCmd>> publisher;

    Impl(const std::string& network, int domain, bool listen_only, const RobotOptions& settings)
        : readonly(listen_only), options(settings), state(std::make_shared<ReceiveState>(settings, domain == 0)) {
        validateOptions(options);
        initializeTransport(network, domain, false);
        watcher = std::make_unique<unitree::robot::ChannelSubscriber<LowCmd>>("rt/lowcmd");
        const auto count = lowcmd_count;
        watcher->InitChannel([count](const void*) { ++*count; }, 0);
        subscriber = std::make_unique<unitree::robot::ChannelSubscriber<LowState>>("rt/lowstate");
        const auto shared = state;
        subscriber->InitChannel([shared](const void* message) {
            shared->accept(*static_cast<const LowState*>(message));
        }, 0);  // Receive in the DDS callback; no queued samples hide stale state.
        {
            std::unique_lock<std::mutex> lock(state->mutex);
            state->changed.wait_for(lock, std::chrono::duration<double>(options.startup_timeout_s), [&] { return state->fresh(); });
            state->requireFresh();
        }
        if (!readonly) {
            quiet(options.quiet_seconds);
            {
                std::lock_guard<std::mutex> lock(state->mutex);
                state->requireFresh();
            }
            publisher = std::make_unique<unitree::robot::ChannelPublisher<LowCmd>>("rt/lowcmd");
            publisher->InitChannel();  // Creating a writer does not send a command.
            status.publisher_created = true;
        }
    }
    void quiet(double duration_s) const {
        if (!std::isfinite(duration_s) || duration_s <= 0.) throw std::invalid_argument("invalid quiet duration");
        const auto before = lowcmd_count->load();
        const auto deadline = Clock::now()+std::chrono::duration<double>(duration_s);
        while (Clock::now() < deadline) {
            {
                std::lock_guard<std::mutex> lock(state->mutex);
                state->requireFresh();
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        if (lowcmd_count->load() != before)
            throw std::runtime_error("another rt/lowcmd controller is active; run g1_prepare first");
    }
    ~Impl() noexcept {
        try { if (subscriber) subscriber->CloseChannel(); } catch (...) {}
        try { if (watcher) watcher->CloseChannel(); } catch (...) {}
        try { if (publisher) publisher->CloseChannel(); } catch (...) {}
    }
};

Robot::Robot(const std::string& network, int domain, bool readonly, const RobotOptions& options)
    : impl_(std::make_unique<Impl>(network, domain, readonly, options)) {}
Robot::~Robot() noexcept = default;
Sensors Robot::read() const {
    return readSnapshot().sensors;
}
RobotSnapshot Robot::readSnapshot() const {
    {
        std::lock_guard<std::mutex> lock(impl_->writer_mutex);
        if (impl_->status.watchdog_latched) throw std::runtime_error("DDS command watchdog latched damping");
        if (impl_->status.consecutive_failures >= 10) throw std::runtime_error("DDS repeatedly failed to publish");
    }
    std::lock_guard<std::mutex> lock(impl_->state->mutex);
    impl_->state->requireFresh();
    auto result = impl_->state->telemetry;
    result.age_s = seconds(Clock::now())-result.received_monotonic_s;
    result.valid = true;
    return result;
}
RobotSnapshot Robot::snapshot() const {
    std::lock_guard<std::mutex> lock(impl_->state->mutex);
    auto result = impl_->state->telemetry;
    result.age_s = result.received_count ? seconds(Clock::now())-result.received_monotonic_s
                                       : std::numeric_limits<double>::infinity();
    result.valid = impl_->state->fresh();
    result.error = result.valid ? "" : impl_->state->error;
    if (!result.valid && result.error.empty()) result.error = "G1 lowstate is stale";
    return result;
}
RobotStatus Robot::status() const {
    std::lock_guard<std::mutex> lock(impl_->writer_mutex);
    auto result = impl_->status;
    result.received_lowcmd_count = impl_->lowcmd_count->load();
    return result;
}
const RobotOptions& Robot::options() const { return impl_->options; }
void Robot::requireQuiet(double duration_s) const {
    impl_->quiet(duration_s < 0. ? impl_->options.quiet_seconds : duration_s);
}
void Robot::latchWatchdog(const std::string& reason) {
    std::lock_guard<std::mutex> lock(impl_->writer_mutex);
    impl_->status.watchdog_latched = true;
    impl_->status.watchdog_reason = reason;
}
bool Robot::available() const {
    std::lock_guard<std::mutex> lock(impl_->state->mutex);
    return impl_->state->fresh();
}
void Robot::write(const Command& command) {
    if (impl_->readonly) throw std::runtime_error("cannot publish in readonly mode");
    std::lock_guard<std::mutex> writer(impl_->writer_mutex);
    Command output = command;
    if (impl_->status.watchdog_latched) { output.kp.setZero(); output.kd.setConstant(3.); }
    std::uint8_t machine;
    std::optional<Sensors> measured;
    {
        std::lock_guard<std::mutex> lock(impl_->state->mutex);
        if (impl_->state->fresh()) {
            impl_->damping_window.reset();
            measured = impl_->state->sensors;
        } else {
            if (!isDamping(output)) impl_->state->requireFresh();
            if (!impl_->damping_window.allow(impl_->state->seen && impl_->state->machine == 5, Clock::now(), impl_->options.damp_s))
                throw std::runtime_error("G1 state unavailable: bounded shutdown damping is not available or has expired");
            output.q = impl_->state->sensors.q;
        }
        if (impl_->status.watchdog_latched) output.q = impl_->state->sensors.q;
        machine = impl_->state->machine;
    }
    const auto message = encodeCommand(output, machine, measured ? &*measured : nullptr, impl_->options.project_effort_limits);
    const auto started = Clock::now();
    bool ok = false;
    try { ok = impl_->publisher->Write(message); } catch (...) {}
    auto& status = impl_->status;
    const double now = seconds(started);
    status.max_publish_write_s = std::max(status.max_publish_write_s, seconds(Clock::now())-now);
    if (status.published_commands) {
        const double gap = now-status.last_publish_monotonic_s;
        status.max_publish_gap_s = std::max(status.max_publish_gap_s, gap);
        if (gap > .004) ++status.late_publish_gaps;
    }
    status.last_publish_monotonic_s = now;
    ++status.published_commands;
    if (ok) status.consecutive_failures = 0;
    else { ++status.publish_failures; ++status.consecutive_failures; }
    if (status.consecutive_failures >= 10) throw std::runtime_error("G1 lowcmd failed to publish ten consecutive times");
}
bool Robot::operatorStopRequested() const {
    std::lock_guard<std::mutex> lock(impl_->state->mutex);
    const auto& remote = impl_->state->telemetry.remote;
    const auto keys = static_cast<std::uint16_t>(remote[2]) | (static_cast<std::uint16_t>(remote[3])<<8);
    constexpr std::uint16_t mask = (1u<<5)|(1u<<9);
    return (keys & mask) == mask;
}
std::string Robot::remoteCommand() {
    std::lock_guard<std::mutex> lock(impl_->state->mutex);
    impl_->state->requireFresh();
    if (impl_->state->remote_commands.empty()) return {};
    const auto command = impl_->state->remote_commands.front();
    impl_->state->remote_commands.pop_front();
    return command;
}
void Robot::officialStand(const std::string& network, int domain) {
    initializeTransport(network, domain, true);
    // This is a high-level RPC to the running built-in controller. Its own
    // LowCmd traffic is expected; low-level takeover's quiet gate does not apply.
    // The operator must first exit external low-level controllers (README).
    unitree::robot::g1::LocoClient client;
    client.SetTimeout(5.f);
    client.Init();
    const auto code = client.StandUp();
    if (code != 0) throw std::runtime_error("official G1 StandUp RPC failed: " + std::to_string(code));
}
}  // namespace g1control
