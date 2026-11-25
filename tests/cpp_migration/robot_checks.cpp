// Pure in-memory transport tests. Including the implementation allows exercising
// private encoding/validation helpers without creating a DDS participant.
#include "../../deploy/robots/g1/src/control/Robot.cpp"
#include <unitree/dds_wrapper/common/crc.h>
#include <iostream>

using namespace g1control;

void expect(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}
template<class Function> void rejected(Function fn) {
    try { fn(); } catch (const std::exception&) { return; }
    throw std::runtime_error("invalid message was accepted");
}
void signedState(LowState& state) { state.crc() = crc(state); }

int main() {
    try {
        LowState initial{};
        initial.mode_machine() = 5;
        initial.imu_state().quaternion()[0] = 1;
        initial.tick() = 1;
        initial.motor_state()[7].q() = .42f;
        float lx = .2f, ly = .6f, rx = -.3f;
        std::memcpy(initial.wireless_remote().data()+4, &lx, 4);
        std::memcpy(initial.wireless_remote().data()+8, &rx, 4);
        std::memcpy(initial.wireless_remote().data()+20, &ly, 4);
        signedState(initial);
        auto state = decodeState(initial);
        expect(std::abs(state.q[7]-.42) < 1e-7, "joint decode mismatch");
        expect((state.velocity_command-Eigen::Vector3d(.6,-.2,.3)).norm() < 1e-7, "remote axes mismatch");

        auto invalid = initial; invalid.motor_state()[0].q() = 4;
        rejected([&] { decodeState(invalid); });  // Corrupt CRC.
        invalid = initial; invalid.mode_machine() = 4; signedState(invalid);
        rejected([&] { decodeState(invalid); });
        invalid = initial; invalid.mode_pr() = 1; signedState(invalid);
        rejected([&] { decodeState(invalid); });
        invalid = initial; invalid.imu_state().quaternion()[0] = .8f; signedState(invalid);
        rejected([&] { decodeState(invalid); });
        invalid = initial; invalid.motor_state()[0].motorstate() = 1; signedState(invalid);
        rejected([&] { decodeState(invalid); });
        invalid = initial; invalid.motor_state()[0].temperature()[0] = 65; signedState(invalid);
        rejected([&] { decodeState(invalid); });
        invalid = initial; invalid.motor_state()[0].dq() = std::numeric_limits<float>::quiet_NaN(); signedState(invalid);
        rejected([&] { decodeState(invalid); });

        Command command;
        command.q.setConstant(.35); command.kp.setConstant(45); command.kd.setConstant(3);
        auto encoded = encodeCommand(command, 5, &state);
        std::array<std::uint32_t, sizeof(LowCmd)/4> raw{};
        std::memcpy(raw.data(), &encoded, sizeof(encoded));
        expect(encoded.crc() == crc32_core(raw.data(), raw.size()-1), "SDK CRC mismatch");
        expect(encoded.mode_machine() == 5 && encoded.mode_pr() == 0, "command mode mismatch");
        for (int i = 0; i < 35; ++i) {
            const auto& motor = encoded.motor_cmd()[i];
            expect(motor.mode() == (i < 29 ? 1 : 0), "command motor mask mismatch");
            expect(motor.dq() == 0 && motor.tau() == 0, "unexpected velocity/feed-forward torque");
            if (i < 29) expect(motor.kp() == 45 && motor.kd() == 3, "gain encoding mismatch");
        }
        command.kp[0] = -1;
        rejected([&] { encodeCommand(command, 5, &state); });
        command.kp[0] = 0; command.q[0] = std::numeric_limits<double>::infinity();
        rejected([&] { encodeCommand(command, 5, &state); });
        command.q.setConstant(3.); command.kp.setConstant(100.);
        command.kd.setConstant(5.); command.effort.setConstant(2.);
        state.q.setConstant(.1); state.dq.setConstant(.2);
        encoded = encodeCommand(command, 5, &state, true);
        for (int i = 0; i < 29; ++i)
            expect(std::abs(100.*(encoded.motor_cmd()[i].q()-.1)-5.*.2-2.) < 1e-5,
                   "positive effort projection failed");
        expect(command.q[0] == 3., "projection modified policy target");
        command.q.setConstant(-3.);
        encoded = encodeCommand(command, 5, &state, true);
        for (int i = 0; i < 29; ++i)
            expect(std::abs(100.*(encoded.motor_cmd()[i].q()-.1)-5.*.2+2.) < 1e-5,
                   "negative effort projection failed");
        rejected([&] { encodeCommand(command, 5, nullptr); });
        encoded = encodeCommand(command, 5, &state);
        expect(encoded.motor_cmd()[0].q() == -3.f, "default encoding modified the Python target contract");
        Command damping;
        damping.q.setConstant(20.); // Irrelevant with kp=0; encode a zero target.
        encoded = encodeCommand(damping, 5, nullptr);
        expect(encoded.motor_cmd()[0].q() == 20 && encoded.motor_cmd()[0].kp() == 0 &&
               encoded.motor_cmd()[0].kd() == 3, "emergency damping encoding failed");
        DampingWindow window;
        const auto now = Clock::now();
        expect(!window.allow(false, now), "shutdown damping accepted without machine credentials");
        expect(window.allow(true, now), "first shutdown damping rejected");
        expect(window.allow(true, now+std::chrono::milliseconds(999)), "shutdown window too short");
        expect(!window.allow(true, now+std::chrono::milliseconds(1001)), "shutdown damping window was extended");

        constexpr std::uint16_t lt=1u<<5, rt=1u<<4, rb=1u, a=1u<<8, b=1u<<9, up=1u<<12;
        expect(pressedCommand(lt|up, 0) == "stand", "stand chord mismatch");
        expect(pressedCommand(lt|b, 0) == "passive", "passive chord mismatch");
        expect(pressedCommand(rt|a, 0) == "rl Velocity", "velocity chord mismatch");
        expect(pressedCommand(rb|a, 0) == "policy dance", "dance chord mismatch");
        expect(pressedCommand(lt|up, lt|up).empty(), "held chord retriggered");
        ReceiveState receiver;
        receiver.accept(initial);
        expect(receiver.fresh(), "valid state unavailable");
        receiver.advanced = Clock::now()-std::chrono::milliseconds(101);
        receiver.accept(initial);
        expect(receiver.fresh(), "repeated tick must remain a diagnostic, matching Python fresh_state");
        initial.tick() = 2; signedState(initial); receiver.accept(initial);
        expect(receiver.fresh(), "advancing tick did not recover");
        receiver.received = Clock::now()-std::chrono::milliseconds(21);
        rejected([&] { receiver.requireFresh(); });
        receiver.accept(invalid);
        expect(!receiver.fresh(), "invalid latest message reused previous state");
        RobotOptions settings;
        expect(settings.max_state_age_s == .02 && settings.max_loop_lateness_s == .04 &&
               settings.watchdog_ms == 100. && settings.damp_s == 1., "site defaults changed");
        settings.stop_temperature_c = 80.;
        auto warm = initial; warm.motor_state()[0].temperature()[0] = 70; signedState(warm);
        decodeState(warm, settings);
        rejected([&] { decodeState(warm); });
        warm.crc() = 0; warm.motor_state()[0].motorstate() = 3;
        decodeState(warm, {}, false);  // Loopback: Python disables CRC/real-motor health gates.
        receiver.accept(initial);
        const auto accepted_count = receiver.telemetry.received_count;
        const auto accepted_time = receiver.received;
        auto corrupt = initial; corrupt.crc() ^= 1;
        receiver.accept(corrupt);
        expect(receiver.telemetry.crc_errors == 1 && receiver.telemetry.received_count == accepted_count &&
               receiver.received == accepted_time, "CRC rejection refreshed the old snapshot");
        expect(receiver.telemetry.remote == initial.wireless_remote() &&
               std::abs(receiver.telemetry.q[7]-.42) < 1e-7, "telemetry lost raw fields");
        std::cout << "PASS: offline G1 CRC/command encoding, state validation, age/tick guards, remote chords\n";
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n'; return 1;
    }
}
