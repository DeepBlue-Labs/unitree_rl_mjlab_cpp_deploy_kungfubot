// Offline numerical probe. It never creates a robot/DDS connection.
#include "control/Physics.h"
#include <cmath>
#include <iomanip>
#include <iostream>
#include <stdexcept>

using namespace g1control;

template<class Vector> void load(const YAML::Node& node, Vector& target) {
    if (node.size() != static_cast<std::size_t>(target.size()))
        throw std::runtime_error("wrong fixture vector length");
    for (int i = 0; i < target.size(); ++i) target[i] = node[i].as<double>();
}
template<class Vector> void vectorJson(const Vector& values) {
    std::cout << '[';
    for (int i = 0; i < values.size(); ++i) { if (i) std::cout << ','; std::cout << values[i]; }
    std::cout << ']';
}
void stateJson(const Sensors& state) {
    std::cout << "{\"q\":"; vectorJson(state.q);
    std::cout << ",\"dq\":"; vectorJson(state.dq);
    std::cout << ",\"quat\":";
    vectorJson(Eigen::Vector4d(state.quat.w(), state.quat.x(), state.quat.y(), state.quat.z()));
    std::cout << ",\"gyro\":"; vectorJson(state.gyro);
    std::cout << ",\"root_pos\":"; vectorJson(state.root_pos);
    std::cout << ",\"root_vel_b\":"; vectorJson(state.root_vel_b);
    std::cout << '}';
}
template<class Function> void mustThrow(Function fn) {
    try { fn(); } catch (const std::exception&) { return; }
    throw std::runtime_error("invalid-input check did not reject fixture");
}
int main(int argc, char** argv) {
    try {
        if (argc != 4) throw std::runtime_error("usage: physics_probe MODEL CONFIG FIXTURE");
        const auto config = YAML::LoadFile(argv[2]);
        const auto fixture = YAML::LoadFile(argv[3]);
        const auto names = config["joint_names"].as<std::vector<std::string>>();
        Sensors initial;
        load(fixture["q"], initial.q); load(fixture["dq"], initial.dq);
        Eigen::Vector4d quat; load(fixture["quat"], quat);
        initial.quat = Eigen::Quaterniond(quat[0], quat[1], quat[2], quat[3]);
        load(fixture["gyro"], initial.gyro); load(fixture["root_pos"], initial.root_pos);
        load(fixture["root_vel_b"], initial.root_vel_b);
        Joints kp, kd, effort, target;
        load(fixture["kp"], kp); load(fixture["kd"], kd);
        load(fixture["effort"], effort); load(fixture["target"], target);
        MujocoPlant plant(argv[1], names, .002, .02);
        FlatGroundEstimator estimator(argv[1], names);
        UpperBodyGravity gravity(argv[1], names, effort);
        plant.reset(initial);
        std::cout << std::setprecision(17) << "{\"version\":\"" << mj_versionString() << "\",\"reset\":";
        stateJson(plant.sensor());
        std::cout << ",\"estimate\":"; stateJson(estimator.estimate(initial));
        std::cout << ",\"gravity\":"; vectorJson(gravity.offset(initial, kp));
        gravity.setEffort(effort*.01);
        std::cout << ",\"gravity_limited\":"; vectorJson(gravity.offset(initial, kp));
        std::cout << ",\"steps\":[";
        for (int tick = 0; tick < 12; ++tick) {
            if (tick) std::cout << ',';
            if (tick == 8) plant.setControlDt(.002);
            stateJson(plant.step(target, kp, kd, effort, tick >= 3 && tick < 9));
        }
        std::cout << "],\"torque\":"; vectorJson(plant.lastTorque());
        std::cout << ",\"saturation_count\":" << plant.saturationCount();
        std::cout << ",\"time\":" << plant.time();
        plant.reset(initial);
        if (plant.time() != 0 || plant.saturationCount() != 0 || !plant.lastTorque().isZero())
            throw std::runtime_error("plant reset did not clear statistics");
        mustThrow([&] { plant.setControlDt(.003); });
        mustThrow([&] { plant.step(target, kp, kd, Joints::Zero()); });
        mustThrow([&] { gravity.offset(initial, Joints::Zero()); });
        auto invalid = initial; invalid.quat.coeffs().setZero();
        mustThrow([&] { estimator.estimate(invalid); });
        mustThrow([&] { gravity.offset(invalid, kp); });
        std::cout << ",\"validation\":true}\n";
        return 0;
    } catch (const std::exception& e) {
        std::cerr << e.what() << '\n'; return 1;
    }
}
