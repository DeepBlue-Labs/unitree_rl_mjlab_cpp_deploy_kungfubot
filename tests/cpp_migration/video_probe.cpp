// Offscreen recording smoke test; no DDS participant or robot connection.
#include "control/Physics.h"
#include "control/VideoRecorder.h"
#include <iostream>

int main(int argc, char** argv) {
    try {
        if (argc != 3) throw std::runtime_error("usage: video_probe ROOT OUTPUT.mp4");
        const std::filesystem::path root(argv[1]);
        const auto config = YAML::LoadFile((root/"policies/supermimic_horse_stance_v2/deploy_config.json").string());
        const auto names = config["joint_names"].as<std::vector<std::string>>();
        g1control::MujocoPlant plant(root/"assets/supermimic_g1/scene_29dof.xml", names);
        g1control::Sensors initial;
        initial.root_pos.z() = .82;
        g1control::Joints kp, kd, effort;
        for (int i = 0; i < 29; ++i) {
            initial.q[i] = config["default_joint_pos"][i].as<double>();
            kp[i] = config["kp"][i].as<double>(); kd[i] = config["kd"][i].as<double>();
            effort[i] = config["effort_limit"][i].as<double>();
        }
        plant.reset(initial);
        g1control::VideoRecorder recorder(plant, argv[2]);
        for (int frame = 0; frame < 25; ++frame) {
            recorder.capture();
            for (int tick = 0; tick < 2; ++tick) plant.step(initial.q, kp, kd, effort, true);
        }
        recorder.finish();
        recorder.finish();  // Idempotent finalization.
        std::cout << "PASS: EGL recorded 25 frames without a display or DDS\n";
    } catch (const std::exception& error) { std::cerr << error.what() << '\n'; return 1; }
}
