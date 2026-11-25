// Hidden, owned GLFW window: exercises MuJoCo perturbation without DDS or input
// injection into any other application. Internal access is limited to this test.
#include <memory>
#include <string>
#include <vector>
#define private public
#include "control/Viewer.h"
#undef private
#include "../../deploy/robots/g1/src/control/Viewer.cpp"
#include <iostream>

void expect(bool condition, const char* message) { if (!condition) throw std::runtime_error(message); }

int main(int argc, char** argv) {
    if(!std::getenv("DISPLAY")&&!std::getenv("WAYLAND_DISPLAY"))return 77;
    using namespace g1control;
    try {
        if (argc != 2) throw std::runtime_error("usage: viewer_checks ROOT");
        const std::filesystem::path root(argv[1]);
        const auto config = YAML::LoadFile((root/"policies/supermimic_horse_stance_v2/deploy_config.json").string());
        MujocoPlant plant(root/"assets/supermimic_g1/scene_29dof.xml", config["joint_names"].as<std::vector<std::string>>());
        Sensors state; state.root_pos.z() = .82;
        for (int i = 0; i < 29; ++i) state.q[i] = config["default_joint_pos"][i].as<double>();
        plant.reset(state);
        if (!glfwInit()) throw std::runtime_error("test requires an available X11/Wayland display");
        glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
        Viewer viewer(plant);
        auto& view = *viewer.impl_;
        viewer.draw(plant, "TEST", "Velocity", false);
        view.selectBody(550., 400.);
        expect(view.perturb.select > 0, "center ray did not select robot body");
        const int body = view.perturb.select;
        auto* force = plant.data()->xfrc_applied+6*body;
        for (int i = 0; i < 6; ++i) force[i] = .1*(i+1);
        std::array<double, 6> baseline{};
        std::copy_n(force, 6, baseline.begin());
        view.startPerturbation(GLFW_MOUSE_BUTTON_RIGHT);
        expect(view.perturb.active == mjPERT_TRANSLATE, "Ctrl-right translation did not start");
        mjv_movePerturb(plant.model(), plant.data(), mjMOUSE_MOVE_V, .1, 0., &view.scene, &view.perturb);
        viewer.applyPerturbation(false);
        double delta = 0.;
        for (int i = 0; i < 6; ++i) delta += std::abs(force[i]-baseline[i]);
        expect(delta > 1e-5, "translation perturbation did not apply force");
        std::array<double, 6> once{}; std::copy_n(force, 6, once.begin());
        viewer.applyPerturbation(false);
        for (int i = 0; i < 6; ++i) expect(std::abs(force[i]-once[i]) < 1e-10, "perturbation accumulated each frame");
        view.stopPerturbation();
        for (int i = 0; i < 6; ++i) expect(force[i] == baseline[i], "release erased another force or retained perturbation");

        view.startPerturbation(GLFW_MOUSE_BUTTON_LEFT);
        expect(view.perturb.active == mjPERT_ROTATE, "Ctrl-left rotation did not start");
        const std::vector<mjtNum> before(plant.data()->qpos, plant.data()->qpos+plant.model()->nq);
        viewer.applyPerturbation(true);
        expect(view.perturb.active == 0, "pause retained active perturbation");
        view.startPerturbation(GLFW_MOUSE_BUTTON_RIGHT);
        expect(view.perturb.active == 0, "paused drag started a physical perturbation");
        expect(std::equal(before.begin(), before.end(), plant.data()->qpos), "paused viewer altered joint pose");
        view.setPaused(false);
        view.startPerturbation(GLFW_MOUSE_BUTTON_RIGHT);
        mjv_movePerturb(plant.model(), plant.data(), mjMOUSE_MOVE_V, .1, 0., &view.scene, &view.perturb);
        viewer.applyPerturbation(false);
        for (int i = 0; i < 6; ++i) force[i] = 7.5+i;  // A newer external/hoist force replaces our write.
        view.stopPerturbation();
        for (int i = 0; i < 6; ++i) expect(force[i] == 7.5+i, "clear overwrote a newer external force");

        const double lookat = view.camera.lookat[0];
        mjv_moveCamera(plant.model(), mjMOUSE_MOVE_V, .1, 0., &view.scene, &view.camera);
        const double panned = view.camera.lookat[0];
        expect(std::abs(panned-lookat) > 1e-6, "camera pan did not move lookat");
        state.root_pos.x() += .2; plant.reset(state);
        viewer.draw(plant, "TEST", "Velocity", false);
        expect(std::abs(view.camera.lookat[0]-panned-.2) < 1e-10, "follow erased manual camera pan");
        view.enqueue("START");
        glfwSetWindowShouldClose(view.window, GLFW_TRUE);
        const auto commands = viewer.pollCommands();
        expect(commands.size() == 1 && commands[0] == "STOP", "close did not prioritize STOP");
        std::cout << "PASS: hidden viewer selection, perturb force/release/pause, force ownership, follow-pan, close STOP\n";
    } catch (const std::exception& error) { std::cerr << error.what() << '\n'; return 1; }
}
