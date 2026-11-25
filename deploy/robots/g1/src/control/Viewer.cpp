#include "control/Viewer.h"
#include "control/Physics.h"

#include <GLFW/glfw3.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace g1control {
namespace {
std::string asciiLine(const std::string& text, std::size_t limit = 160) {
    std::string result;
    result.reserve(std::min(text.size(), limit));
    for (unsigned char c : text) {
        if (result.size() == limit) { result += "..."; break; }
        result.push_back(c >= 32 && c <= 126 ? static_cast<char>(c) : ' ');
    }
    return result;
}

std::string glfwFailure(const char* operation) {
    const char* description = nullptr;
    glfwGetError(&description);
    return std::string(operation) + (description ? ": " + std::string(description) : "")
        + "; use --headless without a graphical display";
}
}  // namespace

struct Viewer::Impl {
    MujocoPlant* plant;
    GLFWwindow* window = nullptr;
    bool glfw_ready = false, scene_ready = false, context_ready = false;
    mjvCamera camera{};
    mjvOption option{};
    mjvScene scene{};
    mjrContext context{};
    mjvPerturb perturb{};
    bool paused = false;
    int force_body = -1, last_button = -1;
    std::array<mjtNum, 6> force_baseline{}, force_written{};
    double last_press = -1., press_x = 0., press_y = 0.;
    Eigen::Vector3d followed_position = Eigen::Vector3d::Zero();
    double cursor_x = 0., cursor_y = 0.;
    std::vector<std::string> commands;

    explicit Impl(MujocoPlant& robot) : plant(&robot) {
        mjv_defaultCamera(&camera);
        mjv_defaultOption(&option);
        mjv_defaultScene(&scene);
        mjr_defaultContext(&context);
        mjv_defaultPerturb(&perturb);
        try {
            if (!glfwInit()) throw std::runtime_error(glfwFailure("GLFW initialization failed"));
            glfw_ready = true;
            window = glfwCreateWindow(1100, 800, "G1 C++ interactive simulation", nullptr, nullptr);
            if (!window) throw std::runtime_error(glfwFailure("Cannot create simulation window"));
            glfwMakeContextCurrent(window);
            glfwSwapInterval(1);

            // Defaults make the corresponding free functions safe if allocation
            // fails part-way through an exception-enabled MuJoCo build.
            scene_ready = true;
            mjv_makeScene(plant->model(), &scene, 2000);
            context_ready = true;
            mjr_makeContext(plant->model(), &context, mjFONTSCALE_150);
            camera.type = mjCAMERA_FREE;
            camera.distance = 3.;
            camera.azimuth = 135.;
            camera.elevation = -15.;
            followed_position = plant->sensor().root_pos;
            for (int axis = 0; axis < 3; ++axis) camera.lookat[axis] = followed_position[axis];

            glfwSetWindowUserPointer(window, this);
            glfwSetKeyCallback(window, onKey);
            glfwSetWindowCloseCallback(window, onClose);
            glfwSetWindowFocusCallback(window, onFocus);
            glfwSetMouseButtonCallback(window, onMouseButton);
            glfwSetCursorPosCallback(window, onCursor);
            glfwSetScrollCallback(window, onScroll);
            glfwGetCursorPos(window, &cursor_x, &cursor_y);
        } catch (...) {
            close();
            throw;
        }
    }

    ~Impl() noexcept { close(); }

    void close() noexcept {
        stopPerturbation();
        if (window) {
            glfwSetWindowUserPointer(window, nullptr);
            glfwMakeContextCurrent(window);
        }
        if (context_ready) { mjr_freeContext(&context); context_ready = false; }
        if (scene_ready) { mjv_freeScene(&scene); scene_ready = false; }
        if (window) { glfwDestroyWindow(window); window = nullptr; }
        if (glfw_ready) { glfwTerminate(); glfw_ready = false; }
    }

    static Impl* self(GLFWwindow* source) {
        return static_cast<Impl*>(glfwGetWindowUserPointer(source));
    }

    void clearForce() noexcept {
        if (force_body <= 0) return;
        auto* applied = plant->data()->xfrc_applied+6*force_body;
        bool unchanged = true;
        for (int i = 0; i < 6; ++i)
            unchanged &= std::abs(applied[i]-force_written[i]) <= 1e-10;
        // The virtual hoist can replace the pelvis force during plant.step.
        // Never erase that newer force while removing our older contribution.
        if (unchanged) std::copy(force_baseline.begin(), force_baseline.end(), applied);
        force_body = -1;
    }

    void stopPerturbation() noexcept {
        clearForce();
        perturb.active = 0;
        perturb.active2 = 0;
    }

    void setPaused(bool value) {
        paused = value;
        if (paused) stopPerturbation();
    }

    void selectBody(double x, double y) {
        stopPerturbation();
        int width = 0, height = 0;
        glfwGetWindowSize(window, &width, &height);
        if (width <= 0 || height <= 0) return;
        mjtNum point[3]{};
        int geom = -1, flex = -1, skin = -1;
        const int body = mjv_select(plant->model(), plant->data(), &option,
            static_cast<double>(width)/height, x/width, 1.-y/height,
            &scene, point, &geom, &flex, &skin);
        if (body > 0) {
            perturb.select = body; perturb.flexselect = flex; perturb.skinselect = skin;
            mjtNum offset[3];
            mju_sub3(offset, point, plant->data()->xpos+3*body);
            mju_mulMatTVec(perturb.localpos, plant->data()->xmat+9*body, offset, 3, 3);
        } else {
            perturb.select = 0; perturb.flexselect = -1; perturb.skinselect = -1;
        }
    }

    void startPerturbation(int button) {
        if (paused || perturb.select <= 0) return;
        const int active = button == GLFW_MOUSE_BUTTON_RIGHT ? mjPERT_TRANSLATE
            : button == GLFW_MOUSE_BUTTON_LEFT ? mjPERT_ROTATE : 0;
        if (!active) return;
        stopPerturbation();
        mjv_initPerturb(plant->model(), plant->data(), &scene, &perturb);
        perturb.active = active;
    }

    void applyPerturbation(bool is_paused) {
        setPaused(is_paused);
        clearForce();
        if (paused || !perturb.active || perturb.select <= 0 || glfwWindowShouldClose(window)) return;
        force_body = perturb.select;
        auto* applied = plant->data()->xfrc_applied+6*force_body;
        std::copy_n(applied, 6, force_baseline.begin());
        std::fill_n(applied, 6, 0.);
        mjv_applyPerturbForce(plant->model(), plant->data(), &perturb);
        for (int i = 0; i < 6; ++i) {
            applied[i] += force_baseline[i];
            force_written[i] = applied[i];
        }
    }

    void enqueue(const char* command) {
        // A stop/close event takes precedence over every other event collected
        // in this poll. It cannot be lost behind selection or start requests.
        if (std::string(command) == "STOP") {
            stopPerturbation();
            commands.clear();
            commands.emplace_back("STOP");
        } else if (commands.empty() || commands.front() != "STOP") {
            if (commands.size() < 64) commands.emplace_back(command);
        }
    }

    static void onKey(GLFWwindow* source, int key, int, int action, int) {
        auto* viewer = self(source);
        if (!viewer) return;
        if ((key == GLFW_KEY_LEFT_CONTROL || key == GLFW_KEY_RIGHT_CONTROL) && action == GLFW_RELEASE)
            viewer->stopPerturbation();
        if (action != GLFW_PRESS) return;
        switch (key) {
        case GLFW_KEY_T: viewer->enqueue("TAKEOVER"); break;
        case GLFW_KEY_S:
        case GLFW_KEY_ENTER:
        case GLFW_KEY_KP_ENTER: viewer->enqueue("START"); break;
        case GLFW_KEY_Q:
        case GLFW_KEY_ESCAPE: viewer->enqueue("STOP"); break;
        case GLFW_KEY_P: viewer->enqueue("passive"); break;
        case GLFW_KEY_SPACE: viewer->enqueue("PAUSE"); break;
        case GLFW_KEY_LEFT_BRACKET:
        case GLFW_KEY_LEFT: viewer->enqueue("SELECT_PREV"); break;
        case GLFW_KEY_RIGHT_BRACKET:
        case GLFW_KEY_RIGHT: viewer->enqueue("SELECT_NEXT"); break;
        default: break;
        }
    }

    static void onClose(GLFWwindow* source) {
        if (auto* viewer = self(source)) viewer->enqueue("STOP");
    }

    static void onFocus(GLFWwindow* source, int focused) {
        if (!focused) if (auto* viewer = self(source)) viewer->stopPerturbation();
    }

    static void onMouseButton(GLFWwindow* source, int button, int action, int mods) {
        auto* viewer = self(source);
        if (!viewer) return;
        glfwGetCursorPos(source, &viewer->cursor_x, &viewer->cursor_y);
        if (action == GLFW_RELEASE) { viewer->stopPerturbation(); return; }
        if (action != GLFW_PRESS) return;
        const auto now = glfwGetTime();
        const bool double_click = button == GLFW_MOUSE_BUTTON_LEFT && button == viewer->last_button &&
            now-viewer->last_press <= .25 && std::hypot(viewer->cursor_x-viewer->press_x, viewer->cursor_y-viewer->press_y) <= 5.;
        viewer->last_button = button; viewer->last_press = now;
        viewer->press_x = viewer->cursor_x; viewer->press_y = viewer->cursor_y;
        if (double_click) {
            viewer->last_button = -1;
            viewer->selectBody(viewer->cursor_x, viewer->cursor_y);
        } else if (mods & GLFW_MOD_CONTROL) viewer->startPerturbation(button);
    }

    static void onCursor(GLFWwindow* source, double x, double y) {
        auto* viewer = self(source);
        if (!viewer) return;
        const double dx = x-viewer->cursor_x, dy = y-viewer->cursor_y;
        viewer->cursor_x = x; viewer->cursor_y = y;
        if (!std::isfinite(dx) || !std::isfinite(dy)) return;
        int width = 0, height = 0;
        glfwGetWindowSize(source, &width, &height);
        if (height <= 0 || width <= 0) return;
        const bool right = glfwGetMouseButton(source, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS;
        const bool left = glfwGetMouseButton(source, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS;
        if (!right && !left) return;
        const bool control = glfwGetKey(source, GLFW_KEY_LEFT_CONTROL) == GLFW_PRESS ||
            glfwGetKey(source, GLFW_KEY_RIGHT_CONTROL) == GLFW_PRESS;
        const bool shift = glfwGetKey(source, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS ||
            glfwGetKey(source, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS;
        const auto action = right ? (shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V)
                                  : (shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V);
        if (viewer->paused || !control) viewer->stopPerturbation();
        if (viewer->perturb.active)
            mjv_movePerturb(viewer->plant->model(), viewer->plant->data(), action,
                            dx/height, dy/height, &viewer->scene, &viewer->perturb);
        else mjv_moveCamera(viewer->plant->model(), action,
                            dx/height, dy/height, &viewer->scene, &viewer->camera);
    }

    static void onScroll(GLFWwindow* source, double, double offset) {
        if (auto* viewer = self(source); viewer && std::isfinite(offset))
            mjv_moveCamera(viewer->plant->model(), mjMOUSE_ZOOM, 0., -.05*offset,
                           &viewer->scene, &viewer->camera);
    }
};

Viewer::Viewer(MujocoPlant& plant) : impl_(std::make_unique<Impl>(plant)) {}
Viewer::~Viewer() noexcept = default;

void Viewer::applyPerturbation(bool paused) { impl_->applyPerturbation(paused); }

std::vector<std::string> Viewer::pollCommands() {
    glfwPollEvents();
    if (glfwWindowShouldClose(impl_->window)) impl_->enqueue("STOP");
    std::vector<std::string> result;
    result.swap(impl_->commands);
    return result;
}

bool Viewer::draw(MujocoPlant& plant, const std::string& status,
                  const std::string& selected, bool paused, const std::string& message) {
    auto& view = *impl_;
    view.setPaused(paused);
    if (&plant != view.plant) throw std::invalid_argument("Viewer must draw its original MuJoCo plant");
    if (glfwWindowShouldClose(view.window)) return false;
    const auto position = plant.sensor().root_pos;
    if (position.allFinite()) {
        const auto delta = position-view.followed_position;
        for (int axis = 0; axis < 3; ++axis) view.camera.lookat[axis] += delta[axis];
        view.followed_position = position;
    }
    int width = 0, height = 0;
    glfwGetFramebufferSize(view.window, &width, &height);
    if (width <= 0 || height <= 0) return true;
    glfwMakeContextCurrent(view.window);
    const mjrRect viewport{0, 0, width, height};
    mjv_updateScene(plant.model(), plant.data(), &view.option, &view.perturb, &view.camera,
                    mjCAT_ALL, &view.scene);
    mjr_render(viewport, &view.scene, &view.context);

    const char* body_name = view.perturb.select > 0 ? mj_id2name(plant.model(), mjOBJ_BODY, view.perturb.select) : nullptr;
    const std::string details = asciiLine(status) + '\n' + asciiLine(selected) + '\n'
        + (paused ? "YES" : "NO") + '\n' + (message.empty() ? "-" : asciiLine(message))
        + '\n' + (body_name ? asciiLine(body_name) : "none") + (view.perturb.active ? " [dragging]" : "");
    mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, viewport,
                "State\nSelected\nPaused\nMessage\nBody", details.c_str(), &view.context);
    mjr_overlay(mjFONT_NORMAL, mjGRID_BOTTOMLEFT, viewport,
                "T: takeover   S / Enter: start   Q / Esc: stop   P: passive\n"
                "Space: pause   [ / Left: previous   ] / Right: next\n"
                "Mouse: left drag rotate | right drag pan | wheel zoom\n"
                "Double left click: select body | Ctrl+right drag: pull | Ctrl+left drag: twist\n"
                "Shift: horizontal drag | Paused: camera only, no physical perturbation",
                nullptr, &view.context);
    const std::string title = "G1 C++ | " + asciiLine(status, 80)
        + (paused ? " | PAUSED" : "");
    glfwSetWindowTitle(view.window, title.c_str());
    glfwSwapBuffers(view.window);
    return true;
}
}  // namespace g1control
