#include "control/VideoRecorder.h"
#include "control/Physics.h"

#include <EGL/egl.h>
#include <EGL/eglext.h>
#include <GL/glx.h>
#include <fcntl.h>
#include <poll.h>
#include <pthread.h>
#include <signal.h>
#include <spawn.h>
#include <sys/wait.h>
#include <unistd.h>

#include <array>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <vector>

extern char** environ;

namespace g1control {
namespace {
constexpr int kWidth = 640, kHeight = 480;
using Clock = std::chrono::steady_clock;

std::runtime_error eglError(const char* operation) {
    std::ostringstream error;
    error << operation << " (EGL 0x" << std::hex << eglGetError() << ')';
    return std::runtime_error(error.str());
}

// Preserve either an EGL context or the GLFW viewer's default GLX context.
// This lets capture coexist with an interactive window on the same thread.
class CurrentContext {
    EGLDisplay display_, previous_display_ = EGL_NO_DISPLAY;
    EGLSurface previous_draw_ = EGL_NO_SURFACE, previous_read_ = EGL_NO_SURFACE;
    EGLContext previous_egl_ = EGL_NO_CONTEXT;
    Display* previous_xdisplay_ = nullptr;
    GLXDrawable previous_xdraw_ = 0, previous_xread_ = 0;
    GLXContext previous_glx_ = nullptr;
public:
    CurrentContext(EGLDisplay display, EGLSurface surface, EGLContext context) : display_(display) {
        previous_egl_ = eglGetCurrentContext();
        if (previous_egl_ != EGL_NO_CONTEXT) {
            previous_display_ = eglGetCurrentDisplay();
            previous_draw_ = eglGetCurrentSurface(EGL_DRAW);
            previous_read_ = eglGetCurrentSurface(EGL_READ);
        } else if ((previous_glx_ = glXGetCurrentContext())) {
            previous_xdisplay_ = glXGetCurrentDisplay();
            previous_xdraw_ = glXGetCurrentDrawable(); previous_xread_ = glXGetCurrentReadDrawable();
            glXMakeContextCurrent(previous_xdisplay_, 0, 0, nullptr);
        }
        if (!eglMakeCurrent(display, surface, surface, context)) {
            restore();
            throw eglError("cannot activate recording context");
        }
    }
    ~CurrentContext() { restore(); }
    void restore() noexcept {
        eglMakeCurrent(display_, EGL_NO_SURFACE, EGL_NO_SURFACE, EGL_NO_CONTEXT);
        if (previous_egl_ != EGL_NO_CONTEXT)
            eglMakeCurrent(previous_display_, previous_draw_, previous_read_, previous_egl_);
        else if (previous_glx_)
            glXMakeContextCurrent(previous_xdisplay_, previous_xdraw_, previous_xread_, previous_glx_);
    }
};

// A failed ffmpeg process must become a C++ error, not a process-wide SIGPIPE.
class BlockPipeSignal {
    sigset_t blocked_{}, previous_{};
    bool previously_pending_ = false, active_ = false;
public:
    BlockPipeSignal() {
        sigemptyset(&blocked_); sigaddset(&blocked_, SIGPIPE);
        sigset_t pending{}; sigpending(&pending); previously_pending_ = sigismember(&pending, SIGPIPE) == 1;
        active_ = pthread_sigmask(SIG_BLOCK, &blocked_, &previous_) == 0;
        if (!active_) throw std::runtime_error("cannot protect ffmpeg pipe writes from SIGPIPE");
    }
    ~BlockPipeSignal() {
        if (!active_) return;
        if (!previously_pending_ && sigismember(&previous_, SIGPIPE) != 1) {
            const timespec zero{};
            while (sigtimedwait(&blocked_, nullptr, &zero) >= 0) {}
        }
        pthread_sigmask(SIG_SETMASK, &previous_, nullptr);
    }
};
}  // namespace

struct VideoRecorder::Impl {
    MujocoPlant& plant;
    std::filesystem::path output, log;
    EGLDisplay display = EGL_NO_DISPLAY;
    EGLSurface surface = EGL_NO_SURFACE;
    EGLContext egl_context = EGL_NO_CONTEXT;
    bool egl_ready = false, scene_ready = false, render_ready = false;
    mjvScene scene{};
    mjvCamera camera{};
    mjvOption option{};
    mjrContext render{};
    std::vector<unsigned char> pixels = std::vector<unsigned char>(kWidth*kHeight*3);
    pid_t encoder = -1;
    int input = -1;
    bool finished = false;

    Impl(MujocoPlant& robot, const std::filesystem::path& path) : plant(robot) {
        if (path.empty()) throw std::invalid_argument("video output path is empty");
        output = std::filesystem::absolute(path);
        log = output.string()+".ffmpeg.log";
        mjv_defaultScene(&scene); mjv_defaultCamera(&camera); mjv_defaultOption(&option); mjr_defaultContext(&render);
        try {
            std::filesystem::create_directories(output.parent_path());
            initializeEgl();
            {
                CurrentContext current(display, surface, egl_context);
                scene_ready = true; mjv_makeScene(plant.model(), &scene, 10000);
                render_ready = true; mjr_makeContext(plant.model(), &render, mjFONTSCALE_100);
                mjr_resizeOffscreen(kWidth, kHeight, &render);
                mjr_setBuffer(mjFB_OFFSCREEN, &render);
                if (render.currentBuffer != mjFB_OFFSCREEN) throw std::runtime_error("MuJoCo offscreen framebuffer unavailable");
            }
            camera.type = mjCAMERA_FREE; camera.distance = 3.; camera.azimuth = 135.; camera.elevation = -15.;
            startEncoder();
        } catch (...) { closeResources(); throw; }
    }
    ~Impl() noexcept {
        try { finish(); } catch (...) {}
        closeResources();
    }

    void initializeEgl() {
        // Mesa's surfaceless platform works without DISPLAY/X/Wayland. If it is
        // unavailable, use the driver's default EGL display (also supports GPU
        // drivers which expose a native display without a visible window).
        const auto platform = reinterpret_cast<PFNEGLGETPLATFORMDISPLAYEXTPROC>(eglGetProcAddress("eglGetPlatformDisplayEXT"));
        if (platform) display = platform(EGL_PLATFORM_SURFACELESS_MESA, EGL_DEFAULT_DISPLAY, nullptr);
        EGLint major = 0, minor = 0;
        if (display == EGL_NO_DISPLAY || !eglInitialize(display, &major, &minor)) {
            display = eglGetDisplay(EGL_DEFAULT_DISPLAY);
            if (display == EGL_NO_DISPLAY || !eglInitialize(display, &major, &minor))
                throw eglError("cannot initialize EGL offscreen recording; install a working EGL/OpenGL driver");
        }
        egl_ready = true;
        if (!eglBindAPI(EGL_OPENGL_API)) throw eglError("cannot bind EGL OpenGL API");
        const EGLint attributes[]{EGL_SURFACE_TYPE, EGL_PBUFFER_BIT, EGL_RENDERABLE_TYPE, EGL_OPENGL_BIT,
            EGL_RED_SIZE, 8, EGL_GREEN_SIZE, 8, EGL_BLUE_SIZE, 8, EGL_DEPTH_SIZE, 24, EGL_NONE};
        EGLConfig config{}; EGLint count = 0;
        if (!eglChooseConfig(display, attributes, &config, 1, &count) || count < 1)
            throw eglError("no compatible EGL pbuffer configuration");
        const char* extensions = eglQueryString(display, EGL_EXTENSIONS);
        const bool surfaceless = (major > 1 || (major == 1 && minor >= 5)) ||
            (extensions && std::strstr(extensions, "EGL_KHR_surfaceless_context"));
        // Match MuJoCo's Python EGL backend: use no default framebuffer when
        // the driver supports it; MuJoCo owns its offscreen framebuffer.
        if (!surfaceless) {
            const EGLint size[]{EGL_WIDTH, kWidth, EGL_HEIGHT, kHeight, EGL_NONE};
            surface = eglCreatePbufferSurface(display, config, size);
            if (surface == EGL_NO_SURFACE) throw eglError("cannot create recording pbuffer");
        }
        egl_context = eglCreateContext(display, config, EGL_NO_CONTEXT, nullptr);
        if (egl_context == EGL_NO_CONTEXT) throw eglError("cannot create recording OpenGL context");
    }

    void startEncoder() {
        int pipe[2];
        if (pipe2(pipe, O_CLOEXEC) != 0) throw std::runtime_error("cannot create ffmpeg input pipe");
        const int errors = open(log.c_str(), O_WRONLY|O_CREAT|O_TRUNC|O_CLOEXEC, 0644);
        if (errors < 0) { close(pipe[0]); close(pipe[1]); throw std::runtime_error("cannot create ffmpeg log"); }
        std::vector<std::string> arguments{"ffmpeg", "-hide_banner", "-loglevel", "error", "-y",
            "-f", "rawvideo", "-pixel_format", "rgb24", "-video_size", "640x480", "-framerate", "25", "-i", "pipe:0",
            "-an", "-vf", "vflip", "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "20",
            "-movflags", "+faststart", "-f", "mp4", output.string()};
        std::vector<char*> argv;
        for (auto& value : arguments) argv.push_back(value.data());
        argv.push_back(nullptr);
        posix_spawn_file_actions_t actions; posix_spawn_file_actions_init(&actions);
        posix_spawn_file_actions_adddup2(&actions, pipe[0], STDIN_FILENO);
        posix_spawn_file_actions_adddup2(&actions, errors, STDERR_FILENO);
        posix_spawn_file_actions_adddup2(&actions, errors, STDOUT_FILENO);
        posix_spawn_file_actions_addclose(&actions, pipe[0]);
        posix_spawn_file_actions_addclose(&actions, pipe[1]);
        posix_spawn_file_actions_addclose(&actions, errors);
        const int code = posix_spawnp(&encoder, "ffmpeg", &actions, nullptr, argv.data(), environ);
        posix_spawn_file_actions_destroy(&actions);
        close(pipe[0]); close(errors);
        if (code != 0) { close(pipe[1]); encoder = -1; throw std::runtime_error("cannot start ffmpeg: "+std::string(std::strerror(code))); }
        input = pipe[1];
        if (fcntl(input, F_SETFL, O_NONBLOCK) != 0) throw std::runtime_error("cannot configure ffmpeg input pipe");
    }

    void capture() {
        if (finished || input < 0) throw std::runtime_error("video recorder has already finished");
        {
            CurrentContext current(display, surface, egl_context);
            const auto position = plant.sensor().root_pos;
            for (int axis = 0; axis < 3; ++axis) camera.lookat[axis] = position[axis];
            mjv_updateScene(plant.model(), plant.data(), &option, nullptr, &camera, mjCAT_ALL, &scene);
            mjr_setBuffer(mjFB_OFFSCREEN, &render);
            const mjrRect viewport{0, 0, kWidth, kHeight};
            mjr_render(viewport, &scene, &render);
            mjr_readPixels(pixels.data(), nullptr, viewport, &render);
        }
        BlockPipeSignal blocked;
        const auto started = Clock::now();
        std::size_t sent = 0;
        while (sent < pixels.size()) {
            const auto count = ::write(input, pixels.data()+sent, pixels.size()-sent);
            if (count > 0) { sent += static_cast<std::size_t>(count); continue; }
            if (count < 0 && errno == EINTR) continue;
            if (count < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
                if (std::chrono::duration<double>(Clock::now()-started).count() > 10.)
                    throw std::runtime_error("ffmpeg input stalled; see "+log.string());
                pollfd descriptor{input, POLLOUT, 0}; poll(&descriptor, 1, 50); continue;
            }
            throw std::runtime_error("ffmpeg rejected video input; see "+log.string());
        }
    }

    void finish() {
        if (finished) return;
        finished = true;
        if (input >= 0) { close(input); input = -1; }
        if (encoder <= 0) return;
        const auto started = Clock::now();
        int status = 0;
        while (true) {
            const auto result = waitpid(encoder, &status, WNOHANG);
            if (result == encoder) { encoder = -1; break; }
            if (result < 0 && errno != EINTR) { encoder = -1; throw std::runtime_error("cannot wait for ffmpeg"); }
            if (std::chrono::duration<double>(Clock::now()-started).count() > 30.) {
                kill(encoder, SIGKILL);
                while (waitpid(encoder, &status, 0) < 0 && errno == EINTR) {}
                encoder = -1;
                throw std::runtime_error("ffmpeg finalization timed out; see "+log.string());
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        if (!WIFEXITED(status) || WEXITSTATUS(status) != 0)
            throw std::runtime_error("ffmpeg could not finish the video; see "+log.string());
    }

    void closeResources() noexcept {
        if (input >= 0) { close(input); input = -1; }
        if (encoder > 0) {
            kill(encoder, SIGKILL);
            int status; while (waitpid(encoder, &status, 0) < 0 && errno == EINTR) {}
            encoder = -1;
        }
        if (egl_context != EGL_NO_CONTEXT && display != EGL_NO_DISPLAY) {
            try {
                CurrentContext current(display, surface, egl_context);
                if (render_ready) mjr_freeContext(&render);
            } catch (...) {}
            render_ready = false;
        }
        if (scene_ready) { mjv_freeScene(&scene); scene_ready = false; }
        if (egl_context != EGL_NO_CONTEXT) { eglDestroyContext(display, egl_context); egl_context = EGL_NO_CONTEXT; }
        if (surface != EGL_NO_SURFACE) { eglDestroySurface(display, surface); surface = EGL_NO_SURFACE; }
        if (egl_ready) { eglTerminate(display); egl_ready = false; }
        display = EGL_NO_DISPLAY;
    }
};

VideoRecorder::VideoRecorder(MujocoPlant& plant, const std::filesystem::path& path)
    : impl_(std::make_unique<Impl>(plant, path)) {}
VideoRecorder::~VideoRecorder() noexcept = default;
void VideoRecorder::capture() { impl_->capture(); }
void VideoRecorder::finish() { impl_->finish(); }
}  // namespace g1control
