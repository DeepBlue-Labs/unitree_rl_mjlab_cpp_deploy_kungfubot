#pragma once

#include <filesystem>
#include <memory>

namespace g1control {
class MujocoPlant;

// EGL offscreen recording using the original Python plant's 640x480 camera.
// Call capture once per two 50 Hz policy ticks to produce a 25 fps MP4.
class VideoRecorder {
public:
    VideoRecorder(MujocoPlant& plant, const std::filesystem::path& output_path);
    ~VideoRecorder() noexcept;
    VideoRecorder(const VideoRecorder&) = delete;
    VideoRecorder& operator=(const VideoRecorder&) = delete;
    void capture();
    void finish();
private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
}  // namespace g1control
