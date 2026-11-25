#pragma once

#include <memory>
#include <string>
#include <vector>

namespace g1control {
class MujocoPlant;

// Simulation-only viewer. Use pollCommands once per UI tick, including while
// paused; draw only renders. The plant must outlive this viewer.
class Viewer {
public:
    explicit Viewer(MujocoPlant& plant);
    ~Viewer() noexcept;
    Viewer(const Viewer&) = delete;
    Viewer& operator=(const Viewer&) = delete;
    Viewer(Viewer&&) = delete;
    Viewer& operator=(Viewer&&) = delete;

    std::vector<std::string> pollCommands();
    // Call immediately before each simulated plant.step(). It refreshes only
    // this viewer's body-force contribution. Passing true clears that force.
    void applyPerturbation(bool paused);
    bool draw(MujocoPlant& plant, const std::string& status,
              const std::string& selected, bool paused,
              const std::string& message = "");

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
}  // namespace g1control
