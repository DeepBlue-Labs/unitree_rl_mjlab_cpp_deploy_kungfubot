#pragma once
#include "control/Policy.h"
#include <map>
#include <optional>

namespace g1control {
class Catalog;
struct SessionGateResult {
    bool ready=false;
    size_t stable_ticks=0;
    std::vector<std::string> reasons;
    std::map<std::string,double> metrics;
};
struct CompletedMotion { std::string policy; size_t frames=0; };
struct SessionOutput {
    Joints q=Joints::Zero(), kp=Joints::Zero(), kd=Joints::Constant(3), effort=Joints::Constant(100);
    Joints raw_action=Joints::Zero();
    std::vector<float> observation;
    std::string phase, active_policy;
    long frame=-1;
    bool support=false, finished=false;
};
// Full Python deployment-session semantics nested inside the top-level RL state.
// Owns session progression; callers must not separately advance its policies.
// All actors are preloaded in Catalog; construction allocates only local kinematics.
class Session {
public:
    Session(Catalog& catalog, const YAML::Node& initial_spec, const YAML::Node& registry,
            const std::filesystem::path& model_path, double damp_s=1.);
    ~Session();
    Session(const Session&)=delete;
    Session& operator=(const Session&)=delete;
    void reset();
    // Automatic simulation starts on the zero-command standing actor, without takeover.
    void startStanding(const Sensors& state);
    void stop(const std::string& reason="STOP requested");
    SessionOutput step(const Sensors& state, const std::string& command={});
    bool acceptsCommand(const std::string& command) const;
    bool finished() const;
    const std::string& phase() const;
    const std::string& event() const;
    const std::string& stopReason() const;
    const std::string& activeName() const;
    const std::string& initialName() const;
    const std::string& lastMotionName() const;
    const std::string& pendingName() const;
    Policy& activePolicy() const;
    size_t policyTicks() const;
    size_t expectedPolicyTicks() const;
    size_t holdTicks() const;
    size_t pendingTicks() const;
    std::optional<double> entryPoseRms() const;
    const std::vector<CompletedMotion>& completedMotions() const;
    const SessionGateResult& gateResult() const;
    std::string status() const;
private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
} // namespace g1control
