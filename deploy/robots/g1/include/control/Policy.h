#pragma once
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

namespace g1control {
using Joints = Eigen::Matrix<double,29,1>;
struct Sensors {
    Joints q=Joints::Zero(), dq=Joints::Zero();
    Eigen::Quaterniond quat{1,0,0,0};
    Eigen::Vector3d gyro=Eigen::Vector3d::Zero();
    Eigen::Vector3d root_pos=Eigen::Vector3d::Zero(), root_vel_b=Eigen::Vector3d::Zero();
    Eigen::Vector3d velocity_command=Eigen::Vector3d::Zero();
};
struct PolicyInfo {
    std::string name, kind, policy_kind, target_processing;
    double dt=.02;
    Joints kp=Joints::Zero(), kd=Joints::Zero(), effort=Joints::Constant(100.);
    Joints entry_q=Joints::Zero();
    std::vector<std::string> joint_names;
    size_t frames=0, observation_width=0;
    bool needs_estimator=false, continuous=false, hold_final=false, robust_proprio=false;
};
struct PolicyOutput {
    Joints q=Joints::Zero(), action=Joints::Zero();
    std::vector<float> observation, goal;
    size_t frame=0;
};
class Policy {
public:
    virtual ~Policy()=default;
    virtual const PolicyInfo& info() const=0;
    virtual void begin(const Sensors& state)=0;
    virtual PolicyOutput infer(const Sensors& state)=0;
    virtual void advance()=0;
    virtual size_t frame() const=0;
    virtual void holdFinal() {}
    // These operations preserve sensor history and the heading anchor. A frozen
    // Hybrid goal repeats one reference pose with zero future joint velocity.
    virtual void seek(size_t) { throw std::logic_error("policy does not support seeking"); }
    virtual void holdReferenceFrame(std::optional<size_t>) { throw std::logic_error("policy does not support reference holding"); }
    // Feed the command that was actually executed back into the actor's action
    // coordinates. It changes only last_action, not already observed history.
    virtual void feedbackTarget(const Joints&) { throw std::logic_error("policy does not support target feedback"); }
    virtual void resetActionHistory() { throw std::logic_error("policy does not support action reset"); }
    virtual Joints referenceJoints(size_t) const { return info().entry_q; }
    virtual Eigen::Quaterniond referenceQuaternion(size_t) const { return Eigen::Quaterniond::Identity(); }
    virtual double referenceHeight(size_t) const { return .78; }
    virtual Eigen::Vector3d referenceGravity(size_t) const { return {0,0,-1}; }
};
// Both factories load/validate local artifacts only. They never create DDS connections.
std::unique_ptr<Policy> makeMotionPolicy(const YAML::Node& spec,const std::filesystem::path& root);
std::unique_ptr<Policy> makeLegacyPolicy(const YAML::Node& spec,const std::filesystem::path& root);
}
