#pragma once

#include "control/Policy.h"
#include <mujoco/mujoco.h>
#include <cstdint>

namespace g1control {

// Joint vectors use exactly the supplied policy joint-name order. Free-joint
// angular velocity and Sensors linear velocity are expressed in the body frame.
class MujocoPlant {
public:
    MujocoPlant(const std::filesystem::path& model_path,
                const std::vector<std::string>& joint_names,
                double physics_dt = .002, double control_dt = .02);
    ~MujocoPlant();
    MujocoPlant(MujocoPlant&&) noexcept;
    MujocoPlant& operator=(MujocoPlant&&) noexcept;
    MujocoPlant(const MujocoPlant&) = delete;
    MujocoPlant& operator=(const MujocoPlant&) = delete;

    void reset(const Sensors& state);
    Sensors sensor() const;
    Sensors step(const Joints& target, const Joints& kp, const Joints& kd,
                 const Joints& effort, bool support = false);
    void applyVelocityPush(const Eigen::Vector3d& linear_body,const Eigen::Vector3d& angular_body);
    void setControlDt(double control_dt);
    double physicsDt() const;
    double time() const;
    const Joints& lastTorque() const;
    std::uint64_t saturationCount() const;

    // For an optional viewer. Pointers remain owned by this plant.
    mjModel* model();
    mjData* data();
    const mjModel* model() const;
    const mjData* data() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

// Planted lowest-foot contact on flat ground determines base height/velocity.
// This replaces missing observations only; it never modifies joint targets.
class FlatGroundEstimator {
public:
    FlatGroundEstimator(const std::filesystem::path& model_path,
                        const std::vector<std::string>& joint_names);
    ~FlatGroundEstimator();
    FlatGroundEstimator(FlatGroundEstimator&&) noexcept;
    FlatGroundEstimator& operator=(FlatGroundEstimator&&) noexcept;
    FlatGroundEstimator(const FlatGroundEstimator&) = delete;
    FlatGroundEstimator& operator=(const FlatGroundEstimator&) = delete;
    Sensors estimate(const Sensors& state);
private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

// Static waist/arm compensation used in preparation. Leg offsets stay zero.
class UpperBodyGravity {
public:
    UpperBodyGravity(const std::filesystem::path& model_path,
                     const std::vector<std::string>& joint_names,
                     const Joints& effort);
    ~UpperBodyGravity();
    UpperBodyGravity(UpperBodyGravity&&) noexcept;
    UpperBodyGravity& operator=(UpperBodyGravity&&) noexcept;
    UpperBodyGravity(const UpperBodyGravity&) = delete;
    UpperBodyGravity& operator=(const UpperBodyGravity&) = delete;
    void setEffort(const Joints& effort);
    Joints offset(const Sensors& state, const Joints& kp);
private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace g1control
