#include "control/Physics.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <set>
#include <stdexcept>

namespace g1control {
namespace {
struct ModelDeleter { void operator()(mjModel* p) const { mj_deleteModel(p); } };
struct DataDeleter { void operator()(mjData* p) const { mj_deleteData(p); } };

int named(const mjModel* model, mjtObj kind, const std::string& name) {
    const int result = mj_name2id(model, kind, name.c_str());
    if (result < 0) throw std::runtime_error("MuJoCo model is missing " + name);
    return result;
}

struct ModelState {
    std::unique_ptr<mjModel, ModelDeleter> model;
    std::unique_ptr<mjData, DataDeleter> data;
    std::array<int, 29> joint{}, qadr{}, vadr{};
    int root_qadr = -1, root_vadr = -1;

    ModelState(const std::filesystem::path& path, const std::vector<std::string>& names) {
        if (mj_version() != mjVERSION_HEADER)
            throw std::runtime_error("MuJoCo header/library versions do not match");
        if (names.size() != 29 || std::set<std::string>(names.begin(), names.end()).size() != 29)
            throw std::invalid_argument("physics requires 29 distinct joint names");
        char error[2048] = {};
        model.reset(mj_loadXML(path.c_str(), nullptr, error, sizeof(error)));
        if (!model) throw std::runtime_error("cannot load MuJoCo model " + path.string() + ": " + error);
        data.reset(mj_makeData(model.get()));
        if (!data) throw std::runtime_error("cannot allocate MuJoCo data");
        for (int i = 0; i < 29; ++i) {
            joint[i] = named(model.get(), mjOBJ_JOINT, names[i]);
            const int type = model->jnt_type[joint[i]];
            if (type != mjJNT_HINGE && type != mjJNT_SLIDE)
                throw std::invalid_argument("expected a scalar joint: " + names[i]);
            qadr[i] = model->jnt_qposadr[joint[i]];
            vadr[i] = model->jnt_dofadr[joint[i]];
        }
        int free_count = 0;
        for (int i = 0; i < model->njnt; ++i) {
            if (model->jnt_type[i] != mjJNT_FREE) continue;
            ++free_count;
            root_qadr = model->jnt_qposadr[i];
            root_vadr = model->jnt_dofadr[i];
        }
        if (free_count != 1) throw std::runtime_error("expected one MuJoCo floating base");
    }

    Eigen::Quaterniond quaternion() const {
        const auto* q = data->qpos + root_qadr + 3;
        return {q[0], q[1], q[2], q[3]};
    }
    void setQuaternion(const Eigen::Quaterniond& q) {
        auto* values = data->qpos + root_qadr + 3;
        values[0] = q.w(); values[1] = q.x(); values[2] = q.y(); values[3] = q.z();
    }
    void setJoints(const Sensors& state) {
        for (int i = 0; i < 29; ++i) {
            data->qpos[qadr[i]] = state.q[i];
            data->qvel[vadr[i]] = state.dq[i];
        }
    }
};

void requireState(const Sensors& state) {
    if (!state.q.allFinite() || !state.dq.allFinite() || !state.gyro.allFinite() ||
        !state.quat.coeffs().allFinite() || state.quat.norm() < 1e-12)
        throw std::runtime_error("non-finite state or invalid quaternion");
}

void requireEffort(const Joints& effort) {
    if (!effort.allFinite() || (effort.array() <= 0).any())
        throw std::invalid_argument("effort limits must be finite and positive");
}

using Vec3Map = Eigen::Map<Eigen::Vector3d>;
using ConstVec3Map = Eigen::Map<const Eigen::Vector3d>;
}  // namespace

struct MujocoPlant::Impl : ModelState {
    std::array<int, 29> actuator{};
    int substeps = 1, pelvis = -1;
    Joints last_torque = Joints::Zero();
    std::uint64_t saturation_count = 0;
    bool supported = false;
    Eigen::Vector3d anchor_position = Eigen::Vector3d::Zero();
    Eigen::Quaterniond anchor_quaternion = Eigen::Quaterniond::Identity();

    Impl(const std::filesystem::path& path, const std::vector<std::string>& names,
         double dt) : ModelState(path, names) {
        if (!std::isfinite(dt) || dt <= 0) throw std::invalid_argument("physics_dt must be positive");
        model->opt.timestep = dt;
        pelvis = named(model.get(), mjOBJ_BODY, "pelvis");
        for (int i = 0; i < 29; ++i) {
            int count = 0;
            for (int a = 0; a < model->nu; ++a) {
                if (model->actuator_trnid[2*a] == joint[i]) { actuator[i] = a; ++count; }
            }
            if (count != 1) throw std::runtime_error("expected one actuator for " + names[i]);
        }
    }

    void support(bool enabled) {
        if (enabled && !supported) {
            anchor_position = ConstVec3Map(data->qpos + root_qadr);
            anchor_quaternion = quaternion();
        }
        if (!enabled && supported)
            std::fill_n(data->xfrc_applied + 6*pelvis, 6, 0.);
        supported = enabled;
    }

    void applySupport() {
        if (!supported) return;
        const auto q = quaternion();
        const auto rotation = q.normalized().toRotationMatrix();
        const Eigen::Vector3d force =
            1200.*(anchor_position - ConstVec3Map(data->qpos + root_qadr))
            - 150.*ConstVec3Map(data->qvel + root_vadr)
            - model->body_subtreemass[pelvis]*ConstVec3Map(model->opt.gravity);
        auto error = anchor_quaternion * q.conjugate();
        if (error.w() < 0.) error.coeffs() *= -1.;
        const Eigen::Vector3d torque = 400.*error.vec()
            - 30.*(rotation * ConstVec3Map(data->qvel + root_vadr + 3));
        Vec3Map(data->xfrc_applied + 6*pelvis) = force;
        Vec3Map(data->xfrc_applied + 6*pelvis + 3) = torque;
    }
};

MujocoPlant::MujocoPlant(const std::filesystem::path& path,
                         const std::vector<std::string>& names, double physics_dt, double control_dt)
    : impl_(std::make_unique<Impl>(path, names, physics_dt)) { setControlDt(control_dt); }
MujocoPlant::~MujocoPlant() = default;
MujocoPlant::MujocoPlant(MujocoPlant&&) noexcept = default;
MujocoPlant& MujocoPlant::operator=(MujocoPlant&&) noexcept = default;

void MujocoPlant::setControlDt(double control_dt) {
    const double ratio = control_dt / impl_->model->opt.timestep;
    if (!std::isfinite(ratio) || ratio < 1. || ratio > std::numeric_limits<int>::max())
        throw std::invalid_argument("control_dt must be an integer multiple of physics_dt");
    const auto count = static_cast<int>(std::round(ratio));
    if (std::abs(count*impl_->model->opt.timestep-control_dt) > 1e-8 + 1e-5*std::abs(control_dt))
        throw std::invalid_argument("control_dt must be an integer multiple of physics_dt");
    impl_->substeps = count;
}

void MujocoPlant::applyVelocityPush(const Eigen::Vector3d& linear,const Eigen::Vector3d& angular) {
    if(!linear.allFinite()||!angular.allFinite())throw std::runtime_error("non-finite push");
    Vec3Map(impl_->data->qvel+impl_->root_vadr)+=impl_->quaternion().toRotationMatrix()*linear;
    Vec3Map(impl_->data->qvel+impl_->root_vadr+3)+=angular;
}
void MujocoPlant::reset(const Sensors& state) {
    requireState(state);
    if (!state.root_pos.allFinite() || !state.root_vel_b.allFinite())
        throw std::runtime_error("non-finite floating-base state");
    auto& p = *impl_;
    mj_resetData(p.model.get(), p.data.get());
    Vec3Map(p.data->qpos + p.root_qadr) = state.root_pos;
    p.setQuaternion(state.quat);
    Vec3Map(p.data->qvel + p.root_vadr) = state.quat.normalized().toRotationMatrix()*state.root_vel_b;
    Vec3Map(p.data->qvel + p.root_vadr + 3) = state.gyro;
    p.setJoints(state);
    p.last_torque.setZero();
    p.saturation_count = 0;
    p.supported = false;
    mj_forward(p.model.get(), p.data.get());
}

Sensors MujocoPlant::sensor() const {
    const auto& p = *impl_;
    Sensors state;
    state.quat = p.quaternion();
    state.root_pos = ConstVec3Map(p.data->qpos + p.root_qadr);
    state.root_vel_b = state.quat.normalized().toRotationMatrix().transpose()
        * ConstVec3Map(p.data->qvel + p.root_vadr);
    state.gyro = ConstVec3Map(p.data->qvel + p.root_vadr + 3);
    for (int i = 0; i < 29; ++i) {
        state.q[i] = p.data->qpos[p.qadr[i]];
        state.dq[i] = p.data->qvel[p.vadr[i]];
    }
    return state;
}

Sensors MujocoPlant::step(const Joints& target, const Joints& kp, const Joints& kd,
                          const Joints& effort, bool support) {
    if (!target.allFinite() || !kp.allFinite() || !kd.allFinite() ||
        (kp.array() < 0.).any() || (kd.array() < 0.).any())
        throw std::invalid_argument("non-finite target or invalid PD gains");
    requireEffort(effort);
    auto& p = *impl_;
    p.support(support);
    for (int n = 0; n < p.substeps; ++n) {
        p.applySupport();
        for (int i = 0; i < 29; ++i) {
            const double requested = kp[i]*(target[i]-p.data->qpos[p.qadr[i]])
                - kd[i]*p.data->qvel[p.vadr[i]];
            if (!std::isfinite(requested)) throw std::runtime_error("non-finite simulated motor torque");
            if (std::abs(requested) > effort[i]) ++p.saturation_count;
            p.last_torque[i] = std::clamp(requested, -effort[i], effort[i]);
            p.data->ctrl[p.actuator[i]] = p.last_torque[i];
        }
        mj_step(p.model.get(), p.data.get());
    }
    return sensor();
}
double MujocoPlant::physicsDt() const { return impl_->model->opt.timestep; }
double MujocoPlant::time() const { return impl_->data->time; }
const Joints& MujocoPlant::lastTorque() const { return impl_->last_torque; }
std::uint64_t MujocoPlant::saturationCount() const { return impl_->saturation_count; }
mjModel* MujocoPlant::model() { return impl_->model.get(); }
mjData* MujocoPlant::data() { return impl_->data.get(); }
const mjModel* MujocoPlant::model() const { return impl_->model.get(); }
const mjData* MujocoPlant::data() const { return impl_->data.get(); }

struct FlatGroundEstimator::Impl : ModelState {
    std::array<std::pair<int, std::vector<int>>, 2> feet;
    std::vector<mjtNum> jacobian;
    Impl(const std::filesystem::path& path, const std::vector<std::string>& names)
        : ModelState(path, names), jacobian(3*model->nv, 0.) {
        const std::array<std::string, 2> sides{"left", "right"};
        for (int i = 0; i < 2; ++i) {
            auto& foot = feet[i];
            foot.first = named(model.get(), mjOBJ_BODY, sides[i] + "_ankle_roll_link");
            for (int g = 0; g < model->ngeom; ++g)
                if (model->geom_bodyid[g] == foot.first && model->geom_type[g] == mjGEOM_SPHERE)
                    foot.second.push_back(g);
            if (foot.second.empty()) throw std::runtime_error(sides[i] + " foot has no spherical contact geoms");
        }
    }
};
FlatGroundEstimator::FlatGroundEstimator(const std::filesystem::path& path,
                                         const std::vector<std::string>& names)
    : impl_(std::make_unique<Impl>(path, names)) {}
FlatGroundEstimator::~FlatGroundEstimator() = default;
FlatGroundEstimator::FlatGroundEstimator(FlatGroundEstimator&&) noexcept = default;
FlatGroundEstimator& FlatGroundEstimator::operator=(FlatGroundEstimator&&) noexcept = default;

Sensors FlatGroundEstimator::estimate(const Sensors& state) {
    requireState(state);
    auto& p = *impl_;
    Sensors result = state;
    result.quat.normalize();
    std::fill_n(p.data->qpos, p.model->nq, 0.);
    std::fill_n(p.data->qvel, p.model->nv, 0.);
    p.setQuaternion(result.quat);
    p.setJoints(state);
    Vec3Map(p.data->qvel + p.root_vadr + 3) = state.gyro;
    mj_forward(p.model.get(), p.data.get());
    double lowest_z = std::numeric_limits<double>::infinity();
    int contact_body = -1;
    Eigen::Vector3d contact = Eigen::Vector3d::Zero();
    for (const auto& foot : p.feet) {
        for (int geom : foot.second) {
            const double z = p.data->geom_xpos[3*geom+2] - p.model->geom_size[3*geom];
            if (z >= lowest_z) continue;
            lowest_z = z;
            contact_body = foot.first;
            contact = ConstVec3Map(p.data->geom_xpos + 3*geom);
            contact.z() = z;
        }
    }
    mj_jac(p.model.get(), p.data.get(), p.jacobian.data(), nullptr, contact.data(), contact_body);
    Eigen::Vector3d world_velocity = Eigen::Vector3d::Zero();
    for (int axis = 0; axis < 3; ++axis)
        for (int v = 0; v < p.model->nv; ++v)
            world_velocity[axis] -= p.jacobian[axis*p.model->nv+v]*p.data->qvel[v];
    result.root_pos = {0., 0., -lowest_z};
    result.root_vel_b = result.quat.toRotationMatrix().transpose()*world_velocity;
    return result;
}

struct UpperBodyGravity::Impl : ModelState {
    std::array<bool, 29> upper{};
    Joints max_torque = Joints::Zero();
    Impl(const std::filesystem::path& path, const std::vector<std::string>& names)
        : ModelState(path, names) {
        for (int i = 0; i < 29; ++i) {
            const auto& name = names[i];
            upper[i] = name.rfind("waist_", 0) == 0 || name.find("shoulder_") != std::string::npos
                || name.find("elbow_") != std::string::npos || name.find("wrist_") != std::string::npos;
        }
    }
};
UpperBodyGravity::UpperBodyGravity(const std::filesystem::path& path,
                                   const std::vector<std::string>& names, const Joints& effort)
    : impl_(std::make_unique<Impl>(path, names)) { setEffort(effort); }
UpperBodyGravity::~UpperBodyGravity() = default;
UpperBodyGravity::UpperBodyGravity(UpperBodyGravity&&) noexcept = default;
UpperBodyGravity& UpperBodyGravity::operator=(UpperBodyGravity&&) noexcept = default;
void UpperBodyGravity::setEffort(const Joints& effort) {
    requireEffort(effort);
    impl_->max_torque = .5*effort;
}
Joints UpperBodyGravity::offset(const Sensors& state, const Joints& kp) {
    if (!state.q.allFinite() || !state.quat.coeffs().allFinite() || !kp.allFinite() ||
        std::abs(state.quat.norm()-1.) > .05 || (kp.array() <= 0.).any())
        throw std::runtime_error("invalid preparation state, quaternion, or gains");
    auto& p = *impl_;
    Vec3Map(p.data->qpos + p.root_qadr) = Eigen::Vector3d(0., 0., .78);
    p.setQuaternion(state.quat);
    for (int i = 0; i < 29; ++i) p.data->qpos[p.qadr[i]] = state.q[i];
    std::fill_n(p.data->qvel, p.model->nv, 0.);
    mj_forward(p.model.get(), p.data.get());
    Joints result = Joints::Zero();
    for (int i = 0; i < 29; ++i) {
        if (!p.upper[i]) continue;
        const double torque = std::clamp(p.data->qfrc_bias[p.vadr[i]], -p.max_torque[i], p.max_torque[i]);
        result[i] = std::clamp(torque/kp[i], -.75, .75);
    }
    return result;
}

}  // namespace g1control
