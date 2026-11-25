#include "control/Policy.h"
#include <onnxruntime_cxx_api.h>
#include <cnpy.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>

namespace g1control {
namespace {
using Floats = std::vector<float>;
using Doubles = std::vector<double>;
using JointF = Eigen::Matrix<float,29,1>;

[[noreturn]] void fail(const std::string& text) { throw std::runtime_error("motion policy: " + text); }
void require(bool ok, const std::string& text) { if (!ok) fail(text); }
Joints vector29(const YAML::Node& cfg,const char* key,bool fp32=false) {
    const auto x=cfg[key];
    require(x.IsSequence() && x.size()==29,std::string(key)+" must contain 29 values");
    Joints out;
    for(int i=0;i<29;++i) out[i]=fp32 ? double(x[i].as<float>()) : x[i].as<double>();
    require(out.allFinite(),std::string(key)+" contains non-finite values");
    return out;
}
const std::array<const char*,29> jointOrder={
    "left_hip_pitch_joint","left_hip_roll_joint","left_hip_yaw_joint","left_knee_joint",
    "left_ankle_pitch_joint","left_ankle_roll_joint","right_hip_pitch_joint","right_hip_roll_joint",
    "right_hip_yaw_joint","right_knee_joint","right_ankle_pitch_joint","right_ankle_roll_joint",
    "waist_yaw_joint","waist_roll_joint","waist_pitch_joint","left_shoulder_pitch_joint",
    "left_shoulder_roll_joint","left_shoulder_yaw_joint","left_elbow_joint","left_wrist_roll_joint",
    "left_wrist_pitch_joint","left_wrist_yaw_joint","right_shoulder_pitch_joint","right_shoulder_roll_joint",
    "right_shoulder_yaw_joint","right_elbow_joint","right_wrist_roll_joint","right_wrist_pitch_joint",
    "right_wrist_yaw_joint"};
void loadInfo(PolicyInfo& info,const YAML::Node& cfg,bool fp32) {
    const auto names=cfg["joint_names"];
    require(names.IsSequence() && names.size()==29,"joint_names must contain 29 DDS-ordered names");
    for(int j=0;j<29;++j) {
        const auto name=names[j].as<std::string>();
        require(name==jointOrder[j],"joint order differs at motor "+std::to_string(j));
        info.joint_names.push_back(name);
    }
    info.dt=cfg["control_dt"].as<double>();
    require(std::isfinite(info.dt) && std::abs(info.dt-.02)<1e-8,"expected 50 Hz control_dt");
    info.kp=vector29(cfg,"kp",fp32); info.kd=vector29(cfg,"kd",fp32);
    info.effort=vector29(cfg,fp32 ? "effort_limits" : "effort_limit",fp32);
    require((info.kp.array()>0).all() && (info.kd.array()>=0).all() && (info.effort.array()>0).all(),
            "invalid gains or effort limits");
}
struct Array {
    std::vector<size_t> shape;
    Doubles values;
    Array(const std::filesystem::path& path,const char* key,std::initializer_list<size_t> tail) {
        // Load only the members needed by the runtime, avoiding extra reference copies.
        const auto a=cnpy::npz_load(path.string(),key);
        shape=a.shape;
        require(shape.size()==tail.size()+1 && shape[0]>0,std::string(key)+" has invalid shape");
        require(std::equal(tail.begin(),tail.end(),shape.begin()+1),std::string(key)+" has invalid trailing dimensions");
        require(a.word_size==4 || a.word_size==8,std::string(key)+" must use float32 or float64");
        values.resize(a.num_vals);
        std::vector<size_t> fortran_stride(shape.size(),1);
        for(size_t d=1;d<shape.size();++d) fortran_stride[d]=fortran_stride[d-1]*shape[d-1];
        for(size_t i=0;i<values.size();++i) {
            size_t source=i;
            if(a.fortran_order) {
                source=0;size_t remaining=i;
                for(size_t d=shape.size();d-->0;) { source+=(remaining%shape[d])*fortran_stride[d];remaining/=shape[d]; }
            }
            values[i]=a.word_size==4 ? double(a.data<float>()[source]) : a.data<double>()[source];
            require(std::isfinite(values[i]),std::string(key)+" contains non-finite data");
        }
    }
    size_t frames() const { return shape[0]; }
    const double* row(size_t f) const { return values.data()+f*(values.size()/frames()); }
    void checkFrames(size_t n,const char* name) const { require(frames()==n,std::string(name)+" frame count mismatch"); }
};
Eigen::Matrix3d rotation(const Eigen::Quaterniond& q) {
    require(q.coeffs().allFinite() && q.norm()>1e-12,"invalid root quaternion");
    return q.normalized().toRotationMatrix();
}
Eigen::Matrix3d heading(const Eigen::Matrix3d& r) {
    return Eigen::AngleAxisd(std::atan2(r(1,0),r(0,0)),Eigen::Vector3d::UnitZ()).toRotationMatrix();
}
void append(Floats& dst,const Eigen::VectorXd& src) { for(Eigen::Index i=0;i<src.size();++i) dst.push_back(float(src[i])); }
void append6(Floats& dst,const Eigen::Matrix3d& r) {
    for(int i=0;i<3;++i) for(int j=0;j<2;++j) dst.push_back(float(r(i,j)));
}
void append(Floats& dst,const Floats& src) { dst.insert(dst.end(),src.begin(),src.end()); }
void checkState(const Sensors& s) {
    require(s.q.allFinite() && s.dq.allFinite() && s.gyro.allFinite(),"non-finite robot state");
    rotation(s.quat);
}

class Actor {
    Ort::Env env_{ORT_LOGGING_LEVEL_WARNING,"motion-policy"};
    Ort::SessionOptions options_;
    std::unique_ptr<Ort::Session> session_;
    std::string first_,second_,output_;
    std::vector<int64_t> first_shape_,second_shape_;
public:
    Actor(const std::filesystem::path& path,bool reference,int width) {
        options_.SetIntraOpNumThreads(1); options_.SetInterOpNumThreads(1);
        options_.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
        options_.AddConfigEntry("session.intra_op.allow_spinning","0");
        session_=std::make_unique<Ort::Session>(env_,path.c_str(),options_);
        first_=reference ? "obs_hist" : "obs";
        second_=reference ? "goal" : "time_step";
        output_=reference ? "action" : "actions";
        first_shape_=reference ? std::vector<int64_t>{1,10,99} : std::vector<int64_t>{1,width};
        second_shape_=reference ? std::vector<int64_t>{1,730} : std::vector<int64_t>{1,1};
        require(session_->GetInputCount()==2,"actor must have two inputs");
        Ort::AllocatorWithDefaultOptions allocator;
        for(size_t i=0;i<2;++i) {
            const std::string name=session_->GetInputNameAllocated(i,allocator).get();
            const auto type=session_->GetInputTypeInfo(i);
            auto tensor=type.GetTensorTypeAndShapeInfo();
            const auto expected=name==first_ ? first_shape_ : second_shape_;
            require(name==first_ || name==second_,"unexpected actor input "+name);
            require(tensor.GetElementType()==ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT,"actor input must be float32");
            auto shape=tensor.GetShape();
            // Exporters may mark only the batch dimension dynamic.
            if(!shape.empty() && shape[0]<0) shape[0]=1;
            require(shape==expected,"actor input shape mismatch: "+name);
        }
        bool found=false;
        for(size_t i=0;i<session_->GetOutputCount();++i) {
            if(std::string(session_->GetOutputNameAllocated(i,allocator).get())!=output_) continue;
            const auto type=session_->GetOutputTypeInfo(i);
            auto tensor=type.GetTensorTypeAndShapeInfo();
            auto shape=tensor.GetShape(); if(!shape.empty() && shape[0]<0) shape[0]=1;
            require(tensor.GetElementType()==ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT && shape==std::vector<int64_t>({1,29}),
                    "actor action must be float32[1,29]");
            found=true;
        }
        require(found,"actor action output missing");
    }
    JointF run(Floats& obs,Floats& aux) {
        const auto mem=Ort::MemoryInfo::CreateCpu(OrtArenaAllocator,OrtMemTypeDefault);
        std::array<Ort::Value,2> inputs={
            Ort::Value::CreateTensor<float>(mem,obs.data(),obs.size(),first_shape_.data(),first_shape_.size()),
            Ort::Value::CreateTensor<float>(mem,aux.data(),aux.size(),second_shape_.data(),second_shape_.size())};
        const char* names[]={first_.c_str(),second_.c_str()}; const char* outputs[]={output_.c_str()};
        auto out=session_->Run(Ort::RunOptions{nullptr},names,inputs.data(),2,outputs,1);
        require(out[0].IsTensor() && out[0].GetTensorTypeAndShapeInfo().GetElementCount()==29,"invalid actor action");
        JointF action=Eigen::Map<const JointF>(out[0].GetTensorData<float>());
        require(action.allFinite(),"non-finite actor action");
        return action;
    }
};

class Hybrid final : public Policy {
    PolicyInfo info_;
    YAML::Node cfg_;
    Array q_,dq_,pos_,quat_;
    JointF q0_,scale_,last_=JointF::Zero();
    std::unique_ptr<Actor> actor_;
    std::array<Floats,5> history_;
    Eigen::Quaterniond alignment_{1,0,0,0};
    size_t frame_=0,history_length_=0;
    bool begun_=false,ufo_=false;
    std::optional<size_t> hold_frame_;
    std::array<Floats,5> terms(const Sensors& s) const {
        std::array<Floats,5> t;
        append(t[0],Eigen::Vector3d(-rotation(s.quat).row(2).transpose())); append(t[1],s.gyro);
        for(int j=0;j<29;++j) t[2].push_back(float(s.q[j])-q0_[j]);
        append(t[3],s.dq); for(int j=0;j<29;++j) t[4].push_back(last_[j]);
        return t;
    }
    Eigen::Quaterniond refQuat(size_t f) const {
        const auto p=quat_.row(std::min(f,info_.frames-1)); return {p[0],p[1],p[2],p[3]};
    }
public:
    Hybrid(const YAML::Node& spec,const std::filesystem::path& root)
      : cfg_(YAML::LoadFile((root/spec["bundle"].as<std::string>()/"contract.json").string())),
        q_(root/spec["bundle"].as<std::string>()/"reference.npz","joint_pos",{29}),
        dq_(root/spec["bundle"].as<std::string>()/"reference.npz","joint_vel",{29}),
        pos_(root/spec["bundle"].as<std::string>()/"reference.npz","root_pos_w",{3}),
        quat_(root/spec["bundle"].as<std::string>()/"reference.npz","root_quat_w",{4}) {
        info_.name=spec["name"].as<std::string>(); info_.kind=spec["adapter"].as<std::string>();
        loadInfo(info_,cfg_,true); info_.frames=q_.frames();
        const auto names=cnpy::npz_load((root/spec["bundle"].as<std::string>()/"reference.npz").string(),"joint_names");
        require(names.shape==std::vector<size_t>{29} && names.word_size%4==0,"invalid reference joint_names");
        for(int j=0;j<29;++j) {
            std::string decoded;
            const auto chars=names.data<uint32_t>()+j*(names.word_size/4);
            for(size_t k=0;k<names.word_size/4 && chars[k]!=0;++k) {
                require(chars[k]<128,"reference joint_names must be ASCII");decoded.push_back(char(chars[k]));
            }
            require(decoded==info_.joint_names[j],"contract and reference joint orders differ");
        }
        dq_.checkFrames(info_.frames,"joint_vel");pos_.checkFrames(info_.frames,"root_pos_w");quat_.checkFrames(info_.frames,"root_quat_w");
        q0_=vector29(cfg_,"default_joint_pos",true).cast<float>(); scale_=vector29(cfg_,"action_scale",true).cast<float>();
        vector29(cfg_,"position_lower",true); vector29(cfg_,"position_upper",true);
        info_.entry_q=Eigen::Map<const Joints>(q_.row(0));
        const auto kind=cfg_["policy_kind"].as<std::string>();
        require(kind=="official_rep_state" || kind=="robust_proprio","unsupported hybrid kind "+kind);
        info_.needs_estimator=kind=="official_rep_state"; history_length_=cfg_["observation_history"].as<size_t>();
        require(history_length_==(info_.needs_estimator ? 1 : 4),"unexpected observation history");
        const int width=cfg_["observation_width"].as<int>();
        require(width==(info_.needs_estimator ? 708 : 1012),"unexpected observation width");
        info_.policy_kind=kind;info_.robust_proprio=kind=="robust_proprio";info_.observation_width=width;
        info_.target_processing="original hybrid mapping; no clipping or filtering";
        ufo_=info_.kind=="ufo_jointtrack";
        if(ufo_) {
            require(!info_.needs_estimator && cfg_["reset_action_history"].as<std::string>()=="measured_pose_hold",
                    "UFO requires measured-pose reset history");
            require((scale_.array()>0).all(),"UFO action scales must be positive");
            info_.target_processing="FP32 SuperMimic actor; measured-pose reset history; original action mapping";
        }
        info_.hold_final=spec["hold_final"] ? spec["hold_final"].as<bool>() :
            (ufo_ && (info_.name.find("entry_star")!=std::string::npos || info_.name.find("home_star")!=std::string::npos || info_.name.find("guard_stand")!=std::string::npos));
        for(size_t f=0;f<info_.frames;++f) rotation(refQuat(f));
        actor_=std::make_unique<Actor>(root/spec["bundle"].as<std::string>()/"policy_motion.onnx",false,width);
    }
    const PolicyInfo& info() const override { return info_; }
    size_t frame() const override { return frame_; }
    Joints referenceJoints(size_t f) const override { return Eigen::Map<const Joints>(q_.row(std::min(f,info_.frames-1))); }
    Eigen::Quaterniond referenceQuaternion(size_t f) const override { return refQuat(f); }
    double referenceHeight(size_t f) const override { return pos_.row(std::min(f,info_.frames-1))[2]; }
    Eigen::Vector3d referenceGravity(size_t f) const override { return -rotation(refQuat(f)).row(2).transpose(); }
    void begin(const Sensors& s) override {
        checkState(s); frame_=0; hold_frame_.reset(); last_.setZero();
        const auto actual=rotation(s.quat),ref=rotation(refQuat(0));
        const double raw=std::atan2(actual(1,0),actual(0,0))-std::atan2(ref(1,0),ref(0,0));
        alignment_=Eigen::AngleAxisd(std::atan2(std::sin(raw),std::cos(raw)),Eigen::Vector3d::UnitZ());
        if(ufo_) for(int j=0;j<29;++j) last_[j]=(float(s.q[j])-q0_[j])/scale_[j];
        require(last_.allFinite(),"non-finite measured-pose holding action");
        const auto t=terms(s);
        for(size_t i=0;i<5;++i) { history_[i].clear(); for(size_t h=0;h<history_length_;++h) append(history_[i],t[i]); }
        begun_=true;
    }
    void seek(size_t f) override { require(f<info_.frames,"reference frame out of range");frame_=f; }
    void holdReferenceFrame(std::optional<size_t> f) override {
        require(!f || *f<info_.frames,"hold reference frame out of range");hold_frame_=f;
    }
    void holdFinal() override { require(ufo_,"holdFinal is only supported by UFO jointtrack");seek(info_.frames-1);holdReferenceFrame(frame_); }
    void feedbackTarget(const Joints& q) override {
        require(q.allFinite(),"non-finite feedback target");
        for(int j=0;j<29;++j) { require(scale_[j]!=0,"zero feedback action scale");last_[j]=float((q[j]-double(q0_[j]))/double(scale_[j])); }
        require(last_.allFinite(),"non-finite feedback action");
    }
    void resetActionHistory() override { last_.setZero(); }
    void advance() override { frame_=std::min(frame_+1,info_.frames-1); }
    PolicyOutput infer(const Sensors& s) override {
        require(begun_,"begin must precede infer"); checkState(s);
        PolicyOutput out;out.frame=frame_;out.goal.reserve(640);
        for(size_t i=0;i<10;++i) {
            const size_t f=hold_frame_ ? *hold_frame_ : std::min(frame_+5*i,info_.frames-1);
            for(int j=0;j<29;++j) out.goal.push_back(float(q_.row(f)[j]));
            for(int j=0;j<29;++j) out.goal.push_back(hold_frame_ ? 0.f : float(dq_.row(f)[j]));
        }
        for(size_t i=0;i<10;++i) {
            const size_t f=hold_frame_ ? *hold_frame_ : std::min(frame_+5*i,info_.frames-1);
            append6(out.goal,rotation(s.quat.conjugate()*alignment_*refQuat(f)));
        }
        if(info_.needs_estimator) {
            require(s.root_pos.allFinite() && s.root_vel_b.allFinite(),"official actor requires finite root estimate");
            out.observation.push_back(float(s.root_pos.z()));
            append(out.observation,Eigen::Vector3d(-rotation(s.quat).row(2).transpose()));
            append(out.observation,s.root_vel_b);append(out.observation,s.gyro);append(out.observation,s.q);append(out.observation,s.dq);
        } else {
            const auto t=terms(s);
            for(size_t i=0;i<5;++i) {
                auto& h=history_[i];std::move(h.begin()+t[i].size(),h.end(),h.begin());
                std::copy(t[i].begin(),t[i].end(),h.end()-t[i].size());append(out.observation,h);
            }
        }
        append(out.observation,out.goal);
        Floats time={float(frame_)};last_=actor_->run(out.observation,time);
        out.action=last_.cast<double>();
        // Deliberately preserve NumPy float32 multiply then add (no target clamps).
        for(int j=0;j<29;++j) { const float scaled=scale_[j]*last_[j];out.q[j]=double(float(q0_[j]+scaled)); }
        require(out.q.allFinite(),"non-finite joint target");return out;
    }
};

class Reference final : public Policy {
    PolicyInfo info_;
    YAML::Node cfg_;
    Array goal_,q_,pos_,r_,gravity_;
    Joints q0_,scale_,velocity_,last_=Joints::Zero();
    std::unique_ptr<Actor> actor_;
    Floats history_;
    Eigen::Matrix3d alignment_=Eigen::Matrix3d::Identity();
    bool begun_=false,aligned_=false;
    size_t frame_=0;
    double max_vel_=.8,max_action_=50,torque_factor_=1.5;
    Eigen::Matrix3d refRotation(size_t f) const {
        return Eigen::Map<const Eigen::Matrix<double,3,3,Eigen::RowMajor>>(r_.row(std::min(f,info_.frames-1)));
    }
public:
    Reference(const YAML::Node& spec,const std::filesystem::path& root)
      : cfg_(YAML::LoadFile((root/spec["bundle"].as<std::string>()/"deploy_config.json").string())),
        goal_(root/spec["clip"].as<std::string>(),"goal",{730}),
        q_(root/spec["clip"].as<std::string>(),"joint_pos",{29}),
        pos_(root/spec["clip"].as<std::string>(),"root_pos",{3}),
        r_(root/spec["clip"].as<std::string>(),"root_R",{3,3}),
        gravity_(root/spec["clip"].as<std::string>(),"proj_grav",{3}) {
        info_.name=spec["name"].as<std::string>();info_.kind="reference";loadInfo(info_,cfg_,false);
        info_.frames=goal_.frames();q_.checkFrames(info_.frames,"joint_pos");pos_.checkFrames(info_.frames,"root_pos");
        r_.checkFrames(info_.frames,"root_R");gravity_.checkFrames(info_.frames,"proj_grav");
        info_.entry_q=Eigen::Map<const Joints>(q_.row(0));
        q0_=vector29(cfg_,"default_joint_pos");scale_=vector29(cfg_,"action_scale");velocity_=vector29(cfg_,"velocity_limit");
        vector29(cfg_,"joint_lower");vector29(cfg_,"joint_upper");
        require((velocity_.array()>0).all(),"velocity limits must be positive");
        const auto stage=spec["stage"] ? spec["stage"].as<std::string>() : "ground";
        if(stage=="air" || stage=="feet") { max_vel_=.3;max_action_=10;torque_factor_=.5; }
        else if(stage=="slack") { max_vel_=.5;max_action_=15;torque_factor_=1; }
        else require(stage=="ground","unknown reference stage "+stage);
        info_.observation_width=1720;info_.policy_kind="vibed_obs_hist";
        info_.target_processing="original DeployCore SafetyCfg/targets, stage="+stage;
        actor_=std::make_unique<Actor>(root/spec["bundle"].as<std::string>()/"policy.onnx",true,990);
    }
    const PolicyInfo& info() const override { return info_; }
    size_t frame() const override { return frame_; }
    void seek(size_t f) override { require(f<info_.frames,"reference frame out of range");frame_=f; }
    Joints referenceJoints(size_t f) const override { return Eigen::Map<const Joints>(q_.row(std::min(f,info_.frames-1))); }
    Eigen::Quaterniond referenceQuaternion(size_t f) const override { return Eigen::Quaterniond(refRotation(f)).normalized(); }
    void resetActionHistory() override { last_.setZero(); }
    double referenceHeight(size_t f) const override { return pos_.row(std::min(f,info_.frames-1))[2]; }
    Eigen::Vector3d referenceGravity(size_t f) const override { return Eigen::Map<const Eigen::Vector3d>(gravity_.row(std::min(f,info_.frames-1))); }
    void begin(const Sensors&) override { frame_=0;last_.setZero();history_.clear();aligned_=false;begun_=true; }
    void advance() override { frame_=std::min(frame_+1,info_.frames-1); }
    PolicyOutput infer(const Sensors& s) override {
        require(begun_,"begin must precede infer");
        require(s.quat.coeffs().allFinite() && s.gyro.allFinite(),"reference guard: non-finite IMU");
        require(std::abs(s.quat.norm()-1)<=.05,"reference guard: bad IMU quaternion norm");
        const auto actual=rotation(s.quat);const Eigen::Vector3d g=-actual.row(2).transpose(),refg=referenceGravity(frame_);
        const double cosang=std::clamp(g.dot(refg)/(g.norm()*refg.norm()+1e-9),-1.,1.);
        require(std::acos(cosang)<=.35,"reference guard: tilt error exceeds 0.35 rad");
        require((s.dq.cwiseAbs().array()<=max_vel_*velocity_.array()).all(),"reference guard: joint velocity limit");
        require(s.q.allFinite() && s.dq.allFinite(),"reference guard: non-finite joint state");
        const auto ref=refRotation(frame_);
        if(!aligned_) { alignment_=heading(actual)*heading(ref).transpose();aligned_=true; }
        Floats obs;obs.reserve(99);
        for(int j=0;j<29;++j) obs.push_back(float(s.q[j])-float(q0_[j]));
        append(obs,s.dq);append(obs,s.gyro);append(obs,g);append(obs,last_);append6(obs,actual.transpose()*alignment_*ref);
        if(history_.empty()) for(int h=0;h<10;++h) append(history_,obs);
        else { std::move(history_.begin()+99,history_.end(),history_.begin());std::copy(obs.begin(),obs.end(),history_.end()-99); }
        PolicyOutput out;out.frame=frame_;out.observation=history_;out.goal.reserve(730);
        for(int i=0;i<730;++i) out.goal.push_back(float(goal_.row(frame_)[i]));
        last_=actor_->run(out.observation,out.goal).cast<double>();
        require(last_.cwiseAbs().maxCoeff()<=max_action_,"reference guard: action magnitude");out.action=last_;
        // Preserve DeployCore torque target clamp; soft position limits are OFF by default.
        for(int j=0;j<29;++j) {
            const double tau=torque_factor_*info_.effort[j];
            const double low=s.q[j]+(-tau+info_.kd[j]*s.dq[j])/info_.kp[j];
            const double high=s.q[j]+(tau+info_.kd[j]*s.dq[j])/info_.kp[j];
            out.q[j]=std::clamp(q0_[j]+scale_[j]*last_[j],low,high);
        }
        return out;
    }
};
} // namespace

std::unique_ptr<Policy> makeMotionPolicy(const YAML::Node& spec,const std::filesystem::path& root) {
    const auto adapter=spec["adapter"].as<std::string>();
    if(adapter=="hybrid" || adapter=="ufo_jointtrack") return std::make_unique<Hybrid>(spec,root);
    if(adapter=="reference") return std::make_unique<Reference>(spec,root);
    fail("unsupported adapter "+adapter);
}
} // namespace g1control
