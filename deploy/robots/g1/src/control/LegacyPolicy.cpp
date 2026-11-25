// C++ adapters for the original Unitree G1 Velocity and Mimic policies.
// Observation/action semantics follow deploy/include/isaaclab and State_Mimic.cpp.
#include "control/Policy.h"
#include <onnxruntime_cxx_api.h>
#include <cnpy.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <deque>
#include <map>
#include <numeric>
#include <stdexcept>
#include <utility>

namespace g1control {
namespace {
using Values = std::vector<float>;
using FJoints = Eigen::Matrix<float,29,1>;
constexpr float motion_dt = 1.0f / 50.0f;
constexpr double pi = 3.14159265358979323846;
const std::vector<std::string> joint_names = {
    "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint", "left_knee_joint",
    "left_ankle_pitch_joint", "left_ankle_roll_joint", "right_hip_pitch_joint", "right_hip_roll_joint",
    "right_hip_yaw_joint", "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
    "waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint", "left_shoulder_pitch_joint",
    "left_shoulder_roll_joint", "left_shoulder_yaw_joint", "left_elbow_joint", "left_wrist_roll_joint",
    "left_wrist_pitch_joint", "left_wrist_yaw_joint", "right_shoulder_pitch_joint", "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint", "right_elbow_joint", "right_wrist_roll_joint", "right_wrist_pitch_joint", "right_wrist_yaw_joint"
};
std::filesystem::path absolute(const std::filesystem::path& root, const std::filesystem::path& path) {
    return path.is_absolute() ? path : root / path;
}
Values floats(const YAML::Node& n) { return n && !n.IsNull() ? n.as<Values>() : Values{}; }
void require(bool condition, const std::string& message) {
    if (!condition) throw std::runtime_error("Legacy policy: " + message);
}
void sizeIs(const Values& values, size_t count, const std::string& name, bool optional=false) {
    require((optional && values.empty()) || values.size()==count, name+" has incorrect length");
    for(float v:values) require(std::isfinite(v), name+" contains a non-finite value");
}
Eigen::Quaternionf torso(Eigen::Quaternionf root, const FJoints& q) {
    return root * Eigen::AngleAxisf(q[12], Eigen::Vector3f::UnitZ())
                * Eigen::AngleAxisf(q[13], Eigen::Vector3f::UnitX())
                * Eigen::AngleAxisf(q[14], Eigen::Vector3f::UnitY());
}
Eigen::Quaternionf yaw(const Eigen::Quaternionf& q) {
    float a=std::atan2(2.f*(q.w()*q.z()+q.x()*q.y()),1.f-2.f*(q.y()*q.y()+q.z()*q.z()));
    return Eigen::Quaternionf(std::cos(a*.5f),0,0,std::sin(a*.5f)).normalized();
}

struct Motion {
    std::vector<Eigen::Vector3f> position;
    std::vector<Eigen::Quaternionf> quat;
    std::vector<FJoints> q, dq;
    explicit Motion(const std::filesystem::path& path) {
        auto arrays=cnpy::npz_load(path.string());
        auto get=[&](const std::string& name)->const cnpy::NpyArray& {
            auto i=arrays.find(name); require(i!=arrays.end(),"motion missing "+name); return i->second;
        };
        const auto& bp=get("body_pos_w"); const auto& bq=get("body_quat_w");
        const auto& jq=get("joint_pos"); const auto& jv=get("joint_vel");
        require(bp.shape.size()==3 && bp.shape[0]>0 && bp.shape[1]>0 && bp.shape[2]==3,"invalid body_pos_w shape");
        const size_t n=bp.shape[0];
        require(bq.shape.size()==3 && bq.shape[0]==n && bq.shape[1]>0 && bq.shape[2]==4,"invalid body_quat_w shape");
        require(jq.shape==std::vector<size_t>({n,29}) && jv.shape==jq.shape,"motion requires 29-joint q/dq arrays");
        for(auto a:{&bp,&bq,&jq,&jv}) require(a->word_size==sizeof(float) && !a->fortran_order,"motion arrays must be C-order float32");
        // The original MotionLoader is fixed at 50 Hz. Reject a different source rate.
        auto fps=arrays.find("fps");
        if(fps!=arrays.end()) {
            require(fps->second.num_vals==1,"invalid motion fps");
            const auto& f=fps->second;
            require(f.word_size==4 || f.word_size==8,"invalid motion fps dtype");
            double rate=f.word_size==8 ? f.data<double>()[0] : f.data<float>()[0];
            require(std::abs(rate-50.)<1e-4,"original Mimic motion must be 50 Hz");
        }
        for(size_t i=0;i<n;++i) {
            position.emplace_back(Eigen::Map<const Eigen::Vector3f>(bp.data<float>()+i*bp.shape[1]*3));
            const float* a=bq.data<float>()+i*bq.shape[1]*4;
            quat.emplace_back(a[0],a[1],a[2],a[3]);
            q.emplace_back(Eigen::Map<const FJoints>(jq.data<float>()+29*i));
            dq.emplace_back(Eigen::Map<const FJoints>(jv.data<float>()+29*i));
            require(position.back().allFinite() && quat.back().coeffs().allFinite() && q.back().allFinite() && dq.back().allFinite(),"non-finite motion data");
            require(std::abs(quat.back().norm()-1.f)<.01f,"motion root quaternion is not normalized");
        }
    }
    size_t index(float time) const {
        float phase=std::clamp(time,0.f,static_cast<float>(q.size())*motion_dt);
        return std::min(static_cast<size_t>(std::floor(phase/motion_dt)),q.size()-1);
    }
};
struct Term {
    std::string name;
    YAML::Node params;
    Values scale, clip;
    size_t history=1;
    bool scale_first=false;
    std::deque<Values> values;
    void add(Values x) {
        sizeIs(scale,x.size(),name+" scale",true);
        for(size_t i=0;i<x.size();++i) {
            if(scale_first && !scale.empty()) x[i]*=scale[i];
            if(!clip.empty()) x[i]=std::clamp(x[i],clip[0],clip[1]);
            if(!scale_first && !scale.empty()) x[i]*=scale[i];
        }
        values.push_back(std::move(x));
        if(values.size()>history) values.pop_front();
    }
};
struct Group { std::vector<Term> terms; bool gym_history=false; };
struct ActionTerm { size_t count=0; Values scale,offset; std::vector<Values> clip; };

class LegacyPolicy final: public Policy {
public:
    LegacyPolicy(const YAML::Node& spec, const std::filesystem::path& root)
        : ort_env_(ORT_LOGGING_LEVEL_WARNING,"g1_legacy") {
        info_.kind=spec["adapter"].as<std::string>();
        require(info_.kind=="velocity" || info_.kind=="balance" || info_.kind=="mimic","unknown adapter "+info_.kind);
        info_.name=spec["name"].as<std::string>(info_.kind);
        info_.continuous=info_.kind!="mimic";
        info_.policy_kind=info_.kind;
        info_.target_processing="original Unitree joint-position action mapping";
        info_.joint_names=joint_names;
        const auto bundle=absolute(root,spec["bundle"].as<std::string>());
        cfg_=YAML::LoadFile((bundle/"params/deploy.yaml").string());
        dt_=cfg_["step_dt"].as<float>();
        require(std::isfinite(dt_) && dt_>0,"invalid step_dt"); info_.dt=dt_;
        mapping_=cfg_["joint_ids_map"].as<std::vector<int>>();
        require(mapping_.size()==29,"G1 requires 29 mapped joints");
        std::array<bool,29> seen{};
        for(int id:mapping_) { require(id>=0 && id<29 && !seen[id],"joint_ids_map must be a permutation of 0..28"); seen[id]=true; }
        default_q_=floats(cfg_["default_joint_pos"]); sizeIs(default_q_,29,"default_joint_pos");
        auto kp=floats(cfg_["stiffness"]),kd=floats(cfg_["damping"]);
        sizeIs(kp,29,"stiffness"); sizeIs(kd,29,"damping");
        for(size_t i=0;i<29;++i) {
            // Original stiffness/damping are already in SDK order; defaults are policy order.
            require(kp[i]>=0 && kd[i]>=0,"negative PD gains");
            info_.kp[i]=kp[i]; info_.kd[i]=kd[i]; info_.entry_q[mapping_[i]]=default_q_[i];
        }
        info_.effort << 88,139,88,139,50,50,88,139,88,139,50,50,88,50,50,25,25,25,25,25,5,5,25,25,25,25,25,5,5;
        if(info_.kind=="mimic") {
            auto file=spec["motion_file"] ? absolute(root,spec["motion_file"].as<std::string>()) : bundle/"params/dance1_subject2.npz";
            motion_=std::make_unique<Motion>(file);
            const float duration=motion_->q.size()*motion_dt;
            start_=std::clamp(spec["time_start"].as<float>(0.f),0.f,duration);
            end_=std::clamp(spec["time_end"].as<float>(duration),0.f,duration);
            require(std::isfinite(start_) && std::isfinite(end_) && end_>start_,"empty/invalid motion time range");
            info_.frames=static_cast<size_t>(std::ceil((end_-start_)/dt_-1e-5f));
            require(info_.frames>0,"empty motion window");
            info_.entry_q=motion_->q[motion_->index(start_)].cast<double>();
        }
        loadActions(); loadObservations();
        options_.SetIntraOpNumThreads(1); options_.SetInterOpNumThreads(1);
        options_.SetGraphOptimizationLevel(ORT_ENABLE_EXTENDED);
        session_=std::make_unique<Ort::Session>(ort_env_,(bundle/"exported/policy.onnx").c_str(),options_);
        Ort::AllocatorWithDefaultOptions allocator;
        for(size_t i=0;i<session_->GetInputCount();++i) {
            auto name=session_->GetInputNameAllocated(i,allocator);
            input_names_.emplace_back(name.get());
            auto input_type=session_->GetInputTypeInfo(i);
            auto tensor=input_type.GetTensorTypeAndShapeInfo();
            require(tensor.GetElementType()==ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT,"only float32 ONNX inputs are supported");
            auto shape=tensor.GetShape();
            require(shape.size()==2 && (shape[0]==1 || shape[0]==-1) && shape[1]>0,"expected ONNX input [1, observations]");
            shape[0]=1; input_shapes_.push_back(shape);
            info_.observation_width+=static_cast<size_t>(shape[1]);
            require(groups_.count(input_names_.back())!=0,"ONNX input has no observation group: "+input_names_.back());
        }
        require(input_names_.size()==groups_.size(),"ONNX input/observation group count mismatch");
        auto output=session_->GetOutputNameAllocated(0,allocator); output_name_=output.get();
        auto output_type=session_->GetOutputTypeInfo(0);
        auto output_info=output_type.GetTensorTypeAndShapeInfo();
        const auto output_shape=output_info.GetShape();
        require(output_info.GetElementType()==ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT && output_shape==std::vector<int64_t>({1,29}),"expected 29 float32 ONNX actions");
    }
    const PolicyInfo& info() const override { return info_; }
    void begin(const Sensors& state) override {
        validateSensors(state); frame_=0; phase_=0; last_action_.assign(29,0.f); initialized_=true;
        if(motion_) {
            const auto robot_yaw=yaw(torso(state.quat.cast<float>(),state.q.cast<float>())).toRotationMatrix();
            const auto ref_yaw=yaw(motion_->quat[motion_->index(start_)]).toRotationMatrix();
            init_quat_=robot_yaw*ref_yaw.transpose();
        }
        // Reset executes each observation once, including advancing gait phase.
        // The first infer therefore uses phase = 2*dt/period, exactly as the old manager.
        for(auto& pair:groups_) for(auto& term:pair.second.terms) {
            term.values.clear(); auto value=observe(term,state);
            for(size_t i=0;i<term.history;++i) term.add(value);
        }
        validateInputSizes();
    }
    PolicyOutput infer(const Sensors& state) override {
        require(initialized_,"begin must be called before infer"); validateSensors(state);
        std::map<std::string,Values> observations;
        for(auto& pair:groups_) {
            auto& group=pair.second;
            for(auto& term:group.terms) term.add(observe(term,state));
            auto& values=observations[pair.first];
            if(group.gym_history) {
                for(size_t h=0;h<group.terms.front().history;++h)
                    for(const auto& term:group.terms) values.insert(values.end(),term.values[h].begin(),term.values[h].end());
            } else {
                for(const auto& term:group.terms)
                    for(const auto& value:term.values) values.insert(values.end(),value.begin(),value.end());
            }
        }
        auto memory=Ort::MemoryInfo::CreateCpu(OrtArenaAllocator,OrtMemTypeDefault);
        std::vector<Ort::Value> tensors; std::vector<const char*> names;
        PolicyOutput result; result.frame=frame_;
        for(size_t i=0;i<input_names_.size();++i) {
            auto& values=observations.at(input_names_[i]); names.push_back(input_names_[i].c_str());
            require(values.size()==static_cast<size_t>(input_shapes_[i][1]),"observation size changed");
            for(float x:values) require(std::isfinite(x),"non-finite observation");
            tensors.push_back(Ort::Value::CreateTensor<float>(memory,values.data(),values.size(),input_shapes_[i].data(),2));
            result.observation.insert(result.observation.end(),values.begin(),values.end());
        }
        const char* output=output_name_.c_str();
        auto outputs=session_->Run(Ort::RunOptions{nullptr},names.data(),tensors.data(),tensors.size(),&output,1);
        const float* raw=outputs.front().GetTensorData<float>();
        require(outputs.front().GetTensorTypeAndShapeInfo().GetElementCount()==29,"unexpected action output count");
        last_action_.assign(raw,raw+29);
        size_t index=0;
        for(const auto& term:actions_) for(size_t j=0;j<term.count;++j,++index) {
            require(std::isfinite(raw[index]),"non-finite policy action");
            float target=raw[index];
            if(!term.scale.empty()) target*=term.scale[j];
            if(!term.offset.empty()) target+=term.offset[j];
            if(!term.clip.empty()) target=std::clamp(target,term.clip[j][0],term.clip[j][1]);
            require(std::isfinite(target),"non-finite target");
            result.q[mapping_[index]]=target; result.action[mapping_[index]]=raw[index];
        }
        if(motion_) {
            const auto i=motionFrame(frame_);
            result.goal.assign(motion_->q[i].data(),motion_->q[i].data()+29);
            result.goal.insert(result.goal.end(),motion_->dq[i].data(),motion_->dq[i].data()+29);
        }
        return result;
    }
    void advance() override { if(info_.continuous || frame_<info_.frames) ++frame_; }
    size_t frame() const override { return frame_; }
    void seek(size_t f) override {
        require(info_.continuous || f<info_.frames,"reference frame out of range");frame_=f;
    }
    void resetActionHistory() override { last_action_.assign(29,0.f); }
    void feedbackTarget(const Joints& q) override {
        require(q.allFinite(),"non-finite feedback target");size_t index=0;
        for(const auto& term:actions_) for(size_t j=0;j<term.count;++j,++index) {
            const double scale=term.scale.empty() ? 1. : double(term.scale[j]);
            const double offset=term.offset.empty() ? 0. : double(term.offset[j]);
            require(scale!=0,"zero feedback action scale");
            last_action_[index]=static_cast<float>((q[mapping_[index]]-offset)/scale);
            require(std::isfinite(last_action_[index]),"non-finite feedback action");
        }
    }
    Joints referenceJoints(size_t f) const override { return motion_ ? motion_->q[motionFrame(f)].cast<double>() : info_.entry_q; }
    Eigen::Quaterniond referenceQuaternion(size_t f) const override { return motion_ ? motion_->quat[motionFrame(f)].cast<double>() : Eigen::Quaterniond::Identity(); }
    double referenceHeight(size_t f) const override { return motion_ ? motion_->position[motionFrame(f)].z() : .78; }
    Eigen::Vector3d referenceGravity(size_t f) const override {
        return motion_ ? (motion_->quat[motionFrame(f)].conjugate()*Eigen::Vector3f(0,0,-1)).cast<double>() : Eigen::Vector3d(0,0,-1);
    }
private:
    void validateSensors(const Sensors& s) const {
        require(s.q.allFinite() && s.dq.allFinite() && s.gyro.allFinite() && s.quat.coeffs().allFinite() && s.velocity_command.allFinite(),"invalid sensor input");
        require(std::abs(s.quat.norm()-1.)<.01,"sensor quaternion is not normalized");
    }
    size_t motionFrame(size_t f) const {
        const size_t limited=std::min(f,info_.frames-1);
        // Keep the original float time / floor indexing, including fractional start times.
        return motion_->index(static_cast<float>(limited)*dt_+start_);
    }
    void loadActions() {
        auto cfg=cfg_["actions"]; require(cfg && cfg.IsMap() && cfg.size()>0,"missing actions");
        size_t total=0;
        for(auto node:cfg) {
            require(node.first.as<std::string>()=="JointPositionAction","only joint position actions are supported");
            auto n=node.second; ActionTerm a;
            a.count=(!n["joint_ids"] || n["joint_ids"].IsNull()) ? 29 : n["joint_ids"].size();
            // This matches original concatenation; original joint_ids only sets action_dim.
            a.scale=floats(n["scale"]); a.offset=floats(n["offset"]);
            sizeIs(a.scale,a.count,"action scale",true); sizeIs(a.offset,a.count,"action offset",true);
            if(n["clip"] && !n["clip"].IsNull()) {
                a.clip=n["clip"].as<std::vector<Values>>(); require(a.clip.size()==a.count,"invalid action clip count");
                for(const auto& c:a.clip) { sizeIs(c,2,"action clip"); require(c[0]<=c[1],"reversed action clip"); }
            }
            total+=a.count; actions_.push_back(std::move(a));
        }
        require(total==29,"action dimensions must total 29");
    }
    void loadObservations() {
        auto obs=cfg_["observations"]; require(obs && obs.IsMap() && obs.size()>0,"missing observations");
        bool flat=false;
        for(auto item:obs) {
            const auto key=item.first.as<std::string>();
            if(key=="scale_first" || key=="use_gym_history") continue;
            flat=item.second["params"].IsDefined(); break;
        }
        if(flat) loadGroup("obs",obs);
        else for(auto group:obs) loadGroup(group.first.as<std::string>(),group.second);
    }
    void loadGroup(const std::string& name,const YAML::Node& cfg) {
        Group group; bool scale_first=false;
        for(auto item:cfg) {
            const auto key=item.first.as<std::string>(); const auto node=item.second;
            if(key=="scale_first") { scale_first=node.as<bool>(); continue; }
            if(key=="use_gym_history") { group.gym_history=node.as<bool>(); continue; }
            Term t; t.name=key; t.params=node["params"]; t.scale_first=scale_first;
            t.scale=floats(node["scale"]); t.clip=floats(node["clip"]);
            const int history=node["history_length"].as<int>(1); require(history>0,"history_length must be positive"); t.history=history;
            if(!t.clip.empty()) { sizeIs(t.clip,2,key+" clip"); require(t.clip[0]<=t.clip[1],"reversed observation clip"); }
            group.terms.push_back(std::move(t));
        }
        require(!group.terms.empty(),"empty observation group");
        if(group.gym_history) for(const auto& term:group.terms) require(term.history==group.terms.front().history,"gym history requires equal term histories");
        groups_.emplace(name,std::move(group));
    }
    void validateInputSizes() const {
        for(size_t i=0;i<input_names_.size();++i) {
            size_t count=0;
            for(const auto& t:groups_.at(input_names_[i]).terms) count+=t.history*t.values.front().size();
            require(count==static_cast<size_t>(input_shapes_[i][1]),"ONNX input size disagrees with observation configuration: "+input_names_[i]);
        }
    }
    Values command(const Sensors& s) const {
        Values result(3,0.f);
        if(info_.kind=="balance") return result;
        auto ranges=cfg_["commands"]["base_velocity"]["ranges"];
        const std::array<std::string,3> keys={"lin_vel_x","lin_vel_y","ang_vel_z"};
        for(size_t i=0;i<3;++i) {
            auto range=floats(ranges[keys[i]]); sizeIs(range,2,"command "+keys[i]); require(range[0]<=range[1],"reversed command range");
            result[i]=std::clamp(static_cast<float>(s.velocity_command[i]),range[0],range[1]);
        }
        return result;
    }
    Values observe(const Term& term,const Sensors& s) {
        const auto& name=term.name;
        if(name=="base_ang_vel") return {static_cast<float>(s.gyro.x()),static_cast<float>(s.gyro.y()),static_cast<float>(s.gyro.z())};
        if(name=="projected_gravity") {
            Eigen::Vector3f g=s.quat.cast<float>().conjugate()*Eigen::Vector3f(0,0,-1); return {g.x(),g.y(),g.z()};
        }
        if(name=="velocity_commands") return command(s);
        if(name=="gait_phase") {
            float period=term.params["period"].as<float>(); require(period>0 && std::isfinite(period),"invalid gait period");
            phase_=std::fmod(phase_+dt_*(1.f/period),1.f);
            const auto cmd=command(s); float norm=std::sqrt(cmd[0]*cmd[0]+cmd[1]*cmd[1]+cmd[2]*cmd[2]);
            if(norm<.1f) return {0,0};
            return {static_cast<float>(std::sin(phase_*2*pi)),static_cast<float>(std::cos(phase_*2*pi))};
        }
        if(name=="last_action") return last_action_;
        if(name=="joint_pos" || name=="joint_pos_rel" || name=="joint_vel_rel") {
            Values values(29);
            for(size_t i=0;i<29;++i) values[i]=name=="joint_vel_rel" ? static_cast<float>(s.dq[mapping_[i]]) : static_cast<float>(s.q[mapping_[i]])-(name=="joint_pos_rel" ? default_q_[i] : 0.f);
            auto ids=term.params["asset_cfg"] ? term.params["asset_cfg"]["joint_ids"] : YAML::Node{};
            if(ids && !ids.IsNull() && ids.size()>0) {
                Values selected; for(int id:ids.as<std::vector<int>>()) { require(id>=0 && id<29,"invalid observation joint id"); selected.push_back(values[id]); } return selected;
            }
            return values;
        }
        if(name=="motion_command" || name=="motion_anchor_ori_b") {
            require(static_cast<bool>(motion_),"motion observation requires mimic adapter"); const auto i=motionFrame(frame_);
            if(name=="motion_command") { Values value(motion_->q[i].data(),motion_->q[i].data()+29); value.insert(value.end(),motion_->dq[i].data(),motion_->dq[i].data()+29); return value; }
            const auto real=torso(s.quat.cast<float>(),s.q.cast<float>());
            const auto reference=torso(motion_->quat[i],motion_->q[i]);
            const Eigen::Matrix3f rot=((init_quat_*reference).conjugate()*real).toRotationMatrix().transpose();
            return {rot(0,0),rot(0,1),rot(1,0),rot(1,1),rot(2,0),rot(2,1)};
        }
        throw std::runtime_error("Legacy policy: unsupported observation "+name);
    }
    PolicyInfo info_;
    YAML::Node cfg_;
    std::vector<int> mapping_;
    Values default_q_,last_action_=Values(29,0.f);
    std::vector<ActionTerm> actions_;
    std::map<std::string,Group> groups_;
    std::unique_ptr<Motion> motion_;
    Eigen::Quaternionf init_quat_=Eigen::Quaternionf::Identity();
    float dt_=.02f,phase_=0,start_=0,end_=0;
    size_t frame_=0;
    bool initialized_=false;
    Ort::Env ort_env_;
    Ort::SessionOptions options_;
    std::unique_ptr<Ort::Session> session_;
    std::vector<std::string> input_names_;
    std::vector<std::vector<int64_t>> input_shapes_;
    std::string output_name_;
};
}
std::unique_ptr<Policy> makeLegacyPolicy(const YAML::Node& spec,const std::filesystem::path& root) {
    return std::make_unique<LegacyPolicy>(spec,root);
}
}
