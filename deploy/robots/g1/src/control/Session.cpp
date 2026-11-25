#include "control/Session.h"
#include "control/Controller.h"
#include "control/Physics.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <set>
#include <sstream>
#include <stdexcept>

namespace g1control {
namespace {
void check(bool yes,const std::string& why) { if(!yes)throw std::runtime_error("session: "+why); }
std::string upper(std::string value) { for(auto& c:value)c=static_cast<char>(std::toupper(static_cast<unsigned char>(c)));return value; }
std::string normalized(const std::string& value) { std::istringstream in(value);std::string part,out;while(in>>part){if(!out.empty())out+=' ';out+=part;}return upper(out); }
size_t ticks(double seconds,double dt) {const double n=seconds/dt;check(std::isfinite(n)&&n>0&&std::abs(n-std::round(n))<=1e-5,"duration must be a positive integer number of control ticks");return static_cast<size_t>(std::llround(n));}
double smooth(double x){return x*x*(3-2*x);}
double quintic(double x){x=std::clamp(x,0.,1.);return x*x*x*(10.+x*(-15.+6.*x));}
YAML::Node merged(const YAML::Node& defaults,const YAML::Node& entry) {auto out=YAML::Clone(defaults);if(!out||!out.IsMap())out=YAML::Node(YAML::NodeType::Map);for(auto v:entry)out[v.first.as<std::string>()]=YAML::Clone(v.second);return out;}

// Exact SwitchGate kinematic proxies: root translation/linear velocity never enter.
class Gate {
    struct ModelDelete{void operator()(mjModel* p)const{mj_deleteModel(p);}};
    struct DataDelete{void operator()(mjData* p)const{mj_deleteData(p);}};
    std::unique_ptr<mjModel,ModelDelete> model_;
    std::unique_ptr<mjData,DataDelete> data_,target_;
    std::array<int,29> qadr_,vadr_;
    std::array<int,2> feet_;
    std::vector<int> leg_;
    int rootq_=0,rootv_=0;
    std::vector<double> jac_,rotjac_;
    size_t required_,stable_=0;
    bool cached_=false;
    Joints last_q_;
    Eigen::Quaterniond last_quat_;
    double target_height_=0;
    const std::array<std::string,8> names_={"pose_rms_rad","leg_pose_rms_rad","joint_speed_rms_rad_s","leg_speed_max_rad_s","body_ang_speed_rad_s","tilt_error_rad","foot_height_error_m","foot_relative_speed_m_s"};
    std::array<double,8> limits_={.15,.12,.6,.8,.25,.12,.025,.08};
    void kinematics(mjData* d,const Joints& q,const Eigen::Quaterniond& quat) {
        d->qpos[rootq_]=0;d->qpos[rootq_+1]=0;d->qpos[rootq_+2]=1;
        const auto unit=quat.normalized();d->qpos[rootq_+3]=unit.w();d->qpos[rootq_+4]=unit.x();d->qpos[rootq_+5]=unit.y();d->qpos[rootq_+6]=unit.z();
        for(int j=0;j<29;++j)d->qpos[qadr_[j]]=q[j];
        mj_kinematics(model_.get(),d);mj_comPos(model_.get(),d);
    }
public:
    Gate(const std::filesystem::path& path,const std::vector<std::string>& joint_names,size_t stable,const std::map<std::string,double>& limits={}):required_(stable) {
        check(stable>0,"gate stable_ticks must be positive");
        check(joint_names.size()==29&&std::set<std::string>(joint_names.begin(),joint_names.end()).size()==29,"gate requires 29 unique joint names");
        for(auto [name,value]:limits){auto it=std::find(names_.begin(),names_.end(),name);check(it!=names_.end(),"unknown gate threshold "+name);check(std::isfinite(value)&&value>0,"invalid gate threshold");limits_[it-names_.begin()]=value;}
        char error[2048]={};model_.reset(mj_loadXML(path.c_str(),nullptr,error,sizeof(error)));check(bool(model_),"gate cannot load model: "+std::string(error));
        data_.reset(mj_makeData(model_.get()));target_.reset(mj_makeData(model_.get()));check(data_&&target_,"cannot allocate gate kinematics");
        int free_count=0;
        for(int j=0;j<model_->njnt;++j)if(model_->jnt_type[j]==mjJNT_FREE){++free_count;rootq_=model_->jnt_qposadr[j];rootv_=model_->jnt_dofadr[j];}
        check(free_count==1,"gate model must have one floating base");
        for(int j=0;j<29;++j){const auto& name=joint_names[j];const int id=mj_name2id(model_.get(),mjOBJ_JOINT,name.c_str());check(id>=0,"gate joint missing: "+name);qadr_[j]=model_->jnt_qposadr[id];vadr_[j]=model_->jnt_dofadr[id];if(name.find("hip_")!=std::string::npos||name.find("knee_")!=std::string::npos||name.find("ankle_")!=std::string::npos)leg_.push_back(j);}
        check(!leg_.empty(),"gate requires leg joints");
        feet_[0]=mj_name2id(model_.get(),mjOBJ_BODY,"left_ankle_roll_link");feet_[1]=mj_name2id(model_.get(),mjOBJ_BODY,"right_ankle_roll_link");check(feet_[0]>=0&&feet_[1]>=0,"gate model missing feet");
        jac_.resize(3*model_->nv);rotjac_.resize(3*model_->nv);
    }
    void reset(){stable_=0;}
    SessionGateResult update(const Sensors& s,const Joints& target_q,const Eigen::Quaterniond& target_quat=Eigen::Quaterniond::Identity()) {
        SessionGateResult out;
        if(!s.q.allFinite()||!s.dq.allFinite()||!s.gyro.allFinite()||!s.quat.coeffs().allFinite()||!target_q.allFinite()||!target_quat.coeffs().allFinite()){reset();out.reasons={"invalid or non-finite gate input"};return out;}
        if(std::abs(s.quat.norm()-1)>.05||std::abs(target_quat.norm()-1)>.05){reset();out.reasons={"invalid quaternion norm"};return out;}
        if(!cached_||!(last_q_.array()==target_q.array()).all()||!(last_quat_.coeffs().array()==target_quat.coeffs().array()).all()) {
            reset();kinematics(target_.get(),target_q,target_quat);target_height_=target_->xpos[feet_[0]*3+2]-target_->xpos[feet_[1]*3+2];last_q_=target_q;last_quat_=target_quat;cached_=true;
        }
        kinematics(data_.get(),s.q,s.quat);std::fill_n(data_->qvel,model_->nv,0.);
        for(int j=0;j<29;++j)data_->qvel[vadr_[j]]=s.dq[j];
        for(int j=0;j<3;++j)data_->qvel[rootv_+3+j]=s.gyro[j];
        std::array<Eigen::Vector3d,2> velocities;
        for(int f=0;f<2;++f){mj_jacBody(model_.get(),data_.get(),jac_.data(),rotjac_.data(),feet_[f]);velocities[f].setZero();for(int a=0;a<3;++a)for(int v=0;v<model_->nv;++v)velocities[f][a]+=jac_[a*model_->nv+v]*data_->qvel[v];}
        const auto delta=(s.q-target_q).eval();double leg_error=0,leg_speed=0;
        for(int j:leg_){leg_error+=delta[j]*delta[j];leg_speed=std::max(leg_speed,std::abs(s.dq[j]));}
        const Eigen::Vector3d g=-s.quat.normalized().toRotationMatrix().row(2).transpose(),tg=-target_quat.normalized().toRotationMatrix().row(2).transpose();
        const std::array<double,8> values={std::sqrt(delta.squaredNorm()/29),std::sqrt(leg_error/leg_.size()),std::sqrt(s.dq.squaredNorm()/29),leg_speed,s.gyro.norm(),std::acos(std::clamp(g.dot(tg),-1.,1.)),std::abs(data_->xpos[feet_[0]*3+2]-data_->xpos[feet_[1]*3+2]-target_height_),(velocities[0]-velocities[1]).norm()};
        bool finite=true;for(size_t i=0;i<values.size();++i){out.metrics[names_[i]]=values[i];if(values[i]>limits_[i])out.reasons.push_back(names_[i]);finite=finite&&std::isfinite(values[i]);}
        if(!out.reasons.empty()||!finite){reset();if(out.reasons.empty())out.reasons={"non-finite kinematic metric"};}else stable_=std::min(stable_+1,required_);
        out.stable_ticks=stable_;out.ready=stable_>=required_;if(!out.ready&&out.reasons.empty())out.reasons={"settling interval incomplete"};return out;
    }
};
} // namespace

struct Session::Impl {
    enum class Kind{OneShot,ReadyHold,Fighting};
    Catalog& catalog;
    YAML::Node registry,initial_spec,fight_config;
    Policy *initial=nullptr,*policy=nullptr,*standing=nullptr,*holder=nullptr,*source=nullptr;
    std::map<std::string,YAML::Node> specs;
    std::map<std::string,Policy*> choices;
    std::map<std::string,std::string> names;
    std::set<std::string> hold_names,next_from_hold,actions;
    std::string initial_name,active_name,holder_name,exit_name,last_motion,pending,phase="WAIT_TAKEOVER",inner="STAND_PRE",event,stop_reason,motion_kind="entry",setup;
    Kind kind=Kind::OneShot;
    std::unique_ptr<UpperBodyGravity> gravity;
    std::unique_ptr<Gate> gate,settle_gate,acquire_gate;
    SessionGateResult gate_result;
    std::vector<CompletedMotion> completed;
    size_t takeover_steps=0,damp_steps=0,prep_steps=0,return_steps=0,entry_blend_steps=0,blend_steps=0;
    size_t index=0,outer_index=0,policy_ticks=0,expected_ticks=0,hold_ticks=0,pending_ticks=0,acquire_ticks=0,settle_ticks=0,warmup_ticks=0;
    size_t request_timeout=0,settle_timeout=0,acquire_timeout=0;
    std::optional<size_t> actor_blend;
    std::optional<double> entry_rms;
    Joints last_q,last_kp,last_kd,origin_q,origin_kp,origin_kd,entry_target,actor_origin,q_start,ready_q;
    bool initialized=false,done=false;
    explicit Impl(Catalog& c,const YAML::Node& initial_cfg,const YAML::Node& full_registry,const std::filesystem::path& path,double damp):catalog(c),registry(YAML::Clone(full_registry)) {
        initial_spec=merged(registry["defaults"],initial_cfg);initial_name=initial_spec["name"].as<std::string>();
        initial=&catalog.get(initial_name);standing=&catalog.get("Balance");policy=initial;
        check(initial->info().frames>0,"sessions require a finite motion policy");
        takeover_steps=ticks(initial_spec["takeover_s"].as<double>(4.),initial->info().dt);damp_steps=ticks(damp,initial->info().dt);
        gravity=std::make_unique<UpperBodyGravity>(path,initial->info().joint_names,initial->info().effort);
        if(initial_spec["ready_hold"]){
            const auto cfg=initial_spec["ready_hold"];kind=cfg["fighting"]?Kind::Fighting:Kind::ReadyHold;
            addChoice(initial_name,initial_spec);
            if(kind==Kind::ReadyHold){
                for(auto name:cfg["next_policies"])addChoice(name.as<std::string>());
                for(auto name:cfg["hold_policies"])hold_names.insert(canonical(name.as<std::string>()));
                for(auto name:cfg["next_from_hold"])next_from_hold.insert(canonical(name.as<std::string>()));
                check(hold_names.count(initial_name)>0,"initial ready-hold policy must hold its final frame");
            }else{
                fight_config=cfg["fighting"];addChoice(fight_config["holder"].as<std::string>());addChoice(fight_config["exit"].as<std::string>());
                for(auto entry:fight_config["actions"])addChoice(entry.first.as<std::string>());
                holder_name=canonical(fight_config["holder"].as<std::string>());exit_name=canonical(fight_config["exit"].as<std::string>());holder=choices.at(holder_name);
                for(auto entry:fight_config["actions"])actions.insert(canonical(entry.first.as<std::string>()));
                ready_q=initial->referenceJoints(initial->info().frames-1);
                check(((holder->referenceJoints(holder->info().frames-1)-ready_q).array().abs() <= (1e-6+1e-5*ready_q.array().abs())).all(),"holder must target the entry final ready pose");
                request_timeout=ticks(fight_config["request_timeout_s"].as<double>(),initial->info().dt);settle_timeout=ticks(fight_config["settle_timeout_s"].as<double>(),initial->info().dt);acquire_timeout=ticks(fight_config["acquire_timeout_s"].as<double>(),initial->info().dt);blend_steps=ticks(fight_config["actor_blend_s"].as<double>(),initial->info().dt);
                gate=std::make_unique<Gate>(path,initial->info().joint_names,25,std::map<std::string,double>{{"body_ang_speed_rad_s",.10},{"leg_speed_max_rad_s",.25},{"joint_speed_rms_rad_s",.20},{"foot_relative_speed_m_s",.06},{"pose_rms_rad",.08},{"tilt_error_rad",.08}});
                settle_gate=std::make_unique<Gate>(path,initial->info().joint_names,5,std::map<std::string,double>{{"leg_speed_max_rad_s",1.5},{"body_ang_speed_rad_s",.7},{"pose_rms_rad",.20}});
                acquire_gate=std::make_unique<Gate>(path,initial->info().joint_names,10);
            }
        }
        reset();
    }
    YAML::Node resolve(const std::string& name)const {
        for(auto candidate:registry["policies"]){if(candidate["name"].as<std::string>()==name)return merged(registry["defaults"],candidate);for(auto alias:candidate["aliases"])if(alias.as<std::string>()==name)return merged(registry["defaults"],candidate);}
        throw std::runtime_error("session: unknown registry policy "+name);
    }
    void addChoice(const std::string& name,YAML::Node spec={}) {
        if(!spec||!spec.IsMap())spec=resolve(name);
        const auto key=spec["name"].as<std::string>();auto* p=&catalog.get(key);
        if(!choices.count(key)) {
            const auto& a=p->info();const auto& b=initial->info();
            check(a.joint_names==b.joint_names&&a.observation_width==b.observation_width&&a.dt==b.dt&&!a.needs_estimator,"incompatible session policy "+key);
            check((a.kp.array()==b.kp.array()).all()&&(a.kd.array()==b.kd.array()).all()&&(a.effort.array()==b.effort.array()).all(),"session policies require identical gains/effort: "+key);
            check(kind==Kind::ReadyHold ? a.kind=="ufo_jointtrack" : ((a.kind=="ufo_jointtrack"||a.kind=="hybrid")&&a.robust_proprio),"session requires compatible proprio policies: "+key);
            choices[key]=p;specs[key]=spec;
        }
        auto alias=[&](const std::string& a){const auto u=upper(a);check(!names.count(u)||names.at(u)==key,"ambiguous session alias "+a);names[u]=key;};alias(key);for(auto a:spec["aliases"])alias(a.as<std::string>());
    }
    std::string canonical(const std::string& name)const{const auto it=names.find(upper(name));check(it!=names.end(),"unknown/unprepared session policy "+name);return it->second;}
    void settings(const YAML::Node& spec){prep_steps=ticks(spec["prepare_s"].as<double>(.4),policy->info().dt);return_steps=ticks(spec["return_s"].as<double>(.5),policy->info().dt);entry_blend_steps=std::min<size_t>(3,policy->info().frames);}
    void reset(){
        policy=initial;source=nullptr;active_name=initial_name;last_motion=initial_name;phase="WAIT_TAKEOVER";inner="STAND_PRE";event.clear();stop_reason.clear();pending.clear();setup.clear();motion_kind="entry";
        index=outer_index=policy_ticks=hold_ticks=pending_ticks=acquire_ticks=settle_ticks=warmup_ticks=0;expected_ticks=initial->info().frames;entry_rms.reset();actor_blend.reset();completed.clear();gate_result={};done=false;initialized=false;
        last_q=standing->info().entry_q;last_kp=standing->info().kp;last_kd=standing->info().kd;settings(initial_spec);
        if(gate){gate->reset();settle_gate->reset();acquire_gate->reset();}
    }
    void stop(const std::string& reason){if(phase!="DAMP"){phase="DAMP";outer_index=0;stop_reason=reason;}}
    bool accepts(const std::string& cmd)const{const auto c=normalized(cmd);return kind==Kind::Fighting ? (!c.empty()&&(names.count(c)||c=="NEXT"||c.rfind("NEXT ",0)==0)) : (kind==Kind::ReadyHold&&(c=="NEXT"||c.rfind("NEXT ",0)==0));}
    std::vector<float> observation(const PolicyOutput& result)const {auto obs=result.observation;if(policy->info().kind=="reference")obs.insert(obs.end(),result.goal.begin(),result.goal.end());return obs;}
    SessionOutput packed(const std::string& p,long frame,const Joints& q,const Joints& kp,const Joints& kd,const PolicyOutput* inferred=nullptr,bool pad=true) const {
        SessionOutput out;out.phase=p;out.frame=frame;out.q=q;out.kp=kp;out.kd=kd;out.effort=policy->info().effort;out.active_policy=active_name;
        out.raw_action.setConstant(std::numeric_limits<double>::quiet_NaN());
        if(pad)out.observation.assign(std::max<size_t>(98,policy->info().observation_width),std::numeric_limits<float>::quiet_NaN());
        if(inferred){auto obs=observation(*inferred);if(pad){check(obs.size()<=out.observation.size(),"unexpected observation width");std::copy(obs.begin(),obs.end(),out.observation.begin());}else out.observation=std::move(obs);out.raw_action=inferred->action;}
        return out;
    }
    void transition(std::string next){
        if(next=="RETURN"&&kind!=Kind::OneShot){
            completed.push_back({active_name,policy_ticks});
            if(kind==Kind::ReadyHold&&hold_names.count(active_name)){next="HOLD";policy->holdFinal();}
            else if(kind==Kind::Fighting&&motion_kind!="exit"){
                if(motion_kind=="entry"){next="READY_BLEND";setup="holder";}
                else{next="SETTLE";policy->holdReferenceFrame(policy->info().frames-1);settle_gate->reset();settle_ticks=0;}
            }
        }
        inner=std::move(next);index=0;origin_q=last_q;origin_kp=last_kp;origin_kd=last_kd;if(inner=="RETURN")standing->resetActionHistory();
    }
    SessionOutput baseStep(const Sensors& s,const std::string& command){
        event.clear();if(command=="START"&&inner=="STAND_PRE"){transition("PREPARE");event="PREPARE: transitioning to motion entry pose";}else if(command=="START")event="START ignored: this sequence runs one motion once";
        const auto current=inner;SessionOutput out;
        if(current=="STAND_PRE"||current=="STAND_POST"||current=="RETURN"){
            const auto result=standing->infer(s);Joints q=result.q,kp=standing->info().kp,kd=standing->info().kd;
            if(current=="RETURN"){
                ++index;const double x=smooth(double(index)/return_steps);q=(1-x)*origin_q+x*q;kp=(1-x)*origin_kp+x*kp;kd=(1-x)*origin_kd+x*kd;standing->feedbackTarget(q);
                if(index==return_steps){inner="STAND_POST";event="STAND_POST: zero-command velocity standing";}
            }
            // Standing observation is already flat even when the selected motion uses Reference.
            out=packed(current,-1,q,kp,kd);std::copy(result.observation.begin(),result.observation.end(),out.observation.begin());out.raw_action=result.action;
        }else if(current=="PREPARE"){
            ++index;const double x=smooth(double(index)/prep_steps);Joints q=(1-x)*origin_q+x*policy->info().entry_q,kp=(1-x)*origin_kp+x*policy->info().kp,kd=(1-x)*origin_kd+x*policy->info().kd;
            q+=x*gravity->offset(s,kp);if(index==prep_steps)inner="POLICY";out=packed(current,-1,q,kp,kd);
        }else if(current=="POLICY"){
            if(policy_ticks==0){entry_rms=std::sqrt((s.q-policy->info().entry_q).squaredNorm()/29.);if(*entry_rms>.25||std::abs(s.q[14]-policy->info().entry_q[14])>.30)throw std::runtime_error("entry not ready: prepare again from standing");entry_target=last_q;policy->begin(s);event="POLICY: frame 0";}
            const auto frame=policy->frame();const auto result=policy->infer(s);Joints q=result.q;++policy_ticks;
            if(policy_ticks<=entry_blend_steps){const double x=smooth(double(policy_ticks)/entry_blend_steps);q=(1-x)*entry_target+x*q;}
            out=packed(current,static_cast<long>(frame),q,policy->info().kp,policy->info().kd,&result);
            if(frame==policy->info().frames-1){last_q=q;last_kp=out.kp;last_kd=out.kd;transition("RETURN");}else policy->advance();
        }else throw std::runtime_error("invalid session phase "+current);
        remember(out);return out;
    }
    void remember(const SessionOutput& out){check(out.q.allFinite()&&out.kp.allFinite()&&out.kd.allFinite(),"non-finite "+out.phase+" command");last_q=out.q;last_kp=out.kp;last_kd=out.kd;}
    SessionOutput readyStep(const Sensors& s,std::string command){
        std::string notice;
        if(command=="NEXT"||command.rfind("NEXT ",0)==0){
            const auto name=command=="NEXT"?"":command.substr(5);
            if(inner!="HOLD"&&inner!="STAND_POST")notice="NEXT ignored in "+inner+"; wait for HOLD or STAND_POST";
            else if(!names.count(name))notice="NEXT rejected: unknown or unprepared policy";
            else if(inner=="HOLD"&&!next_from_hold.count(names.at(name)))notice="NEXT rejected in HOLD: use NEXT exit_star to return to standing before entry_star/guard_stand";
            else{active_name=names.at(name);policy=choices.at(active_name);settings(specs.at(active_name));policy_ticks=0;expected_ticks=policy->info().frames;last_motion=active_name;entry_rms.reset();transition("PREPARE");notice="NEXT "+active_name+": PREPARE from current controlled pose";}
            command.clear();
        }
        SessionOutput out;
        if(inner=="HOLD"){
            const auto result=policy->infer(s);out=packed("HOLD",policy->frame(),result.q,policy->info().kp,policy->info().kd,&result,false);remember(out);++hold_ticks;event=notice.empty()?(command=="START"?"START ignored in HOLD; use NEXT <policy>":""):notice;
        }else{out=baseStep(s,command);if(inner=="HOLD"&&out.phase=="POLICY")event="HOLD: closed-loop final-pose tracking. NEXT <policy> switches; STOP enters damping.";if(!notice.empty())event=notice;}
        return out;
    }
    void startBlend(){actor_blend=0;actor_origin=last_q;}
    SessionOutput actorCommand(const Sensors& s,const std::string& current,size_t frame){
        const auto result=policy->infer(s);Joints q=result.q;
        if(actor_blend){
            const double weight=quintic(double(*actor_blend)/blend_steps);const Joints from=source!=policy?source->infer(s).q:actor_origin;
            q=(1-weight)*from+weight*q;policy->feedbackTarget(q);source->feedbackTarget(q);++*actor_blend;if(*actor_blend>blend_steps)actor_blend.reset();
        }
        return packed(current,frame,q,policy->info().kp,policy->info().kd,&result,false);
    }
    bool holderReady(const Sensors& s){
        ++acquire_ticks;const auto result=acquire_gate->update(s,ready_q);if(actor_blend)acquire_gate->reset();const bool acquired=!actor_blend&&result.ready;
        if(acquire_ticks>=acquire_timeout&&!acquired)throw std::runtime_error("ready controller did not settle");
        return acquired;
    }
    void beginHolder(const Sensors& s){source=policy;active_name=holder_name;policy=holder;policy->begin(s);policy->holdFinal();startBlend();inner="READY_BLEND";setup.clear();gate->reset();acquire_gate->reset();acquire_ticks=0;}
    void beginAction(const std::string& name,const Sensors& s){source=policy;active_name=name;policy=choices.at(name);setup.clear();pending.clear();policy->begin(s);policy->holdReferenceFrame(std::nullopt);startBlend();acquire_gate->reset();acquire_ticks=warmup_ticks=0;inner="WARMUP";policy_ticks=0;expected_ticks=policy->info().frames;last_motion=name;motion_kind=name==exit_name?"exit":"action";entry_rms.reset();}
    std::string request(const std::string& command){
        if(!accepts(command))return {};
        const auto name=command.rfind("NEXT ",0)==0?command.substr(5):command;
        if(!names.count(name))return "Unknown/unverified action";
        const auto target=names.at(name);
        if(inner=="STAND_POST"&&target==initial_name){setup="entry:"+target;return "Starting entry_star from standing";}
        if(inner!="HOLD")return "Action ignored in "+inner+"; wait for HOLD";
        if(target==holder_name)return "Already holding the ready pose with home_star";
        if(!actions.count(target)&&target!=exit_name)return "Use a verified action or exit_star from HOLD";
        pending=target;pending_ticks=0;return "WAIT_READY: "+target+"; maintaining ready pose until switching conditions hold";
    }
    SessionOutput fightStep(const Sensors& s,std::string command){
        event.clear();auto notice=command.empty()?std::string{}:request(command);if(!notice.empty())command.clear();
        if(setup.rfind("entry:",0)==0){active_name=setup.substr(6);policy=choices.at(active_name);policy_ticks=0;expected_ticks=policy->info().frames;last_motion=active_name;motion_kind="entry";setup.clear();transition("PREPARE");}
        else if(setup=="holder"||setup=="recover")beginHolder(s);
        if(inner=="HOLD"){
            gate_result=gate->update(s,ready_q);
            if(!pending.empty()){++pending_ticks;if(gate_result.ready){const auto name=pending;beginAction(name,s);notice="TRANSITION: "+name+"; measured-state gate passed";}else if(pending_ticks>=request_timeout){pending.clear();notice="Action cancelled: ready-state conditions not met; continuing HOLD";}}
        }
        const auto current=inner;SessionOutput out;
        if(current=="HOLD"||current=="READY_BLEND"){
            out=actorCommand(s,current,policy->frame());bool acquired=false;if(current=="READY_BLEND")acquired=holderReady(s);
            if(current=="HOLD")++hold_ticks;else if(acquired){inner="HOLD";event="HOLD: stable ready controller; type an action name, NEXT <name>, or STOP";}
        }else if(current=="WARMUP"||current=="FIGHT_POLICY"){
            const auto frame=policy->frame();out=actorCommand(s,current=="WARMUP"?"WARMUP":"POLICY",frame);
            if(current=="WARMUP"){++warmup_ticks;if(!actor_blend){entry_rms=std::sqrt((s.q-policy->info().entry_q).squaredNorm()/29.);inner="FIGHT_POLICY";policy->holdReferenceFrame(std::nullopt);}}
            else{++policy_ticks;if(frame==policy->info().frames-1){last_q=out.q;transition("RETURN");}else policy->advance();}
        }else if(current=="SETTLE"){
            out=actorCommand(s,current,policy->frame());++settle_ticks;const auto result=settle_gate->update(s,policy->referenceJoints(policy->info().frames-1),policy->referenceQuaternion(policy->info().frames-1));
            if(result.ready){setup="recover";event="RECOVER: motion settled; returning to ready pose";}else if(settle_ticks>=settle_timeout)throw std::runtime_error("motion did not settle for return to ready");
        }else out=baseStep(s,command);
        remember(out);if(out.phase=="POLICY"&&inner=="READY_BLEND")event="READY_BLEND: entering the ready-pose controller";else if(out.phase=="POLICY"&&inner=="SETTLE")event="SETTLE: motion complete; waiting for stable return conditions";if(!notice.empty())event=notice;return out;
    }
    SessionOutput step(const Sensors& s,const std::string& raw){
        if(!initialized){standing->begin(s);initialized=true;}
        const auto command=normalized(raw);event.clear();
        if(command=="STOP")stop("STOP requested");
        if(phase=="WAIT_TAKEOVER"&&command=="TAKEOVER"){q_start=s.q;phase="TAKEOVER";outer_index=0;event="TAKEOVER: virtual hoist ON; moving to normal standing pose";}
        else if(command=="TAKEOVER")event="TAKEOVER ignored in "+phase;
        else if(command=="START"&&(phase=="WAIT_TAKEOVER"||phase=="TAKEOVER"||phase=="DAMP"))event="START ignored in "+phase+"; wait for STAND_PRE";
        SessionOutput out;
        if(phase=="WAIT_TAKEOVER"||phase=="DAMP"){
            if(phase=="DAMP"){++outer_index;done=outer_index>=damp_steps;}
            out=packed(phase,-1,s.q,Joints::Zero(),Joints::Constant(3));
        }else if(phase=="TAKEOVER"){
            ++outer_index;const double x=smooth(double(outer_index)/takeover_steps);const Joints q=q_start+x*(standing->info().entry_q-q_start);
            out=packed("TAKEOVER",-1,q,standing->info().kp,standing->info().kd);
            if(outer_index==takeover_steps){phase="STAND_PRE";event="STAND_PRE: virtual hoist OFF; zero-command standing. Type START when ready.";}
        }else{
            out=kind==Kind::Fighting?fightStep(s,command):kind==Kind::ReadyHold?readyStep(s,command):baseStep(s,command);phase=inner;
        }
        check(out.q.allFinite()&&out.kp.allFinite()&&out.kd.allFinite(),"non-finite "+out.phase+" command");out.support=out.phase=="WAIT_TAKEOVER"||out.phase=="TAKEOVER"||out.phase=="DAMP";out.finished=done;return out;
    }
};
Session::Session(Catalog& c,const YAML::Node& spec,const YAML::Node& registry,const std::filesystem::path& model,double damp):impl_(std::make_unique<Impl>(c,spec,registry,model,damp)){}
Session::~Session()=default;
void Session::reset(){impl_->reset();}
void Session::startStanding(const Sensors& state){impl_->reset();impl_->standing->begin(state);impl_->initialized=true;impl_->phase=impl_->inner="STAND_PRE";}
void Session::stop(const std::string& why){impl_->stop(why);}
SessionOutput Session::step(const Sensors& state,const std::string& command){return impl_->step(state,command);}
bool Session::acceptsCommand(const std::string& command)const{return impl_->accepts(command);}
bool Session::finished()const{return impl_->done;}
const std::string& Session::phase()const{return impl_->phase;}
const std::string& Session::event()const{return impl_->event;}
const std::string& Session::stopReason()const{return impl_->stop_reason;}
const std::string& Session::activeName()const{return impl_->active_name;}
const std::string& Session::initialName()const{return impl_->initial_name;}
const std::string& Session::lastMotionName()const{return impl_->last_motion;}
const std::string& Session::pendingName()const{return impl_->pending;}
Policy& Session::activePolicy()const{return *impl_->policy;}
size_t Session::policyTicks()const{return impl_->policy_ticks;}
size_t Session::expectedPolicyTicks()const{return impl_->expected_ticks;}
size_t Session::holdTicks()const{return impl_->hold_ticks;}
size_t Session::pendingTicks()const{return impl_->pending_ticks;}
std::optional<double> Session::entryPoseRms()const{return impl_->entry_rms;}
const std::vector<CompletedMotion>& Session::completedMotions()const{return impl_->completed;}
const SessionGateResult& Session::gateResult()const{return impl_->gate_result;}
std::string Session::status()const{auto out=impl_->phase+"/"+impl_->active_name;if(!impl_->pending.empty())out+=" -> "+impl_->pending;return out;}
} // namespace g1control
