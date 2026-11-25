#include "control/Controller.h"
#include "control/Registry.h"
#include <algorithm>
#include <cctype>
#include <sstream>
#include <cmath>
#include <stdexcept>
#include <utility>
namespace g1control {
namespace {
Joints vec(const YAML::Node& n) { if(!n.IsSequence()||n.size()!=29) throw std::runtime_error("expected 29 joints in FSM config"); Joints v; for(int i=0;i<29;i++) v[i]=n[i].as<double>(); return v; }
double smooth(double x) { x=std::clamp(x,0.,1.); return x*x*(3-2*x); }
Command blend(const Command& a,const Command& b,double x) { Command o; o.q=a.q+(b.q-a.q)*x;o.kp=a.kp+(b.kp-a.kp)*x;o.kd=a.kd+(b.kd-a.kd)*x;o.effort=a.effort.cwiseMin(b.effort);return o; }
Command target(const PolicyInfo& i,const Joints& q) {Command o;o.q=q;o.kp=i.kp;o.kd=i.kd;o.effort=i.effort;return o;}
void valid(const Sensors& s) { if(!s.q.allFinite()||!s.dq.allFinite()||!s.gyro.allFinite()||!s.quat.coeffs().allFinite()||std::abs(s.quat.norm()-1)>.05) throw std::runtime_error("invalid measured state"); }
}
Catalog::Catalog(const std::filesystem::path& root,const std::filesystem::path& registry_path,const std::filesystem::path& standing_path) {
    Registry registry(root,registry_path);
    for(auto spec:registry.data()["policies"]) {
        std::vector<std::string> aliases;
        if(spec["aliases"]) for(auto a:spec["aliases"]) aliases.push_back(a.as<std::string>());
        add(registry.load(spec["name"].as<std::string>()),aliases);
    }
    for(const std::string kind:{"velocity","balance","mimic"}) {
        YAML::Node spec;spec["adapter"]=kind;
        spec["name"]=kind=="mimic"?"Mimic_Dance1_subject2":kind=="velocity"?"Velocity":"Balance";
        spec["bundle"]=kind=="mimic"?"deploy/robots/g1/config/policy/mimic/dance1_subject2":"deploy/robots/g1/config/policy/velocity/v0";
        if(kind=="balance"&&!standing_path.empty())spec["bundle"]=standing_path.string();
        if(kind=="mimic") spec["motion_file"]="deploy/robots/g1/config/policy/mimic/dance1_subject2/params/dance1_subject2.npz";
        add(makeLegacyPolicy(spec,root),kind=="mimic"?std::vector<std::string>{"dance","mimic"}:std::vector<std::string>{kind});
    }
}
void Catalog::add(std::unique_ptr<Policy> p,const std::vector<std::string>& aliases) {
    const auto name=p->info().name;
    if(name.empty()||policies_.count(name)||aliases_.count(name)) throw std::runtime_error("duplicate policy "+name);
    if(std::abs(p->info().dt-.02)>1e-8) throw std::runtime_error("policy must run at 50 Hz: "+name);
    for(const auto& a:aliases) {if(a!=name && (aliases_.count(a)||policies_.count(a))) throw std::runtime_error("duplicate alias "+a);aliases_[a]=name;}
    policies_[name]=std::move(p);
}
Policy& Catalog::get(const std::string& name,bool ignore_case) {
    const auto key=aliases_.count(name)?aliases_.at(name):name;
    if(auto p=policies_.find(key);p!=policies_.end())return *p->second;
    if(ignore_case) {
        const auto lower=[](std::string value){std::transform(value.begin(),value.end(),value.begin(),[](unsigned char c){return std::tolower(c);});return value;};
        const auto wanted=lower(name);std::string found;
        const auto match=[&](const std::string& spelling,const std::string& canonical){
            if(lower(spelling)!=wanted)return;
            if(!found.empty()&&found!=canonical)throw std::runtime_error("ambiguous policy spelling: "+name);
            found=canonical;
        };
        for(const auto& [candidate,policy]:policies_)match(candidate,candidate);
        for(const auto& [alias,canonical]:aliases_)match(alias,canonical);
        if(!found.empty())return *policies_.at(found);
    }
    throw std::runtime_error("unknown policy: "+name);
}
void Catalog::warmup() {
    for(auto& [name,p]:policies_) {
        Sensors s;s.q=p->info().entry_q;s.root_pos.z()=p->referenceHeight(0);
        s.quat=Eigen::Quaterniond::FromTwoVectors(p->referenceGravity(0),Eigen::Vector3d(0,0,-1));
        p->begin(s);auto out=p->infer(s);if(!out.q.allFinite()) throw std::runtime_error("warmup produced invalid target: "+name);p->begin(s);
    }
}
Controller::Controller(Catalog& cat,const YAML::Node& original,Gravity gravity):catalog_(cat),gravity_(std::move(gravity)) {
    auto stand=original["FSM"]["FixStand"];
    stand_q_=vec(stand["qs"][1]);stand_kp_=vec(stand["kp"]);stand_kd_=vec(stand["kd"]);
    stand_s_=stand["ts"][1].as<double>();
    if(stand_s_<=0||!stand_q_.allFinite()||(stand_kp_.array()<=0).any()||(stand_kd_.array()<0).any()) throw std::runtime_error("invalid original stand configuration");
}
void Controller::passive() {session_=nullptr;session_command_.clear();session_after_return_.clear();state_=State::Passive;active_=pending_=nullptr;holding_=false;phase_time_=0;output_.kp.setZero();output_.kd.setConstant(3);}
void Controller::fault(const std::string& why) {error_=why;if(session_)session_->stop(why);else passive();}
void Controller::prepare(Policy& policy,const Sensors& s) {
    active_=&policy;pending_=nullptr;phase_=Phase::Prepare;phase_time_=0;holding_=false;start_=output_;active_->begin(s);
}
void Controller::returning(Policy* next,const Sensors& s) {
    pending_=next;active_=&catalog_.get("Balance");active_->begin(s);phase_=Phase::Return;phase_time_=0;holding_=false;start_=output_;
}
void Controller::configureSessions(const YAML::Node& registry,const std::filesystem::path& model,double damp_s) {
    registry_=YAML::Clone(registry);model_=model;damp_s_=damp_s;
    // Model/kinematic allocation happens before the first robot connection.
    // Switching sessions during control only resets preallocated state.
    sessions_.clear();session_=nullptr;
    for(auto entry:registry_["policies"]) {
        auto spec=YAML::Clone(registry_["defaults"]);
        for(auto field:entry)spec[field.first.as<std::string>()]=YAML::Clone(field.second);
        const auto name=spec["name"].as<std::string>();
        sessions_[name]=std::make_unique<Session>(catalog_,spec,registry_,model_,damp_s_);
    }
}
void Controller::selectSession(const std::string& name,const Sensors& s,bool automatic) {
    if(!registry_)throw std::runtime_error("session registry is not configured");
    const auto canonical=catalog_.get(name,true).info().name;
    auto found=sessions_.find(canonical);
    if(found==sessions_.end())throw std::runtime_error("not a registered motion session: "+name);
    auto* next=found->second.get();next->reset();
    if(automatic)next->startStanding(s);
    session_=next;session_command_.clear();active_=pending_=nullptr;error_.clear();
    state_=automatic?State::Stand:State::Passive;
}
std::string Controller::request(const std::string& text,const Sensors& s) {
    std::istringstream in(text);std::string cmd,name,extra;in>>cmd>>name>>extra;
    if(cmd.empty()) return "";
    std::transform(cmd.begin(),cmd.end(),cmd.begin(),[](unsigned char c){return std::tolower(c);});
    if(session_&&cmd=="takeover"&&!name.empty()&&session_->phase()=="WAIT_TAKEOVER"){
        const auto canonical=catalog_.get(name,true).info().name;
        if(sessions_.count(canonical)){selectSession(canonical,s);name.clear();}
        else {session_=nullptr;cmd="stand";name.clear();}
    }
    if(session_) {
        if(cmd=="stop") {session_->stop();return session_->status();}
        if(cmd=="status") {
            std::ostringstream message;message<<status()<<" | motion="<<session_->lastMotionName()<<" "<<session_->policyTicks()<<"/"<<session_->expectedPolicyTicks()<<" | pending="<<session_->pendingName()<<" | gate=";
            if(session_->gateResult().ready)message<<"ready";else for(auto& why:session_->gateResult().reasons)message<<why<<" ";return message.str();
        }
        if(cmd=="help") {
            std::string help="TAKEOVER -> STAND_PRE -> START; STOP enters damping. stand: original FixStand; passive: damping; policy <name>: switch when standing. ";
            for(auto entry:registry_["policies"])if(entry["name"].as<std::string>()==session_->initialName()) {
                auto ready=entry["ready_hold"];
                if(ready&&ready["fighting"]){help+="In HOLD verified actions: ";for(auto action:ready["fighting"]["actions"])help+=action.first.as<std::string>()+" ";help+="; NEXT <name> is equivalent. exit_star -> STAND_POST; entry_star re-enters.";}
                else if(ready)help+="In HOLD: NEXT exit_star/home_star. In STAND_POST: NEXT entry_star/guard_stand/home_star/exit_star.";
                else help+="One-shot motion: PREPARE -> POLICY -> RETURN -> STAND_POST. START never replays it.";
            }return help;
        }
        if(cmd=="passive") {passive();return status();}
        if(cmd=="stand"||cmd=="fixstand") {session_=nullptr;session_command_.clear();}
        else if(cmd=="reset") {return "Use passive before reset";}
        else {
            if(!error_.empty())return "fault latched: "+error_;
            if(cmd=="rl"||cmd=="policy") {
                if(name.empty())name=session_->initialName();
                const auto canonical=catalog_.get(name,true).info().name;
                if(canonical==session_->initialName()&&session_->phase()=="STAND_PRE")session_command_="START";
                else if(session_->phase()=="STAND_PRE"||session_->phase()=="STAND_POST") {
                    bool registered=false;
                    for(auto spec:registry_["policies"])if(spec["name"].as<std::string>()==canonical)registered=true;
                    if(registered){selectSession(canonical,s,true);session_command_="START";}
                    else {session_=nullptr;state_=State::Stand;phase_time_=stand_s_;}
                } else if(session_->acceptsCommand("NEXT "+canonical))session_command_="NEXT "+canonical;
                else return "Policy switch rejected in "+session_->phase()+"; use its session transitions or stand/passive";
                if(session_)return status();
            } else {
                std::string upper=cmd;std::transform(upper.begin(),upper.end(),upper.begin(),[](unsigned char c){return std::toupper(c);});
                session_command_=upper+(name.empty()?"":" "+name);
                return status();
            }
        }
    }
    if(cmd=="passive"||cmd=="Passive"||cmd=="stop") {passive();return status();}
    if(cmd=="reset") {if(state_!=State::Passive)return "reset requires Passive";error_.clear();return status();}
    if(cmd=="status") return status();
    if(!error_.empty()) return "fault latched: "+error_+"; inspect cause, then reset in Passive";
    if(!extra.empty()) return "too many command arguments";
    if(cmd=="stand"||cmd=="Stand"||cmd=="fixstand") {
        valid(s);start_=output_;start_.q=s.q;state_=State::Stand;phase_time_=0;active_=pending_=nullptr;return status();
    }
    if(cmd=="rl"||cmd=="RL"||cmd=="policy"||cmd=="next") {
        if(name.empty()&&(cmd=="policy"||cmd=="next"))return "policy requires a policy name";
        if(name.empty()) name="Velocity";
        Policy& p=catalog_.get(name); // Resolve before changing any state.
        if(state_==State::Passive) return "enter Stand first; RL is disabled in Passive";
        if(state_==State::Stand&&!standReady())return "Stand interpolation is not complete";
        bool registered=false;
        if(registry_)for(auto spec:registry_["policies"])if(spec["name"].as<std::string>()==p.info().name)registered=true;
        if(registered){
            if(state_!=State::Stand){returning(nullptr,s);session_after_return_=p.info().name;}
            else {selectSession(p.info().name,s,true);session_command_="START";}
        }else if(state_==State::Stand) {state_=State::RL;prepare(p,s);} else {returning(&p,s);}
        return status();
    }
    return "commands: stand | rl [name] | policy <name> | passive | status | reset | stop";
}
Command Controller::tick(const Sensors& s) {
    try {
        if(session_) {
            const auto command=std::exchange(session_command_,std::string{});
            session_output_=session_->step(s,command);
            output_.q=session_output_.q;output_.kp=session_output_.kp;output_.kd=session_output_.kd;output_.effort=session_output_.effort;
            const auto& phase=session_->phase();
            state_=(phase=="WAIT_TAKEOVER"||phase=="DAMP")?State::Passive:
                   (phase=="TAKEOVER"||phase=="STAND_PRE"||phase=="STAND_POST")?State::Stand:State::RL;
            return output_;
        }
        if(state_==State::Passive) {output_.q=s.q.allFinite()?s.q:Joints::Zero();output_.kp.setZero();output_.kd.setConstant(3);return output_;}
        valid(s);
        if(s.dq.cwiseAbs().maxCoeff()>60)throw std::runtime_error("joint speed above global guard");
        const auto gravity=s.quat.normalized().conjugate()*Eigen::Vector3d(0,0,-1);
        if(state_==State::Stand) {
            if(std::acos(std::clamp(-gravity.z(),-1.,1.))>.8)throw std::runtime_error("stand tilt guard");
            phase_time_+=dt_;Command to;to.q=stand_q_;to.kp=stand_kp_;to.kd=stand_kd_;
            output_=to;output_.q=start_.q+(stand_q_-start_.q)*std::clamp(phase_time_/stand_s_,0.,1.);
        } else if(phase_==Phase::Prepare) {
            const auto& info=active_->info();
            if(std::acos(std::clamp(gravity.dot(active_->referenceGravity(0)),-1.,1.))>.55)throw std::runtime_error("preparation orientation differs from reference");
            phase_time_+=dt_;Joints q=info.entry_q;if(gravity_)q+=gravity_(s,info);
            output_=blend(start_,target(info,q),smooth(phase_time_/prepare_s_));
            if(phase_time_>=prepare_s_) {
                if(std::sqrt((s.q-info.entry_q).squaredNorm()/29.)>.25 || std::abs(s.q[14]-info.entry_q[14])>.30) {
                    if(phase_time_>prepare_s_+4.) throw std::runtime_error("policy entry pose did not settle: "+info.name);
                } else {active_->begin(s);phase_=Phase::Active;phase_time_=0;start_=output_;}
            }
        } else {
            const auto& info=active_->info();
            if(std::acos(std::clamp(gravity.dot(active_->referenceGravity(active_->frame())),-1.,1.))>1.)throw std::runtime_error("RL orientation guard");
            auto out=active_->infer(s);
            if(!out.q.allFinite())throw std::runtime_error("non-finite policy target");
            Command next=target(info,out.q);phase_time_+=dt_;
            if(phase_==Phase::Return) {
                output_=blend(start_,next,smooth(phase_time_/return_s_));active_->advance();
                if(phase_time_>=return_s_) {
                    if(!session_after_return_.empty()){auto name=std::exchange(session_after_return_,std::string{});selectSession(name,s,true);session_command_="START";return output_;}
                    if(pending_) {auto* p=pending_;prepare(*p,s);}else{phase_=Phase::Active;phase_time_=.06;}}
            } else {
                output_=blend(start_,next,smooth(phase_time_/.06));
                if(!info.continuous && !holding_ && active_->frame()+1>=info.frames) {
                    if(info.hold_final) {active_->holdFinal();holding_=true;}
                    else returning(nullptr,s);
                } else if(!holding_) active_->advance();
            }
        }
        if(!output_.q.allFinite()||!output_.kp.allFinite()||!output_.kd.allFinite())throw std::runtime_error("non-finite motor command");
    } catch(const std::exception& e) {
        fault(e.what());output_.q=s.q.allFinite()?s.q:Joints::Zero();output_.kp.setZero();output_.kd.setConstant(3);
        if(session_){session_output_=session_->step(s);output_.q=session_output_.q;output_.kp=session_output_.kp;output_.kd=session_output_.kd;output_.effort=session_output_.effort;state_=State::Passive;}
    }
    return output_;
}
std::string Controller::status() const {
    std::string out=state_==State::Passive?"Passive":state_==State::Stand?"Stand":"RL";
    if(session_)out+="/"+session_->status();
    if(state_==State::RL&&active_) out+="/"+std::string(phase_==Phase::Prepare?"Prepare":phase_==Phase::Return?"Return":holding_?"Hold":"Active")+"/"+active_->info().name;
    if(pending_)out+=" -> "+pending_->info().name;
    if(!error_.empty())out+=" [FAULT: "+error_+"]";
    return out;
}
}
