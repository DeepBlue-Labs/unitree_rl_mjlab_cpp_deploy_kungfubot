// Replay real Python session fixtures against the C++ state machines and actors.
#include "control/Session.h"
#include "control/Controller.h"
#include <fstream>
#include <iostream>
#include <iomanip>
#include <cmath>
#include <stdexcept>
using namespace g1control;
Joints joints(const YAML::Node& n){Joints q;for(int j=0;j<29;++j)q[j]=n[j].as<double>();return q;}
void demand(bool ok,const std::string& why){if(!ok)throw std::runtime_error(why);}
int main(int argc,char** argv){
 try{
  if(argc!=3)throw std::runtime_error("usage: session_probe ROOT MANIFEST.yaml");
  const std::filesystem::path root=argv[1];const auto manifest=YAML::LoadFile(argv[2]);
  Catalog catalog(root,{},root/"policies/standing");const auto registry=YAML::LoadFile((root/"policies/registry.json").string());size_t total=0;
  std::cout<<std::setprecision(12);
  for(auto test:manifest){
   const auto name=test["name"].as<std::string>();Session session(catalog,test["spec"],registry,root/"assets/supermimic_g1/scene_29dof.xml",test["damp_s"].as<double>(1.));
   std::ifstream input(test["fixture"].as<std::string>());demand(bool(input),"cannot read fixture");std::string line;size_t count=0;double maxq=0,maxobs=0,maxgain=0;
   while(std::getline(input,line)){
    const auto f=YAML::Load(line);Sensors s;s.q=joints(f["q"]);s.dq=joints(f["dq"]);s.quat={f["quat"][0].as<double>(),f["quat"][1].as<double>(),f["quat"][2].as<double>(),f["quat"][3].as<double>()};for(int j=0;j<3;++j){s.gyro[j]=f["gyro"][j].as<double>();s.root_pos[j]=f["root_pos"][j].as<double>();s.root_vel_b[j]=f["root_vel"][j].as<double>();}
    if(count==0&&test["automatic"].as<bool>(false))session.startStanding(s);
    const auto command=f["command"].as<std::string>("");SessionOutput out;std::string error;
    try{out=session.step(s,command);}catch(const std::exception& e){error=e.what();}
    const auto where=name+" tick "+std::to_string(count)+" "+f["phase"].as<std::string>("error");
    if(f["error"]){demand(!error.empty(),where+": expected exception");++count;continue;}
    demand(error.empty(),where+": unexpected exception "+error);
    demand(out.phase==f["out_phase"].as<std::string>(),where+": output phase "+out.phase);
    demand(session.phase()==f["phase"].as<std::string>(),where+": next phase "+session.phase());
    demand(out.frame==f["frame"].as<long>(),where+": reference frame differs");
    demand(session.policyTicks()==f["policy_ticks"].as<size_t>(),where+": policy count differs");
    demand(session.activeName()==f["active"].as<std::string>(),where+": actor differs "+session.activeName());
    demand(session.pendingName()==f["pending"].as<std::string>(),where+": pending action differs");
    demand(session.holdTicks()==f["hold_ticks"].as<size_t>(),where+": hold count differs");
    demand(session.finished()==f["finished"].as<bool>(),where+": finished differs");
    demand(session.completedMotions().size()==f["completed"].size(),where+": completion count differs");
    for(size_t j=0;j<session.completedMotions().size();++j){auto x=f["completed"][j];demand(session.completedMotions()[j].policy==x["policy"].as<std::string>()&&session.completedMotions()[j].frames==x["frames"].as<size_t>(),where+": completion differs");}
    const double qerror=(out.q-joints(f["target"])).cwiseAbs().maxCoeff();maxq=std::max(maxq,qerror);demand(qerror<test["q_tolerance"].as<double>(.002),where+": target mismatch "+std::to_string(qerror));
    const double gain=std::max((out.kp-joints(f["kp"])).cwiseAbs().maxCoeff(),(out.kd-joints(f["kd"])).cwiseAbs().maxCoeff());maxgain=std::max(maxgain,gain);demand(gain<2e-5,where+": gains differ");
    if(f["observation"]){demand(out.observation.size()==f["observation"].size(),where+": obs width differs");for(size_t j=0;j<out.observation.size();++j){auto x=f["observation"][j];if(x.IsNull()){demand(std::isnan(out.observation[j]),where+": missing NaN padding");continue;}const double err=std::abs(out.observation[j]-x.as<double>());maxobs=std::max(maxobs,err);demand(err<test["obs_tolerance"].as<double>(.005),where+": observation mismatch "+std::to_string(err));}}
    if(f["gate"]){const auto& gate=session.gateResult();auto g=f["gate"];demand(gate.ready==g["ready"].as<bool>()&&gate.stable_ticks==g["stable_ticks"].as<size_t>(),where+": ready gate differs");for(auto metric:g["metrics"]){auto key=metric.first.as<std::string>();demand(gate.metrics.count(key)&&std::abs(gate.metrics.at(key)-metric.second.as<double>())<1e-7,where+": gate metric differs "+key);}}
    ++count;
   }
   total+=count;std::cout<<"{\"case\":\""<<name<<"\",\"ticks\":"<<count<<",\"max_target_error\":"<<maxq<<",\"max_observation_error\":"<<maxobs<<",\"max_gain_error\":"<<maxgain<<"}\n";
  }
  std::cout<<"Compared "<<total<<" real-actor session ticks against original Python\n";
 }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
}
