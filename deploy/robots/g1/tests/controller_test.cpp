#include "control/Controller.h"
#include <iostream>
#include <stdexcept>
using namespace g1control;
void check(bool b,const char* m){if(!b)throw std::runtime_error(m);}
class Fake:public Policy {
public:
 PolicyInfo i;size_t f=0;int begins=0;bool bad=false;
 Fake(std::string n,bool continuous,bool hold=false){i.name=n;i.continuous=continuous;i.hold_final=hold;i.frames=4;i.kp.setConstant(30);i.kd.setConstant(2);}
 const PolicyInfo& info()const override{return i;}
 void begin(const Sensors&)override{f=0;++begins;}
 PolicyOutput infer(const Sensors&)override{if(bad)throw std::runtime_error("inference failure");PolicyOutput o;o.q=i.entry_q;return o;}
 void advance()override{++f;}size_t frame()const override{return f;}
};
int main(){try{
 Catalog catalog;auto vel=std::make_unique<Fake>("Velocity",true);auto* vp=vel.get();catalog.add(std::move(vel),{"velocity"});
 catalog.add(std::make_unique<Fake>("Balance",true));catalog.add(std::make_unique<Fake>("Motion",false));catalog.add(std::make_unique<Fake>("Hold",false,true));
 YAML::Node cfg;for(int j=0;j<29;j++){cfg["FSM"]["FixStand"]["qs"][1].push_back(0.);cfg["FSM"]["FixStand"]["kp"].push_back(40.);cfg["FSM"]["FixStand"]["kd"].push_back(2.);}cfg["FSM"]["FixStand"]["ts"].push_back(0.);cfg["FSM"]["FixStand"]["ts"].push_back(2.);
 Controller c(catalog,cfg);Sensors s;
 check(c.request("rl",s).find("first")!=std::string::npos,"RL bypassed Stand");
 c.request("stand",s);c.request("rl",s);check(c.state()==Controller::State::Stand,"unfinished Stand bypassed");
 for(int j=0;j<100;j++)c.tick(s);check(c.standReady(),"stand not ready");
 c.request("rl velocity",s);for(int j=0;j<30;j++)c.tick(s);check(c.status()=="RL/Active/Velocity","velocity not active");check(vp->begins>=2,"policy reset missing");
 try{c.request("policy absent",s);throw std::logic_error("unknown accepted");}catch(const std::runtime_error&){}check(c.status()=="RL/Active/Velocity","unknown changed state");
 c.request("policy Motion",s);bool saw_motion=false;for(int j=0;j<100;j++){c.tick(s);saw_motion|=c.status()=="RL/Active/Motion";}check(saw_motion,"nested policy switch missing");check(c.status()=="RL/Active/Balance","finite motion did not return to Balance");
 c.request("policy Hold",s);for(int j=0;j<65;j++)c.tick(s);check(c.status()=="RL/Hold/Hold","hold final not active");
 c.request("stand",s);check(c.state()==Controller::State::Stand,"RL to Stand failed");c.request("passive",s);auto out=c.tick(s);check(out.kp.isZero()&&(out.kd.array()==3).all(),"Passive retained actuation gains");
 c.request("stand",s);for(int j=0;j<100;j++)c.tick(s);c.request("rl",s);for(int j=0;j<25;j++)c.tick(s);vp->bad=true;c.tick(s);check(c.state()==Controller::State::Passive&&!c.error().empty(),"fault did not latch Passive");
 c.request("stand",s);check(c.state()==Controller::State::Passive,"fault restart accepted");c.request("reset",s);vp->bad=false;c.request("stand",s);check(c.state()==Controller::State::Stand,"explicit reset failed");
 s.q[0]=std::nan("");c.tick(s);check(c.state()==Controller::State::Passive,"NaN not rejected");
 std::cout<<"hierarchy: parent transitions, internal switching, completion, hold, reset and failure handling passed\n";
 return 0;}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
