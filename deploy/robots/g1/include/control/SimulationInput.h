#pragma once
#include "control/Controller.h"
#include <algorithm>
#include <cctype>
#include <sstream>
namespace g1control {
struct SimulationAction { std::string command, message; };
// Input/UI state is simulation-only; it never owns a robot or advances a policy.
class SimulationInput {
    Catalog& catalog_;
    std::vector<std::string> choices_;
    std::string selected_;
    bool paused_=false, native_session_=false, selected_by_ui_=false;
public:
    SimulationInput(Catalog& catalog,const std::string& selected):catalog_(catalog) {
        for(const auto& [name,policy]:catalog.policies()) if(name!="Balance") choices_.push_back(name);
        selected_=catalog_.get(selected,true).info().name;
    }
    static std::string verb(const std::string& text) {
        std::istringstream in(text);std::string word;in>>word;
        std::transform(word.begin(),word.end(),word.begin(),[](unsigned char c){return std::tolower(c);});return word;
    }
    const std::string& selected() const { return selected_; }
    bool paused() const { return paused_; }
    void nativeSession(bool enabled){native_session_=enabled;}
    SimulationAction consume(const std::string& text) {
        std::istringstream in(text);std::string op,name,extra;in>>op>>name>>extra;op=verb(op);
        if(op.empty())return {};
        if(op=="stop") {paused_=false;return {"stop",{}};}
        if(op=="passive")return {"passive",{}};
        if(!extra.empty())return {{},"Too many command arguments"};
        if(op=="pause") {paused_=!paused_;return {{},paused_?"Simulation paused":"Simulation resumed"};}
        if(op=="resume") {paused_=false;return {{},"Simulation resumed"};}
        if(op=="select_next"||op=="select_prev") {
            const auto found=std::find(choices_.begin(),choices_.end(),selected_);
            const auto at=found==choices_.end()?0:std::distance(choices_.begin(),found);
            const auto n=static_cast<int>(choices_.size());
            if(n){selected_=choices_[(at+(op=="select_next"?1:n-1))%n];selected_by_ui_=true;}
            return {{},"Selected: "+selected_+" (press S to run)"};
        }
        if(op=="select") {
            if(name.empty())return {{},"SELECT requires a policy name"};
            selected_=catalog_.get(name,true).info().name;selected_by_ui_=true;return {{},"Selected: "+selected_};
        }
        if(op=="help")return {{},"TAKEOVER/stand | START/rl [name] | NEXT/policy <name> | SELECT <name> | PAUSE | RESUME | passive | STOP | status | reset | list"};
        if(op=="list") {std::string list="Policies:";for(const auto& n:choices_)list+="\n  "+n;return {{},list};}
        const bool starts=op=="takeover"||op=="stand"||op=="fixstand"||op=="start"||op=="rl"||op=="next"||op=="policy";
        if(starts&&paused_)return {{},"Simulation is paused; use RESUME or Space before switching state/policy"};
        if(op=="takeover"){
            if(native_session_&&selected_by_ui_){selected_by_ui_=false;return {"TAKEOVER "+selected_,{}};}
            return {native_session_?"TAKEOVER":"stand",{}};
        }
        if(op=="stand"||op=="fixstand")return {"stand",{}};
        if(op=="start"||op=="rl") {
            if(!name.empty())selected_=catalog_.get(name,true).info().name;
            const bool selected=selected_by_ui_;selected_by_ui_=false;
            return {native_session_&&op=="start"&&name.empty()&&!selected?"START":"rl "+selected_,{}};
        }
        if(op=="next"||op=="policy") {
            if(name.empty())return {{},"NEXT/policy requires a policy name"};
            selected_=catalog_.get(name,true).info().name;selected_by_ui_=false;return {(native_session_&&op=="next"?"NEXT ":"policy ")+selected_,{}};
        }
        return {op+(name.empty()?"":" "+name),{}};
    }
};
}
