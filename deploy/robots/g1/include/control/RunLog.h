#pragma once
#include "control/Session.h"
#include "control/Robot.h"
#include <jsoncpp/json/json.h>
namespace g1control {
Json::Value jsonNode(const YAML::Node& node);
void writeJson(const std::filesystem::path& path,const Json::Value& value);
class RunLog {
public:
    explicit RunLog(std::filesystem::path output);
    void record(const SessionOutput& out,const Sensors& before,const Sensors& after,double sim_time,const RobotSnapshot* snapshot=nullptr);
    Json::Value metrics() const;
    Json::Value holdMetrics(const Joints& reference) const;
    void finish(Json::Value summary);
    const std::map<std::string,size_t>& counts() const {return counts_;}
    size_t count(const std::string& phase) const;
    size_t size() const {return rows_.size();}
    const std::filesystem::path& directory() const{return directory_;}
private:
    struct Row {SessionOutput output;Sensors before,after;double time=0,send=0;RobotSnapshot snapshot;};
    std::vector<Row> rows_;
    std::filesystem::path directory_;
    std::map<std::string,size_t> counts_;
};
// Same motion-relative screening as the original Python simulator.
class SimulationScreen {
public:
    std::string check(const Sensors&,const SessionOutput&,const Policy&);
private:
    const Policy* policy_=nullptr;
    size_t return_index_=0;
};
}
