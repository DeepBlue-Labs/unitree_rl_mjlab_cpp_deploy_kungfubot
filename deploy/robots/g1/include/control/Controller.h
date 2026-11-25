#pragma once
#include "control/Physics.h"
#include "control/Session.h"
#include <map>
#include <functional>
namespace g1control {
struct Command {
    Joints q=Joints::Zero(), kp=Joints::Zero(), kd=Joints::Constant(3), effort=Joints::Constant(100);
};
class Catalog {
public:
    Catalog()=default;
    explicit Catalog(const std::filesystem::path& root, const std::filesystem::path& registry_path={}, const std::filesystem::path& standing_path={});
    void add(std::unique_ptr<Policy> policy, const std::vector<std::string>& aliases={});
    Policy& get(const std::string& name, bool ignore_case=false);
    const std::map<std::string,std::unique_ptr<Policy>>& policies() const { return policies_; }
    void warmup();
private:
    std::map<std::string,std::unique_ptr<Policy>> policies_;
    std::map<std::string,std::string> aliases_;
};
// All policy histories belong to their adapter. Only this 50 Hz controller owns
// the active policy and motor command. Transport never switches policies itself.
class Controller {
public:
    enum class State { Passive, Stand, RL };
    using Gravity = std::function<Joints(const Sensors&,const PolicyInfo&)>;
    Controller(Catalog& catalog, const YAML::Node& original_config, Gravity gravity={});
    void configureSessions(const YAML::Node& registry, const std::filesystem::path& model, double damp_s=1.);
    void selectSession(const std::string& name, const Sensors& state, bool automatic=false);
    const Session* session() const { return session_; }
    const SessionOutput& sessionOutput() const { return session_output_; }
    bool finished() const { return session_ && session_->finished(); }
    std::string request(const std::string& command, const Sensors& state);
    Command tick(const Sensors& state);
    void fault(const std::string& reason);
    State state() const { return state_; }
    size_t policyFrame() const { return session_?session_->activePolicy().frame():(active_?active_->frame():0); }
    bool standReady() const { return session_?(session_->phase()=="STAND_PRE"||session_->phase()=="STAND_POST"):(state_==State::Stand && phase_time_>=stand_s_); }
    std::string status() const;
    const std::string& error() const { return error_; }
    const Command& output() const { return output_; }
private:
    enum class Phase { Prepare, Active, Return };
    void prepare(Policy&,const Sensors&);
    void returning(Policy* next,const Sensors&);
    void passive();
    std::map<std::string,std::unique_ptr<Session>> sessions_;
    Session* session_=nullptr;
    SessionOutput session_output_;
    YAML::Node registry_;
    std::filesystem::path model_;
    double damp_s_=1.;
    std::string session_command_, session_after_return_;
    Catalog& catalog_;
    Gravity gravity_;
    State state_=State::Passive;
    Phase phase_=Phase::Active;
    Policy* active_=nullptr;
    Policy* pending_=nullptr;
    Command output_, start_;
    Joints stand_q_, stand_kp_, stand_kd_;
    double phase_time_=0,stand_s_=2, prepare_s_=.4,return_s_=.5,dt_=.02;
    bool holding_=false;
    std::string error_;
};
}
