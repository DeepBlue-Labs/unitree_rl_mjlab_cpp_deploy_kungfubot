#include "control/Controller.h"
#include "control/Robot.h"
#include "control/Registry.h"
#include "control/RunLog.h"
#include "control/VideoRecorder.h"
#include "control/SimulationInput.h"
#include <boost/program_options.hpp>
#include <atomic>
#include <chrono>
#include <csignal>
#include <fstream>
#include <sstream>
#include <iostream>
#include <iomanip>
#include <thread>
#include <mutex>
#include <deque>
#include <fcntl.h>
#include <unistd.h>
#ifdef G1_VIEWER
#include "control/Viewer.h"
#endif
using namespace g1control;
using Clock=std::chrono::steady_clock;
namespace {
volatile std::sig_atomic_t interrupted=0;
void signalHandler(int) {interrupted=1;}
class Terminal {
    int flags_=-1;std::string pending_;bool closed_=false;
public:
    Terminal(){flags_=fcntl(STDIN_FILENO,F_GETFL,0);if(flags_>=0)fcntl(STDIN_FILENO,F_SETFL,flags_|O_NONBLOCK);}
    ~Terminal(){if(flags_>=0)fcntl(STDIN_FILENO,F_SETFL,flags_);}
    bool closed() const {return closed_;}
    std::vector<std::string> lines(){char b[4096];ssize_t n=-1;if(!closed_)while((n=::read(STDIN_FILENO,b,sizeof(b)))>0)pending_.append(b,n);if(n==0)closed_=true;std::vector<std::string> out;size_t end;while((end=pending_.find('\n'))!=std::string::npos){out.push_back(pending_.substr(0,end));pending_.erase(0,end+1);}if(closed_&&!pending_.empty()){out.push_back(pending_);pending_.clear();}return out;}
};
struct SharedCommand {std::mutex mutex;Command value;Clock::time_point updated=Clock::now();bool stop=false,publish=false;std::string fault;};
void update(SharedCommand& sh,const Command& c){std::lock_guard<std::mutex> l(sh.mutex);sh.value=c;sh.updated=Clock::now();}
int ticks(double seconds){const double n=seconds/.02;if(!std::isfinite(n)||n<=0||std::abs(n-std::round(n))>1e-7)throw std::runtime_error("duration must be a positive integer number of 20 ms ticks");return std::lround(n);}
std::string timestamp(){const auto now=std::chrono::system_clock::now();const auto t=std::chrono::system_clock::to_time_t(now);std::ostringstream s;s<<std::put_time(std::localtime(&t),"%Y%m%d_%H%M%S")<<'_'<<std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch()).count()%1000000;return s.str();}
Json::Value sessionSummary(const Session* s){Json::Value j;if(!s)return j;j["policy_ticks"]=Json::UInt64(s->policyTicks());j["expected_policy_ticks"]=Json::UInt64(s->expectedPolicyTicks());j["motion_complete"]=s->policyTicks()==s->expectedPolicyTicks();j["active_policy"]=s->activeName();j["last_motion"]=s->lastMotionName();j["entry_pose_rms"]=s->entryPoseRms()?Json::Value(*s->entryPoseRms()):Json::Value();j["completed_motions"]=Json::Value(Json::arrayValue);for(auto& m:s->completedMotions()){Json::Value v;v["policy"]=m.policy;v["frames"]=Json::UInt64(m.frames);j["completed_motions"].append(v);}j["gate"]["ready"]=s->gateResult().ready;j["gate"]["stable_ticks"]=Json::UInt64(s->gateResult().stable_ticks);j["gate"]["reasons"]=Json::Value(Json::arrayValue);for(auto& r:s->gateResult().reasons)j["gate"]["reasons"].append(r);for(auto& [k,v]:s->gateResult().metrics)j["gate"]["metrics"][k]=v;return j;}
}
int main(int argc,char** argv) {
 try {
    namespace po=boost::program_options;
    po::options_description options("Native C++ G1 deployment");
    options.add_options()("help,h","show help")("root",po::value<std::string>()->default_value(G1_DEPLOY_ROOT),"deployment root")
      ("mode",po::value<std::string>()->default_value("sim"),"sim (automatic) | sim2sim (interactive) | readonly | real | official-stand")
      ("network,n",po::value<std::string>(),"robot interface (alias of --iface)")("iface",po::value<std::string>(),"override site network interface")("domain",po::value<int>(),"DDS domain")
      ("registry",po::value<std::string>(),"policy registry JSON/YAML")("site",po::value<std::string>(),"site JSON/YAML")
      ("list","grouped policy list; no DDS")("inspect","validate selected/all policies and print artifacts; no DDS")
      ("headless","disable interactive viewer")("fast","simulation without wall-clock pacing")("no-support","disable simulation hoist")
      ("steps",po::value<int>()->default_value(0),"readonly samples / loopback or local test limit")("script",po::value<std::string>(),"local simulation YAML command timeline")
      ("policy",po::value<std::string>()->default_value("Velocity"),"policy name or alias")("trace",po::value<std::string>(),"additional compact CSV trace")
      ("output",po::value<std::string>(),"new run directory (must not exist)")("render","save automatic simulation MP4")("push","automatic simulation velocity pushes")
      ("pre-s",po::value<double>()->default_value(5.),"automatic standing before motion")("post-s",po::value<double>()->default_value(10.),"automatic standing/hold after motion")
      ("auto-start","positive-domain loopback DDS only")("sim-band-file",po::value<std::string>(),"loopback elastic-support flag")
      ("max-state-age-s",po::value<double>(),"override site state age")("watchdog-ms",po::value<double>(),"override command watchdog")
      ("max-loop-lateness-s",po::value<double>(),"override control deadline")("stop-temperature-c",po::value<double>(),"override temperature stop")("damp-s",po::value<double>(),"override damping time");
    po::variables_map args;po::store(po::parse_command_line(argc,argv,options),args);po::notify(args);
    if(args.count("help")){std::cout<<options<<'\n';return 0;}
    const std::filesystem::path root=std::filesystem::canonical(args["root"].as<std::string>());
    const auto mode=args["mode"].as<std::string>();const bool sim=mode=="sim"||mode=="sim2sim", automatic=mode=="sim"&&!args.count("script");
    if(!sim&&mode!="real"&&mode!="readonly"&&mode!="official-stand")throw std::runtime_error("unknown mode: "+mode);
    auto path=[&](const std::string& p){return std::filesystem::path(p).is_absolute()?std::filesystem::path(p):root/p;};
    const auto registry_path=args.count("registry")?path(args["registry"].as<std::string>()):root/"policies/registry.json";
    const auto site_path=args.count("site")?path(args["site"].as<std::string>()):root/"site.json";
    const auto site=YAML::LoadFile(site_path.string());Registry registry(root,registry_path);
    if(args.count("list")){std::cout<<registry.formatPolicyList()<<"\nOriginal C++: Velocity (velocity), Mimic_Dance1_subject2 (dance); Balance is internal standing.\n";return 0;}
    if(args.count("network")&&args.count("iface"))throw std::runtime_error("use either --iface or --network");
    const auto iface=args.count("iface")?args["iface"].as<std::string>():args.count("network")?args["network"].as<std::string>():site["iface"].as<std::string>();
    const int domain=args.count("domain")?args["domain"].as<int>():site["domain"].as<int>();const int steps=args["steps"].as<int>();
    RobotOptions ro;
    auto setting=[&](const char* option,const char* key,double& target){target=args.count(option)?args[option].as<double>():site[key].as<double>();if(!std::isfinite(target)||target<=0)throw std::runtime_error(std::string("invalid setting: ")+key);};
    setting("max-state-age-s","max_state_age_s",ro.max_state_age_s);setting("watchdog-ms","watchdog_ms",ro.watchdog_ms);setting("max-loop-lateness-s","max_loop_lateness_s",ro.max_loop_lateness_s);setting("stop-temperature-c","stop_temperature_c",ro.stop_temperature_c);setting("damp-s","damp_s",ro.damp_s);
    const int pre=ticks(args["pre-s"].as<double>()),post=ticks(args["post-s"].as<double>());ticks(ro.damp_s);
    if(!args["steps"].defaulted()&&steps<=0)throw std::runtime_error("explicit --steps must be positive");
    if(domain<0||steps<0)throw std::runtime_error("domain/steps must be nonnegative");
    if((args.count("render")||args.count("push"))&&!automatic)throw std::runtime_error("--render/--push require automatic --mode sim");
    if(args.count("script")&&!sim)throw std::runtime_error("--script is local simulation only");
    if(mode=="real"&&domain==0&&(args.count("auto-start")||steps||args.count("sim-band-file")))throw std::runtime_error("robot execution must be interactive; automatic/bounded runs are loopback-only");
    if((args.count("auto-start")||args.count("sim-band-file"))&&(mode!="real"||iface!="lo"||domain<=0))throw std::runtime_error("DDS simulation options require --mode real --iface lo --domain > 0");
    if(!sim&&!args.count("inspect")&&((domain==0&&iface=="lo")||(domain>0&&iface!="lo")))throw std::runtime_error("use wired domain 0 for robot; lo with positive domain for DDS tests");
    if(mode=="official-stand"&&!args.count("inspect")){Robot::officialStand(iface,domain);return 0;}
    const auto model=path(site["model"].as<std::string>()),standing=path(site["standing_policy"].as<std::string>());
    Catalog catalog(root,registry_path,standing);catalog.warmup();
    auto& selected=catalog.get(args["policy"].as<std::string>(),true);auto& balance=catalog.get("Balance");
    YAML::Node spec;for(auto item:registry.data()["policies"])if(item["name"].as<std::string>()==selected.info().name)spec=registry.resolve(selected.info().name);
    Json::Value manifest;
    if(spec.IsMap())manifest=jsonNode(registry.inspect(selected.info().name));
    else{manifest["policy"]["name"]=selected.info().name;manifest["frame_count"]=Json::UInt64(selected.info().frames);manifest["control_dt"]=selected.info().dt;}
    manifest["standing_sha256"]["params/deploy.yaml"]=fileSha256(standing/"params/deploy.yaml");manifest["standing_sha256"]["exported/policy.onnx"]=fileSha256(standing/"exported/policy.onnx");
    manifest["runtime"]=jsonNode(site);manifest["runtime"]["mode"]=mode;manifest["runtime"]["iface"]=iface;manifest["runtime"]["domain"]=domain;manifest["runtime"]["model"]=model.string();manifest["runtime"]["standing_policy"]=standing.string();manifest["runtime"]["registry"]=registry_path.string();manifest["runtime"]["site"]=site_path.string();manifest["runtime"]["pre_s"]=pre*.02;manifest["runtime"]["post_s"]=post*.02;manifest["runtime"]["steps"]=steps;manifest["runtime"]["max_state_age_s"]=ro.max_state_age_s;manifest["runtime"]["watchdog_ms"]=ro.watchdog_ms;manifest["runtime"]["max_loop_lateness_s"]=ro.max_loop_lateness_s;manifest["runtime"]["stop_temperature_c"]=ro.stop_temperature_c;manifest["runtime"]["damp_s"]=ro.damp_s;
    for(auto flag:{"render","push","headless","fast","no-support","auto-start"})manifest["runtime"][flag]=bool(args.count(flag));
    for(auto name:{"script","trace","sim-band-file"})if(args.count(name))manifest["runtime"][name]=args[name].as<std::string>();
    manifest["runtime"]["policy"]=selected.info().name;
    for(int i=0;i<argc;i++)manifest["argv"].append(argv[i]);
    if(args.count("inspect")){
        if(args["policy"].defaulted())for(auto& [name,p]:catalog.policies()){
            manifest["actors"][name]["kind"]=p->info().kind;
            manifest["actors"][name]["frames"]=Json::UInt64(p->info().frames);
            manifest["actors"][name]["control_dt"]=p->info().dt;
        }
        std::cout<<manifest<<'\n';return 0;
    }
    if(automatic&&!spec.IsMap())throw std::runtime_error("automatic sim requires a registered finite motion; use --mode sim2sim for original C++ Velocity/dance");
    const auto output=args.count("output")?std::filesystem::absolute(args["output"].as<std::string>()):root/"results"/(selected.info().name+"_"+mode+"_"+timestamp());
    RunLog log(output);manifest["runtime"]["output"]=output.string();writeJson(output/"run.json",manifest);
    if(manifest.isMember("session_artifacts")){Json::Value j;j["artifact_sha256"]=manifest["session_artifacts"];j["fighting"]=jsonNode(spec["ready_hold"]["fighting"]);writeJson(output/"session.json",j);}
    FlatGroundEstimator estimator(model,balance.info().joint_names);UpperBodyGravity gravity(model,balance.info().joint_names,balance.info().effort);
    auto config=YAML::LoadFile((root/"deploy/robots/g1/config/config.yaml").string());
    Controller ctrl(catalog,config,[&](const Sensors& s,const PolicyInfo& i){gravity.setEffort(i.effort);return gravity.offset(s,i.kp);});ctrl.configureSessions(registry.data(),model,ro.damp_s);
    SimulationInput input(catalog,selected.info().name);std::signal(SIGINT,signalHandler);std::signal(SIGTERM,signalHandler);Terminal terminal;
    std::ofstream trace;if(args.count("trace")){trace.open(args["trace"].as<std::string>());if(!trace)throw std::runtime_error("cannot open trace");trace<<"tick,time,state,root_z,fault,policy_frame,paused,selected\n";}
    std::map<int,std::vector<std::string>> timeline;if(args.count("script"))for(auto e:YAML::LoadFile(args["script"].as<std::string>()))timeline[e["tick"].as<int>()].push_back(e["command"].as<std::string>());
    std::unique_ptr<MujocoPlant> plant;std::unique_ptr<Robot> robot;std::unique_ptr<VideoRecorder> video;
#ifdef G1_VIEWER
    std::unique_ptr<Viewer> viewer;
#endif
    Sensors sensors;SharedCommand shared;std::thread publisher;bool had_fault=false,quit=false,stop_requested=false,readonly_begun=false,band_released=false;
    std::deque<std::string> pending_commands;size_t push_index=0;
    std::string previous,last_message,reason="";SimulationScreen screen;auto started=Clock::now(),deadline=started;
    std::cout<<"Policy: "<<selected.info().name<<" | mode: "<<mode<<" | Logs: "<<output<<std::endl;
    try {
        if(sim){plant=std::make_unique<MujocoPlant>(model,balance.info().joint_names);sensors.root_pos.z()=automatic?.78:.80;sensors.q=automatic?balance.info().entry_q:Joints::Zero();plant->reset(sensors);if(args.count("render"))video=std::make_unique<VideoRecorder>(*plant,output/"handoff.mp4");
#ifdef G1_VIEWER
            if(!automatic&&!args.count("headless"))viewer=std::make_unique<Viewer>(*plant);
#else
            if(!automatic&&!args.count("headless"))throw std::runtime_error("viewer unavailable; rebuild with GLFW or use --headless");
#endif
        }else{robot=std::make_unique<Robot>(iface,domain,mode=="readonly",ro);sensors=estimator.estimate(robot->read());if(mode=="real")robot->requireQuiet();}
        if(spec.IsMap()&&mode!="readonly")ctrl.selectSession(selected.info().name,sensors,automatic);
        input.nativeSession(ctrl.session()!=nullptr);
        if(mode=="real")publisher=std::thread([&]{auto next=Clock::now();while(true){Command command;bool active=false,expired=false;{std::lock_guard<std::mutex> l(shared.mutex);if(shared.stop)break;active=shared.publish;command=shared.value;expired=active&&std::chrono::duration<double,std::milli>(Clock::now()-shared.updated).count()>ro.watchdog_ms;if(expired&&shared.fault.empty())shared.fault="command watchdog expired";if(!shared.fault.empty()){command.kp.setZero();command.kd.setConstant(3);}}if(active){if(expired)robot->latchWatchdog();try{robot->write(command);}catch(const std::exception& e){std::lock_guard<std::mutex> l(shared.mutex);if(shared.fault.empty())shared.fault=e.what();}}next+=std::chrono::milliseconds(2);std::this_thread::sleep_until(next);if(Clock::now()-next>std::chrono::milliseconds(10))next=Clock::now();}});
        if(sim&&!automatic)std::cout<<"Interactive C++ sim2sim ready. No robot/DDS connection.\n";
        std::cout<<"TAKEOVER -> STAND_PRE -> START; session actions / NEXT <name>; STOP.\nstand = original C++ FixStand; passive = damping; rl/policy <name> = RL controller.\n";
        if(args.count("auto-start"))ctrl.request("TAKEOVER",sensors);
        deadline=Clock::now();
        const size_t auto_limit=automatic?size_t(pre+ticks(spec["prepare_s"].as<double>())+selected.info().frames+(spec["ready_hold"]?0:ticks(spec["return_s"].as<double>()))+post+(spec["ready_hold"]&&spec["ready_hold"]["fighting"]?ticks(spec["ready_hold"]["fighting"]["acquire_timeout_s"].as<double>(10.)+1.):0)):0;
        for(size_t tick=0;!quit;++tick){
            if(interrupted){reason="Ctrl-C";stop_requested=true;interrupted=0;ctrl.request("stop",sensors);if(!ctrl.session())quit=true;}
            if((steps&&(mode=="real"?log.size():tick)>=size_t(steps))||(automatic&&tick>=auto_limit)){reason=automatic?"sequence complete":"bounded run complete";break;}
            if(plant&&args.count("push")&&(tick==150||tick==size_t(pre+ticks(spec["prepare_s"].as<double>())+30)||tick==size_t(pre+ticks(spec["prepare_s"].as<double>())+selected.info().frames+150))){
                // Exact NumPy default_rng(2026) impulses used by the Python oracle.
                static const Eigen::Vector3d linear[]={Eigen::Vector3d(-0.12842607452982552,0.0559652662860618,-0.0),Eigen::Vector3d(0.16205753467086959,-0.12905872327078055,0.0),Eigen::Vector3d(0.05434832165786052,0.10109283883163467,0.0)};
                static const Eigen::Vector3d angular[]={Eigen::Vector3d(-0.051799789156780796,-0.05803306627613952,0.116207298341306),Eigen::Vector3d(-0.0806788930577723,0.18678488006495625,0.1679400642149113),Eigen::Vector3d(0.13035810335022802,-0.020647781839958718,-0.06447501867449526)};
                const size_t i=push_index++;
                plant->applyVelocityPush(linear[i],angular[i]);
            }
            const auto snapshot=robot?robot->readSnapshot():RobotSnapshot{};
            sensors=plant?plant->sensor():estimator.estimate(snapshot.sensors);
            if(mode=="readonly"&&robot->operatorStopRequested()){reason="L2+B pressed";break;}
            if(mode=="real"){
                if(robot->operatorStopRequested()){reason="L2+B pressed";stop_requested=true;ctrl.request("stop",sensors);if(!ctrl.session())quit=true;}
                {std::lock_guard<std::mutex> l(shared.mutex);if(!shared.fault.empty())throw std::runtime_error(shared.fault);}
                if(Clock::now()-deadline>std::chrono::duration<double>(ro.max_loop_lateness_s))throw std::runtime_error("policy loop missed its deadline");
            }
            auto commands=automatic||mode=="readonly"?std::vector<std::string>{}:terminal.lines();
#ifdef G1_VIEWER
            if(viewer){auto events=viewer->pollCommands();commands.insert(commands.end(),events.begin(),events.end());}
#endif
            if(mode!="readonly"&&!automatic&&terminal.closed()&&!args.count("script")&&!steps&&!args.count("auto-start"))commands.push_back("STOP");
            if(timeline.count(tick))commands.insert(commands.end(),timeline[tick].begin(),timeline[tick].end());
            if(robot&&mode!="readonly"){for(std::string remote=robot->remoteCommand();!remote.empty();remote=robot->remoteCommand())commands.push_back(remote);}
            if(automatic&&tick==size_t(pre))commands.push_back("START");
            if(args.count("auto-start")&&ctrl.session()&&ctrl.session()->phase()=="STAND_PRE"&&log.count("STAND_PRE")>=size_t(pre))commands.push_back("START");
            if(args.count("sim-band-file")&&!band_released&&log.count("STAND_PRE")>=100){std::filesystem::remove(args["sim-band-file"].as<std::string>());band_released=true;}
            for(auto& c:commands)pending_commands.push_back(c);
            commands.assign(pending_commands.begin(),pending_commands.end());
            bool emergency=false;for(auto& c:commands)if(SimulationInput::verb(c)=="stop"||SimulationInput::verb(c)=="passive"){emergency=true;break;}
            if(emergency){pending_commands.clear();bool stop=false;for(auto& c:commands)stop|=SimulationInput::verb(c)=="stop";if(stop){input.consume("STOP");ctrl.request("stop",sensors);reason="STOP requested";stop_requested=true;if(!ctrl.session())quit=true;}else ctrl.request("passive",sensors);}
            else {
                commands.clear();if(!pending_commands.empty()){commands.push_back(pending_commands.front());pending_commands.pop_front();}
                for(auto& c:commands){try{
                input.nativeSession(ctrl.session()!=nullptr);auto action=(ctrl.session()&&SimulationInput::verb(c)=="help")?SimulationAction{"help",{}}:sim?input.consume(c):SimulationAction{c,{}};
                if(!action.message.empty()){last_message=action.message;std::cout<<last_message<<'\n';}
                if(!action.command.empty()){
                    if(mode=="real"&&SimulationInput::verb(action.command)=="takeover"&&ctrl.session()&&ctrl.session()->phase()=="WAIT_TAKEOVER"){robot->requireQuiet();deadline=Clock::now();}
                    last_message=ctrl.request(action.command,sensors);std::cout<<last_message<<'\n';
                }
            }catch(const std::exception& e){std::cerr<<"Rejected command: "<<e.what()<<'\n';last_message=e.what();}}
            }
            const bool paused=sim&&input.paused();Command command;SessionOutput out;
            if(mode=="readonly"){
                if(!readonly_begun){selected.begin(sensors);readonly_begun=true;}
                auto result=selected.infer(sensors);out.phase="READONLY";out.active_policy=selected.info().name;out.frame=selected.frame();out.q=result.q;out.kp=selected.info().kp;out.kd=selected.info().kd;out.effort=selected.info().effort;out.observation=result.observation;if(selected.info().kind=="reference")out.observation.insert(out.observation.end(),result.goal.begin(),result.goal.end());out.raw_action=result.action;selected.advance();
            }else{
                command=paused?ctrl.output():ctrl.tick(sensors);
                if(ctrl.session())out=ctrl.sessionOutput();else{out.q=command.q;out.kp=command.kp;out.kd=command.kd;out.effort=command.effort;out.phase=ctrl.status();out.frame=ctrl.policyFrame();out.active_policy=input.selected();out.support=ctrl.state()!=Controller::State::RL;}
                if(!ctrl.error().empty()){had_fault=true;reason=ctrl.error();}
            }
            if(ctrl.status()!=previous){previous=ctrl.status();std::cout<<"tick "<<tick<<": "<<previous<<std::endl;}
            if(ctrl.session()&&!ctrl.session()->event().empty()&&!paused){last_message=ctrl.session()->event();std::cout<<last_message<<'\n';}
            if(trace)trace<<tick<<','<<(plant?plant->time():tick*.02)<<','<<ctrl.status()<<','<<sensors.root_pos.z()<<','<<ctrl.error()<<','<<ctrl.policyFrame()<<','<<paused<<','<<input.selected()<<'\n';
            Sensors after=sensors;
            if(!paused){
                if(plant){
#ifdef G1_VIEWER
                    if(viewer)viewer->applyPerturbation(false);
#endif
                    after=plant->step(out.q,out.kp,out.kd,out.effort,!automatic&&!args.count("no-support")&&out.support);
                    if(video&&tick%2==0)video->capture();
                    if(ctrl.session()){auto fail=screen.check(after,out,ctrl.session()->activePolicy());if(!fail.empty()&&out.phase!="DAMP"){had_fault=true;reason=fail;ctrl.fault(fail);if(automatic)quit=true;}}
                }else if(mode=="real"){
                    if(Clock::now()-deadline>std::chrono::duration<double>(ro.max_loop_lateness_s))throw std::runtime_error("policy inference missed its deadline");
                    update(shared,command);if(out.phase!="WAIT_TAKEOVER"&&out.phase!="DAMP"){std::lock_guard<std::mutex> l(shared.mutex);shared.publish=true;}
                }
                if(mode!="real"||(out.phase!="WAIT_TAKEOVER"&&out.phase!="TAKEOVER"))log.record(out,sensors,after,plant?plant->time():tick*.02,robot?&snapshot:nullptr);
            }
#ifdef G1_VIEWER
            if(viewer&&!viewer->draw(*plant,ctrl.status(),input.selected(),paused,last_message)){input.consume("STOP");ctrl.request("stop",sensors);stop_requested=true;reason="window closed";if(!ctrl.session())quit=true;}
#endif
            if(ctrl.finished())quit=true;
            if(mode=="readonly"&&tick+1>=size_t(steps?steps:100)){reason="read-only steps complete";quit=true;}
            if(automatic&&spec["ready_hold"]&&log.count("HOLD")>=size_t(post)){reason="sequence complete";quit=true;}
            std::cout.flush();
            deadline+=std::chrono::milliseconds(20);
            if(!sim||(!args.count("fast")&&!automatic)||(paused&&!args.count("script")))std::this_thread::sleep_until(deadline);
            deadline=std::max(deadline,Clock::now());
        }
    }catch(const std::exception& e){had_fault=true;reason=e.what();std::cerr<<reason<<'\n';ctrl.fault(reason);}
    if(publisher.joinable()){
        bool published;{std::lock_guard<std::mutex> l(shared.mutex);published=shared.publish;}
        if(published){Command damping;damping.q=sensors.q;for(int i=log.count("DAMP");i<ticks(ro.damp_s);i++){update(shared,damping);std::this_thread::sleep_for(std::chrono::milliseconds(20));}}
        {std::lock_guard<std::mutex> l(shared.mutex);shared.stop=true;}publisher.join();
    }
    if(video){try{video->finish();}catch(const std::exception& e){had_fault=true;reason=e.what();}}
    Json::Value summary=sessionSummary(ctrl.session());summary["policy"]=selected.info().name;summary["mode"]=mode;summary["interactive"]=!automatic&&mode!="readonly";summary["viewer"]=sim&&!automatic&&!args.count("headless");summary["domain"]=domain;summary["iface"]=iface;summary["error"]=had_fault;summary["stop_reason"]=reason.empty()?(ctrl.session()?ctrl.session()->stopReason():"finished"):reason;summary["hold_ticks"]=Json::UInt64(log.count("HOLD"));summary["standing_after_motion"]=log.count("STAND_POST")>0;summary["wall_seconds"]=std::chrono::duration<double>(Clock::now()-started).count();summary["robot_commands_published"]=Json::UInt64(robot?robot->status().published_commands:0);summary["push"]=bool(args.count("push"));summary["target_processing"]=selected.info().target_processing;
    if(robot){auto status=robot->status();summary["bridge_published_commands"]=Json::UInt64(status.published_commands);summary["bridge_publish_failures"]=Json::UInt64(status.publish_failures);summary["bridge_watchdog"]=status.watchdog_latched;summary["bridge_publisher_created"]=status.publisher_created;writeJson(output/"g1_bridge.json",summary);}
    if(automatic){auto m=log.metrics();const bool holding=bool(spec["ready_hold"]);summary["complete"]=!had_fault&&summary["motion_complete"].asBool()&&log.count(holding?"HOLD":"STAND_POST")==size_t(post);auto hold=log.holdMetrics(ctrl.session()->activePolicy().referenceJoints(ctrl.session()->activePolicy().info().frames-1));
        summary["hold_pose_rmse_last_1s"]=hold["hold_pose_rmse_last_1s"];
        summary["standing_recovered_last_1s"]=summary["complete"].asBool()&&m["standing_recovered_last_1s"].asBool();
        summary["hold_stable_last_1s"]=summary["complete"].asBool()&&m["hold_envelope_last_1s"].asBool()&&!hold["hold_pose_rmse_last_1s"].isNull()&&hold["hold_pose_rmse_last_1s"].asDouble()<.15&&(!spec["ready_hold"]["fighting"]||(m["hold_pitch_ptp_deg_last_1s"].asDouble()<.2&&m["hold_max_joint_speed_last_1s"].asDouble()<.1));
        summary["screen_passed"]=holding?summary["hold_stable_last_1s"].asBool():summary["standing_recovered_last_1s"].asBool();summary["completion_behavior"]=holding?"HOLD":"STAND_POST";summary["screen_criteria"]="motion-relative height/tilt, complete sequence and final recovery; not hardware qualification";}
    log.finish(summary);std::cout<<"Finished: "<<ctrl.status()<<"\nLogs: "<<output<<'\n';return had_fault||(automatic&&!summary["screen_passed"].asBool())?2:0;
 }catch(const std::exception& e){std::cerr<<"g1_deploy: "<<e.what()<<'\n';return 1;}
}
