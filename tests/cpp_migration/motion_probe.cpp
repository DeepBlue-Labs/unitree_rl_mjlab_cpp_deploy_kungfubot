// Offline actor probe: no DDS or robot commands are constructed.
#include "control/Policy.h"
#include <cnpy.h>
#include <iostream>
#include <stdexcept>
using namespace g1control;
int main(int argc,char** argv) {
    try {
        if(argc<5) throw std::runtime_error("usage: motion_probe ROOT POLICY SENSOR_NPZ OUTPUT_NPZ [STAGE]");
        const std::filesystem::path root=argv[1];const std::string name=argv[2];
        const auto registry=YAML::LoadFile((root/"policies/registry.json").string());
        YAML::Node spec;
        for(const auto& x:registry["policies"]) if(x["name"].as<std::string>()==name) spec=YAML::Clone(x);
        if(name=="PythonStanding") {spec["name"]=name;spec["adapter"]="balance";spec["bundle"]="policies/standing";}
        if(!spec) throw std::runtime_error("unknown policy "+name);
        spec["stage"]=argc>5 ? argv[5] : "ground";
        auto actor=name=="PythonStanding" ? makeLegacyPolicy(spec,root) : makeMotionPolicy(spec,root);
        const auto samples=cnpy::npz_load(argv[3],"states"),commands=cnpy::npz_load(argv[3],"commands");
        if(samples.word_size!=8 || samples.shape.size()!=2 || samples.shape[1]!=71 || commands.word_size!=8 || commands.shape.size()!=2 || commands.shape[0]!=samples.shape[0] || (commands.shape[1]!=3 && commands.shape[1]!=5))
            throw std::runtime_error("invalid fixture: states Nx71, commands Nx3/Nx5, float64 required");
        const bool transitions=commands.shape[1]==5;
        cnpy::NpyArray feedback;
        if(transitions) {feedback=cnpy::npz_load(argv[3],"feedback");if(feedback.word_size!=8 || feedback.shape!=std::vector<size_t>{samples.shape[0],29}) throw std::runtime_error("invalid feedback fixture");}
        std::vector<float> observations,goals;std::vector<double> actions,targets,reference_joints,reference_quaternions;
        size_t obs_width=0,goal_width=0;
        for(size_t i=0;i<samples.shape[0];++i) {
            const double* p=samples.data<double>()+i*71;const double* cmd=commands.data<double>()+i*commands.shape[1];
            Sensors s;s.q=Eigen::Map<const Joints>(p);s.dq=Eigen::Map<const Joints>(p+29);
            s.quat=Eigen::Quaterniond(p[58],p[59],p[60],p[61]);s.gyro=Eigen::Map<const Eigen::Vector3d>(p+62);
            s.root_pos=Eigen::Map<const Eigen::Vector3d>(p+65);s.root_vel_b=Eigen::Map<const Eigen::Vector3d>(p+68);
            if(i==0 || cmd[1]==1) actor->begin(s);
            else if(cmd[1]==2) actor->resetActionHistory();
            const auto requested=size_t(cmd[0]);
            if(transitions) {
                actor->seek(requested);
                if(cmd[2]==1) actor->holdReferenceFrame(size_t(cmd[3]));
                else if(cmd[2]==2) actor->holdReferenceFrame(std::nullopt);
                if(cmd[4]!=0) actor->feedbackTarget(Eigen::Map<const Joints>(feedback.data<double>()+i*29));
            } else {
                if(requested<actor->frame() || requested>=actor->info().frames) throw std::runtime_error("invalid frame command");
                while(actor->frame()<requested) actor->advance();
                if(cmd[2]!=0) actor->holdFinal();
            }
            const auto out=actor->infer(s);
            if(actor->frame()!=requested || out.frame!=requested) throw std::runtime_error("infer advanced frame");
            obs_width=out.observation.size();goal_width=out.goal.size();
            observations.insert(observations.end(),out.observation.begin(),out.observation.end());
            goals.insert(goals.end(),out.goal.begin(),out.goal.end());
            actions.insert(actions.end(),out.action.data(),out.action.data()+29);targets.insert(targets.end(),out.q.data(),out.q.data()+29);
            const auto ref=actor->referenceJoints(requested);const auto quat=actor->referenceQuaternion(requested);
            reference_joints.insert(reference_joints.end(),ref.data(),ref.data()+29);
            for(double x:{quat.w(),quat.x(),quat.y(),quat.z()}) reference_quaternions.push_back(x);
        }
        const size_t n=samples.shape[0];
        cnpy::npz_save(argv[4],"observation",observations.data(),{n,obs_width},"w");
        // zlib treats a null pointer as CRC initialization even at length zero.
        const float empty=0;
        cnpy::npz_save(argv[4],"goal",goals.empty() ? &empty : goals.data(),{n,goal_width},"a");
        cnpy::npz_save(argv[4],"action",actions.data(),{n,29},"a");
        cnpy::npz_save(argv[4],"target",targets.data(),{n,29},"a");
        cnpy::npz_save(argv[4],"reference_joints",reference_joints.data(),{n,29},"a");
        cnpy::npz_save(argv[4],"reference_quaternion",reference_quaternions.data(),{n,4},"a");
        std::cout<<name<<" "<<n<<" steps\n";
        return 0;
    } catch(const std::exception& error) { std::cerr<<error.what()<<"\n";return 1; }
}
