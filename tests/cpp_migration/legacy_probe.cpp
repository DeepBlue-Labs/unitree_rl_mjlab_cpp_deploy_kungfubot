// Deterministic offline probe: no DDS initialization or robot commands.
#include "control/Policy.h"
#include <cmath>
#include <iomanip>
#include <iostream>
#include <stdexcept>

using namespace g1control;
template<class T> void array(const T& values) {
    std::cout << '[';
    for(size_t i=0;i<static_cast<size_t>(values.size());++i) { if(i) std::cout << ','; std::cout << values[i]; }
    std::cout << ']';
}
Sensors sample(const PolicyInfo& info,int step) {
    Sensors s;
    for(int j=0;j<29;++j) {
        s.q[j]=info.entry_q[j]+.08*std::sin(.17*j+.03*step);
        s.dq[j]=.15*std::cos(.11*j+.07*step);
    }
    s.quat=Eigen::AngleAxisd(.47+.002*step,Eigen::Vector3d::UnitZ())
          *Eigen::AngleAxisd(-.09+.001*step,Eigen::Vector3d::UnitY())
          *Eigen::AngleAxisd(.12-.003*step,Eigen::Vector3d::UnitX());
    s.gyro={.1+.01*step,-.2+.005*step,.03-.004*step};
    s.velocity_command={step%3==0 ? 1.3 : .4,step%3==1 ? -.9 : .2,step%3==2 ? 1.4 : -.3};
    return s;
}
int main(int argc,char** argv) {
    try {
        if(argc<4) throw std::runtime_error("usage: legacy_probe ROOT SPEC_YAML STEPS");
        const auto spec=YAML::LoadFile(argv[2]);
        auto policy=makeLegacyPolicy(spec,argv[1]);
        const int steps=std::stoi(argv[3]);
        std::cout << std::setprecision(17);
        for(int episode=0;episode<2;++episode) {
            policy->begin(sample(policy->info(),0));
            for(int i=0;i<steps;++i) {
                auto s=sample(policy->info(),i); auto out=policy->infer(s);
                if(out.frame!=static_cast<size_t>(i) || policy->frame()!=out.frame) throw std::runtime_error("infer advanced frame");
                std::cout << "{\"episode\":" << episode << ",\"frame\":" << out.frame << ",\"total_frames\":" << policy->info().frames << ",\"dt\":" << policy->info().dt << ",\"q\":"; array(s.q);
                std::cout << ",\"dq\":"; array(s.dq);
                std::cout << ",\"quat\":[" << s.quat.w() << ',' << s.quat.x() << ',' << s.quat.y() << ',' << s.quat.z() << ']';
                std::cout << ",\"gyro\":"; array(s.gyro);
                std::cout << ",\"command\":"; array(s.velocity_command);
                std::cout << ",\"observation\":"; array(out.observation);
                std::cout << ",\"goal\":"; array(out.goal);
                std::cout << ",\"action\":"; array(out.action);
                std::cout << ",\"target\":"; array(out.q);
                std::cout << ",\"height\":" << policy->referenceHeight(i) << ",\"gravity\":"; array(policy->referenceGravity(i));
                std::cout << "}\n";
                policy->advance();
            }
        }
    } catch(const std::exception& e) { std::cerr << e.what() << '\n'; return 1; }
}
