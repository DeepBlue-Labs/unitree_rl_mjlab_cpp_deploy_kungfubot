#include "control/Registry.h"
#include <iostream>
#include <stdexcept>
using namespace g1control;
int main(int argc,char** argv) {
    try {
        if(argc<2) throw std::runtime_error("usage: registry_probe ROOT [POLICY|--list|--self-test] [REGISTRY]");
        Registry registry(argv[1],argc>3 ? argv[3] : "");
        const std::string mode=argc>2 ? argv[2] : "entry_star";
        if(mode=="--list") {std::cout<<registry.formatPolicyList();return 0;}
        if(mode=="--self-test") {
            size_t count=0;
            for(const auto entry:registry.data()["policies"]) {
                const auto name=entry["name"].as<std::string>();auto spec=registry.resolve(name);
                for(const auto alias:entry["aliases"]) if(registry.resolve(alias.as<std::string>())["name"].as<std::string>()!=name) throw std::runtime_error("alias mismatch");
                auto actor=registry.load(spec);
                if(actor->info().name!=name || actor->info().observation_width==0) throw std::runtime_error("invalid loaded contract");
                spec["prepare_s"]=99.;
                if(registry.resolve(name)["prepare_s"].as<double>()==99.) throw std::runtime_error("resolve must return detached defaults");
                ++count;
            }
            bool rejected=false;try {registry.resolve("not_a_registered_policy");} catch(const std::exception&) {rejected=true;}
            if(!rejected) throw std::runtime_error("unknown policy accepted");
            std::cout<<"validated "<<count<<" policies, aliases, merged defaults and detached specs\n";return 0;
        }
        std::cout<<registry.inspect(mode)<<"\n";return 0;
    } catch(const std::exception& error) {std::cerr<<error.what()<<"\n";return 1;}
}
