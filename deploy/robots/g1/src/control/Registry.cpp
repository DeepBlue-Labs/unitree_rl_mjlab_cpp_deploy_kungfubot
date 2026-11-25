#include "control/Registry.h"
#include <openssl/evp.h>
#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <set>
#include <sstream>

namespace g1control {
namespace {
void need(bool ok,const std::string& message) { if(!ok) throw std::runtime_error("policy registry: "+message); }
std::string commaNumber(const std::string& value) {
    std::string out;size_t count=0;
    for(auto i=value.rbegin();i!=value.rend();++i) { if(count && count%3==0) out.push_back(',');out.push_back(*i);++count; }
    std::reverse(out.begin(),out.end());return out;
}
std::string upper(std::string text) { for(auto& c:text) c=char(std::toupper(static_cast<unsigned char>(c)));return text; }
void ticks(const YAML::Node& spec,const char* name,double dt) {
    need(bool(spec[name]),std::string("missing duration ")+name);
    const double value=spec[name].as<double>()/dt;
    need(std::isfinite(value) && value>0 && std::abs(value-std::round(value))<=1e-8+1e-5*std::abs(std::round(value)),
         std::string(name)+" must be a positive integer number of control ticks");
}
}
YAML::Node readRegistry(const std::filesystem::path& path) {
    auto data=YAML::LoadFile(path.string());
    need(data["schema"] && data["schema"].as<int>()==1,"unsupported schema");
    need(data["policies"].IsSequence(),"policies must be a sequence");
    need(data["defaults"].IsMap(),"defaults must be a map");
    std::set<std::string> names;
    for(const auto entry:data["policies"]) {
        need(entry.IsMap() && entry["name"],"policy name missing");
        const auto add=[&](const YAML::Node& n) {const auto name=n.as<std::string>();need(!name.empty() && names.insert(name).second,"duplicate/empty policy name or alias: "+name);};
        add(entry["name"]);
        if(entry["aliases"]) {need(entry["aliases"].IsSequence(),"aliases must be a sequence");for(const auto alias:entry["aliases"]) add(alias);}
    }
    return data;
}
std::string formatPolicyList(const YAML::Node& data) {
    std::vector<std::string> order;
    std::map<std::string,std::vector<YAML::Node>> groups;
    for(const auto entry:data["policies"]) {
        const auto source=entry["training_source"].as<std::string>("unknown");
        if(!groups.count(source)) order.push_back(source);
        groups[source].push_back(entry);
    }
    std::ostringstream out;out<<"已注册 "<<data["policies"].size()<<" 个策略，按训练来源分组：\n";
    for(const auto& source:order) {
        const auto sources=data["training_sources"];
        const auto metadata=sources && sources[source] ? sources[source] : YAML::Node(YAML::NodeType::Map);
        out<<"\n【"<<metadata["label"].as<std::string>("训练来源未记录")<<"】（"<<groups[source].size()<<" 个）\n";
        if(metadata["note"]) out<<"  "<<metadata["note"].as<std::string>()<<"\n";
        for(const auto& entry:groups[source]) {
            std::string checkpoint="未记录";
            if(entry["checkpoint_iteration"] && !entry["checkpoint_iteration"].IsNull()) checkpoint=commaNumber(entry["checkpoint_iteration"].as<std::string>());
            out<<"  "<<entry["name"].as<std::string>()<<" | 动作："<<entry["motion_label"].as<std::string>("未记录")<<" | checkpoint："<<checkpoint<<"\n    可用别名：";
            if(!entry["aliases"] || !entry["aliases"].size()) out<<"无（使用完整名称）";
            else for(size_t i=0;i<entry["aliases"].size();++i) out<<(i?", ":"")<<entry["aliases"][i].as<std::string>();
            out<<"\n";
        }
    }
    out<<"\n用法：./run_policy.sh --policy <完整名称或别名> --mode sim2sim\n";return out.str();
}
std::string fileSha256(const std::filesystem::path& path) {
    std::ifstream input(path,std::ios::binary);need(bool(input),"cannot read artifact: "+path.string());
    std::unique_ptr<EVP_MD_CTX,decltype(&EVP_MD_CTX_free)> context(EVP_MD_CTX_new(),&EVP_MD_CTX_free);
    need(context && EVP_DigestInit_ex(context.get(),EVP_sha256(),nullptr)==1,"cannot initialize SHA256");
    std::array<char,65536> buffer{};
    while(input) {input.read(buffer.data(),buffer.size());if(input.gcount()) need(EVP_DigestUpdate(context.get(),buffer.data(),size_t(input.gcount()))==1,"SHA256 update failed");}
    need(input.eof(),"artifact read failed: "+path.string());
    std::array<unsigned char,EVP_MAX_MD_SIZE> digest{};unsigned length=0;
    need(EVP_DigestFinal_ex(context.get(),digest.data(),&length)==1 && length==32,"SHA256 finalization failed");
    std::ostringstream hex;hex<<std::hex<<std::setfill('0');for(unsigned i=0;i<length;++i) hex<<std::setw(2)<<unsigned(digest[i]);return hex.str();
}
Registry::Registry(std::filesystem::path root,std::filesystem::path path):root_(std::filesystem::absolute(root).lexically_normal()) {
    if(path.empty()) path=root_/"policies/registry.json";
    data_=readRegistry(path);
}
YAML::Node Registry::resolve(const std::string& name) const {
    for(const auto entry:data_["policies"]) {
        bool found=entry["name"].as<std::string>()==name;
        if(entry["aliases"]) for(const auto alias:entry["aliases"]) found=found || alias.as<std::string>()==name;
        if(found) {
            auto out=YAML::Clone(data_["defaults"]);
            for(const auto field:entry) out[field.first.as<std::string>()]=YAML::Clone(field.second);
            return out;
        }
    }
    throw std::runtime_error("unknown policy '"+name+"'; use --list");
}
std::string Registry::formatPolicyList() const { return g1control::formatPolicyList(data_); }
std::vector<std::filesystem::path> Registry::artifactPaths(const YAML::Node& spec) const {
    const auto bundle=root_/spec["bundle"].as<std::string>();const auto kind=spec["adapter"].as<std::string>();
    if(kind=="hybrid" || kind=="ufo_jointtrack") return {bundle/"policy_motion.onnx",bundle/"contract.json",bundle/"reference.npz"};
    if(kind=="reference") return {bundle/"policy.onnx",bundle/"deploy_config.json",root_/spec["clip"].as<std::string>()};
    throw std::runtime_error("unknown registry adapter: "+kind);
}
YAML::Node Registry::identity(const YAML::Node& spec) const {
    YAML::Node hashes(YAML::NodeType::Map);
    for(const auto& path:artifactPaths(spec)) {
        const auto rel=path.lexically_normal().lexically_relative(root_);
        const auto key=!rel.empty() && *rel.begin()!=".." ? rel.generic_string() : path.string();
        hashes[key]=fileSha256(path);
    }
    return hashes;
}
std::unique_ptr<Policy> Registry::load(const std::string& name) const { return load(resolve(name)); }
std::unique_ptr<Policy> Registry::load(const YAML::Node& spec) const {
    for(const auto& path:artifactPaths(spec)) need(std::filesystem::is_regular_file(path),"missing artifact: "+path.string());
    auto policy=makeMotionPolicy(spec,root_);const auto& info=policy->info();
    for(const auto name:{"prepare_s","return_s","takeover_s"}) ticks(spec,name,info.dt);
    need(info.frames>0,"empty reference motion");
    need(info.kp.allFinite() && info.kd.allFinite() && info.effort.allFinite() && info.entry_q.allFinite(),"invalid policy vectors");
    for(size_t f=0;f<info.frames;++f) {
        const auto height=policy->referenceHeight(f);const auto gravity=policy->referenceGravity(f);
        need(std::isfinite(height) && height>0 && gravity.allFinite() && std::abs(gravity.norm()-1)<=.01001,"invalid reference height/gravity envelope");
    }
    return policy;
}
YAML::Node Registry::inspect(const std::string& name) const {
    const auto spec=resolve(name);auto policy=load(spec);const auto& info=policy->info();
    YAML::Node result(YAML::NodeType::Map);result["policy"]=spec;result["artifact_sha256"]=identity(spec);
    result["frame_count"]=info.frames;result["control_dt"]=info.dt;result["observation_width"]=info.observation_width;
    result["target_processing"]=info.target_processing;result["needs_estimator"]=info.needs_estimator;
    for(const auto file:{"params/deploy.yaml","exported/policy.onnx"}) result["standing_sha256"][file]=fileSha256(root_/"policies/standing"/file);
    result["standing_source"]["bundle"]="policies/standing";
    result["standing_source"]["contract"]="original unified Python zero-command velocity standing";
    result["standing_source"]["waist_yaw_action_scale"]=YAML::LoadFile((root_/"policies/standing/params/deploy.yaml").string())["actions"]["JointPositionAction"]["scale"][12].as<double>();
    result["standing_source"]["original_cpp_velocity_waist_yaw_action_scale"]=YAML::LoadFile((root_/"deploy/robots/g1/config/policy/velocity/v0/params/deploy.yaml").string())["actions"]["JointPositionAction"]["scale"][12].as<double>();
    const auto ready=spec["ready_hold"];
    if(!ready) return result;
    const auto fight=ready["fighting"];
    std::vector<std::string> choices={spec["name"].as<std::string>()};
    if(fight) {
        for(const auto key:{"actor_blend_s","request_timeout_s","settle_timeout_s","acquire_timeout_s"}) ticks(fight,key,info.dt);
        choices.push_back(fight["holder"].as<std::string>());choices.push_back(fight["exit"].as<std::string>());
        need(fight["actions"].IsMap(),"fighting actions must be a map");
        for(const auto action:fight["actions"]) {const auto key=action.first.as<std::string>();choices.push_back(key);result["verified_actions"].push_back(key);}
    } else for(const auto next:ready["next_policies"]) choices.push_back(next.as<std::string>());
    std::set<std::string> loaded;std::map<std::string,std::string> aliases;
    for(const auto& key:choices) {
        const auto candidate=resolve(key);const auto canonical=candidate["name"].as<std::string>();
        if(loaded.insert(canonical).second) {
            auto actor=load(candidate);const auto& other=actor->info();
            need(other.joint_names==info.joint_names && other.observation_width==info.observation_width && other.dt==info.dt && !other.needs_estimator,
                 "session candidate has incompatible observation contract: "+canonical);
            need(fight ? (other.robust_proprio && (other.kind=="hybrid" || other.kind=="ufo_jointtrack")) : other.kind=="ufo_jointtrack",
                 "session candidate has incompatible adapter: "+canonical);
            need((other.kp.array()==info.kp.array()).all() && (other.kd.array()==info.kd.array()).all() && (other.effort.array()==info.effort.array()).all(),
                 "session candidate gains/efforts differ: "+canonical);
            if(fight && canonical==resolve(fight["holder"].as<std::string>())["name"].as<std::string>()) {
                const auto a=actor->referenceJoints(other.frames-1),b=policy->referenceJoints(info.frames-1);
                need(((a-b).cwiseAbs().array()<=1e-6+1e-5*b.cwiseAbs().array()).all(),"holder must target entry final ready pose");
            }
            result["session_artifacts"][canonical]=identity(candidate);
        }
        std::vector<std::string> keys={canonical};if(candidate["aliases"]) for(const auto alias:candidate["aliases"]) keys.push_back(alias.as<std::string>());
        for(const auto& alias:keys) {const auto normalized=upper(alias);need(!aliases.count(normalized) || aliases[normalized]==canonical,"ambiguous session alias: "+alias);aliases[normalized]=canonical;}
    }
    if(!fight) {
        std::set<std::string> holders;
        for(const auto key:ready["hold_policies"]) {const auto normalized=upper(key.as<std::string>());need(aliases.count(normalized),"unprepared hold policy");holders.insert(aliases[normalized]);}
        need(holders.count(spec["name"].as<std::string>()),"initial ready-hold policy must hold its final frame");
        for(const auto key:ready["next_from_hold"]) need(aliases.count(upper(key.as<std::string>())),"unprepared next_from_hold policy");
    }
    return result;
}
}
