#include "control/RunLog.h"
#include <fstream>
#include <chrono>
#include <cmath>
#include <cstring>
#include <limits>
#include <zlib.h>
#include <codecvt>
#include <locale>
namespace g1control {
Json::Value jsonNode(const YAML::Node& n) {
    if(!n||n.IsNull())return {};
    if(n.IsSequence()){Json::Value v(Json::arrayValue);for(auto x:n)v.append(jsonNode(x));return v;}
    if(n.IsMap()){Json::Value v(Json::objectValue);for(auto x:n)v[x.first.as<std::string>()]=jsonNode(x.second);return v;}
    const auto s=n.Scalar();
    if(n.Tag()=="!"||n.Tag()=="tag:yaml.org,2002:str")return s;
    if(s=="true")return true;
    if(s=="false")return false;
    char* end=nullptr;const auto number=std::strtod(s.c_str(),&end);
    if(!s.empty()&&end==s.c_str()+s.size()&&std::isfinite(number))return number;
    return s;
}
void writeJson(const std::filesystem::path& path,const Json::Value& value) {
    std::ofstream stream(path);if(!stream)throw std::runtime_error("cannot write "+path.string());
    Json::StreamWriterBuilder b;b["indentation"]="  ";stream<<Json::writeString(b,value)<<'\n';
    if(!stream)throw std::runtime_error("write failed "+path.string());
}
namespace {
double tilt(const Sensors& s){return std::acos(std::clamp(s.quat.toRotationMatrix()(2,2),-1.,1.));}
double pitch(const Sensors& s){return std::asin(std::clamp(-s.quat.toRotationMatrix()(2,0),-1.,1.));}
void u16(std::ostream& f,uint16_t x){char b[2]={char(x),char(x>>8)};f.write(b,2);}
void u32(std::ostream& f,uint32_t x){u16(f,x&65535);u16(f,x>>16);}
// Standard NumPy NPY v1 entries in a ZIP archive, with raw DEFLATE compression.
class Npz {
    struct Entry{std::string name;uint32_t crc,size,compressed,offset;};
    std::ofstream f;std::vector<Entry> entries;
public:
    explicit Npz(const std::filesystem::path& p):f(p,std::ios::binary){if(!f)throw std::runtime_error("cannot write NPZ");}
    void add(std::string name,const std::string& dtype,std::vector<size_t> shape,const void* data,size_t bytes){
        std::string header="{'descr': '"+dtype+"', 'fortran_order': False, 'shape': (";
        for(auto n:shape)header+=std::to_string(n)+", ";
        header+="), }";
        header.append((64-(10+header.size()+1)%64)%64,' ');header+='\n';
        std::string raw("\x93NUMPY\x01\x00",8);raw+=char(header.size());raw+=char(header.size()>>8);raw+=header;
        if(bytes)raw.append(static_cast<const char*>(data),bytes);
        z_stream z{};if(deflateInit2(&z,1,Z_DEFLATED,-MAX_WBITS,8,Z_DEFAULT_STRATEGY)!=Z_OK)throw std::runtime_error("NPZ compression init failed");
        std::vector<unsigned char> compressed(compressBound(raw.size())+64);
        z.next_in=reinterpret_cast<Bytef*>(raw.data());z.avail_in=raw.size();z.next_out=compressed.data();z.avail_out=compressed.size();
        const int status=deflate(&z,Z_FINISH);const size_t n=z.total_out;deflateEnd(&z);
        if(status!=Z_STREAM_END)throw std::runtime_error("NPZ compression failed");
        Entry e{name+".npy",uint32_t(crc32(0,reinterpret_cast<const Bytef*>(raw.data()),raw.size())),uint32_t(raw.size()),uint32_t(n),uint32_t(f.tellp())};
        u32(f,0x04034b50);u16(f,20);u16(f,0);u16(f,8);u16(f,0);u16(f,0);u32(f,e.crc);u32(f,e.compressed);u32(f,e.size);u16(f,e.name.size());u16(f,0);f<<e.name;f.write(reinterpret_cast<const char*>(compressed.data()),n);entries.push_back(e);
    }
    template<class T>void numbers(const std::string& name,const std::string& type,const std::vector<T>& values,std::vector<size_t> shape){add(name,type,std::move(shape),values.data(),values.size()*sizeof(T));}
    void strings(const std::string& name,const std::vector<std::string>& values){
        std::wstring_convert<std::codecvt_utf8<char32_t>,char32_t> convert;
        std::vector<std::u32string> text;size_t width=1;
        for(auto& v:values){text.push_back(convert.from_bytes(v));width=std::max(width,text.back().size());}
        std::vector<uint32_t> bytes(values.size()*width);
        for(size_t i=0;i<text.size();i++)for(size_t j=0;j<text[i].size();j++)bytes[i*width+j]=text[i][j];
        add(name,"<U"+std::to_string(width),{values.size()},bytes.data(),bytes.size()*4);
    }
    void close(){const uint32_t start=f.tellp();for(auto& e:entries){u32(f,0x02014b50);u16(f,20);u16(f,20);u16(f,0);u16(f,8);u16(f,0);u16(f,0);u32(f,e.crc);u32(f,e.compressed);u32(f,e.size);u16(f,e.name.size());u16(f,0);u16(f,0);u16(f,0);u16(f,0);u32(f,0);u32(f,e.offset);f<<e.name;}const uint32_t size=uint32_t(f.tellp())-start;u32(f,0x06054b50);u16(f,0);u16(f,0);u16(f,entries.size());u16(f,entries.size());u32(f,size);u32(f,start);u16(f,0);if(!f)throw std::runtime_error("NPZ write failed");}
};
}
RunLog::RunLog(std::filesystem::path output):directory_(std::move(output)){
    if(std::filesystem::exists(directory_))throw std::runtime_error("output directory already exists: "+directory_.string());
    std::filesystem::create_directories(directory_);
}
void RunLog::record(const SessionOutput& out,const Sensors& before,const Sensors& after,double time,const RobotSnapshot* snapshot){
    Row row;row.output=out;row.before=before;row.after=after;row.time=time;row.send=std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();if(snapshot)row.snapshot=*snapshot;
    rows_.push_back(std::move(row));counts_[out.phase]++;
}
size_t RunLog::count(const std::string& phase)const{auto p=counts_.find(phase);return p==counts_.end()?0:p->second;}
Json::Value RunLog::metrics()const{
    Json::Value result,phases;double height=INFINITY,angle=0;
    for(auto& [name,n]:counts_){double h=INFINITY,t=0;for(auto& r:rows_)if(r.output.phase==name){h=std::min(h,r.after.root_pos.z());t=std::max(t,tilt(r.after));}phases[name]["ticks"]=Json::UInt64(n);phases[name]["min_height_m"]=h;phases[name]["max_tilt_rad"]=t;height=std::min(height,h);angle=std::max(angle,t);result["phase_ticks"][name]=Json::UInt64(n);}
    result["phase_metrics"]=phases;result["min_height_m"]=std::isfinite(height)?Json::Value(height):Json::Value();result["max_tilt_rad"]=angle;
    for(const auto name:{"STAND_POST","HOLD"}){std::vector<const Row*> last;for(auto it=rows_.rbegin();it!=rows_.rend()&&last.size()<50;++it)if(it->output.phase==name)last.push_back(&*it);double h=INFINITY,t=0,speed=0,pmin=INFINITY,pmax=-INFINITY;for(auto r:last){h=std::min(h,r->after.root_pos.z());t=std::max(t,tilt(r->after));speed=std::max(speed,r->after.dq.cwiseAbs().maxCoeff());pmin=std::min(pmin,pitch(r->after));pmax=std::max(pmax,pitch(r->after));}if(std::string(name)=="STAND_POST")result["standing_recovered_last_1s"]=last.size()==50&&h>.6&&t<.45;else{result["hold_envelope_last_1s"]=last.size()==50&&h>.6&&t<.45;result["hold_pitch_ptp_deg_last_1s"]=last.empty()?Json::Value():Json::Value((pmax-pmin)*180/M_PI);result["hold_max_joint_speed_last_1s"]=last.empty()?Json::Value():Json::Value(speed);}}
    result["boxing_envelope_passed"]=height>.6&&angle<.45;return result;
}
Json::Value RunLog::holdMetrics(const Joints& reference) const {
    double squared=0;size_t count=0;
    for(auto it=rows_.rbegin();it!=rows_.rend()&&count<50;++it)if(it->output.phase=="HOLD"){squared+=(it->before.q-reference).squaredNorm();++count;}
    Json::Value value;value["hold_pose_rmse_last_1s"]=count?Json::Value(std::sqrt(squared/(29*count))):Json::Value();return value;
}
void RunLog::finish(Json::Value summary){
    Npz npz(directory_/"trace.npz");const size_t n=rows_.size();std::vector<std::string> phase,active;std::vector<int64_t> frames,ticks;std::vector<double> height,angles,pitches,speed,time,send,receive,age;std::vector<uint8_t> support;size_t width=0;
    for(auto& r:rows_){phase.push_back(r.output.phase);active.push_back(r.output.active_policy);frames.push_back(r.output.frame);ticks.push_back(r.snapshot.tick);height.push_back(r.after.root_pos.z());angles.push_back(tilt(r.after));pitches.push_back(pitch(r.after));speed.push_back(r.after.dq.cwiseAbs().maxCoeff());time.push_back(r.time);send.push_back(r.send);receive.push_back(r.snapshot.received_monotonic_s);age.push_back(r.snapshot.age_s);support.push_back(r.output.support);width=std::max(width,r.output.observation.size());}
    npz.strings("phase",phase);npz.strings("active_policy",active);npz.numbers("motion_frame","<i8",frames,{n});npz.numbers("lowstate_tick","<i8",ticks,{n});
    for(auto field:std::vector<std::pair<std::string,const std::vector<double>*>>{{"height",&height},{"tilt",&angles},{"pitch",&pitches},{"max_joint_speed",&speed},{"sim_time",&time},{"t_send",&send},{"t_receive",&receive},{"state_age",&age}})npz.numbers(field.first,"<f8",*field.second,{n});
    npz.numbers("support","|u1",support,{n});
    for(const auto name:{"target","joint_target","kp","kd","measured","measured_joint_pos","measured_joint_vel","raw_action","measured_imu_quat_wxyz","temperature","motor_errors"}){
        std::vector<double> values;const std::string key=name;size_t columns=key=="measured_imu_quat_wxyz"?4:key=="temperature"?70:key=="motor_errors"?35:29;
        values.reserve(n*columns);for(auto& r:rows_){if(columns==4){values.insert(values.end(),{r.before.quat.w(),r.before.quat.x(),r.before.quat.y(),r.before.quat.z()});continue;}if(key=="temperature"){for(auto pair:r.snapshot.temperature)values.insert(values.end(),pair.begin(),pair.end());continue;}if(key=="motor_errors"){for(auto error:r.snapshot.motor_errors)values.push_back(error);continue;}const Joints* v=key=="target"||key=="joint_target"?&r.output.q:key=="kp"?&r.output.kp:key=="kd"?&r.output.kd:key=="measured_joint_vel"?&r.before.dq:key=="raw_action"?&r.output.raw_action:&r.before.q;values.insert(values.end(),v->data(),v->data()+29);}npz.numbers(key,"<f8",values,{n,columns});}
    std::vector<float> observations(n*width,std::numeric_limits<float>::quiet_NaN());std::vector<int64_t> widths;for(size_t i=0;i<n;i++){std::copy(rows_[i].output.observation.begin(),rows_[i].output.observation.end(),observations.begin()+i*width);widths.push_back(rows_[i].output.observation.size());}npz.numbers("observation","<f4",observations,{n,width});npz.numbers("observation_width","<i8",widths,{n});npz.close();
    auto m=metrics();for(auto it=m.begin();it!=m.end();++it)if(!summary.isMember(it.name()))summary[it.name()]=*it;writeJson(directory_/"summary.json",summary);
}
std::string SimulationScreen::check(const Sensors& s,const SessionOutput& out,const Policy& p){
    if(policy_!=&p){policy_=&p;return_index_=0;}
    if(!s.q.allFinite()||!s.dq.allFinite()||!s.quat.coeffs().allFinite()||!s.gyro.allFinite()||!s.root_pos.allFinite()||!s.root_vel_b.allFinite())return "simulation screen: non-finite state";
    if(std::abs(s.quat.norm()-1)>.05)return "simulation screen: invalid root quaternion";
    if(out.phase=="WAIT_TAKEOVER"||out.phase=="TAKEOVER"||out.phase=="DAMP")return {};
    double floor=.35;Eigen::Vector3d gravity(0,0,-1);
    if(out.phase=="POLICY"||out.phase=="HOLD"||out.phase=="RETURN"){
        const size_t frame=out.phase=="RETURN"?p.info().frames-1:std::clamp<long>(out.frame,0,p.info().frames-1);
        floor=std::max(.20,std::min(.35,p.referenceHeight(frame)-.12));gravity=p.referenceGravity(frame);
        if(out.phase=="RETURN"){const double x=std::min(++return_index_*p.info().dt,1.);floor=(1-x)*floor+x*.35;gravity=(1-x)*gravity+x*Eigen::Vector3d(0,0,-1);}
    }
    if(s.root_pos.z()<floor)return "simulation screen: pelvis below motion-relative floor";
    const Eigen::Vector3d measured=s.quat.conjugate()*Eigen::Vector3d(0,0,-1);
    if(std::acos(std::clamp(measured.dot(gravity)/gravity.norm(),-1.,1.))>.7)return "simulation screen: reference-relative tilt exceeds 0.7 rad";
    return {};
}
}
