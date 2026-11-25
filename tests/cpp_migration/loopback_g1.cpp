// Local DDS fixture only: no configurable network interface, no domain 0.
// Publishes synthetic state and records commands; does not control any plant.
#include <unitree/idl/hg/LowCmd_.hpp>
#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <algorithm>
#include <array>
#include <cmath>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <stdexcept>
#include <thread>
using State=unitree_hg::msg::dds_::LowState_;
using Command=unitree_hg::msg::dds_::LowCmd_;
using Clock=std::chrono::steady_clock;
volatile std::sig_atomic_t stopped=0;
void signalHandler(int){stopped=1;}
template<class T> uint32_t crc(const T& message){
 const auto* bytes=reinterpret_cast<const unsigned char*>(&message);uint32_t result=0xffffffff;
 for(size_t i=0;i<sizeof(T)/4-1;++i){uint32_t word;std::memcpy(&word,bytes+i*4,4);for(uint32_t mask=0x80000000;mask;mask>>=1){const bool high=result&0x80000000;result<<=1;if(high)result^=0x04c11db7;if(word&mask)result^=0x04c11db7;}}return result;
}
int main(int argc,char** argv){
 try{
  if(argc!=3)throw std::runtime_error("usage: loopback_g1 DOMAIN>0 COMMAND_CSV");
  const int domain=std::stoi(argv[1]);if(domain<=0||domain>200)throw std::runtime_error("fixture requires a positive local DDS domain <=200");
  std::ofstream log(argv[2]);if(!log)throw std::runtime_error("cannot open command capture");log<<"time,kp_max,kd_min,kd_max,q_max,crc_ok\n"<<std::setprecision(12);
  std::signal(SIGTERM,signalHandler);std::signal(SIGINT,signalHandler);
  unitree::robot::ChannelFactory::Instance()->Init(domain,"lo");
  std::array<float,29> q={-.1,0,0,.3,-.2,0,-.1,0,0,.3,-.2,0,0,0,0,.35,.18,0,.87,0,0,0,.35,-.18,0,.87,0,0,0};
  std::mutex mutex;const auto started=Clock::now();std::atomic<size_t> count{0};
  unitree::robot::ChannelSubscriber<Command> commands("rt/lowcmd");
  commands.InitChannel([&](const void* raw){
   const auto& c=*static_cast<const Command*>(raw);float kp=0,kdmin=1e9,kdmax=0,qmax=0;const bool good=crc(c)==c.crc();
   std::lock_guard<std::mutex> guard(mutex);
   for(int j=0;j<29;++j){auto& m=c.motor_cmd()[j];kp=std::max(kp,m.kp());kdmin=std::min(kdmin,m.kd());kdmax=std::max(kdmax,m.kd());qmax=std::max(qmax,std::abs(m.q()));if(good&&m.kp()>0)q[j]=m.q();}
   log<<std::chrono::duration<double>(Clock::now()-started).count()<<','<<kp<<','<<kdmin<<','<<kdmax<<','<<qmax<<','<<good<<'\n';log.flush();++count;
  },1);
  unitree::robot::ChannelPublisher<State> states("rt/lowstate");states.InitChannel();
  std::cout<<"LOOPBACK_READY domain="<<domain<<" iface=lo"<<std::endl;
  State state{};state.mode_machine()=5;state.mode_pr()=0;state.imu_state().quaternion()[0]=1;auto deadline=Clock::now();
  while(!stopped){
   state.tick()+=1;
   {std::lock_guard<std::mutex> guard(mutex);for(int j=0;j<29;++j){auto& m=state.motor_state()[j];m.q()=q[j];m.dq()=0;m.motorstate()=0;for(auto& t:m.temperature())t=25;}}
   state.crc()=crc(state);states.Write(state);deadline+=std::chrono::milliseconds(2);std::this_thread::sleep_until(deadline);if(Clock::now()-deadline>std::chrono::milliseconds(20))deadline=Clock::now();
  }
  commands.CloseChannel();states.CloseChannel();std::cout<<"CAPTURED "<<count.load()<<" local commands"<<std::endl;return 0;
 }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
}
