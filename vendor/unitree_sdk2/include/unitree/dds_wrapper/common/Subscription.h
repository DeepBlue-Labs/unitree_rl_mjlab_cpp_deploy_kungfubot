// Copyright (c) 2025, Unitree Robotics Co., Ltd.
// All rights reserved.

#pragma once

#include <unitree/robot/channel/channel_subscriber.hpp>
#include <mutex>
#include <thread>
#include <atomic>

namespace unitree
{
namespace robot
{
/**
 * @brief Base class for convenient subscription to topics.
 * This class provides an easy-to-use interface for subscribing to topics, 
 * but the continuous background updates may increase CPU usage.
 *
 * @tparam MessageType The type of message being subscribed to.
 */
template <typename MessageType>
class SubscriptionBase
{
public:
  using MsgType = MessageType;
  using SharedPtr = std::shared_ptr<SubscriptionBase<MsgType>>;

  SubscriptionBase(const std::string& topic, const std::function<void(const void*)>& handler = nullptr)
  {
    sub_ = std::make_shared<unitree::robot::ChannelSubscriber<MessageType>>(topic);
    if (handler) {
      sub_->InitChannel(handler);
    } else {
      sub_->InitChannel([this](const void *msg){
        std::lock_guard<std::mutex> lock(mutex_);
        pre_communication();
        msg_ = *(const MessageType*)msg;
        post_communication();
        last_update_ns_.store(now_ns(),std::memory_order_release);
      });
    }
  }

  void set_timeout_ms(uint32_t timeout_ms) { timeout_ms_.store(timeout_ms); }

  bool isTimeout() {
    const auto updated=last_update_ns_.load(std::memory_order_acquire);
    return updated==0 || now_ns()-updated>int64_t(timeout_ms_.load())*1000000;
  }

  void wait_for_connection() {
    auto t0 = std::chrono::steady_clock::now();
    bool warn_info = false;
    while(isTimeout()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      if (!warn_info && std::chrono::steady_clock::now() - t0 > std::chrono::seconds(2)) {
        warn_info = true;
        std::cout << "Waiting for connection " << sub_->GetChannelName() << std::endl;
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100)); // wait for stable communicaiton
    if (warn_info) {
      std::cout << "Connected " << sub_->GetChannelName() << std::endl;
    }
  }

  MessageType msg_;
  std::mutex mutex_;

protected:
  virtual void pre_communication() {}  // something before receiving message
  virtual void post_communication() {} // something after receiving message

  static int64_t now_ns() { return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count(); }
  std::atomic<uint32_t> timeout_ms_{1000};
  std::atomic<int64_t> last_update_ns_{0};
  unitree::robot::ChannelSubscriberPtr<MessageType> sub_;
};


}; // namespace robot
}; // namespace unitree
