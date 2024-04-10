// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <functional>
#include <memory>
#include <map>

#include "rclcpp/rclcpp.hpp"
#include "interfaces/msg/heartbeat.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

struct heartbeat_payload {
  bool received;
  uint32_t tick;

  heartbeat_payload() {
    this->received = false;
    this->tick = 0;
  }
};

class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber()
  : Node("minimal_subscriber")
  {
    heartbeat_map["/node_1"] = heartbeat_payload();
    heartbeat_map["/node_2"] = heartbeat_payload();

    subscription_ = this->create_subscription<interfaces::msg::Heartbeat>(
      "heartbeat", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
    timer_ = this->create_wall_timer(
      1000ms, std::bind(&MinimalSubscriber::timer_callback, this));
  }

private:
  void topic_callback(const interfaces::msg::Heartbeat &msg)
  {
    if(heartbeat_map.find(msg.sender_id) == heartbeat_map.end()) {
        RCLCPP_ERROR(this->get_logger(), "Received unregistered heartbeat: %s", msg.sender_id.c_str());
        return;
    }

    heartbeat_payload &heartbeat = heartbeat_map[msg.sender_id];
    if((msg.tick == heartbeat.tick) || (msg.tick <= heartbeat.tick && msg.tick != 0)) {
        RCLCPP_ERROR(this->get_logger(), "Invalid tick received from: %s", msg.sender_id.c_str());
        return;
    }
    heartbeat.tick = msg.tick;
    heartbeat.received = true;

    //RCLCPP_INFO(this->get_logger(), "I heard: '%s', tick: %u, active: %d", msg.sender_id.c_str(), msg.tick, msg.active);
  }
  void timer_callback()
  {
    bool err_flag = false;

    for(auto &hm: heartbeat_map) {
      if(hm.second.received == false) {
        err_flag = true;
        RCLCPP_ERROR(this->get_logger(), "No heartbeat from '%s' received!", hm.first.c_str());
      }

      hm.second.received = false;
    }

    if(!err_flag) {
      RCLCPP_INFO(this->get_logger(), "All heartbeats received");
    }
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<interfaces::msg::Heartbeat>::SharedPtr subscription_;
  std::map<std::string, heartbeat_payload> heartbeat_map;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
