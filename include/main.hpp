#pragma once

#include <chrono>
#include <functional>
#include <memory>
#include <map>

#include "rclcpp/rclcpp.hpp"
#include "interfaces/msg/heartbeat.hpp"


struct heartbeat_payload {
  bool received;
  uint32_t tick;

  heartbeat_payload() {
    this->received = false;
    this->tick = 0;
  }
};

class MissionControl : public rclcpp::Node
{
private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<interfaces::msg::Heartbeat>::SharedPtr subscription_;
    std::map<std::string, heartbeat_payload> heartbeat_map;

public:
    MissionControl();

private:
    void topic_callback(const interfaces::msg::Heartbeat &msg);
    void timer_callback();
};
