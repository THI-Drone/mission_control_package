#include "main.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;


MissionControl::MissionControl() : Node("mission_control")
{
  heartbeat_map["/node_1"] = heartbeat_payload();
  heartbeat_map["/node_2"] = heartbeat_payload();

  heartbeat_subscription = this->create_subscription<interfaces::msg::Heartbeat>(
    "heartbeat", 10, std::bind(&MissionControl::heartbeat_callback, this, _1));
  heartbeat_timer = this->create_wall_timer(
    1000ms, std::bind(&MissionControl::heartbeat_timer_callback, this));
}

void MissionControl::heartbeat_callback(const interfaces::msg::Heartbeat &msg)
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

void MissionControl::heartbeat_timer_callback()
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

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MissionControl>());
  rclcpp::shutdown();
  return 0;
}
