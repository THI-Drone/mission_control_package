#include "main.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;


MissionControl::MissionControl() : CommonNode("mission_control")
{
  // Initialize Heartbeat
  heartbeat_map["/node_1"] = heartbeat_payload();
  heartbeat_map["/node_2"] = heartbeat_payload();

  heartbeat_subscription = this->create_subscription<interfaces::msg::Heartbeat>(
    "heartbeat", 10, std::bind(&MissionControl::heartbeat_callback, this, _1));
  heartbeat_timer = this->create_wall_timer(
    1000ms, std::bind(&MissionControl::heartbeat_timer_callback, this));

  // Initialize Event Loop
  event_loop_timer = this->create_wall_timer(
    100ms, std::bind(&MissionControl::event_loop, this));
}

// Event Loop
/**
 * @brief The event loop function of the MissionControl class.
 * 
 * This function is responsible for handling events and controlling the mission.
 * It is called every 100ms.
 * 
 * @note Can be disabled with the "event_loop_active" variable set to false
 */
void MissionControl::event_loop()
{
  if(!event_loop_active) return;

  switch (mission_state)
  {
  case selfcheck:
    self_check();
    break;
  // TODO implement other cases
  
  default:
    RCLCPP_ERROR(this->get_logger(), "MissionControl::event_loop: Unknown mission_state: %d", mission_state);
    mission_abort("MissionControl::event_loop: Unknown mission_state");
  }
}

// Selfcheck
void MissionControl::self_check()
{
  EventLoopGuard elg = EventLoopGuard(&event_loop_active, false);
  RCLCPP_INFO(this->get_logger(), "MissionControl::self_check: Self check started");

  // TODO implement Selfcheck

  RCLCPP_INFO(this->get_logger(), "MissionControl::self_check: Self check finished");
  
  mission_state = armed;
}

// Mission Abort
/**
 * @brief Aborts the mission.
 * 
 * This function is responsible for aborting the mission and exiting the program.
 * 
 * @param reason The reason for aborting the mission.
 */
void MissionControl::mission_abort(std::string reason)
{
  RCLCPP_INFO(this->get_logger(), "MissionControl::mission_abort: Aborting mission, reason: %s", reason.c_str());

  // TODO abort mission

  exit(EXIT_FAILURE);
}

// Heartbeat
void MissionControl::heartbeat_callback(const interfaces::msg::Heartbeat &msg)
{
  // Throw away own heartbeat
  if(msg.sender_id == this->get_fully_qualified_name()) return;

  if(heartbeat_map.find(msg.sender_id) == heartbeat_map.end()) {
      RCLCPP_ERROR(this->get_logger(), "MissionControl::heartbeat_callback: Received unregistered heartbeat: %s", msg.sender_id.c_str());
      return;
  }

  heartbeat_payload &heartbeat = heartbeat_map[msg.sender_id];
  if((msg.tick == heartbeat.tick) || (msg.tick <= heartbeat.tick && msg.tick != 0)) {
      RCLCPP_ERROR(this->get_logger(), "MissionControl::heartbeat_callback: Invalid tick received from: %s", msg.sender_id.c_str());
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
      RCLCPP_ERROR(this->get_logger(), "MissionControl::heartbeat_timer_callback: No heartbeat from '%s' received!", hm.first.c_str());
    }

    hm.second.received = false;
  }

  if(!err_flag) {
    RCLCPP_INFO(this->get_logger(), "MissionControl::heartbeat_timer_callback: All heartbeats received");
  }
}

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MissionControl>());
  rclcpp::shutdown();
  return 0;
}
