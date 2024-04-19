#pragma once

#include <chrono>
#include <functional>
#include <memory>
#include <map>
#include <stddef.h>
#include "rclcpp/rclcpp.hpp"

#include "common_package/common_node.hpp"
#include "event_loop_guard.hpp"
#include "structs.hpp"

// Message includes
#include "interfaces/msg/heartbeat.hpp"
#include "interfaces/msg/control.hpp"
#include "interfaces/msg/mission_start.hpp"
#include "interfaces/msg/mission_abort.hpp"
#include "interfaces/msg/fly_to_coord.hpp"

/**
 * @brief Enumeration representing the different states of a mission.
 */
typedef enum MissionState
{
    selfcheck,
    armed,
    takeoff,
    waypoint,
    marker_recognition,
    drop_payload,
    rth,
    land,
} MissionState_t;

class MissionControl : public common_lib::CommonNode
{
private:
    // General
    MissionState_t mission_state = selfcheck; /// Main mission state
    bool state_first_loop = true;             /// If set to true, the first event loop after a mission_state change is happening. Manually set to false in your function after first call.
    std::map<std::string, ros_node> node_map; /// Has an entry for every ros node
    std::string active_node = "";             /// node_id that is currently allowed to send data to the FCC interface, set to "" if none is allowed

    // Event Loop
    const uint32_t event_loop_time_delta_ms = 100;
    bool event_loop_active = true; /// If set to true, the event loop will be executed periodically
    rclcpp::TimerBase::SharedPtr event_loop_timer;

    // Mission Start
    rclcpp::Subscription<interfaces::msg::MissionStart>::SharedPtr mission_start_subscription;

    // Mission Abort
    rclcpp::Publisher<interfaces::msg::MissionAbort>::SharedPtr mission_abort_publisher;

    // Fail-Safe Checks
    rclcpp::Subscription<interfaces::msg::FlyToCoord>::SharedPtr control_subscription;

    // Heartbeat
    bool heartbeat_received_all = false; /// true if all heartbeats we're received in the last timeframe, otherwise false
    rclcpp::TimerBase::SharedPtr heartbeat_timer;
    rclcpp::Subscription<interfaces::msg::Heartbeat>::SharedPtr heartbeat_subscription;

public:
    MissionControl();

private:
    // Event Loop
    void event_loop();

    // Configuration Changes
    void set_standby_config();
    void set_mission_state(const MissionState_t new_mission_state);
    constexpr MissionState_t get_mission_state() const
    {
        return mission_state;
    }
    bool get_state_first_loop();

    // Selfcheck
    void mode_self_check();

    // Mission start
    void mission_start(const interfaces::msg::MissionStart &msg);

    // Takeoff
    void initiate_takeoff();

    // Mission Abort
    void mission_abort(std::string reason);

    // Fail-Safe checks
    void check_control(const interfaces::msg::FlyToCoord &msg);

    // Heartbeat
    void heartbeat_callback(const interfaces::msg::Heartbeat &msg);
    void heartbeat_timer_callback();
};
