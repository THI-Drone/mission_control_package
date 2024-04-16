#pragma once

#include <chrono>
#include <functional>
#include <memory>
#include <map>
#include <stddef.h>
#include "rclcpp/rclcpp.hpp"

#include "common_package/common_node.hpp"
#include "event_loop_guard.hpp"

// Message includes
#include "interfaces/msg/heartbeat.hpp"
#include "interfaces/msg/control.hpp"


/**
 * @brief Struct representing a heartbeat payload.
 */
struct heartbeat_payload {
    bool received; /**< Flag indicating if the payload has been received. */
    uint32_t tick; /**< The tick value of the payload. */

    /**
     * @brief Default constructor for heartbeat_payload.
     * Initializes the received flag to false and the tick value to 0.
     */
    heartbeat_payload() {
        this->received = false;
        this->tick = 0;
    }
};

/**
 * @brief Enumeration representing the different states of a mission.
 */
typedef enum MissionState {
    selfcheck,
    armed,
    takeoff,
    waypoint,
    marker_recognition,
    drop_payload,
    rth,
    land,
} MissionState_t;

class MissionControl : public CommonNode
{
private:
    // General
    MissionState_t mission_state = selfcheck;

    // Event Loop
    bool event_loop_active = true;
    rclcpp::TimerBase::SharedPtr event_loop_timer;

    // Fail-Safe Checks
    rclcpp::Subscription<interfaces::msg::Control>::SharedPtr control_subscription;

    // Heartbeat
    rclcpp::TimerBase::SharedPtr heartbeat_timer;
    rclcpp::Subscription<interfaces::msg::Heartbeat>::SharedPtr heartbeat_subscription;
    std::map<std::string, heartbeat_payload> heartbeat_map;

public:
    MissionControl();

private:
    // Event Loop
    void event_loop();

    // Selfcheck
    void self_check();

    // Mission Abort
    void mission_abort(std::string reason);

    // Fail-Safe checks
    void check_control(const interfaces::msg::Control &msg);

    // Heartbeat
    void heartbeat_callback(const interfaces::msg::Heartbeat &msg);
    void heartbeat_timer_callback();
};
