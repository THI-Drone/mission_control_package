#pragma once

#include <stddef.h>

#include <chrono>
#include <functional>
#include <map>
#include <memory>
#include <nlohmann/json.hpp>
#include <unordered_set>
#include <vector>

#include "common_package/common_node.hpp"
#include "event_loop_guard.hpp"
#include "mission_definition_file.hpp"
#include "rclcpp/rclcpp.hpp"
#include "structs.hpp"

// Message includes
#include "interfaces/msg/control.hpp"
#include "interfaces/msg/flight_state.hpp"
#include "interfaces/msg/gps_position.hpp"
#include "interfaces/msg/heartbeat.hpp"
#include "interfaces/msg/job_finished.hpp"
#include "interfaces/msg/mission_finished.hpp"
#include "interfaces/msg/mission_progress.hpp"
#include "interfaces/msg/mission_start.hpp"
#include "interfaces/msg/uav_health.hpp"
#include "interfaces/msg/uav_waypoint_command.hpp"
#include "interfaces/msg/waypoint.hpp"
#include "interfaces/msg/uav_command.hpp"

/**
 * @brief Enumeration representing the different states of a mission.
 */
typedef enum MissionState {
    prepare_mission,
    selfcheck,
    check_drone_configuration,
    armed,
    takeoff,
    decision_maker,
    fly_to_waypoint,
    detect_marker,
} MissionState_t;

class MissionControl : public common_lib::CommonNode {
    /**
     * @brief Class for handling the mission logic.
     */

   private:
    // General
    MissionState_t mission_state = prepare_mission;  //!< Main mission state
    std::string active_marker_name =
        "init";  //!< Name of the marker that is currently active
    bool job_finished_successfully =
        false;  //!< Will be set to true after a successfull job_finished
                //!< message was received from the active node
    nlohmann::json
        job_finished_payload;  //!< If a job finished successfully, the payload
                               //!< will be stored here for future usage
    bool state_first_loop =
        true;  //!< If set to true, the first event loop after a mission_state
               //!< change is happening. Will be set to false when calling
               //!< get_state_first_loop().
    std::map<std::string, ros_node>
        node_map;  //!< Has an entry for every ros node
    std::string active_node_id =
        "";  //!< node_id that is currently allowed to send data to the FCC
             //!< interface, set to "" if none is allowed

    size_t current_command_id = 0;
    std::vector<mission_file_lib::command>
        commands;  //!< Storage for current commands that
                   //!< will be executed one by one

    Position home_position;     //!< Storage for takeoff position
    Position current_position;  //!< Current position of the drone

    uint8_t current_flight_mode =
        interfaces::msg::FlightState::UNKNOWN;  //!< Current flight mode of the
                                                //!< drone as published in
                                                //!< FlightState

    bool drone_health_ok =
        false;  //!< True if all of the health indicators in the UAVHealth.msg
                //!< are ok, otherwise false

    // Wait Time
    static constexpr uint16_t wait_time_between_msgs =
        50;  //!< Wait time between messages in ms to avoid confusion
    bool wait_time_finished_ok = false; //!< True if wait time finished, otherwise false
    rclcpp::TimerBase::SharedPtr wait_time_timer;

    // Mission Definition File
    mission_file_lib::MissionDefinitionReader mission_definition_reader;
    std::unordered_set<std::string> executed_marker_names;

    // Mission Progress
    float mission_progress = 0.0;

    // Event Loop
    const uint32_t event_loop_time_delta_ms = 100;
    bool event_loop_active =
        true;  //!< If set to true, the event loop will be executed periodically
    rclcpp::TimerBase::SharedPtr event_loop_timer;

    // Job finished
    rclcpp::Subscription<interfaces::msg::JobFinished>::SharedPtr
        job_finished_subscription;

    // Waypoint Publisher
    rclcpp::Publisher<interfaces::msg::UAVWaypointCommand>::SharedPtr uav_waypoint_command_publisher;
    rclcpp::Publisher<interfaces::msg::UAVCommand>::SharedPtr uav_command_publisher;

    // Control
    rclcpp::Publisher<interfaces::msg::Control>::SharedPtr control_publisher;

    // Mission Start
    rclcpp::Subscription<interfaces::msg::MissionStart>::SharedPtr
        mission_start_subscription;

    // Mission Finished
    rclcpp::Publisher<interfaces::msg::MissionFinished>::SharedPtr
        mission_finished_publisher;

    // Fail-Safe Checks
    rclcpp::Subscription<interfaces::msg::UAVWaypointCommand>::SharedPtr
        control_subscription;

    // Heartbeat
    static constexpr uint16_t heartbeat_period_ms =
        500;  //!< Heartbeat period in ms
    static constexpr uint16_t heartbeat_max_timestamp_age_ms =
        50;  //!< Maximum allowed time difference between timestamp in heartbeat
             //!< message and current time
    bool heartbeat_received_all =
        false;  //!< true if all heartbeats we're received in the last
                //!< timeframe, otherwise false
    rclcpp::TimerBase::SharedPtr heartbeat_timer;
    rclcpp::Subscription<interfaces::msg::Heartbeat>::SharedPtr
        heartbeat_subscription;

    // FCC Bridge callbacks
    static constexpr uint16_t max_position_msg_time_difference_ms =
        100;  //!< Maximum age of a position message from FCC bridge

    static constexpr uint16_t max_progress_msg_time_difference_ms =
        500;  //!< Maximum age of a progress message from FCC bridge

    static constexpr uint16_t max_flight_state_msg_time_difference_ms =
        200;  //!< Maximum age of a flight state message from FCC bridge

    static constexpr uint16_t max_health_msg_time_difference_ms =
        500;  //!< Maximum age of a health message from FCC bridge

    // FCC Bridge callbacks
    rclcpp::Subscription<interfaces::msg::GPSPosition>::SharedPtr
        position_subscription;
    rclcpp::Subscription<interfaces::msg::MissionProgress>::SharedPtr
        mission_progress_subscription;
    rclcpp::Subscription<interfaces::msg::FlightState>::SharedPtr
        flight_state_subscription;
    rclcpp::Subscription<interfaces::msg::UAVHealth>::SharedPtr
        health_subscription;

   public:
    MissionControl();

   private:
    // Event Loop
    void event_loop();

    // Configuration Changes
    void set_standby_config();
    void set_mission_state(const MissionState_t new_mission_state);
    constexpr MissionState_t get_mission_state() const { return mission_state; }
    bool get_state_first_loop();
    bool get_job_finished_successfully();
    nlohmann::json get_job_finished_payload();

    // Wait time
    void init_wait(uint16_t wait_time_ms);
    bool wait_time_finished();
    void callback_wait_time();

    // Mission Progress
    bool current_mission_finished();

    // Active Node
    void set_active_node_id(std::string node_id);
    void clear_active_node_id();
    std::string get_active_node_id() const { return active_node_id; }

    // Active Marker
    void set_active_marker_name(const std::string &new_active_marker_name);
    std::string get_active_marker_name() const { return active_marker_name; }

    // Prepare Mission
    void mode_prepare_mission();

    // Selfcheck
    void mode_self_check();

    // Check drone configuration
    void mode_check_drone_configuration();

    // Job finished
    void job_finished_callback(const interfaces::msg::JobFinished &msg);

    // Mission start
    void mission_start(const interfaces::msg::MissionStart &msg);

    // Takeoff
    void initiate_takeoff();

    // Decision Maker
    void mode_decision_maker();

    // Fly to waypoint
    void mode_fly_to_waypoint();

    // Control Functions
    void send_control(const std::string &target_id, const bool active,
                      const std::string payload);
    void send_control_json(const std::string &target_id, const bool active,
                           const nlohmann::json payload_json);

    // Mission Abort
    void mission_abort(std::string reason);

    // Mission Finished successfully
    void mission_finished();

    // Fail-Safe checks
    void check_control(const interfaces::msg::UAVWaypointCommand &msg);

    // Heartbeat
    void heartbeat_callback(const interfaces::msg::Heartbeat &msg);
    void heartbeat_timer_callback();

    // FCC Bridge callbacks
    void position_callback(const interfaces::msg::GPSPosition &msg);
    void mission_progress_callback(const interfaces::msg::MissionProgress &msg);
    void flight_state_callback(const interfaces::msg::FlightState &msg);
    void health_callback(const interfaces::msg::UAVHealth &msg);
};