#pragma once

#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <functional>
#include <map>
#include <memory>
#include <unordered_set>
#include <nlohmann/json.hpp>
#include <stddef.h>
#include <vector>

#include "common_package/common_node.hpp"
#include "event_loop_guard.hpp"
#include "structs.hpp"
#include "mission_definition_file.hpp"

// Message includes
#include "interfaces/msg/control.hpp"
#include "interfaces/msg/fly_to_coord.hpp"
#include "interfaces/msg/heartbeat.hpp"
#include "interfaces/msg/job_finished.hpp"
#include "interfaces/msg/mission_abort.hpp"
#include "interfaces/msg/mission_start.hpp"

/**
 * @brief Enumeration representing the different states of a mission.
 */
typedef enum MissionState
{
    prepare_mission,
    selfcheck,
    check_drone_configuration,
    armed,
    takeoff,
    decision_maker,
    fly_to_waypoint,
    detect_marker,
    drop_payload,
    end_mission,
} MissionState_t;

class MissionControl : public common_lib::CommonNode
{
    /**
     * @brief Class for handling the mission logic.
     */

private:
    // General
    MissionState_t mission_state = prepare_mission; /// Main mission state
    std::string active_marker_name = "init";        /// Name of the marker that is currently active
    bool job_finished_successfully =
        false; /// Will be set to true after a successfull job_finished message
               /// was received from the active node
    nlohmann::json
        job_finished_payload; /// If a job finished successfully, the payload will
                              /// be stored here for future usage
    bool state_first_loop =
        true;                                 /// If set to true, the first event loop after a mission_state
                                              /// change is happening. Will be set to false when calling
                                              /// get_state_first_loop().
    std::map<std::string, ros_node> node_map; /// Has an entry for every ros node
    std::string active_node_id =
        ""; /// node_id that is currently allowed to send data to the FCC
            /// interface, set to "" if none is allowed

    size_t current_command_id = 0;
    std::vector<mission_file_lib::command> commands; /// Storage for current commands that
                                                     /// will be executed one by one

    // Mission Definition File
    mission_file_lib::MissionDefinitionReader mission_definition_reader;
    std::unordered_set<std::string> executed_marker_names;

    // Event Loop
    const uint32_t event_loop_time_delta_ms = 100;
    bool event_loop_active =
        true; /// If set to true, the event loop will be executed periodically
    rclcpp::TimerBase::SharedPtr event_loop_timer;

    // Job finished
    rclcpp::Subscription<interfaces::msg::JobFinished>::SharedPtr
        job_finished_subscription;

    // Control
    rclcpp::Publisher<interfaces::msg::Control>::SharedPtr control_publisher;

    // Mission Start
    rclcpp::Subscription<interfaces::msg::MissionStart>::SharedPtr
        mission_start_subscription;

    // Mission Abort
    rclcpp::Publisher<interfaces::msg::MissionAbort>::SharedPtr
        mission_abort_publisher;

    // Fail-Safe Checks
    rclcpp::Subscription<interfaces::msg::FlyToCoord>::SharedPtr
        control_subscription;

    // Heartbeat
    static constexpr uint16_t heartbeat_period_ms =
        500; /// Heartbeat period in ms
    bool heartbeat_received_all =
        false; /// true if all heartbeats we're received in the last timeframe,
               /// otherwise false
    rclcpp::TimerBase::SharedPtr heartbeat_timer;
    rclcpp::Subscription<interfaces::msg::Heartbeat>::SharedPtr
        heartbeat_subscription;

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
    void send_control(const std::string &target_id, const bool active, const std::string payload);
    void send_control_json(const std::string &target_id, const bool active, const nlohmann::json payload_json);

    // Mission Abort
    void mission_abort(std::string reason);

    // Mission Finished successfully
    void mission_finished();

    // Fail-Safe checks
    void check_control(const interfaces::msg::FlyToCoord &msg);

    // Heartbeat
    void heartbeat_callback(const interfaces::msg::Heartbeat &msg);
    void heartbeat_timer_callback();
};
