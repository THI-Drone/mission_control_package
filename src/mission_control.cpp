#include "mission_control.hpp"

using std::placeholders::_1;

MissionControl::MissionControl() : CommonNode("mission_control")
{
    // Register nodes
    const std::string node_names[] = {"telemetry_node", "waypoint_node"};

    for (const std::string &name : node_names)
    {
        node_map[name] = ros_node();
    }

    // Allowing waypoint_node to start the mission
    node_map["waypoint_node"].can_start_mission = true; // TODO change with real node_id

    // Initialize Heartbeat
    heartbeat_subscription =
        this->create_subscription<interfaces::msg::Heartbeat>(
            "heartbeat", 1,
            std::bind(&MissionControl::heartbeat_callback, this, _1));
    heartbeat_timer = this->create_wall_timer(
        1000ms, std::bind(&MissionControl::heartbeat_timer_callback, this));

    // Init Job_Finished
    job_finished_subscription =
        this->create_subscription<interfaces::msg::JobFinished>(
            "job_finished", 10,
            std::bind(&MissionControl::job_finished_callback, this, _1));

    // Mission Start
    mission_start_subscription =
        this->create_subscription<interfaces::msg::MissionStart>(
            "mission_start", 10,
            std::bind(&MissionControl::mission_start, this, _1));

    // Mission Finished
    mission_finished_publisher =
        this->create_publisher<interfaces::msg::MissionFinished>("mission_finished",
                                                              10);

    // Control Publisher
    control_publisher =
        this->create_publisher<interfaces::msg::Control>("control", 10);

    // Fail-Safe checks
    control_subscription = this->create_subscription<interfaces::msg::UAVWaypointCommand>(
        "uav_waypoint_command", 10, std::bind(&MissionControl::check_control, this, _1));

    // Initialize Event Loop
    event_loop_timer = this->create_wall_timer(
        std::chrono::milliseconds(event_loop_time_delta_ms),
        std::bind(&MissionControl::event_loop, this));
}

/**
 * @brief The event loop function of the MissionControl class.
 *
 * This function is responsible for handling events and controlling the mission.
 * It is called every {event_loop_time_delta_ms} ms.
 *
 * @note Can be disabled with the "event_loop_active" variable set to false
 */
void MissionControl::event_loop()
{
    if (!event_loop_active)
        return;

    switch (get_mission_state())
    {
    case prepare_mission:
        mode_prepare_mission();
        break;
    case selfcheck:
        mode_self_check();
        break;
    case check_drone_configuration:
        mode_check_drone_configuration();
        break;
    case armed:
        if (get_state_first_loop())
            set_standby_config();
        break;
    case takeoff:
        initiate_takeoff();
        break;
    case decision_maker:
        mode_decision_maker();
        break;
    case fly_to_waypoint:
        mode_fly_to_waypoint();
        break;

        // TODO implement other states

    default:
        RCLCPP_ERROR(this->get_logger(),
                     "MissionControl::event_loop: Unknown mission_state: %d",
                     get_mission_state());
        mission_abort("MissionControl::event_loop: Unknown mission_state");
    }
}

/**
 * @brief Sends a control message to a target node.
 *
 * This function sends a control message to a target node identified by its target_id.
 * The control message can be set as active or inactive, and can include a payload.
 * If the control message is set as active, the function also updates the active node ID.
 * If the target_id matches the current active node ID and the control message is set as inactive,
 * the function clears the active node ID.
 *
 * @param target_id The ID of the target node.
 * @param active Whether the control message is active or inactive.
 * @param payload The payload of the control message.
 */
void MissionControl::send_control(const std::string &target_id, const bool active, const std::string payload)
{
    interfaces::msg::Control msg;
    msg.target_id = target_id;
    msg.active = active;
    msg.payload = payload;

    if (active)
    {
        set_active_node_id(target_id);
    }
    else if (target_id == get_active_node_id())
    {
        clear_active_node_id();
    }

    RCLCPP_DEBUG(this->get_logger(),
                 "MissionControl::send_control: Publishing control message for node '%s': active: %d, payload: %s",
                 target_id.c_str(), active, payload.c_str());

    control_publisher->publish(msg);
}

/**
 * @brief Sends a control JSON to a target ID.
 *
 * This function sends a control JSON to the specified target ID. The control JSON contains information about the control action to be performed.
 *
 * @note Look at the docs at `send_control` for more information
 *
 * @param target_id The ID of the target to which the control JSON is to be sent.
 * @param active A boolean value indicating whether the control action is active or not.
 * @param payload_json The JSON payload containing the control action information.
 */
void MissionControl::send_control_json(const std::string &target_id, const bool active, const nlohmann::json payload_json)
{
    std::string payload = "";

    try
    {
        payload = payload_json.dump();
    }
    catch (const nlohmann::json::type_error &e)
    {
        mission_abort("MissionControl::send_control_json: Failed to dump payload_json: " + (std::string)e.what());
    }

    return send_control(target_id, active, payload);
}

/**
 * @brief Aborts the mission.
 *
 * This function is responsible for aborting the mission and exiting the
 * program. A MissionAbort message will be published before exiting.
 *
 * @param reason The reason for aborting the mission.
 */
void MissionControl::mission_abort(std::string reason)
{
    RCLCPP_FATAL(this->get_logger(),
                 "MissionControl::mission_abort: Aborting mission, reason: %s",
                 reason.c_str());
    RCLCPP_FATAL(this->get_logger(),
                 "MissionControl::mission_abort: Last active node: %s",
                 get_active_node_id().c_str());
    RCLCPP_FATAL(this->get_logger(),
                 "MissionControl::mission_abort: Internal state: %d",
                 get_mission_state());

    // Reset internal state
    clear_active_node_id();

    // Publish abort message
    interfaces::msg::MissionFinished msg;
    msg.sender_id = get_name();
    msg.error_code = EXIT_FAILURE;
    msg.reason = reason;
    mission_finished_publisher->publish(msg);

    // Stop node
    RCLCPP_INFO(
        this->get_logger(),
        "MissionControl::mission_abort: Mission finished failure message sent, exiting now");

    exit(EXIT_FAILURE);
}

/**
 * @brief Handles the completion of a mission.
 *
 * This function is called when a mission is finished successfully. It performs the following tasks:
 * 1. Resets the internal state of the mission control.
 * 2. Publishes a mission abort message with the sender ID and reason for mission completion.
 * 3. Stops the node by exiting with a success status code.
 */
void MissionControl::mission_finished()
{
    RCLCPP_INFO(this->get_logger(),
                "MissionControl::mission_finished: Mission finished successfully");

    // Reset internal state
    clear_active_node_id();

    // Publish abort message
    interfaces::msg::MissionFinished msg;
    msg.sender_id = get_name();
    msg.error_code = EXIT_SUCCESS;
    msg.reason = "Successfully finished mission";
    mission_finished_publisher->publish(msg);

    // Stop node
    RCLCPP_INFO(
        this->get_logger(),
        "MissionControl::mission_finished: Mission finished message sent, exiting now");

    exit(EXIT_SUCCESS);
}