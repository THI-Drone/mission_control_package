#include "mission_control.hpp"

using std::placeholders::_1;

/**
 * @brief Constructor for the MissionControl class.
 *
 * @note The `MDF_FILE_PATH` must be provided either through the NodeOptions
 * or as a ROS parameter. Set it to `DEFAULT` to use the default file path.
 *
 * @param options The optional NodeOptions for the node.
 */
MissionControl::MissionControl(const rclcpp::NodeOptions &options)
    : CommonNode(common_lib::node_names::MISSION_CONTROL, options) {
    // Path to mission definition file
    try {
        const std::string mdf_default_file_path =
            "src/mission_control_package/assets/mission_test.json";

        // Try to read parameter
        mdf_file_path =
            this->declare_parameter<std::string>("MDF_FILE_PATH", "");

        // Check if no parameter was provided
        if (mdf_file_path.size() <= 0) {
            RCLCPP_FATAL(this->get_logger(),
                         "MissionControl::%s: 'MDF_FILE_PATH' parameter not "
                         "set. Exiting.",
                         __func__);

            exit(EXIT_FAILURE);
        }

        if (mdf_file_path == "DEFAULT") {
            // Default parameter was provided. Therefore, the default file path
            // will be used.

            mdf_file_path = mdf_default_file_path;

            RCLCPP_WARN(this->get_logger(),
                        "MissionControl::%s: Using default Mission Definition "
                        "File at: '%s'",
                        __func__, mdf_default_file_path.c_str());
        } else {
            // A custom parameter was provided and will be used

            RCLCPP_INFO(this->get_logger(),
                        "MissionControl::%s: Using custom Mission Definition "
                        "File at: '%s'",
                        __func__, mdf_file_path.c_str());
        }
    } catch (const rclcpp::exceptions::InvalidParameterTypeException &e) {
        // Parameter is not a string
        RCLCPP_FATAL(this->get_logger(),
                     "MissionControl::%s: 'MDF_FILE_PATH' parameter is of "
                     "wrong type. Expected string.",
                     __func__);

        mission_abort(
            "'MDF_FILE_PATH' parameter is of wrong type. Expected string.");
    }

    // Initialize Heartbeat
    heartbeat_subscription =
        this->create_subscription<interfaces::msg::Heartbeat>(
            common_lib::topic_names::Heartbeat, 1,
            std::bind(&MissionControl::heartbeat_callback, this, _1));
    heartbeat_timer = this->create_wall_timer(
        1000ms, std::bind(&MissionControl::heartbeat_timer_callback, this));

    // Init Job_Finished
    job_finished_subscription =
        this->create_subscription<interfaces::msg::JobFinished>(
            common_lib::topic_names::JobFinished, 10,
            std::bind(&MissionControl::job_finished_callback, this, _1));

    // Mission Start
    mission_start_subscription =
        this->create_subscription<interfaces::msg::MissionStart>(
            common_lib::topic_names::MissionStart, 10,
            std::bind(&MissionControl::mission_start_callback, this, _1));

    // Mission Finished
    mission_finished_publisher =
        this->create_publisher<interfaces::msg::MissionFinished>(
            common_lib::topic_names::MissionFinished, 10);

    // UAV Command Publisher
    uav_waypoint_command_publisher =
        this->create_publisher<interfaces::msg::UAVWaypointCommand>(
            common_lib::topic_names::UAVWaypointCommand, 10);

    uav_command_publisher = this->create_publisher<interfaces::msg::UAVCommand>(
        common_lib::topic_names::UAVCommand, 10);

    // FCC Bridge subscribes
    mission_progress_subscription =
        this->create_subscription<interfaces::msg::MissionProgress>(
            common_lib::topic_names::MissionProgress, 10,
            std::bind(&MissionControl::mission_progress_callback, this, _1));

    position_subscription =
        this->create_subscription<interfaces::msg::GPSPosition>(
            common_lib::topic_names::GPSPosition, 10,
            std::bind(&MissionControl::position_callback, this, _1));

    flight_state_subscription =
        this->create_subscription<interfaces::msg::FlightState>(
            common_lib::topic_names::FlightState, 10,
            std::bind(&MissionControl::flight_state_callback, this, _1));

    health_subscription = this->create_subscription<interfaces::msg::UAVHealth>(
        common_lib::topic_names::UAVHealth, 10,
        std::bind(&MissionControl::health_callback, this, _1));

    // Control Publisher
    control_publisher = this->create_publisher<interfaces::msg::Control>(
        common_lib::topic_names::Control, 10);

    // Safety Limits Publisher
    safety_limits_publisher =
        this->create_publisher<interfaces::msg::SafetyLimits>(
            common_lib::topic_names::SafetyLimits, 10);

    // Fail-Safe checks
    waypoint_command_subscription =
        this->create_subscription<interfaces::msg::UAVWaypointCommand>(
            common_lib::topic_names::UAVWaypointCommand, 10,
            std::bind(&MissionControl::waypoint_command_callback, this, _1));

    command_subscription =
        this->create_subscription<interfaces::msg::UAVCommand>(
            common_lib::topic_names::UAVCommand, 10,
            std::bind(&MissionControl::command_callback, this, _1));

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
void MissionControl::event_loop() {
    if (!event_loop_active) return;

    switch (get_mission_state()) {
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
            if (get_state_first_loop()) set_standby_config();
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
        case detect_marker:
            mode_detect_marker();
            break;
        case set_marker:
            mode_set_marker();
            break;
        default:
            RCLCPP_ERROR(this->get_logger(),
                         "MissionControl::%s: Unknown mission_state: %d",
                         __func__, get_mission_state());
            mission_abort("MissionControl::" + (std::string) __func__ +
                          ": Unknown mission_state");
    }
}

/**
 * @brief Sends a control message to a target node.
 *
 * This function sends a control message to a target node identified by its
 * target_id. The control message can be set as active or inactive, and can
 * include a payload. If the control message is set as active, the function also
 * updates the active node ID. If the target_id matches the current active node
 * ID and the control message is set as inactive, the function clears the active
 * node ID.
 *
 * @note Resets `mission_progress` to 0
 *
 * @param target_id The ID of the target node.
 * @param active Whether the control message is active or inactive.
 * @param payload The payload of the control message.
 */
void MissionControl::send_control(const std::string &target_id,
                                  const bool active,
                                  const std::string &payload) {
    interfaces::msg::Control msg;
    msg.target_id = target_id;
    msg.active = active;
    msg.payload = payload;

    if (active) {
        // Set node as active
        set_active_node_id(target_id);
    } else if (target_id == get_active_node_id()) {
        // Reset active node id if it is currently active
        clear_active_node_id();
    }

    RCLCPP_DEBUG(this->get_logger(),
                 "MissionControl::%s: Publishing control message for "
                 "node '%s': active: %d, payload: %s",
                 __func__, target_id.c_str(), active, payload.c_str());

    control_publisher->publish(msg);

    // Reset mission progress
    mission_progress = 0.0;
}

/**
 * @brief Sends a control JSON to a target ID.
 *
 * This function sends a control JSON to the specified target ID. The control
 * JSON contains information about the control action to be performed.
 *
 * @note Look at the docs at `send_control` for more information
 *
 * @param target_id The ID of the target to which the control JSON is to be
 * sent.
 * @param active A boolean value indicating whether the control action is active
 * or not.
 * @param payload_json The JSON payload containing the control action
 * information.
 */
void MissionControl::send_control_json(const std::string &target_id,
                                       const bool active,
                                       const nlohmann::json &payload_json) {
    std::string payload = "";

    try {
        payload = payload_json.dump();
    } catch (const nlohmann::json::type_error &e) {
        mission_abort((std::string) "MissionControl::" + __func__ +
                      ": Failed to dump payload_json: " + e.what());
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
void MissionControl::mission_abort(std::string reason) {
    RCLCPP_FATAL(this->get_logger(),
                 "MissionControl::%s: Aborting mission, reason: %s", __func__,
                 reason.c_str());

    std::string last_active_node_id_str = "";
    if (get_last_active_node_id().size() > 0) {
        last_active_node_id_str =
            " - Last active node id: '" + get_last_active_node_id() + "'";
    }

    RCLCPP_INFO(this->get_logger(),
                "MissionControl::%s: Current active node: '%s'%s", __func__,
                get_active_node_id().c_str(), last_active_node_id_str.c_str());

    try {
        RCLCPP_INFO(this->get_logger(),
                    "MissionControl::%s: Internal state: '%s'", __func__,
                    get_mission_state_str());
    } catch (const std::runtime_error &e) {
        RCLCPP_ERROR(this->get_logger(),
                     "MissionControl::%s: Failed to print mission state: %s",
                     __func__, e.what());

        RCLCPP_ERROR(this->get_logger(),
                     "MissionControl::%s: Internal state: '%d' [INVALID]",
                     __func__, get_mission_state());
    }

    // Explicitly deactivate currently active node (if not empty)
    if (get_active_node_id().size() > 0) {
        RCLCPP_INFO(
            this->get_logger(),
            "MissionControl::%s: Explicitly deactivated active node: '%s'",
            __func__, get_active_node_id().c_str());

        send_control(get_active_node_id(), false, "{}");
    }

    // Reset internal state
    clear_active_node_id();

    // Publish abort message
    interfaces::msg::MissionFinished msg;
    msg.sender_id = get_name();
    msg.error_code = EXIT_FAILURE;
    msg.reason = reason;
    mission_finished_publisher->publish(msg);

    // Stop node
    RCLCPP_INFO(this->get_logger(),
                "MissionControl::%s: Mission finished failure "
                "message sent, exiting now",
                __func__);

    exit(EXIT_FAILURE);
}

/**
 * @brief Handles the completion of a mission.
 *
 * This function is called when a mission is finished successfully. It performs
 * the following tasks:
 * 1. Resets the internal state of the mission control.
 * 2. Publishes a mission abort message with the sender ID and reason for
 * mission completion.
 * 3. Stops the node by exiting with a success status code.
 */
void MissionControl::mission_finished() {
    RCLCPP_INFO(this->get_logger(),
                "MissionControl::%s: Mission finished successfully", __func__);

    // Explicitly deactivate currently active node (if not empty)
    if (get_active_node_id().size() > 0) {
        RCLCPP_INFO(
            this->get_logger(),
            "MissionControl::%s: Explicitly deactivated active node: '%s'",
            __func__, get_active_node_id().c_str());

        send_control(get_active_node_id(), false, "{}");
    }

    // Reset internal state
    clear_active_node_id();

    // Publish abort message
    interfaces::msg::MissionFinished msg;
    msg.sender_id = get_name();
    msg.error_code = EXIT_SUCCESS;
    msg.reason = "Successfully finished mission";
    mission_finished_publisher->publish(msg);

    // Stop node
    RCLCPP_INFO(this->get_logger(),
                "MissionControl::%s: Mission finished message "
                "sent, exiting now",
                __func__);

    exit(EXIT_SUCCESS);
}
