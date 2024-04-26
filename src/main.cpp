#include "main.hpp"

using std::placeholders::_1;

MissionControl::MissionControl() : CommonNode("mission_control")
{
    // Register nodes
    const std::string node_names[] = {"/telemetry_node", "/waypoint_node"};

    for (const std::string &name : node_names)
    {
        node_map[name] = ros_node();
    }

    // Allowing waypoint_node to start the mission
    node_map["/waypoint_node"].can_start_mission = true; // TODO change with real node_id

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

    // Mission Abort
    mission_abort_publisher =
        this->create_publisher<interfaces::msg::MissionAbort>("mission_abort",
                                                              10);

    // Control Publisher
    control_publisher =
        this->create_publisher<interfaces::msg::Control>("control", 10);

    // Fail-Safe checks
    control_subscription = this->create_subscription<interfaces::msg::FlyToCoord>(
        "fly_to_coord", 10, std::bind(&MissionControl::check_control, this, _1));

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
 * @brief Sets the standby configuration
 *
 * @note No nodes are allowed to send to the FCC interface in this configuration
 */
void MissionControl::set_standby_config() { clear_active_node_id(); }

/**
 * @brief Sets the mission state to a new value.
 *
 * This function sets the mission state to the specified new value. If the new
 * value is different from the current mission state, the
 * `job_finished_successfully` flag is set to false and the `state_first_loop`
 * flag is set to true. After updating the mission state, a debug message is
 * logged.
 *
 * @param new_mission_state The new mission state to set.
 */
void MissionControl::set_mission_state(const MissionState_t new_mission_state)
{
    if (mission_state != new_mission_state)
    {
        job_finished_successfully = false;
        state_first_loop = true;
    }

    mission_state = new_mission_state;

    RCLCPP_DEBUG(this->get_logger(),
                 "MissionControl::set_mission_state: Set mission state to %d",
                 mission_state);
}

/**
 * @brief Sets the active node ID.
 *
 * This function sets the active node ID to the specified value.
 * Use the `clear_active_node_id` function instead if no node is allowed to send
 * to the FCC interface.
 *
 * @param node_id The ID of the node to set as active.
 */
void MissionControl::set_active_node_id(std::string node_id)
{
    RCLCPP_DEBUG(this->get_logger(),
                 "MissionControl::set_active_node_id: Set active_node_id to %s",
                 node_id.c_str());
    active_node_id = node_id;
}

/**
 * @brief Sets the active marker name.
 *
 * This function sets the active marker name to the provided value.
 *
 * @param new_active_marker_name The new active marker name.
 */
void MissionControl::set_active_marker_name(const std::string &new_active_marker_name)
{
    RCLCPP_DEBUG(this->get_logger(),
                 "MissionControl::set_active_marker_name: Set active_marker_name to %s",
                 new_active_marker_name.c_str());

    active_marker_name = new_active_marker_name;
}

/**
 * @brief Clears the active node ID.
 *
 * This function clears the active node ID by setting it to an empty string.
 */
void MissionControl::clear_active_node_id()
{
    RCLCPP_DEBUG(this->get_logger(),
                 "MissionControl::clear_active_node_id: Cleared active node id");
    active_node_id = "";
}

/**
 * @brief Checks if the first loop was already triggered and returns the result.
 *
 * This function checks the state of the first loop and returns false if it was
 * already triggered, otherwise it returns true. After returning the state, it
 * sets the first loop state to false.
 *
 * @return False if the first loop was already triggered, true otherwise.
 */
bool MissionControl::get_state_first_loop()
{
    // Check if first loop was already triggered
    if (!state_first_loop)
        return false;

    // Set first loop to false and return true
    state_first_loop = false;
    return true;
}

/**
 * @brief Checks if the job finished successfully.
 *
 * This function checks if the job finished successfully and returns the result.
 * If the job has finished successfully, the flag is reset to false.
 *
 * @return true if the job finished successfully, false otherwise.
 */
bool MissionControl::get_job_finished_successfully()
{
    if (job_finished_successfully)
    {
        job_finished_successfully = false;
        return true;
    }
    else
    {
        return false;
    }
}

/**
 * @brief Retrieves the payload for a finished job.
 *
 * This function returns the payload for a finished job and clears the internal
 * `job_finished_payload` variable.
 *
 * @return The payload for the finished job.
 */
nlohmann::json MissionControl::get_job_finished_payload()
{
    nlohmann::json res = job_finished_payload;

    // Clear the job_finished_payload
    job_finished_payload = nlohmann::json();

    return res;
}

/**
 * @brief Prepares the mission by reading the mission file and setting the standby configuration.
 */
void MissionControl::mode_prepare_mission()
{
    if (get_state_first_loop())
    {
        RCLCPP_INFO(this->get_logger(), "MissionControl::mode_prepare_mission: Prepare mission started");
        set_standby_config();

        std::string file_path = "src/mission_control_package/assets/mission_test.json";
        try
        {
            mission_definition_reader.read_file(file_path, false);
        }
        catch (const std::runtime_error &e)
        {
            mission_abort("MissionControl::mode_prepare_mission: Failed to read mission file: " + (std::string)e.what());
        }

        RCLCPP_INFO(this->get_logger(),
                    "MissionControl::mode_prepare_mission: Prepare mission finished");
        set_mission_state(selfcheck);
    }
}

/**
 * @brief Executes the self-check of the mission control.
 *
 * This function performs a self-check by waiting for a good GPS signal and
 * checking if all heartbeats are received within a given timeframe (30s). If
 * the self-check fails, the mission is aborted.
 */
void MissionControl::mode_self_check()
{
    static uint32_t i;

    if (get_state_first_loop())
    {
        RCLCPP_INFO(this->get_logger(),
                    "MissionControl::mode_self_check: Self check started");
        set_standby_config();
        i = 0;

        // TODO implement waiting for good GPS signal
    }

    const uint32_t max_wait_time = (30 * 1000) / event_loop_time_delta_ms;
    if (i % (1000 / event_loop_time_delta_ms) == 0)
    {
        RCLCPP_INFO(
            this->get_logger(),
            "MissionControl::mode_self_check: Waiting for heartbeats: %us / %us",
            (i * event_loop_time_delta_ms) / 1000,
            (max_wait_time * event_loop_time_delta_ms) / 1000);
    }

    i++;

    if (i >= max_wait_time)
    {
        RCLCPP_ERROR(this->get_logger(),
                     "MissionControl::mode_self_check: Self check failed: Not all "
                     "heartbeats received in the given timeframe (%us)",
                     (max_wait_time * event_loop_time_delta_ms) / 1000);
        mission_abort("Self check failed: Not all heartbeats received in the given "
                      "timeframe");
    }

    if (heartbeat_received_all)
    {
        RCLCPP_INFO(this->get_logger(),
                    "MissionControl::mode_self_check: Self check finished");
        set_mission_state(check_drone_configuration);
    }
}

/**
 * @brief Checks the drone configuration and sets the mission state to armed.
 *
 * This function checks if the drone configuration is valid by performing the following checks:
 * 1. Check if position is inside of geofence.
 * 2. Check that drone is on the ground.
 *
 * @note After performing the checks, the mission state is set to armed.
 */
void MissionControl::mode_check_drone_configuration()
{
    if (get_state_first_loop())
    {
        RCLCPP_INFO(this->get_logger(), "MissionControl::mode_check_drone_configuration: Check drone configuration started");
        set_standby_config();

        // TODO implement check if position is inside of geofence

        // TODO implement check that drone is on the ground
    }

    RCLCPP_INFO(this->get_logger(),
                "MissionControl::mode_check_drone_configuration: Check drone configuration finished");
    set_mission_state(armed);
    RCLCPP_INFO(this->get_logger(),
                "MissionControl::mode_check_drone_configuration: Mission Control armed");
}

/**
 * @brief Callback function for handling job_finished messages.
 *
 * This function is called when a job_finished message is received. It performs the following tasks:
 * 1. Parses the JSON payload from the message.
 * 2. Checks the error code of the message. If it is not EXIT_SUCCESS, aborts the mission.
 * 3. Ignores the message if the sender node is not active.
 * 4. Sends a control message to explicitly deactivate the sender node.
 * 5. Updates the internal state by clearing the active node ID.
 * 6. Stores the payload for future use.
 * 7. Sets the job_finished_successfully flag to true.
 *
 * @param msg The job_finished message received.
 */
void MissionControl::job_finished_callback(
    const interfaces::msg::JobFinished &msg)
{
    // Try to parse JSON
    nlohmann::json payload;
    try
    {
        payload = nlohmann::json::parse(msg.payload);
    }
    catch (const nlohmann::json::exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "MissionControl::job_finished_callback: "
                                         "Failed to parse json in payload");
    }

    // Check error code
    if (msg.error_code != EXIT_SUCCESS)
    {
        // Abort mission
        std::string error_msg = "MissionControl::job_finished_callback: Received "
                                "job_finished message with error_code: " +
                                msg.error_code;

        if (payload.contains("error_msg"))
            error_msg += ". Error Message: " + payload.at("error_msg").dump();

        mission_abort(error_msg);
        return;
    }

    // Ignore message if sender node is not active as this might be a delayed
    // message or a programming error in the sender node
    if (msg.sender_id != get_active_node_id())
    {
        RCLCPP_WARN(this->get_logger(),
                    "MissionControl::job_finished_callback: Got a successfull "
                    "job_finished message from an inactive node: %s",
                    msg.sender_id.c_str());
        return;
    }

    // Send control message to explicitly deactivate node
    send_control(msg.sender_id, false, "{}");

    // Deactivate node in internal state
    clear_active_node_id();

    // Store payload for future use
    job_finished_payload = msg.payload;

    // Set job_finished_successfully flag to true indicating receiving a
    // successfull job_finished message
    RCLCPP_INFO(this->get_logger(),
                "MissionControl::job_finished_callback: Job finished "
                "successfully: node_id %s",
                msg.sender_id.c_str());
    job_finished_successfully = true;
}

/**
 * @brief Starts the mission based on the received command.
 *
 * This function is called when a mission start command is received. It verifies
 * the current mission state (must be armed), checks if the sender node is known
 * and allowed to start the mission, and then initiates the mission.
 *
 * @param msg The mission start command message
 */
void MissionControl::mission_start(const interfaces::msg::MissionStart &msg)
{
    RCLCPP_INFO(
        this->get_logger(),
        "MissionControl::mission_start: Received mission start command from %s",
        msg.sender_id.c_str());

    // Check mission state is armed
    if (get_mission_state() != armed)
    {
        RCLCPP_ERROR(
            get_logger(),
            "Mission start command received without being in 'armed' state");
        return;
    }

    // Check that sender node id is known
    if (node_map.find(msg.sender_id) == node_map.end())
    {
        RCLCPP_ERROR(get_logger(),
                     "MissionControl::mission_start: Sender node unknown: %s",
                     msg.sender_id.c_str());
        return;
    }

    // Check that sender has permission to start mission
    if (!node_map[msg.sender_id].can_start_mission)
    {
        RCLCPP_ERROR(get_logger(),
                     "MissionControl::mission_start: Sender is not allowed to "
                     "start the mission: %s",
                     msg.sender_id.c_str());
        return;
    }

    // Set mission to 'init' marker
    set_active_marker_name("init");

    // Start mission
    RCLCPP_INFO(this->get_logger(),
                "MissionControl::mission_start: Mission started");
    set_mission_state(takeoff);
    RCLCPP_INFO(this->get_logger(),
                "MissionControl::mission_start: Takeoff initiated");
}

void MissionControl::initiate_takeoff()
{
    if (get_state_first_loop())
    {
        RCLCPP_INFO(this->get_logger(),
                    "MissionControl::initiate_takeoff: Takeoff initated");

        // TODO check if drone is in the air -> abort if false

        // TODO save takeoff coordinates

        // TODO initiate takeoff
    }

    // TODO monitor takeoff

    // If takeoff finished
    RCLCPP_INFO(this->get_logger(),
                "MissionControl::initiate_takeoff: Takeoff finished");

    set_mission_state(decision_maker);
    RCLCPP_INFO(
        this->get_logger(),
        "MissionControl::initiate_takeoff: make_decision state activated");
}

/**
 * @brief Makes decisions on the mode of operation for the mission control.
 *
 * This function is responsible for making decisions on the mode of operation for the mission control.
 * It checks if there are commands left to be executed and moves to the next command if necessary.
 * If there are no more commands, it moves to the next marker and retrieves new commands from the marker.
 * It also checks the type of the current command and sets the mission state accordingly.
 *
 * @note This function assumes that the `commands` vector and `current_command_id` variable have been properly initialized and that the active_marker_name is updated beforehand if a new marker was detected.
 */
void MissionControl::mode_decision_maker()
{
    EventLoopGuard elg(&event_loop_active, false);

    RCLCPP_INFO(this->get_logger(), "MissionControl::mode_decision_maker: Decision Maker started");

    RCLCPP_INFO(this->get_logger(), "MissionControl::mode_decision_maker: active_marker_name: '%s', current_command_id: %ld, command count: %ld", get_active_marker_name().c_str(), current_command_id, commands.size());

    // Check if there are commands left that need to be executed
    if (commands.size() > 0 && current_command_id < commands.size() - 1)
    {
        current_command_id++; // move command id one further
        RCLCPP_DEBUG(this->get_logger(), "MissionControl::mode_decision_maker: Moved command id to: %ld", current_command_id);
    }
    else // Move to next marker
    {
        // Store new commands from active marker in storage
        current_command_id = 0;

        // Check that marker has not been executed before
        std::string active_marker = get_active_marker_name();
        RCLCPP_DEBUG(this->get_logger(), "MissionControl::mode_decision_maker: Getting new commands for active marker name: '%s'", active_marker.c_str());

        if (executed_marker_names.find(active_marker) != executed_marker_names.end())
            mission_abort("MissionControl::mode_decision_maker: Got the same active marker two times: " + active_marker);

        executed_marker_names.insert(active_marker);

        try
        {
            // Get new marker commands and store them in cache
            commands = mission_definition_reader.get_marker_commands(active_marker);
        }
        catch (const std::runtime_error &e)
        {
            mission_abort("MissionControl::mode_decision_maker: Failed to get new commands: " + (std::string)e.what());
        }
    }

    // Check if command list is empty
    if (commands.size() <= 0)
        mission_abort("MissionControl::mode_decision_maker: Comand list is empty");

    // Switch mode based on the current command
    std::string &current_command_type = commands.at(current_command_id).type;
    RCLCPP_DEBUG(this->get_logger(), "MissionControl::mode_decision_maker: Switching mode based on command type: '%s'", current_command_type.c_str());

    if (current_command_type == "waypoint")
        set_mission_state(fly_to_waypoint);
    else if (current_command_type == "detect_marker")
        set_mission_state(detect_marker);
    else if (current_command_type == "drop_payload")
        set_mission_state(drop_payload);
    else if (current_command_type == "end_mission")
        mission_finished();
    else
        mission_abort("MissionControl::mode_decision_maker: Unknown command type");
}

/**
 * Executes the "fly to waypoint" mode of the mission control.
 * Activates the Waypoint Node and sends the command data as payload.
 * If the job is finished successfully, returns to the decision maker for the next command.
 */
void MissionControl::mode_fly_to_waypoint()
{
    if (get_state_first_loop())
    {
        // Activate Waypoint Node and send the command data as payload
        send_control_json("/waypoint_node", true, commands.at(current_command_id).data);
    }

    // If job finished, return to decision maker for next command
    if (get_job_finished_successfully())
    {
        set_mission_state(decision_maker);
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
    interfaces::msg::MissionAbort msg;
    msg.sender_id = get_fully_qualified_name();
    msg.reason = reason;
    mission_abort_publisher->publish(msg);

    // Stop node
    RCLCPP_INFO(
        this->get_logger(),
        "MissionControl::mission_abort: Mission abort message sent, exiting now");

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
    interfaces::msg::MissionAbort msg;
    msg.sender_id = get_fully_qualified_name();
    msg.reason = "Successfully finished mission";
    mission_abort_publisher->publish(msg);

    // Stop node
    RCLCPP_INFO(
        this->get_logger(),
        "MissionControl::mission_abort: Mission abort message sent, exiting now");

    exit(EXIT_SUCCESS);
}

// Fail-Safe checks
/**
 * @brief Checks the sender of each FlyToCoord message
 *
 * Aborts mission if an unauthorized sender is sending on the fly_to_coord
 * topic.
 *
 * @param msg The FlyToCoord message to be checked.
 */
void MissionControl::check_control(const interfaces::msg::FlyToCoord &msg)
{
    RCLCPP_DEBUG(
        this->get_logger(),
        "MissionControl::check_control: Checking FlyToCoord message from %s",
        msg.sender_id.c_str());

    if (msg.sender_id != get_active_node_id())
        mission_abort("Unauthorized node sending on fly_to_coord topic registered");

    RCLCPP_DEBUG(
        this->get_logger(),
        "MissionControl::check_control: Checking FlyToCoord message successfull");
}

// Heartbeat
/**
 * @brief Callback function for handling heartbeat messages.
 *
 * This function is called when a heartbeat message is received. It performs the
 * following tasks:
 * - Checks if the heartbeat message is from the current node itself and
 * discards it.
 * - Checks if the sender of the heartbeat message is registered in the node
 * map. If not, it logs an error and returns.
 * - Checks if the active state of the sender matches the internal state. If
 * not, it logs a fatal error and aborts the mission.
 * - Checks the tick value of the heartbeat message. If it is invalid or has
 * already been received, it logs an error and returns.
 * - Checks that the received timestamp is not older than 10ms. If it is too
 * old, it logs an error and aborts the mission.
 * - Updates the tick value and sets the received flag in the heartbeat payload
 * of the sender.
 * - Logs the received heartbeat information (sender ID, tick value, and active
 * state) in debug mode.
 *
 * @param msg The received heartbeat message.
 */
void MissionControl::heartbeat_callback(const interfaces::msg::Heartbeat &msg)
{
    // Throw away own heartbeat
    if (msg.sender_id == get_fully_qualified_name())
        return;

    if (node_map.find(msg.sender_id) == node_map.end())
    {
        RCLCPP_ERROR(get_logger(),
                     "MissionControl::heartbeat_callback: Received unregistered "
                     "heartbeat: %s",
                     msg.sender_id.c_str());
        return;
    }

    // Check that active state matches the internal state
    if (msg.active && (msg.sender_id != get_active_node_id()))
    {
        RCLCPP_FATAL(get_logger(),
                     "MissionControl::heartbeat_callback: Node '%s' is active even "
                     "though it should be deactive",
                     msg.sender_id.c_str());
        mission_abort("A node was active even though it should be deactive");
    }
    if ((!msg.active) && (msg.sender_id == get_active_node_id()))
    {
        RCLCPP_FATAL(get_logger(),
                     "MissionControl::heartbeat_callback: Node '%s' is deactive "
                     "even though it should be active",
                     msg.sender_id.c_str());
        mission_abort("A node was deactive even though it should be active");
    }

    // Check tick
    heartbeat_payload &heartbeat = node_map[msg.sender_id].hb_payload;
    if ((msg.tick == heartbeat.tick) ||
        (msg.tick <= heartbeat.tick && msg.tick != 0))
    {
        RCLCPP_ERROR(
            get_logger(),
            "MissionControl::heartbeat_callback: Invalid tick received from: %s",
            msg.sender_id.c_str());
        return;
    }
    heartbeat.tick = msg.tick;

    // Check timestamp
    rclcpp::Time timestamp_now = this->now();
    if (timestamp_now - rclcpp::Time(msg.time_stamp) >
        rclcpp::Duration(std::chrono::duration<int64_t, std::milli>(10)))
    {
        RCLCPP_ERROR(
            get_logger(),
            "MissionControl::heartbeat_callback: Received too old timestamp: %s",
            msg.sender_id.c_str());
        mission_abort("A too old timestamp was received in a heartbeat message");
    }

    heartbeat.received = true;

    RCLCPP_DEBUG(this->get_logger(),
                 "Received heartbeat from: '%s', tick: %u, active: %d",
                 msg.sender_id.c_str(), msg.tick, msg.active);
}

/**
 * @brief Callback function for the heartbeat timer.
 *
 * This function checks if heartbeats have been received from all nodes in the
 * node map. If a heartbeat is not received from a node, an error message is
 * logged. If the mission is not in 'selfcheck' state, the mission will be
 * aborted.
 */
void MissionControl::heartbeat_timer_callback()
{
    bool err_flag = false;

    for (auto &nm : node_map)
    {
        heartbeat_payload &hp = nm.second.hb_payload;

        if (hp.received == false)
        {
            err_flag = true;
            RCLCPP_FATAL(this->get_logger(),
                         "MissionControl::heartbeat_timer_callback: No heartbeat "
                         "from '%s' received!",
                         nm.first.c_str());
        }

        hp.received = false;
    }

    heartbeat_received_all = !err_flag;

    if (heartbeat_received_all)
    {
        RCLCPP_DEBUG(
            this->get_logger(),
            "MissionControl::heartbeat_timer_callback: All heartbeats received");
    }
    else
    {
        if (get_mission_state() != selfcheck && get_mission_state() != prepare_mission)
        {
            mission_abort("Missing heartbeat while not in 'selfcheck' or 'prepare_mission' state");
        }
    }
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MissionControl>());
    rclcpp::shutdown();
    return 0;
}
