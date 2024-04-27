#include "main.hpp"

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
