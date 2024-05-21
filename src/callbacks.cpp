#include "mission_control.hpp"

/**
 * @brief Callback function for handling job_finished messages.
 *
 * This function is called when a job_finished message is received. It performs
 * the following tasks:
 * 1. Parses the JSON payload from the message.
 * 2. Checks the error code of the message. If it is not EXIT_SUCCESS, aborts
 * the mission.
 * 3. Ignores the message if the sender node is not active.
 * 4. Sends a control message to explicitly deactivate the sender node.
 * 5. Updates the internal state by clearing the active node ID.
 * 6. Stores the payload for future use.
 * 7. Sets the job_finished_successfully flag to true.
 *
 * @param msg The job_finished message received.
 */
void MissionControl::job_finished_callback(
    const interfaces::msg::JobFinished &msg) {
    // Try to parse JSON
    nlohmann::json payload;
    try {
        payload = nlohmann::json::parse(msg.payload);
    } catch (const nlohmann::json::exception &e) {
        RCLCPP_FATAL(
            this->get_logger(),
            "MissionControl::%s: "
            "Failed to parse json formatted payload: '%s' - Payload: '%s'",
            __func__, e.what(), msg.payload.c_str());

        mission_abort((std::string) "MissionControl::" + __func__ +
                      ": Failed to parse json formatted payload: '" + e.what() +
                      "' - Payload: '" + msg.payload + "'");
    }

    // Check error code
    if (msg.error_code != EXIT_SUCCESS) {
        // Check if sender node is currently active
        if (msg.sender_id == get_active_node_id()) {
            // Send control message to explicitly deactivate node
            send_control_json(msg.sender_id, false, {});
        }

        // Abort mission
        std::string error_msg = "MissionControl::" + (std::string) __func__ +
                                ": Received "
                                "job_finished message with error_code: " +
                                std::to_string(msg.error_code);

        if (payload.contains("error_msg"))
            error_msg += ". Error Message: " + payload.at("error_msg").dump();
        else
            error_msg += ". Payload: '" + msg.payload + "'";

        mission_abort(error_msg);
        return;
    }

    // Ignore message if sender node is not active as this might be a delayed
    // message or a programming error in the sender node
    if (msg.sender_id != get_active_node_id()) {
        RCLCPP_ERROR(
            this->get_logger(),
            "MissionControl::%s: Got a successfull "
            "job_finished message from an inactive node: %s. Ignoring message.",
            __func__, msg.sender_id.c_str());
        return;
    }

    // Send control message to explicitly deactivate node
    send_control_json(msg.sender_id, false, {});

    // Deactivate node in internal state
    clear_active_node_id();

    // Store payload for future use
    job_finished_payload = payload;

    // Set job_finished_successfully flag to true indicating receiving a
    // successfull job_finished message
    RCLCPP_INFO(this->get_logger(),
                "MissionControl::%s: Job finished "
                "successfully: node_id: '%s', payload: '%s'",
                __func__, msg.sender_id.c_str(), payload.dump().c_str());

    // Set flag to true to indicate a successfully finished job
    job_finished_successfully = true;
}

/**
 * @brief Starts the mission based on the received command.
 *
 * This function is called when a mission start command is received. It verifies
 * the current mission state (must be `armed`), checks if the sender node is
 * known and allowed to start the mission, and then initiates the mission.
 *
 * @param msg The mission start command message
 */
void MissionControl::mission_start_callback(
    const interfaces::msg::MissionStart &msg) {
    RCLCPP_INFO(this->get_logger(),
                "MissionControl::%s: Received mission start command from %s",
                __func__, msg.sender_id.c_str());

    // Check mission state is armed
    if (get_mission_state() != armed) {
        RCLCPP_ERROR(get_logger(),
                     "MissionControl::%s: Mission start command received "
                     "without being in '%s' state",
                     __func__, get_mission_state_str(armed));
        return;
    }

    // Check that sender node id is known
    if (node_map.find(msg.sender_id) == node_map.end()) {
        RCLCPP_ERROR(get_logger(),
                     "MissionControl::%s: Sender node unknown: '%s'", __func__,
                     msg.sender_id.c_str());
        return;
    }

    // Check that sender has permission to start mission
    if (!node_map[msg.sender_id].can_start_mission) {
        RCLCPP_ERROR(get_logger(),
                     "MissionControl::%s: Sender is not allowed to "
                     "start the mission: '%s'",
                     __func__, msg.sender_id.c_str());
        return;
    }

    // Set mission to 'init' marker
    set_active_marker_name("init");

    // Start mission
    set_mission_state(takeoff);
    RCLCPP_INFO(this->get_logger(),
                "MissionControl::%s: Mission started. New state: '%s'",
                __func__, get_mission_state_str());
}

/**
 * @brief Callback function for handling UAV waypoint command messages.
 *
 * This function is called when a UAV waypoint command message is received.
 * It performs several checks on the message, including checking the sender's
 * ID and the timestamp of the message. If the checks pass, the function
 * continues with the necessary operations. If the checks fail, appropriate
 * error messages are logged and the mission aborts.
 *
 * @note Aborts the mission if an unauthorized sender is sending on the
 * UAVWaypointCommand topic. Too old messages are ignored.
 *
 * @param msg The UAV waypoint command message received.
 */
void MissionControl::waypoint_command_callback(
    const interfaces::msg::UAVWaypointCommand &msg) {
    // Store current timestamp for later
    rclcpp::Time timestamp_now = this->now();

    RCLCPP_DEBUG(this->get_logger(),
                 "MissionControl::%s: Checking '%s' message from %s", __func__,
                 common_lib::topic_names::UAVWaypointCommand,
                 msg.sender_id.c_str());

    // Check that the mission is running
    if (get_mission_state() == prepare_mission ||
        get_mission_state() == selfcheck ||
        get_mission_state() == check_drone_configuration ||
        get_mission_state() == armed) {
        // Abort mission
        RCLCPP_FATAL(get_logger(),
                     "MissionControl::%s: The node '%s' sent a message on the "
                     "'%s' topic, but the mission has not started yet",
                     __func__, msg.sender_id.c_str(),
                     common_lib::topic_names::UAVWaypointCommand);

        mission_abort((std::string) "MissionControl::" + __func__ +
                      ": The node '" + msg.sender_id +
                      "' sent a message on the '" +
                      common_lib::topic_names::UAVWaypointCommand +
                      "' topic, but the mission has not started yet");
    }

    // Check that sender is allowed to send on the topic
    if (msg.sender_id != get_active_node_id()) {
        // Check timestamp: If it is too old, ingore message as sender might
        // have been allowed to send at that time (defined by
        // 'max_waypoint_command_msg_time_difference_ms' constant)
        if (timestamp_now - rclcpp::Time(msg.time_stamp) >
            rclcpp::Duration(std::chrono::duration<int64_t, std::milli>(
                max_waypoint_command_msg_time_difference_ms))) {
            // Send Error message
            RCLCPP_ERROR(get_logger(),
                         "MissionControl::%s: Received too "
                         "old timestamp in waypoint command message from '%s' "
                         "that is currently not allowed to send. "
                         "Currently allowed node: '%s'. Ignoring message.",
                         __func__, msg.sender_id.c_str(),
                         get_active_node_id().c_str());

            // Deactivate node to be sure that it is not accidentally still
            // active
            send_control_json(msg.sender_id, false, {});

            // Ignore message
            return;
        }

        // Check if sender is in probation period
        if (get_probation_period() &&
            msg.sender_id == get_last_active_node_id()) {
            // Sending error message and don't abort the mission
            RCLCPP_ERROR(
                get_logger(),
                "MissionControl::%s: The node '%s' sent a message on the "
                "'%s' topic, but it is not allowed to do so. "
                "Currently allowed node: '%s'. This fact is ignored because of "
                "the current probation period.",
                __func__, msg.sender_id.c_str(),
                common_lib::topic_names::UAVWaypointCommand,
                get_active_node_id().c_str());

            return;
        }

        RCLCPP_FATAL(get_logger(),
                     "MissionControl::%s: The node '%s' sent a message on the "
                     "'%s' topic, but it is not allowed to do so. "
                     "Currently allowed node: '%s'",
                     __func__, msg.sender_id.c_str(),
                     common_lib::topic_names::UAVWaypointCommand,
                     get_active_node_id().c_str());

        mission_abort((std::string) "MissionControl::" + __func__ +
                      ": Unauthorized node '" + msg.sender_id +
                      "' sending on '" +
                      common_lib::topic_names::UAVWaypointCommand +
                      "' topic registered. Currently active node: '" +
                      get_active_node_id() + "'");
    }

    RCLCPP_DEBUG(this->get_logger(),
                 "MissionControl::%s: Checking '%s' message "
                 "successfull",
                 __func__, common_lib::topic_names::UAVWaypointCommand);
}

/**
 * @brief Callback function for handling the UAVCommand message.
 *
 * This function is called when a new UAVCommand message is received. It checks
 * the sender ID of the message and verifies if it is allowed to send on the
 * UAVCommand topic. If the sender is unauthorized, it calls the mission_abort
 * function with an appropriate error message.
 *
 * @note Only Mission Control is allowed to send on this topic.
 *
 * @param msg The UAVCommand message received.
 */
void MissionControl::command_callback(const interfaces::msg::UAVCommand &msg) {
    RCLCPP_DEBUG(this->get_logger(),
                 "MissionControl::%s: Checking '%s' message from %s", __func__,
                 common_lib::topic_names::UAVCommand, msg.sender_id.c_str());

    // Only Mission Control is allowed to send on this topic
    if (msg.sender_id != this->get_name()) {
        mission_abort(
            "MissionControl::" + (std::string) __func__ +
            ": Unauthorized node '" + msg.sender_id + "' sending on '" +
            (std::string)common_lib::topic_names::UAVCommand +
            "' topic registered. Only allowed node to send on this topic: '" +
            (std::string)this->get_name() + "'");
    }

    RCLCPP_DEBUG(this->get_logger(),
                 "MissionControl::%s: Checking '%s' message "
                 "successfull",
                 __func__, common_lib::topic_names::UAVCommand);
}

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
 * not, it logs a fatal error and aborts the mission. Exception: FCC Bridge
 * which must always be active when not in `selfcheck` or `prepare_mission`
 * state.
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
void MissionControl::heartbeat_callback(const interfaces::msg::Heartbeat &msg) {
    // Save current timestamp for later
    rclcpp::Time timestamp_now = this->now();

    // Throw away own heartbeat
    if (msg.sender_id == get_name()) return;

    // Check if the sender id of the heartbeat is known, otherwise log error and
    // ignore message
    if (node_map.find(msg.sender_id) == node_map.end()) {
        RCLCPP_ERROR(get_logger(),
                     "MissionControl::%s: Received unregistered "
                     "heartbeat: %s",
                     __func__, msg.sender_id.c_str());
        return;
    }

    // Separate check for FCC bridge as that is a special case
    if (node_map[msg.sender_id].is_fcc_bridge) {
        if ((!msg.active) && get_mission_state() != selfcheck &&
            get_mission_state() != prepare_mission) {
            RCLCPP_FATAL(get_logger(),
                         "MissionControl::%s: Node '%s' is no longer "
                         "active but is required to be active for the mission",
                         __func__, msg.sender_id.c_str());

            mission_abort("MissionControl::" + (std::string) __func__ +
                          ": Node '" + msg.sender_id +
                          "' is no longer active but is required to be active "
                          "for the mission");
        } else {
            // Nothing to do here, as the FCC bridge should always be active if
            // not in the 'selfcheck' or 'prepare_mission' state
        }
    } else {
        // Check that active state matches the internal state
        if (msg.active && (msg.sender_id != get_active_node_id())) {
            if (get_probation_period() &&
                msg.sender_id == get_last_active_node_id()) {
                RCLCPP_ERROR(get_logger(),
                             "MissionControl::%s: Node '%s' is active even "
                             "though it should be deactive. Ignoring this fact "
                             "because of the probation period.",
                             __func__, msg.sender_id.c_str());
            } else {
                RCLCPP_FATAL(get_logger(),
                             "MissionControl::%s: Node '%s' is active even "
                             "though it should be deactive",
                             __func__, msg.sender_id.c_str());
                mission_abort("MissionControl::" + (std::string) __func__ +
                              ": Node '" + msg.sender_id +
                              "' was active even though it should be deactive");
            }
        }
        if ((!msg.active) && (msg.sender_id == get_active_node_id())) {
            if (get_probation_period() &&
                msg.sender_id == get_last_active_node_id()) {
                RCLCPP_ERROR(get_logger(),
                             "MissionControl::%s: Node '%s' is deactive even "
                             "though it should be active. Ignoring this fact "
                             "because of the probation period.",
                             __func__, msg.sender_id.c_str());
            } else {
                RCLCPP_FATAL(get_logger(),
                             "MissionControl::%s: Node '%s' is deactive "
                             "even though it should be active",
                             __func__, msg.sender_id.c_str());
                mission_abort("MissionControl::" + (std::string) __func__ +
                              ": Node '" + msg.sender_id +
                              "' was deactive even though it should be active");
            }
        }
    }

    // Check tick
    HeartbeatPayload &heartbeat = node_map[msg.sender_id].hb_payload;
    if ((msg.tick == heartbeat.tick) ||
        (msg.tick <= heartbeat.tick && msg.tick != 0)) {
        RCLCPP_ERROR(get_logger(),
                     "MissionControl::%s: Invalid tick '%u' "
                     "received from: %s. Ignoring heartbeat message",
                     __func__, msg.tick, msg.sender_id.c_str());
        return;
    }
    heartbeat.tick = msg.tick;

    // Check if timestamp is too old (defined by
    // 'heartbeat_max_timestamp_age_ms' constant)
    if (timestamp_now - rclcpp::Time(msg.time_stamp) >
        rclcpp::Duration(std::chrono::duration<int64_t, std::milli>(
            heartbeat_max_timestamp_age_ms))) {
        RCLCPP_FATAL(get_logger(),
                     "MissionControl::%s: Received too old "
                     "timestamp from: %s",
                     __func__, msg.sender_id.c_str());
        mission_abort(
            "MissionControl::" + (std::string) __func__ +
            ": Received too old timestamp in heartbeat message from '" +
            msg.sender_id + "'");
    }

    heartbeat.received = true;
    heartbeat.active = msg.active;

    RCLCPP_DEBUG(this->get_logger(),
                 "MissionControl::%s: Received heartbeat from: '%s', tick: %u, "
                 "active: %d",
                 __func__, msg.sender_id.c_str(), msg.tick, msg.active);
}

/**
 * @brief Callback function for the heartbeat timer.
 *
 * This function checks if heartbeats have been received from all nodes in the
 * node map. If a heartbeat is not received from a node, an error message is
 * logged. If the mission is not in 'selfcheck' state, the mission will be
 * aborted.
 */
void MissionControl::heartbeat_timer_callback() {
    bool err_flag = false;  // If set to true, a heartbeat was not received or a
                            // node was in an incorrect state

    for (auto &nm : node_map) {
        // Create local reference for easier access
        HeartbeatPayload &hp = nm.second.hb_payload;

        // Check if no heartbeat was received
        if (hp.received == false) {
            err_flag = true;
            RCLCPP_ERROR(this->get_logger(),
                         "MissionControl::%s: No heartbeat "
                         "from '%s' received!",
                         __func__, nm.first.c_str());
        }

        // Check for FCC Bridge that the active flag is set to true
        if (nm.second.is_fcc_bridge && !nm.second.hb_payload.active) {
            err_flag = true;
            // Log an error if the FCC bridge is not active but required for the
            // mission
            RCLCPP_ERROR(this->get_logger(),
                         "MissionControl::%s: Node '%s' "
                         "is not active but this is required for the mission!",
                         __func__, nm.first.c_str());
        }

        // Deactivate the received flag for the next heartbeat intervall
        hp.received = false;
    }

    // Set the global variable to let the selfcheck mode know, that all the
    // heartbeats were received and all the nodes are in their correct states
    heartbeat_received_all = !err_flag;

    // Check if all heartbeats were received
    if (heartbeat_received_all) {
        // Log a debug message if all heartbeats are received
        RCLCPP_DEBUG(this->get_logger(),
                     "MissionControl::%s: All heartbeats "
                     "received",
                     __func__);
    } else {
        if (get_mission_state() != selfcheck &&
            get_mission_state() != prepare_mission) {
            // Abort the mission if a heartbeat is missing or the FCC bridge is
            // not active while not in 'selfcheck' or 'prepare_mission' state.
            // During these two phases, the ros network is still starting up and
            // therefore not all nodes have to be active at that time.
            mission_abort(
                "MissionControl::" + (std::string) __func__ +
                ": Missing heartbeat or FCC bridge not active while not in "
                "'" +
                get_mission_state_str(selfcheck) + "' or '" +
                get_mission_state_str(prepare_mission) + "' state");
        }
    }
}

/**
 * @brief Callback function for the probation period timer.
 *
 * This function is called when the probation period timer expires. It cancels
 * the timer, sets the probation period flag to false, and resets the last
 * active node ID. It also logs a debug message indicating that the probation
 * period is over for the last active node.
 */
void MissionControl::probation_period_timer_callback() {
    probation_period_timer->cancel();

    probation_period = false;
    last_active_node_id = "";

    RCLCPP_DEBUG(
        this->get_logger(),
        "MissionControl::%s: Probation period over for last_active_node: '%s'",
        __func__, last_active_node_id.c_str());
}

/**
 * @brief Callback function for receiving GPS position messages.
 *
 * This function is called when a GPS position message is received. It logs the
 * received position information and performs checks on the timestamp of the
 * message. If the timestamp is too old, it resets the current position and
 * takes appropriate actions based on the node's active state. The function also
 * stores the received position values in the `current_position` object.
 *
 * @param msg The GPS position message received.
 */
void MissionControl::position_callback(
    const interfaces::msg::GPSPosition &msg) {
    // Store current timestamp for later
    rclcpp::Time timestamp_now = this->now();

    RCLCPP_DEBUG(this->get_logger(),
                 "WaypointNode::%s: Received position from "
                 "'%s': lat: %f, lon: %f, height: %f",
                 __func__, msg.sender_id.c_str(), msg.latitude_deg,
                 msg.longitude_deg, msg.relative_altitude_m);

    // Check if timestamp is too old (defined by
    // 'max_position_msg_time_difference_ms' constant)
    if (timestamp_now - rclcpp::Time(msg.time_stamp) >
        rclcpp::Duration(std::chrono::duration<int64_t, std::milli>(
            max_position_msg_time_difference_ms))) {
        // Reset current_position as we can no longer reliabely know where we
        // are
        current_position = Position();

        // Check if Mission Control is currently active and thereby sending
        // commands to the FCC Bridge (e.g. during takeoff). If this is the
        // case, the position needs to be reliable and therefore a mission abort
        // will be triggered.
        if (this->get_active()) {
            RCLCPP_FATAL(get_logger(),
                         "WaypointNode::%s: Received too old "
                         "timestamp in position message: %s. Aborting Job.",
                         __func__, msg.sender_id.c_str());
            mission_abort("MissionControl::" + (std::string) __func__ +
                          ": A too old timestamp was received in a position "
                          "message while "
                          "being active");
        } else {
            // Warn but still store values
            RCLCPP_WARN(get_logger(),
                        "WaypointNode::%s: Received too old "
                        "timestamp in position message: %s. Values will be "
                        "used regardless.",
                        __func__, msg.sender_id.c_str());
        }
    }

    // Store values
    current_position.set_position(msg.latitude_deg, msg.longitude_deg,
                                  msg.relative_altitude_m * 100.0);
}

/**
 * @brief Callback function for handling mission progress messages.
 *
 * This function is called when a mission progress message is received. It logs
 * the received progress information and performs checks on the timestamp of the
 * message. If the timestamp is too old, a warning is issued and the message is
 * ignored. Otherwise, the progress value is stored in the `mission_progress`
 * member variable.
 *
 * @param msg The mission progress message received.
 */
void MissionControl::mission_progress_callback(
    const interfaces::msg::MissionProgress &msg) {
    // Store current timestamp for later
    rclcpp::Time timestamp_now = this->now();

    RCLCPP_DEBUG(this->get_logger(),
                 "MissionControl::%s: Received mission "
                 "progress from '%s': progress: %f / 1.0",
                 __func__, msg.sender_id.c_str(), msg.progress);

    // Check if timestamp is too old (defined by
    // 'max_progress_msg_time_difference_ms' constant)
    if (timestamp_now - rclcpp::Time(msg.time_stamp) >
        rclcpp::Duration(std::chrono::duration<int64_t, std::milli>(
            max_progress_msg_time_difference_ms))) {
        // Warn
        RCLCPP_WARN(get_logger(),
                    "MissionControl::%s: Received too "
                    "old timestamp in progress message: %s. Values will be "
                    "used regardless.",
                    __func__, msg.sender_id.c_str());
    }

    // Store value
    mission_progress = msg.progress;
}

/**
 * @brief Callback function for receiving flight state messages.
 *
 * This function is called when a flight state message is received. It extracts
 * the flight mode and landed state from the message and stores them in the
 * `current_flight_mode` and `current_landed_state` variables respectively.
 * It also checks the timestamp of the message and ignores it if it is too old.
 *
 * @param msg The flight state message received.
 */
void MissionControl::flight_state_callback(
    const interfaces::msg::FlightState &msg) {
    // Store current timestamp for later
    rclcpp::Time timestamp_now = this->now();

    RCLCPP_DEBUG(this->get_logger(),
                 "MissionControl::%s: Received flight "
                 "state from '%s': mode: %u, landed_state: %u",
                 __func__, msg.sender_id.c_str(), msg.mode.mode,
                 msg.state.state);

    // Check if timestamp is too old (defined by
    // 'max_flight_state_msg_time_difference_ms' constant)
    if (timestamp_now - rclcpp::Time(msg.time_stamp) >
        rclcpp::Duration(std::chrono::duration<int64_t, std::milli>(
            max_flight_state_msg_time_difference_ms))) {
        // Warn and ignore message
        RCLCPP_WARN(get_logger(),
                    "MissionControl::%s: Received too "
                    "old timestamp in flight state message: %s, mode: %u, "
                    "landed_state: %u. Ignoring message.",
                    __func__, msg.sender_id.c_str(), msg.mode.mode,
                    msg.state.state);

        return;
    }

    // Store values
    current_flight_mode = msg.mode.mode;
    current_landed_state = msg.state.state;
}

/**
 * @brief Callback function for receiving UAV health messages.
 *
 * This function is called when a UAV health message is received. It checks the
 * timestamp of the message and stores the health status of the drone. If the
 * drone health is no longer good, it logs a warning and aborts the mission.
 *
 * @param msg The UAV health message received.
 */
void MissionControl::health_callback(const interfaces::msg::UAVHealth &msg) {
    // Store current timestamp for later
    rclcpp::Time timestamp_now = this->now();

    RCLCPP_DEBUG(
        this->get_logger(),
        "MissionControl::%s: Received flight "
        "state from '%s': GYRO: %d, ACCEL: %d, MAGN: %d, LOCALPOS: "
        "%d, GLOBALPOS: %d, HOMEPOS: %d, ARMABLE: %d",
        __func__, msg.sender_id.c_str(), msg.is_gyrometer_calibration_ok,
        msg.is_accelerometer_calibration_ok, msg.is_magnetometer_calibration_ok,
        msg.is_local_position_ok, msg.is_global_position_ok,
        msg.is_home_position_ok, msg.is_armable);

    // Check if timestamp is too old (defined by
    // 'max_health_msg_time_difference_ms' constant)
    if (timestamp_now - rclcpp::Time(msg.time_stamp) >
        rclcpp::Duration(std::chrono::duration<int64_t, std::milli>(
            max_health_msg_time_difference_ms))) {
        // Warn and ignore message
        RCLCPP_WARN(
            get_logger(),
            "MissionControl::%s: Received too "
            "old timestamp in flight state message: %s. Ignoring message.",
            __func__, msg.sender_id.c_str());

        return;
    }

    // Store value
    drone_health_ok = msg.is_gyrometer_calibration_ok &&
                      msg.is_accelerometer_calibration_ok &&
                      msg.is_magnetometer_calibration_ok &&
                      msg.is_local_position_ok && msg.is_global_position_ok &&
                      msg.is_home_position_ok && msg.is_armable;
}

/**
 * @brief Callback function for wait time completion.
 *
 * This function is called when the wait time is finished. It cancels the wait
 * time timer and sets the `wait_time_finished_ok` flag to true.
 */
void MissionControl::callback_wait_time() {
    RCLCPP_DEBUG(this->get_logger(), "MissionControl::%s: Finished wait time",
                 __func__);

    wait_time_timer->cancel();

    wait_time_finished_ok = true;
}
