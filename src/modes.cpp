#include "mission_control.hpp"

/**
 * @brief Prepares the mission by reading the mission file and setting the
 * standby configuration.
 */
void MissionControl::mode_prepare_mission() {
    // Deactivate Event Loop as it is not needed for prepare mission
    EventLoopGuard elg(&event_loop_active, false);

    RCLCPP_INFO(this->get_logger(),
                "MissionControl::%s: Prepare mission started", __func__);
    set_standby_config();

    try {
        mission_definition_reader.read_file(mdf_file_path, false);
    } catch (const std::runtime_error &e) {
        mission_abort("MissionControl::" + (std::string) __func__ +
                      ": Failed to read mission "
                      "file: " +
                      (std::string)e.what());
    }

    RCLCPP_INFO(this->get_logger(),
                "MissionControl::%s: Prepare mission finished", __func__);
    set_mission_state(selfcheck);
}

/**
 * @brief Executes the self-check of the mission control.
 *
 * This function performs a self-check by waiting for a good drone health and
 * checking if all heartbeats are received within a given timeframe (30s). If
 * the self-check fails, the mission is aborted.
 *
 * @note Aborts after 30s if not all conditions were met
 */
void MissionControl::mode_self_check() {
    static uint32_t i;

    if (get_state_first_loop()) {
        RCLCPP_INFO(this->get_logger(),
                    "MissionControl::%s: Self check started", __func__);
        set_standby_config();
        i = 0;
    }

    const uint32_t max_wait_time =
        (30 /* [s] */ * 1000) / event_loop_time_delta_ms;

    if (i % (1000 / event_loop_time_delta_ms) == 0) {
        // Creating helpful log message
        std::vector<std::string> missing_conditions = {};
        if (!heartbeat_received_all) missing_conditions.push_back("heartbeats");
        if (!drone_health_ok) missing_conditions.push_back("drone health");

        std::string missing_conditions_str = "";
        {
            size_t i = 0;

            for (const auto &mc : missing_conditions) {
                if (i > 0) missing_conditions_str += ", ";
                missing_conditions_str += mc;
                i++;
            }
        }

        if (missing_conditions.size() <= 0) {
            // Output that all conditions are met
            RCLCPP_INFO(this->get_logger(),
                        "MissionControl::%s: [OK] All conditions met: "
                        "%us / %us",
                        __func__, (i * event_loop_time_delta_ms) / 1000,
                        (max_wait_time * event_loop_time_delta_ms) / 1000);
        } else {
            // Output that conditions are still missing
            RCLCPP_INFO(this->get_logger(),
                        "MissionControl::%s: [WAIT] Waiting for: [%s]: "
                        "%us / %us",
                        __func__, missing_conditions_str.c_str(),
                        (i * event_loop_time_delta_ms) / 1000,
                        (max_wait_time * event_loop_time_delta_ms) / 1000);
        }
    }

    i++;

    if (i >= max_wait_time) {
        RCLCPP_FATAL(this->get_logger(),
                     "MissionControl::%s: Self check failed: Not all "
                     "conditions met in the given timeframe (%us)",
                     __func__,
                     (max_wait_time * event_loop_time_delta_ms) / 1000);
        mission_abort(
            "MissionControl::" + (std::string) __func__ +
            ": Self check failed: Not all conditions met in the given "
            "timeframe");
    }

    // Check that all heartbeats were received and drone health is good
    if (heartbeat_received_all && drone_health_ok) {
        {
            // Send safety settings to FCC bridge
            mission_file_lib::safety safety_settings =
                mission_definition_reader.get_safety_settings();

            interfaces::msg::SafetyLimits msg;
            msg.time_stamp = this->now();
            msg.sender_id = this->get_name();
            msg.max_speed_m_s =
                std::max(safety_settings.max_horizontal_speed_mps,
                         safety_settings.max_vertical_speed_mps);
            msg.max_height_m = safety_settings.max_height_cm / 100.0;
            msg.min_soc = safety_settings.min_soc_percent;

            std::vector<interfaces::msg::Waypoint> geofence_points;

            for (const auto &p : safety_settings.get_geofence_points()) {
                interfaces::msg::Waypoint geofence_point;
                geofence_point.latitude_deg = p.at(0);
                geofence_point.longitude_deg = p.at(1);
                geofence_point.relative_altitude_m =
                    interfaces::msg::Waypoint::INVALID_ALTITUDE;

                geofence_points.push_back(geofence_point);
            }

            msg.geofence_points = geofence_points;

            safety_limits_publisher->publish(msg);

            RCLCPP_INFO(this->get_logger(),
                        "MissionControl::%s: Sent safety limits to FCC bridge",
                        __func__);
        }

        RCLCPP_INFO(this->get_logger(),
                    "MissionControl::%s: Self check finished", __func__);
        set_mission_state(check_drone_configuration);
    }
}

/**
 * @brief Checks the drone configuration and sets the mission state to armed.
 *
 * This function checks if the drone configuration is valid by performing the
 * following checks:
 * 1. Check if position is inside of geofence.
 * 2. Check that drone is on the ground.
 * 3. Check that the drone health is good.
 * 4. Check that the current flight mode is `HOLD`.
 *
 * @note Aborts after 60s if not all conditions were met. After performing the
 * checks, the mission state is set to `armed`.
 */
void MissionControl::mode_check_drone_configuration() {
    static uint32_t i;

    if (get_state_first_loop()) {
        RCLCPP_INFO(this->get_logger(),
                    "MissionControl::%s: Check "
                    "drone configuration started",
                    __func__);
        set_standby_config();
        i = 0;
    }

    const uint32_t max_wait_time =
        (60 /* [s] */ * 1000) / event_loop_time_delta_ms;

    if (i % (1000 / event_loop_time_delta_ms) == 0) {
        // Creating helpful log message
        std::vector<std::string> missing_conditions = {};
        if (!drone_health_ok) missing_conditions.push_back("drone health");
        if (current_landed_state != interfaces::msg::LandedState::ON_GROUND)
            missing_conditions.push_back("landed state 'ON_GROUND'");
        if (current_flight_mode != interfaces::msg::FlightMode::HOLD)
            missing_conditions.push_back("flight mode 'HOLD'");

        std::string missing_conditions_str = "";
        {
            size_t i = 0;

            for (const auto &mc : missing_conditions) {
                if (i > 0) missing_conditions_str += ", ";
                missing_conditions_str += mc;
                i++;
            }
        }

        if (missing_conditions.size() <= 0) {
            // Output that all conditions are met
            RCLCPP_INFO(this->get_logger(),
                        "MissionControl::%s: [OK] All conditions met: "
                        "%us / %us",
                        __func__, (i * event_loop_time_delta_ms) / 1000,
                        (max_wait_time * event_loop_time_delta_ms) / 1000);
        } else {
            // Output that conditions are still missing
            RCLCPP_INFO(this->get_logger(),
                        "MissionControl::%s: [WAIT] Waiting for: [%s]: "
                        "%us / %us",
                        __func__, missing_conditions_str.c_str(),
                        (i * event_loop_time_delta_ms) / 1000,
                        (max_wait_time * event_loop_time_delta_ms) / 1000);
        }
    }

    i++;

    if (i >= max_wait_time) {
        RCLCPP_FATAL(
            this->get_logger(),
            "MissionControl::%s: Check drone configuration failed: Not all "
            "conditions met in the given timeframe (%us)",
            __func__, (max_wait_time * event_loop_time_delta_ms) / 1000);
        mission_abort("MissionControl::" + (std::string) __func__ +
                      ": Check drone configuration failed: Not all conditions "
                      "met in the given "
                      "timeframe");
    }

    // Check that drone health is good, FCC is in 'HOLD' state, and drone is on
    // the ground
    if (drone_health_ok &&
        current_landed_state == interfaces::msg::LandedState::ON_GROUND /* &&
        current_flight_mode == interfaces::msg::FlightMode::HOLD */) {
        // Check that current position is in geofence
        try {
            if (!mission_definition_reader.check_in_geofence(
                    current_position.get_position_array())) {
                RCLCPP_FATAL(this->get_logger(),
                             "MissionControl::%s: "
                             "Drone is outside of geofence. Can't arm.",
                             __func__);

                mission_abort("MissionControl::" + (std::string) __func__ +
                              ": Drone is "
                              "outside of geofence. Can't arm.");
            }
        } catch (const std::runtime_error &e) {
            RCLCPP_FATAL(this->get_logger(),
                         "MissionControl::%s: "
                         "Position could not be verified to be inside of the "
                         "geofence: %s",
                         __func__, e.what());

            mission_abort((std::string) "MissionControl::" + __func__ +
                          ": Position could not be verified to be inside of "
                          "the geofence: " +
                          e.what());
        }

        // All checks finished, go to 'armed' state
        RCLCPP_INFO(this->get_logger(),
                    "MissionControl::%s: Check drone "
                    "configuration finished",
                    __func__);
        set_mission_state(armed);
        RCLCPP_INFO(this->get_logger(),
                    "MissionControl::%s: Mission "
                    "Control armed",
                    __func__);
    }
}

/**
 * @brief Initiates the takeoff procedure.
 *
 * This function is responsible for initiating the takeoff procedure of the
 * drone. It performs several checks to ensure that the drone is in the correct
 * state and is currently inside the geofence before sending the takeoff
 * command.
 */
void MissionControl::initiate_takeoff() {
    static bool takeoff_initiated;  // True, if the takeoff command was sent,
                                    // otherwise false

    if (get_state_first_loop()) {
        RCLCPP_INFO(this->get_logger(), "MissionControl::%s: Takeoff initated",
                    __func__);

        takeoff_initiated = false;  // reset flag

        // Activate Mission Control
        send_control_json(this->get_name(), true, {});
        this->activate();
        set_active_node_id(this->get_name());

        // Check that drone is in 'HOLD' state and landed state is 'ON_GROUND'
        if (current_flight_mode != interfaces::msg::FlightMode::HOLD ||
            current_landed_state != interfaces::msg::LandedState::ON_GROUND) {
            RCLCPP_FATAL(this->get_logger(),
                         "MissionControl::%s: Drone is not in "
                         "'READY' flight mode or not on the ground. Current "
                         "flight mode: %u, Current landed state: %u",
                         __func__, current_flight_mode, current_landed_state);

            mission_abort(
                "MissionControl::" + (std::string) __func__ +
                ": Drone is not in 'HOLD' "
                "flight mode or not on the ground. Current flight mode: " +
                std::to_string(current_flight_mode) +
                ", Current landed state: " +
                std::to_string(current_landed_state));
        }

        // Check that the current position is known
        if (!current_position.values_set) {
            RCLCPP_FATAL(this->get_logger(),
                         "MissionControl::%s: Cannot takeoff "
                         "without a valid position",
                         __func__);

            mission_abort("MissionControl::" + (std::string) __func__ +
                          ": Cannot takeoff without a "
                          "valid position");
        }

        home_position = current_position;

        // Wait for a predefined time before sending waypoint command to avoid
        // confusion
        init_wait(wait_time_between_msgs_ms);
    }

    if (wait_time_finished()) {
        // Check that takeoff wasn't executed before
        if (!takeoff_initiated) {
            // Execute takeoff

            takeoff_initiated =
                true;  // set flag to true, because the takeoff is executing now

            // Send takeoff command
            interfaces::msg::Waypoint waypoint_msg;
            waypoint_msg.latitude_deg = current_position.coordinate_lat;
            waypoint_msg.longitude_deg = current_position.coordinate_lon;
            waypoint_msg.relative_altitude_m =
                2.0;  // Takeoff to a height of 2 m

            interfaces::msg::UAVCommand msg;
            msg.sender_id = this->get_name();
            msg.time_stamp = this->now();
            msg.speed_m_s = mission_definition_reader.get_safety_settings()
                                .max_vertical_speed_mps;
            msg.waypoint = waypoint_msg;
            msg.type = interfaces::msg::UAVCommand::TAKE_OFF;

            uav_command_publisher->publish(msg);

            // Start timeout
            init_wait(takeoff_timeout_ms);

            RCLCPP_INFO(this->get_logger(),
                        "MissionControl::%s: Takeoff command sent. Starting "
                        "timeout timer: %us",
                        __func__, takeoff_timeout_ms / 1000);
        } else {
            // Timeout occured, abort mission
            RCLCPP_FATAL(this->get_logger(),
                         "MissionControl::%s: Takeoff timed out after %u ms",
                         __func__, takeoff_timeout_ms);

            mission_abort("MissionControl::" + (std::string) __func__ +
                          ": Takeoff timed out after " +
                          std::to_string(takeoff_timeout_ms) + " ms");
        }
    }

    if (current_mission_finished()) {
        // Cancel timeout timer
        cancel_wait();

        // Takeoff finished
        RCLCPP_INFO(this->get_logger(), "MissionControl::%s: Takeoff finished",
                    __func__);

        // Deactivate Mission Control
        this->deactivate();
        send_control_json(this->get_name(), false, {});
        clear_active_node_id();

        set_mission_state(decision_maker);
        RCLCPP_INFO(this->get_logger(),
                    "MissionControl::%s: '%s' state activated", __func__,
                    get_mission_state_str(decision_maker));
    }
}

/**
 * @brief Makes decisions on the mode of operation for the mission control.
 *
 * This function is responsible for making decisions on the mode of operation
 * for the mission control. It checks if there are commands left to be executed
 * and moves to the next command if necessary. If there are no more commands, it
 * moves to the next marker and retrieves new commands from the marker. It also
 * checks the type of the current command and sets the mission state
 * accordingly.
 *
 * @note This function assumes that the `commands` vector and
 * `current_command_id` variable have been properly initialized and that the
 * active_marker_name is updated beforehand if a new marker was detected.
 */
void MissionControl::mode_decision_maker() {
    // Deactivate Event Loop as it is not needed for decision maker
    EventLoopGuard elg(&event_loop_active, false);

    RCLCPP_INFO(this->get_logger(),
                "MissionControl::%s: Decision Maker started", __func__);

    // Check if there are commands left that need to be executed
    if (commands.size() > 0 && current_command_id < commands.size() - 1) {
        current_command_id++;  // move command id one further
        RCLCPP_DEBUG(this->get_logger(),
                     "MissionControl::%s: Moved command id to: %ld", __func__,
                     current_command_id);
    } else  // Move to next marker
    {
        // Store new commands from active marker in storage
        current_command_id = 0;

        // Check that marker has not been executed before
        std::string active_marker = get_active_marker_name();
        RCLCPP_INFO(this->get_logger(),
                    "MissionControl::%s: Getting new "
                    "commands for active marker name: '%s'",
                    __func__, active_marker.c_str());

        if (executed_marker_names.find(active_marker) !=
            executed_marker_names.end())
            mission_abort("MissionControl::" + (std::string) __func__ +
                          ": Got the same active "
                          "marker two times: " +
                          active_marker);

        executed_marker_names.insert(active_marker);

        try {
            // Get new marker commands and store them in cache
            commands =
                mission_definition_reader.get_marker_commands(active_marker);
        } catch (const std::runtime_error &e) {
            mission_abort("MissionControl::" + (std::string) __func__ +
                          ": Failed to get new "
                          "commands: " +
                          (std::string)e.what());
        }
    }

    // Check if command list is empty
    if (commands.size() <= 0)
        mission_abort("MissionControl::" + (std::string) __func__ +
                      ": Comand list is empty");

    // Log current status
    {
        const std::string standby_text =
            (executed_marker_names.find(get_active_marker_name()) ==
             executed_marker_names.end())
                ? " [STANDBY]"
                : "";

        RCLCPP_INFO(
            this->get_logger(),
            "MissionControl::%s: active_marker_name: "
            "'%s'%s, current_command_type: '%s', current_command_id: %ld, "
            "command count: %ld",
            __func__, get_active_marker_name().c_str(), standby_text.c_str(),
            commands.at(current_command_id).type.c_str(), current_command_id,
            commands.size());
    }

    // Switch mode based on the current command
    std::string &current_command_type = commands.at(current_command_id).type;
    RCLCPP_DEBUG(this->get_logger(),
                 "MissionControl::%s: Switching mode based on "
                 "command type: '%s'",
                 __func__, current_command_type.c_str());

    if (current_command_type == "waypoint")
        set_mission_state(fly_to_waypoint);
    else if (current_command_type == "detect_marker")
        set_mission_state(detect_marker);
    else if (current_command_type == "end_mission")
        mission_finished();
    else
        mission_abort((std::string) "MissionControl::" + __func__ +
                      ": Unknown command type: '" + current_command_type + "'");
}

/**
 * @brief Executes the "fly_to_waypoint" mode of the mission control.
 *
 * This function activates the waypoint node and sends the command data as
 * payload. It checks if the current command is of the correct type and aborts
 * the mission if it's not. If the job finished successfully, it sets the
 * mission state to `decision_maker` for the next command.
 *
 * @note This function ignores `job_finished_payload` as it is not relevant.
 */
void MissionControl::mode_fly_to_waypoint() {
    // Maximum time before waypoint must be reached, otherwise a mission abort
    // will be triggered
    constexpr uint32_t max_allowed_time_to_waypoint_ms =
        2 /* min */ * 60 * 1000;

    if (get_state_first_loop()) {
        RCLCPP_INFO(this->get_logger(),
                    "MissionControl::%s: Activating waypoint node", __func__);
        RCLCPP_DEBUG(this->get_logger(),
                     "MissionControl::%s: Data sent to "
                     "waypoint node: %s",
                     __func__,
                     commands.at(current_command_id).data.dump().c_str());

        // Check that current command is of the correct type
        if (commands.at(current_command_id).type != "waypoint") {
            mission_abort("MissionControl::" + (std::string) __func__ +
                          ": command has the wrong "
                          "type: Expected: 'waypoint', Got: '" +
                          commands.at(current_command_id).type + "'");
        }

        // Activate Waypoint Node and send the command data as payload
        send_control_json(common_lib::node_names::WAYPOINT, true,
                          commands.at(current_command_id).data);

        // Calculate timeout
        uint32_t timeout_ms = max_allowed_time_to_waypoint_ms;
        if (commands.at(current_command_id).data.contains("pre_wait_time_ms")) {
            timeout_ms += commands.at(current_command_id)
                              .data.at("pre_wait_time_ms")
                              .get<uint32_t>();
        }

        if (commands.at(current_command_id).data.contains("pre_wait_time_ms")) {
            timeout_ms += commands.at(current_command_id)
                              .data.at("post_wait_time_ms")
                              .get<uint32_t>();
        }

        // Start timeout timer
        init_wait(timeout_ms);
        RCLCPP_INFO(this->get_logger(),
                    "MissionControl::%s: Timeout timer started: %u ms "
                    "(= %f s)",
                    __func__, timeout_ms, timeout_ms / 1000.0);
    }

    if (wait_time_finished()) {
        // Timeout reached, aborting mission
        mission_abort("MissionControl::" + (std::string) __func__ +
                      ": Timeout reached without "
                      "arriving at waypoint");
    }

    // If job finished, return to decision maker for next command
    if (get_job_finished_successfully()) {
        // Cancel timeout timer
        cancel_wait();

        RCLCPP_INFO(this->get_logger(),
                    "MissionControl::%s: Job finished "
                    "successfully. Set mission state to 'decision_maker'.",
                    __func__);
        set_mission_state(decision_maker);
    }
}

/**
 * @brief Executes the "detect_marker" mode of the mission control.
 *
 * This function activates the QR code scanner node, sends the command data as
 * payload, starts a timeout timer, and waits for the job to finish
 * successfully. If the timeout is reached without decoding a marker, the
 * mission is aborted. Once the job is finished successfully, the payload is
 * parsed, the new active marker is set, and the mission state is set to
 * `decision_maker`.
 */
void MissionControl::mode_detect_marker() {
    if (get_state_first_loop()) {
        RCLCPP_INFO(this->get_logger(),
                    "MissionControl::%s: Activating qr code "
                    "scanner node",
                    __func__);
        RCLCPP_DEBUG(this->get_logger(),
                     "MissionControl::%s: Data sent to "
                     "qr code scanner node: %s",
                     __func__,
                     commands.at(current_command_id).data.dump().c_str());

        // Check that current command is of the correct type
        if (commands.at(current_command_id).type != "detect_marker")
            mission_abort("MissionControl::" + (std::string) __func__ +
                          ": command has the wrong "
                          "type: Expected: 'detect_marker', Got: '" +
                          commands.at(current_command_id).type + "'");

        // Activate QR Code Scanner Node and send the command data as payload
        send_control_json(common_lib::node_names::QRCODE_SCANNER, true,
                          commands.at(current_command_id).data);

        // Start timeout timer
        init_wait(commands.at(current_command_id).data.at("timeout_ms"));
        RCLCPP_INFO(this->get_logger(),
                    "MissionControl::%s: Timeout timer started: %u ms "
                    "(= %f s)",
                    __func__,
                    commands.at(current_command_id)
                        .data.at("timeout_ms")
                        .get<uint32_t>(),
                    commands.at(current_command_id)
                            .data.at("timeout_ms")
                            .get<uint32_t>() /
                        1000.0);
    }

    if (wait_time_finished()) {
        // Timeout reached, aborting mission
        mission_abort("MissionControl::" + (std::string) __func__ +
                      ": Timeout reached without "
                      "decoding a marker");
    }

    // If job finished, parse payload and return to decision maker for next
    // command
    if (get_job_finished_successfully()) {
        // Cancel timeout timer
        cancel_wait();

        // Decode JobFinished payload
        nlohmann::json payload = get_job_finished_payload();

        std::map<const std::string, const common_lib::JsonKeyDefinition>
            definition = {{"marker", {true, common_lib::string}}};

        try {
            common_lib::CommandDefinitions::parse_check_json(payload,
                                                             definition);
        } catch (const std::runtime_error &e) {
            mission_abort("MissionControl::" + (std::string) __func__ +
                          ": Failed to parse "
                          "JobFinished payload: " +
                          (std::string)e.what());
        }

        // Set new active marker
        set_active_marker_name(payload.at("marker"));

        // Set mission state to 'decision_maker'
        RCLCPP_INFO(this->get_logger(),
                    "MissionControl::%s: Job finished "
                    "successfully. Set mission state to '%s'.",
                    __func__, get_mission_state_str(decision_maker));
        set_mission_state(decision_maker);
    }
}
