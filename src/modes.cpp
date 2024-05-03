#include "mission_control.hpp"

/**
 * @brief Prepares the mission by reading the mission file and setting the
 * standby configuration.
 */
void MissionControl::mode_prepare_mission() {
    if (get_state_first_loop()) {
        RCLCPP_INFO(
            this->get_logger(),
            "MissionControl::mode_prepare_mission: Prepare mission started");
        set_standby_config();

        std::string file_path =
            "src/mission_control_package/assets/mission_test.json";
        try {
            mission_definition_reader.read_file(file_path, false);
        } catch (const std::runtime_error &e) {
            mission_abort(
                "MissionControl::mode_prepare_mission: Failed to read mission "
                "file: " +
                (std::string)e.what());
        }

        RCLCPP_INFO(
            this->get_logger(),
            "MissionControl::mode_prepare_mission: Prepare mission finished");
        set_mission_state(selfcheck);
    }
}

void MissionControl::initiate_takeoff() {
    if (get_state_first_loop()) {
        RCLCPP_INFO(this->get_logger(),
                    "MissionControl::initiate_takeoff: Takeoff initated");

        this->activate();
        set_active_node_id(this->get_name());

        // TODO check if drone is in the air -> abort if false

        // TODO save takeoff coordinates

        // TODO initiate takeoff
    }

    // TODO monitor takeoff

    // If takeoff finished
    RCLCPP_INFO(this->get_logger(),
                "MissionControl::initiate_takeoff: Takeoff finished");

    this->deactivate();
    clear_active_node_id();

    set_mission_state(decision_maker);
    RCLCPP_INFO(
        this->get_logger(),
        "MissionControl::initiate_takeoff: make_decision state activated");
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
                "MissionControl::mode_decision_maker: Decision Maker started");

    RCLCPP_INFO(this->get_logger(),
                "MissionControl::mode_decision_maker: active_marker_name: "
                "'%s', current_command_id: %ld, command count: %ld",
                get_active_marker_name().c_str(), current_command_id,
                commands.size());

    // Check if there are commands left that need to be executed
    if (commands.size() > 0 && current_command_id < commands.size() - 1) {
        current_command_id++;  // move command id one further
        RCLCPP_DEBUG(
            this->get_logger(),
            "MissionControl::mode_decision_maker: Moved command id to: %ld",
            current_command_id);
    } else  // Move to next marker
    {
        // Store new commands from active marker in storage
        current_command_id = 0;

        // Check that marker has not been executed before
        std::string active_marker = get_active_marker_name();
        RCLCPP_DEBUG(this->get_logger(),
                     "MissionControl::mode_decision_maker: Getting new "
                     "commands for active marker name: '%s'",
                     active_marker.c_str());

        if (executed_marker_names.find(active_marker) !=
            executed_marker_names.end())
            mission_abort(
                "MissionControl::mode_decision_maker: Got the same active "
                "marker two times: " +
                active_marker);

        executed_marker_names.insert(active_marker);

        try {
            // Get new marker commands and store them in cache
            commands =
                mission_definition_reader.get_marker_commands(active_marker);
        } catch (const std::runtime_error &e) {
            mission_abort(
                "MissionControl::mode_decision_maker: Failed to get new "
                "commands: " +
                (std::string)e.what());
        }
    }

    // Check if command list is empty
    if (commands.size() <= 0)
        mission_abort(
            "MissionControl::mode_decision_maker: Comand list is empty");

    // Switch mode based on the current command
    std::string &current_command_type = commands.at(current_command_id).type;
    RCLCPP_DEBUG(this->get_logger(),
                 "MissionControl::mode_decision_maker: Switching mode based on "
                 "command type: '%s'",
                 current_command_type.c_str());

    if (current_command_type == "waypoint")
        set_mission_state(fly_to_waypoint);
    else if (current_command_type == "detect_marker")
        set_mission_state(detect_marker);
    else if (current_command_type == "end_mission")
        mission_finished();
    else
        mission_abort(
            "MissionControl::mode_decision_maker: Unknown command type");
}

/**
 * @brief Executes the "fly_to_waypoint" mode of the mission control.
 *
 * This function activates the waypoint node and sends the command data as
 * payload. It checks if the current command is of the correct type and aborts
 * the mission if it's not. If the job finished successfully, it sets the
 * mission state to "decision_maker" for the next command.
 *
 * @note This function ignores `job_finished_payload` as it is not relevant.
 */
void MissionControl::mode_fly_to_waypoint() {
    if (get_state_first_loop()) {
        RCLCPP_INFO(
            this->get_logger(),
            "MissionControl::mode_fly_to_waypoint: Activating waypoint node");
        RCLCPP_DEBUG(this->get_logger(),
                     "MissionControl::mode_fly_to_waypoint: Data sent to "
                     "waypoint node: %s",
                     commands.at(current_command_id).data.dump().c_str());

        // Check that current command is of the correct type
        if (commands.at(current_command_id).type != "waypoint")
            mission_abort(
                "MissionControl::mode_fly_to_waypoint: command has the wrong "
                "type: Expected: 'waypoint', Got: '" +
                commands.at(current_command_id).type + "'");

        // Activate Waypoint Node and send the command data as payload
        send_control_json("waypoint_node", true,
                          commands.at(current_command_id).data);
    }

    // If job finished, return to decision maker for next command
    if (get_job_finished_successfully()) {
        RCLCPP_INFO(this->get_logger(),
                    "MissionControl::mode_fly_to_waypoint: Job finished "
                    "successfully. Set mission state to 'decision_maker'.");
        set_mission_state(decision_maker);
    }
}

/**
 * @brief Executes the self-check of the mission control.
 *
 * This function performs a self-check by waiting for a good GPS signal and
 * checking if all heartbeats are received within a given timeframe (30s). If
 * the self-check fails, the mission is aborted.
 */
void MissionControl::mode_self_check() {
    static uint32_t i;

    if (get_state_first_loop()) {
        RCLCPP_INFO(this->get_logger(),
                    "MissionControl::mode_self_check: Self check started");
        set_standby_config();
        i = 0;

        // TODO implement waiting for good GPS signal
    }

    const uint32_t max_wait_time = (30 * 1000) / event_loop_time_delta_ms;
    if (i % (1000 / event_loop_time_delta_ms) == 0) {
        RCLCPP_INFO(this->get_logger(),
                    "MissionControl::mode_self_check: Waiting for heartbeats: "
                    "%us / %us",
                    (i * event_loop_time_delta_ms) / 1000,
                    (max_wait_time * event_loop_time_delta_ms) / 1000);
    }

    i++;

    if (i >= max_wait_time) {
        RCLCPP_ERROR(
            this->get_logger(),
            "MissionControl::mode_self_check: Self check failed: Not all "
            "heartbeats received in the given timeframe (%us)",
            (max_wait_time * event_loop_time_delta_ms) / 1000);
        mission_abort(
            "Self check failed: Not all heartbeats received in the given "
            "timeframe");
    }

    if (heartbeat_received_all) {
        RCLCPP_INFO(this->get_logger(),
                    "MissionControl::mode_self_check: Self check finished");
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
 *
 * @note After performing the checks, the mission state is set to armed.
 */
void MissionControl::mode_check_drone_configuration() {
    if (get_state_first_loop()) {
        RCLCPP_INFO(this->get_logger(),
                    "MissionControl::mode_check_drone_configuration: Check "
                    "drone configuration started");
        set_standby_config();

        // TODO implement check if position is inside of geofence

        // TODO implement check that drone is on the ground
    }

    RCLCPP_INFO(this->get_logger(),
                "MissionControl::mode_check_drone_configuration: Check drone "
                "configuration finished");
    set_mission_state(armed);
    RCLCPP_INFO(this->get_logger(),
                "MissionControl::mode_check_drone_configuration: Mission "
                "Control armed");
}
