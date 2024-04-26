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
        set_standby_config();
        break;
    case takeoff:
        initiate_takeoff();
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
    interfaces::msg::Control control_msg;
    control_msg.target_id = msg.sender_id;
    control_msg.active = false;
    control_msg.payload = "{}";
    control_publisher->publish(control_msg);

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
    // TODO save initial coordinates
    // TODO check for inital command instead of switching to fly_to_waypoint
    // directly
    set_mission_state(fly_to_waypoint);
    RCLCPP_INFO(
        this->get_logger(),
        "MissionControl::initiate_takeoff: fly_to_waypoint state activated");
}

void MissionControl::mode_fly_to_waypoint()
{
    if (get_state_first_loop())
    {
        // Activate WaypointNode
        // TODO continue here after reading in the Mission Definition File to know
        // where to fly
    }
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
