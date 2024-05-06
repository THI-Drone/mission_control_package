#include "mission_control.hpp"

#define ENUM_TO_STR(member) \
    case member:            \
        return #member

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
 * Additionally resets `mission_progress` to 0.
 *
 * @param new_mission_state The new mission state to set.
 */
void MissionControl::set_mission_state(const MissionState_t new_mission_state) {
    if (mission_state != new_mission_state) {
        job_finished_successfully = false;
        state_first_loop = true;
    }

    mission_progress = 0.0;
    mission_state = new_mission_state;

    RCLCPP_DEBUG(this->get_logger(),
                 "MissionControl::%s: Set mission state to %d", __func__,
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
void MissionControl::set_active_node_id(std::string node_id) {
    RCLCPP_DEBUG(this->get_logger(),
                 "MissionControl::%s: Set active_node_id to %s", __func__,
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
void MissionControl::set_active_marker_name(
    const std::string& new_active_marker_name) {
    RCLCPP_DEBUG(this->get_logger(),
                 "MissionControl::%s: Set active_marker_name to %s", __func__,
                 new_active_marker_name.c_str());

    active_marker_name = new_active_marker_name;
}

/**
 * @brief Clears the active node ID.
 *
 * This function clears the active node ID by setting it to an empty string.
 */
void MissionControl::clear_active_node_id() {
    RCLCPP_DEBUG(this->get_logger(),
                 "MissionControl::%s: Cleared active node id", __func__);
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
bool MissionControl::get_state_first_loop() {
    // Check if first loop was already triggered
    if (!state_first_loop) return false;

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
bool MissionControl::get_job_finished_successfully() {
    if (job_finished_successfully) {
        job_finished_successfully = false;
        return true;
    } else {
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
nlohmann::json MissionControl::get_job_finished_payload() {
    nlohmann::json res = job_finished_payload;

    // Clear the job_finished_payload
    job_finished_payload = nlohmann::json();

    return res;
}

/**
 * Checks if the current mission is finished.
 *
 * @return true if the mission is finished, false otherwise.
 */
bool MissionControl::current_mission_finished() {
    if (mission_progress >= 1.0) {
        mission_progress = 0.0;
        return true;
    } else {
        return false;
    }
}

/**
 * @brief Initializes the wait time timer.
 *
 * This function initializes the wait time timer with the specified wait time in
 * milliseconds. It creates a wall timer that calls the callback function
 * MissionControl::callback_wait_time after the specified wait time has elapsed.
 *
 * @note Poll `wait_time_finished` to continue after the wait time is over
 *
 * @param wait_time_ms The wait time in milliseconds.
 */
void MissionControl::init_wait(uint16_t wait_time_ms) {
    wait_time_finished_ok = false;

    // Initialize Wait Time Timer
    wait_time_timer = this->create_wall_timer(
        std::chrono::milliseconds(wait_time_ms),
        std::bind(&MissionControl::callback_wait_time, this));

    RCLCPP_DEBUG(this->get_logger(),
                 "MissionControl::%s: Started wait time for %u ms", __func__,
                 wait_time_ms);
}

/**
 * Checks if the wait time has finished.
 *
 * @return true if the wait time has finished, false otherwise.
 */
bool MissionControl::wait_time_finished() {
    if (wait_time_finished_ok) {
        wait_time_finished_ok = false;
        return true;
    } else {
        return false;
    }
}

/**
 * @brief Cancels the current wait time and sets the flag to false.
 *
 * This function cancels the current timer used for waiting and sets the
 * `wait_time_finished_ok` flag to false. It is used to cancel the wait time
 * early when needed.
 */
void MissionControl::cancel_wait() {
    // Cancel current timer
    wait_time_timer->cancel();

    // Make sure that the flag is set to false
    wait_time_finished_ok = false;

    RCLCPP_DEBUG(this->get_logger(), "MissionControl::%s: Canceled wait time",
                 __func__);
}

/**
 * @brief Get the string representation of the current mission state.
 *
 * This function returns the string representation of the current mission state.
 * The mission state is obtained using the `get_mission_state()` function.
 *
 * @return const char* The string representation of the mission state.
 *
 * @throws std::runtime_error if the mission state is unknown.
 */
const char* MissionControl::get_mission_state_str() const {
    return get_mission_state_str(get_mission_state());
}

/**
 * @brief Converts a MissionState_t enum value to its corresponding string representation.
 * 
 * @param mission_state The MissionState_t enum value to convert.
 * @return const char* The string representation of the mission state.
 * @throws std::runtime_error if the mission state is unknown.
 */
const char* MissionControl::get_mission_state_str(
    MissionState_t mission_state) const {
    switch (mission_state) {
        ENUM_TO_STR(prepare_mission);
        ENUM_TO_STR(selfcheck);
        ENUM_TO_STR(check_drone_configuration);
        ENUM_TO_STR(armed);
        ENUM_TO_STR(takeoff);
        ENUM_TO_STR(decision_maker);
        ENUM_TO_STR(fly_to_waypoint);
        ENUM_TO_STR(detect_marker);
        default:
            throw std::runtime_error(
                "MissionControl::" + (std::string) __func__ +
                ": Unknown mission state");
    }
}
