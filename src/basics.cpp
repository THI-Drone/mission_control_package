#include "mission_control.hpp"

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
