#pragma once

#include <array>
#include <stdexcept>
#include <string>

/**
 * @brief Struct representing a heartbeat payload.
 */
struct heartbeat_payload {
    bool received; /**< Flag indicating if the payload has been received. */
    uint32_t tick; /**< The tick value of the payload. */
    bool active;   /**< The active state of the payload. */

    /**
     * @brief Default constructor for heartbeat_payload.
     * Initializes the received flag to false and the tick value to 0.
     */
    heartbeat_payload() {
        received = false;
        tick = 0;
        active = false;
    }
};

/**
 * @brief A struct representing a mission control payload.
 *
 * This struct contains a heartbeat payload and a flag indicating whether the
 * node is allowed to start the mission.
 *
 * @note Only set the `is_fcc_bridge` flag for the actual fcc bridge
 */
struct ros_node {
    struct heartbeat_payload hb_payload;
    bool can_start_mission = false;
    bool is_fcc_bridge = false;

    /**
     * @brief Default constructor for the ros_node class.
     */
    ros_node() = default;

    /**
     * @brief Initialize a node and provide values for the `can_start_mission`
     * and `is_fcc_bridge` flags.
     *
     * @note Only set the `is_fcc_bridge` flag for the actual fcc bridge
     */
    ros_node(const bool can_start_mission, const bool is_fcc_bridge) {
        this->can_start_mission = can_start_mission;
        this->is_fcc_bridge = is_fcc_bridge;
    }
};

/**
 * @brief Represents a position with latitude, longitude, and height.
 */
struct Position {
    bool values_set =
        false;  //!< Flag that is true when the values have been set and false
                //!< when not initialized with values
    double coordinate_lat;
    double coordinate_lon;
    uint32_t height_cm;

    /**
     * @brief Get the position as an array of doubles.
     *
     * @return std::array<double, 2> The position array containing latitude and
     * longitude.
     * @throws std::runtime_error if the values are not set.
     */
    std::array<double, 2> get_position_array() const {
        // Check that values are set
        if (!values_set)
            throw std::runtime_error(
                "Position::get_position_array: values are not set");

        return {coordinate_lat, coordinate_lon};
    }
};
