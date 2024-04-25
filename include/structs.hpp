#pragma once

#include <string>

/**
 * @brief Struct representing a heartbeat payload.
 */
struct heartbeat_payload {
  bool received; /**< Flag indicating if the payload has been received. */
  uint32_t tick; /**< The tick value of the payload. */

  /**
   * @brief Default constructor for heartbeat_payload.
   * Initializes the received flag to false and the tick value to 0.
   */
  heartbeat_payload() {
    received = false;
    tick = 0;
  }
};

/**
 * @brief A struct representing a mission control payload.
 *
 * This struct contains a heartbeat payload and a flag indicating whether the
 * node is allowed to start the mission.
 */
struct ros_node {
  struct heartbeat_payload hb_payload;
  bool can_start_mission = false;
};
