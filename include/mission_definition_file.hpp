#pragma once

#include <array>
#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <vector>

#include "common_package/commands.hpp"
#include "external_libs/geofence.hpp"

namespace mission_file_lib {
/**
 * @brief Struct representing the safety settings of a mission.
 */
struct safety {
   private:
    std::vector<std::array<double, 2>>
        geofence_points;  //!< Vector of geofence points. Each point is
                          //!< represented by an array of floats (latitude,
                          //!< longitude).

   public:
    std::vector<std::array<double, 2>>
        geofence_points_convex_hull;  //!< Calculated convex hull of the
                                      //!< geofence points

    uint16_t max_height_cm =
        common_lib::MAX_FLIGHT_HEIGHT_CM;  //!< Maximum flight height in
                                           //!< centimeters

    uint16_t min_cruise_height_cm =
        common_lib::MIN_CRUISE_HEIGHT_CM;  //!< Minimum cruise height in
                                           //!< centimeters

    float max_horizontal_speed_mps =
        common_lib::MAX_HORIZONTAL_SPEED_MPS;  //!< Maximum horizontal speed in
                                               //!< meters per second

    float max_vertical_speed_mps =
        common_lib::MAX_VERTICAL_SPEED_MPS;  //!< Maximum vertical speed in
                                             //!< meters per second

    /**
     * @brief Get the geofence points.
     *
     * This function returns a vector of arrays containing the geofence points.
     * Each geofence point is represented by a 2D coordinate (latitude,
     * longitude).
     *
     * @note These are the raw geofence points as defined in the Mission File
     *
     * @return std::vector<std::array<double, 2>> The geofence points.
     */
    std::vector<std::array<double, 2>> get_geofence_points() const {
        return geofence_points;
    }

    /**
     * @brief Sets the geofence points for the mission.
     *
     * This function sets the geofence points for the mission by accepting a
     * vector of 2D points. The geofence points are used to define a boundary
     * for the mission area.
     *
     * @note The points will be used to calculate a convex hull used for
     * checking the geofence
     *
     * @param points A vector of 2D points representing the geofence points.
     */
    void set_geofence_points(const std::vector<std::array<double, 2>> &points) {
        geofence_points = points;

        geofence_points_convex_hull = geofence::getConvexHull(points);
    }

    /**
     * Checks if a given point is inside the geofence.
     *
     * @note Make sure the geofence points are set before calling this function
     *
     * @param point The point to check, represented as a std::array<double, 2>
     * containing the lat and lon coordinates.
     * @return true if the point is inside the geofence, false otherwise.
     */
    bool check_in_geofence(const std::array<double, 2> &point) {
        return geofence::isIn(geofence_points_convex_hull, point);
    }
};

/**
 * @brief Struct representing a command.
 *
 * A marker can have several commands.
 */
struct command {
    std::string type;
    nlohmann::ordered_json data;
};

class MissionDefinitionReader {
   private:
    safety safety_settings;

    /**
     * @brief A map that associates marker names with marker commands.
     *
     * This map is used to store and retrieve marker commands based on their
     * associated unique ids. The keys of the map are strings, and the values
     * are of type `std::vector<command>`. The commands will be executed in the
     * given order on marker recognition.
     *
     * @note Use 'init' for commands that shall be executed right after takeoff.
     */
    std::map<std::string, std::vector<command>> markers;

   public:
    MissionDefinitionReader() = default;

    /**
     * @brief Reads a mission definition file and performs validation checks.
     *
     * This function reads a mission definition file located at the specified
     * file path. It performs various validation checks on the JSON content of
     * the file to ensure that it conforms to the expected structure and values.
     *
     * @param file_path The path to the mission definition file.
     * @param dry_run Flag indicating whether to perform a dry run without
     * storing the result.
     *
     * @throws std::runtime_error if the file fails to open, parse, or if the
     * JSON content is invalid.
     *
     * @note This function stores the result in an internal data structure for
     * future use, unless it is a dry run.
     */
    void read_file(const std::string &file_path, const bool dry_run);

    safety get_safety_settings() const { return safety_settings; }

    /**
     * @brief Retrieves the marker commands associated with a given marker name.
     *
     * This function returns a vector of commands associated with the specified
     * marker name. If the marker name is not found in the markers map, a
     * runtime_error is thrown.
     *
     * @param marker_name The name of the marker.
     * @return std::vector<command> The vector of commands associated with the
     * marker name.
     * @throws std::runtime_error if the marker name is not found.
     */
    std::vector<command> get_marker_commands(
        const std::string &marker_name) const;
};
}  // namespace mission_file_lib
