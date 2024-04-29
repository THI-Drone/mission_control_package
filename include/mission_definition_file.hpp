#pragma once

#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <vector>
#include <nlohmann/json.hpp>

#include "common_package/commands.hpp"

namespace mission_file_lib
{
    /**
     * @brief Struct representing the safety settings of a mission.
     */
    struct safety
    {
        /// Vector of geofence points. Each point is represented by a pair of floats (latitude, longitude).
        std::vector<std::pair<double, double>> geofence_points;
        /// Maximum flight height in centimeters
        uint16_t max_height_cm = common_lib::MAX_FLIGHT_HEIGHT_CM;
        /// Minimum cruise height in centimeters
        uint16_t min_cruise_height_cm = common_lib::MIN_CRUISE_HEIGHT_CM;
        /// Maximum horizontal speed in meters per second
        float max_horizontal_speed_mps = common_lib::MAX_HORIZONTAL_SPEED_MPS;
        /// Maximum vertical speed in meters per second
        float max_vertical_speed_mps = common_lib::MAX_VERTICAL_SPEED_MPS;
    };

    /**
     * @brief Struct representing a command.
     *
     * A marker can have several commands.
     */
    struct command
    {
        std::string type;
        nlohmann::ordered_json data;
    };

    class MissionDefinitionReader
    {
    private:
        safety safety_settings;

        /**
         * @brief A map that associates marker names with marker commands.
         *
         * This map is used to store and retrieve marker commands based on their associated unique ids.
         * The keys of the map are strings, and the values are of type `std::vector<command>`.
         * The commands will be executed in the given order on marker recognition.
         *
         * @note Use 'init' for commands that shall be executed right after takeoff.
         */
        std::map<std::string, std::vector<command>> markers;

    public:
        MissionDefinitionReader() = default;

        /**
         * @brief Reads a mission definition file and performs validation checks.
         *
         * This function reads a mission definition file located at the specified file path.
         * It performs various validation checks on the JSON content of the file to ensure
         * that it conforms to the expected structure and values.
         *
         * @param file_path The path to the mission definition file.
         * @param dry_run Flag indicating whether to perform a dry run without storing the result.
         *
         * @throws std::runtime_error if the file fails to open, parse, or if the JSON content is invalid.
         *
         * @note This function stores the result in an internal data structure for future use, unless it is a dry run.
         */
        void read_file(const std::string &file_path, const bool dry_run);

        safety get_safety_settings() const { return safety_settings; }

        /**
         * @brief Retrieves the marker commands associated with a given marker name.
         *
         * This function returns a vector of commands associated with the specified marker name.
         * If the marker name is not found in the markers map, a runtime_error is thrown.
         *
         * @param marker_name The name of the marker.
         * @return std::vector<command> The vector of commands associated with the marker name.
         * @throws std::runtime_error if the marker name is not found.
         */
        std::vector<command> get_marker_commands(const std::string &marker_name) const;
    };
}
