#include "mission_definition_file.hpp"

using namespace mission_file_lib;

/**
 * @brief Reads a mission definition file and performs validation checks.
 *
 * This function reads a mission definition file located at the specified file
 * path. It performs various validation checks on the JSON content of the file
 * to ensure that it conforms to the expected structure and values.
 *
 * @param file_path The path to the mission definition file.
 * @param dry_run Flag indicating whether to perform a dry run without storing
 * the result.
 *
 * @throws std::runtime_error if the file fails to open, parse, or if the JSON
 * content is invalid.
 *
 * @note This function stores the result in an internal data structure for
 * future use, unless it is a dry run.
 */
void MissionDefinitionReader::read_file(const std::string &file_path,
                                        const bool dry_run) {
    std::ifstream file;
    file.open(file_path, std::ios::in);

    if (!file.good() || !file.is_open())
        throw std::runtime_error(
            "MissionDefinitionReader::" + (std::string) __func__ +
            ": [READ FILE] Failed to open "
            "file: " +
            file_path);

    // Parse json
    nlohmann::ordered_json json;
    try {
        json = nlohmann::ordered_json::parse(file);
    } catch (const nlohmann::json::parse_error &e) {
        file.close();
        throw std::runtime_error(
            "MissionDefinitionReader::" + (std::string) __func__ +
            ": [READ FILE] Failed to parse "
            "JSON: " +
            std::string(e.what()));
    }

    // Close file as it is no longer needed
    file.close();

    /*
     * Check first level of json
     */
    try {
        const std::map<const std::string, const common_lib::JsonKeyDefinition>
            definition = {{"safety", {true, common_lib::object}},
                          {"markers", {true, common_lib::object}}};

        common_lib::CommandDefinitions::parse_check_json(json, definition);
    } catch (const std::runtime_error &e) {
        printf(
            "MissionDefinitionReader::%s: [FIRST LEVEL] Checking first "
            "level of json "
            "failed:\nMake sure that 'safety' and 'markers' exist and have the "
            "correct value type.\n",
            __func__);

        // Forward exception
        throw e;
    }

    /*
     * Check second level of json
     */
    // Check "safety" value
    nlohmann::ordered_json safety_json = json.at("safety");

    try {
        const std::map<const std::string, const common_lib::JsonKeyDefinition>
            definition = {{"geofence", {true, common_lib::array}},
                          {"max_height_cm",
                           {false, common_lib::number_unsigned,
                            common_lib::MIN_CRUISE_HEIGHT_CM,
                            common_lib::MAX_FLIGHT_HEIGHT_CM}},
                          {"min_cruise_height_cm",
                           {false, common_lib::number_unsigned,
                            common_lib::MIN_CRUISE_HEIGHT_CM,
                            common_lib::MAX_FLIGHT_HEIGHT_CM}},
                          {"max_horizontal_speed_mps",
                           {false, common_lib::number, 0.0,
                            common_lib::MAX_HORIZONTAL_SPEED_MPS}},
                          {"max_vertical_speed_mps",
                           {false, common_lib::number, 0.0,
                            common_lib::MAX_VERTICAL_SPEED_MPS}},
                          {"min_soc_percent",
                           {false, common_lib::number_unsigned,
                            common_lib::MIN_SOC_PERCENT, 99}}};

        common_lib::CommandDefinitions::parse_check_json(safety_json,
                                                         definition);

        // Additional step required as `min_cruise_height_cm` cannot be higher
        // than `max_height_cm`
        uint16_t max_height_cm = common_lib::MAX_FLIGHT_HEIGHT_CM;
        if (safety_json.contains("max_height_cm"))
            max_height_cm = safety_json.at("max_height_cm");

        uint16_t min_cruise_height_cm = common_lib::MIN_CRUISE_HEIGHT_CM;
        if (safety_json.contains("min_cruise_height_cm"))
            min_cruise_height_cm = safety_json.at("min_cruise_height_cm");

        if (min_cruise_height_cm > max_height_cm)
            throw std::runtime_error(
                "MissionDefinitionReader::" + (std::string) __func__ +
                ": [SAFETY] "
                "'min_cruise_height_cm' "
                "can not be higher than 'max_height_cm'");

        // Check geofence
        printf(
            "MissionDefinitionReader::%s: [SAFETY] Checking geofence "
            "points:\n",
            __func__);

        nlohmann::ordered_json geofence_json = safety_json.at("geofence");

        std::vector<std::array<double, 2>> geofence_points;
        for (const auto &[key, geofence_point] : geofence_json.items()) {
            // Loop through all geofence points

            const std::map<const std::string,
                           const common_lib::JsonKeyDefinition>
                geofence_definition = {
                    {"lat", {true, common_lib::number_float}},
                    {"lon", {true, common_lib::number_float}}};

            printf(
                "MissionDefinitionReader::%s: [SAFETY] Checking "
                "geofence point "
                "number: %ld\n",
                __func__, geofence_points.size());
            common_lib::CommandDefinitions::parse_check_json(
                geofence_point, geofence_definition);

            std::array<double, 2> geofence_point_pair = {
                geofence_point.at("lat"), geofence_point.at("lon")};

            // Check that point is unique
            for (const auto &gp : geofence_points) {
                if (gp.at(0) == geofence_point_pair.at(0) &&
                    gp.at(0) == geofence_point_pair.at(1)) {
                    throw std::runtime_error(
                        "MissionDefinitionReader::" + (std::string) __func__ +
                        ": [SAFETY] Geofence "
                        "point is "
                        "not unique: LAT: " +
                        std::to_string(geofence_point_pair.at(0)) +
                        ", LON: " + std::to_string(geofence_point_pair.at(1)));
                }
            }

            geofence_points.push_back(geofence_point_pair);
        }

        // Checking that at least 3 geofence points are provided
        if (geofence_points.size() < 3)
            throw std::runtime_error(
                "MissionDefinitionReader::" + (std::string) __func__ +
                ": [SAFETY] At least 3 "
                "points most be "
                "provided (currently: " +
                std::to_string(geofence_points.size()) + ")");

        // Always store geofence points as they're needed for future checks
        safety_settings.set_geofence_points(geofence_points);

        printf(
            "MissionDefinitionReader::%s: [SAFETY] Geofence points "
            "checked "
            "successfully.\n",
            __func__);
    } catch (const std::runtime_error &e) {
        printf(
            "MissionDefinitionReader::%s: [SAFETY] Checking second "
            "level of json "
            "failed: Values of key 'safety' are incorrect.\n",
            __func__);

        // Forward exception
        throw e;
    }

    // Check 'markers' value
    nlohmann::ordered_json markers_json = json.at("markers");

    try {
        printf(
            "MissionDefinitionReader::%s: [MARKERS] Checking marker "
            "values:\n",
            __func__);

        // Loop through all markers
        std::unordered_set<std::string> marker_names;
        bool end_mission_command_exists = false;

        for (const auto &[marker_name, marker_content] : markers_json.items()) {
            // Loop through every marker
            printf(
                "MissionDefinitionReader::%s: [MARKERS] Checking marker "
                "'%s':\n",
                __func__, marker_name.c_str());

            if (marker_names.find(marker_name) != marker_names.end()) {
                // duplicate marker detected
                throw std::runtime_error(
                    "MissionDefinitionReader::" + (std::string) __func__ +
                    ": [MARKERS] Duplicate "
                    "marker "
                    "detected: " +
                    marker_name);
            }

            // Check that value is of type array
            if (!marker_content.is_array())
                throw std::runtime_error(
                    "MissionDefinitionReader::" + (std::string) __func__ +
                    ": [MARKERS] Marker "
                    "value of key "
                    "'" +
                    marker_name + "' is not of type array");

            // Check that at least one command is specified
            if (marker_content.size() <= 0)
                throw std::runtime_error(
                    "MissionDefinitionReader::" + (std::string) __func__ +
                    ": [MARKERS] Marker "
                    "value of key "
                    "'" +
                    marker_name + "' must have at least one command");

            for (const auto &[key, val] : marker_content.items()) {
                // Loop through every command in the marker

                // Check correct marker value
                {
                    printf(
                        "MissionDefinitionReader::%s: "
                        "[MARKERS::COMMAND] Checking that "
                        "'type' and 'data' keys of marker '%s' exist.\n",
                        __func__, marker_name.c_str());

                    const std::map<const std::string,
                                   const common_lib::JsonKeyDefinition>
                        definition = {{"type", {true, common_lib::string}},
                                      {"data", {true, common_lib::object}}};

                    common_lib::CommandDefinitions::parse_check_json(
                        val, definition);
                }

                // Check marker content
                {
                    const std::string marker_type = val.at("type");
                    const nlohmann::ordered_json json_marker_data =
                        val.at("data");

                    if (marker_type == "end_mission")
                        end_mission_command_exists = true;

                    printf(
                        "MissionDefinitionReader::%s: "
                        "[MARKERS::COMMAND] Checking that "
                        "'data' is formatted correctly for the command of type "
                        "'%s'.\n",
                        __func__, marker_type.c_str());

                    const std::map<const std::string,
                                   const common_lib::JsonKeyDefinition>
                        definition =
                            common_lib::CommandDefinitions::get_definition(
                                marker_type);

                    common_lib::CommandDefinitions::parse_check_json(
                        json_marker_data, definition);

                    if (marker_type == "waypoint") {
                        // Check that waypoint is inside of geofence

                        std::array<double, 2> point = {
                            json_marker_data["target_coordinate_lat"],
                            json_marker_data["target_coordinate_lon"]};

                        printf(
                            "MissionDefinitionReader::%s: "
                            "[MARKERS::COMMAND] Checking that "
                            "the waypoint is inside of the geofence: lat: %f, "
                            "lon :%f\n",
                            __func__, point.at(0), point.at(1));

                        if (!safety_settings.check_in_geofence(point)) {
                            throw std::runtime_error(
                                "MissionDefinitionReader::" +
                                (std::string) __func__ +
                                ": "
                                "[MARKERS::COMMAND] Waypoint "
                                "outside of geofence: lat: " +
                                std::to_string(point.at(0)) +
                                ", lon: " + std::to_string(point.at(1)));
                        }

                        printf(
                            "MissionDefinitionReader::%s: "
                            "[MARKERS::COMMAND] Waypoint "
                            "check successfull\n",
                            __func__);
                    }
                }
            }

            // Add marker name to set
            marker_names.insert(marker_name);
        }

        if (marker_names.find("init") == marker_names.end())
            throw std::runtime_error(
                "MissionDefinitionReader::" + (std::string) __func__ +
                ": [MARKERS] required 'init' "
                "marker is "
                "not specified");

        if (!end_mission_command_exists) {
            throw std::runtime_error(
                "MissionDefinitionReader::" + (std::string) __func__ +
                ": [MARKERS] At least one "
                "command of "
                "type 'end_mission' must exist");
        }

        printf(
            "MissionDefinitionReader::%s: [MARKERS] Successfully read "
            "%ld "
            "markers: ",
            __func__, marker_names.size());
        bool first_loop = true;
        for (const auto &marker_name : marker_names) {
            if (!first_loop)
                printf(", ");
            else
                first_loop = false;

            printf("'%s'", marker_name.c_str());
        }
        printf("\n");
    } catch (const std::runtime_error &e) {
        printf(
            "MissionDefinitionReader::%s: [MARKERS] Checking second "
            "level of json "
            "failed: Values of key 'markers' are incorrect.\n",
            __func__);

        // Forward exception
        throw e;
    }

    // Check successfull

    if (dry_run)  // Dry run ends here
        return;

    /*
     * Store safety settings
     */
    // Check that safety settings were defined and if yes store them, otherwise
    // use default values Note: Geofence points were already stored earlier
    if (safety_json.contains("max_height_cm"))
        safety_settings.max_height_cm = safety_json.at("max_height_cm");
    if (safety_json.contains("min_cruise_height_cm"))
        safety_settings.min_cruise_height_cm =
            safety_json.at("min_cruise_height_cm");
    if (safety_json.contains("max_horizontal_speed_mps"))
        safety_settings.max_horizontal_speed_mps =
            safety_json.at("max_horizontal_speed_mps");
    if (safety_json.contains("max_vertical_speed_mps"))
        safety_settings.max_vertical_speed_mps =
            safety_json.at("max_vertical_speed_mps");
    if (safety_json.contains("min_soc_percent"))
        safety_settings.min_soc_percent = safety_json.at("min_soc_percent");

    // Store marker details
    for (const auto &[marker_name, marker_content] : markers_json.items()) {
        // Loop through every marker

        std::vector<command> marker_commands;

        for (const auto &[key, val] : marker_content.items()) {
            // Loop through every command in the marker
            command com;
            com.type = val.at("type");
            com.data = val.at("data");
            marker_commands.push_back(com);
        }

        markers[marker_name] = marker_commands;
    }
}

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
std::vector<command> MissionDefinitionReader::get_marker_commands(
    const std::string &marker_name) const {
    if (markers.find(marker_name) == markers.end())
        throw std::runtime_error(
            "MissionDefinitionReader::" + (std::string) __func__ +
            ": Marker name not "
            "found");

    return markers.at(marker_name);
}
