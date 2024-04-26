#include "mission_definition_file.hpp"

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
void MissionDefinitionReader::read_file(const std::string &file_path, const bool dry_run)
{
    std::ifstream file;
    file.open(file_path, std::ios::in);

    if (!file.good() || !file.is_open())
        throw std::runtime_error(
            "MissionDefinitionReader::read_file: Failed to open file: " +
            file_path);

    // Parse json
    nlohmann::ordered_json json;
    try
    {
        json = nlohmann::ordered_json::parse(file);
    }
    catch (const nlohmann::json::parse_error &e)
    {
        file.close();
        throw std::runtime_error(
            "MissionDefinitionReader::read_file: Failed to parse JSON: " +
            std::string(e.what()));
    }

    // Close file as it is no longer needed
    file.close();

    /*
     * Check first level of json
     */
    try
    {
        const std::map<const std::string, const common_lib::JsonKeyDefinition>
            definition = {{"safety", {true, common_lib::object}},
                          {"markers", {true, common_lib::object}}};

        common_lib::CommandDefinitions::parse_check_json(json, definition);
    }
    catch (const std::runtime_error &e)
    {
        printf("MissionDefinitionReader::read_file: Checking first level of json failed:\nMake sure that 'safety' and 'markers' exist and have the correct value type.\n");

        // Forward exception
        throw e;
    }

    /*
     *Check second level of json
     */
    // Check "safety" value
    nlohmann::ordered_json safety_json = json.at("safety");

    try
    {
        const std::map<const std::string, const common_lib::JsonKeyDefinition>
            definition = {{"geofence", {true, common_lib::array}},
                          {"max_height_cm", {false, common_lib::number_unsigned, common_lib::MIN_CRUISE_HEIGHT_CM, common_lib::MAX_FLIGHT_HEIGHT_CM}},
                          {"min_cruise_height_cm", {false, common_lib::number_unsigned, common_lib::MIN_CRUISE_HEIGHT_CM, common_lib::MAX_FLIGHT_HEIGHT_CM}},
                          {"max_horizontal_speed_mps", {false, common_lib::number, 0.0, common_lib::MAX_HORIZONTAL_SPEED_MPS}},
                          {"max_vertical_speed_mps", {false, common_lib::number, 0.0, common_lib::MAX_VERTICAL_SPEED_MPS}}};

        common_lib::CommandDefinitions::parse_check_json(safety_json, definition);

        // Additional step required as `min_cruise_height_cm` cannot be higher than `max_height_cm`
        uint16_t max_height_cm = common_lib::MAX_FLIGHT_HEIGHT_CM;
        if (safety_json.contains("max_height_cm"))
            max_height_cm = safety_json.at("max_height_cm");

        uint16_t min_cruise_height_cm = common_lib::MIN_CRUISE_HEIGHT_CM;
        if (safety_json.contains("min_cruise_height_cm"))
            min_cruise_height_cm = safety_json.at("min_cruise_height_cm");

        if (min_cruise_height_cm > max_height_cm)
            throw std::runtime_error("MissionDefinitionReader::read_file: 'min_cruise_height_cm' can not be higher than 'max_height_cm'");

        // Check geofence
        printf("MissionDefinitionReader::read_file: Checking geofence points.\n");

        nlohmann::ordered_json geofence_json = safety_json.at("geofence");

        std::vector<std::pair<float, float>> geofence_points;
        for (const auto &[key, geofence_point] : geofence_json.items())
        {
            // Loop through all geofence points

            const std::map<const std::string, const common_lib::JsonKeyDefinition>
                geofence_definition = {
                    {"lat", {true, common_lib::number_float}},
                    {"lon", {true, common_lib::number_float}}};

            printf("MissionDefinitionReader::read_file: Checking geofence point number: %ld\n", geofence_points.size());
            common_lib::CommandDefinitions::parse_check_json(geofence_point, geofence_definition);

            std::pair<float, float> geofence_point_pair = {geofence_point.at("lat"), geofence_point.at("lon")};

            // Check that point is unique
            for (const auto &gp : geofence_points)
            {
                if (gp.first == geofence_point_pair.first && gp.second == geofence_point_pair.second)
                {
                    throw std::runtime_error("MissionDefinitionReader::read_file: Geofence point is not unique: LAT: " + std::to_string(geofence_point_pair.first) + ", LON: " + std::to_string(geofence_point_pair.second));
                }
            }

            geofence_points.push_back(geofence_point_pair);
        }

        // Checking that at least 2 geofence points are provided
        if (geofence_points.size() < 2)
            throw std::runtime_error("MissionDefinitionReader::read_file: At least 2 points most be provided (currently: " + std::to_string(geofence_points.size()) + ")");
        
        printf("MissionDefinitionReader::read_file: Geofence points checked successfully.\n");
    }
    catch (const std::runtime_error &e)
    {
        printf("MissionDefinitionReader::read_file: Checking second level of json failed: Values of key 'safety' are incorrect.\n");

        // Forward exception
        throw e;
    }

    // Check 'markers' value
    nlohmann::ordered_json markers_json = json.at("markers");

    try
    {
        printf("MissionDefinitionReader::read_file: Checking marker values.\n");

        // Loop through all markers
        std::unordered_set<std::string> marker_names;

        for (const auto &[marker_name, marker_content] : markers_json.items())
        {
            // Loop through every marker
            printf("MissionDefinitionReader::read_file: Checking marker '%s':\n", marker_name.c_str());

            if (marker_names.find(marker_name) != marker_names.end())
            {
                // duplicate marker detected
                throw std::runtime_error("MissionDefinitionReader::read_file: Duplicate marker detected: " + marker_name);
            }

            // Check that value is of type array
            if (!marker_content.is_array())
                throw std::runtime_error("MissionDefinitionReader::read_file: Marker value of key '" + marker_name + "' is not of type array");

            for (const auto &[key, val] : marker_content.items())
            {
                // Loop through every command in the marker

                // Check correct marker value
                {
                    printf("MissionDefinitionReader::read_file: Checking that 'type' and 'data' keys of marker '%s' exist.\n", marker_name.c_str());

                    const std::map<const std::string, const common_lib::JsonKeyDefinition>
                        definition = {{"type", {true, common_lib::string}},
                                      {"data", {true, common_lib::object}}};

                    common_lib::CommandDefinitions::parse_check_json(val, definition);
                }

                // Check marker content
                {
                    const std::string marker_type = val.at("type");
                    const nlohmann::ordered_json json_marker_data = val.at("data");

                    printf("MissionDefinitionReader::read_file: Checking that 'data' is formatted correctly for the marker '%s' of type '%s'.\n", marker_name.c_str(), marker_type.c_str());

                    const std::map<const std::string, const common_lib::JsonKeyDefinition>
                        definition = common_lib::CommandDefinitions::get_definition(marker_type);

                    common_lib::CommandDefinitions::parse_check_json(json_marker_data, definition);
                }
            }

            // Add marker name to set
            marker_names.insert(marker_name);
        }

        if (marker_names.find("init") == marker_names.end())
            throw std::runtime_error("MissionDefinitionReader::read_file: required 'init' marker is not specified");

        printf("MissionDefinitionReader::read_file: Successfully read %ld markers: ", marker_names.size());
        bool first_loop = true;
        for (const auto &marker_name : marker_names)
        {
            if (!first_loop)
                printf(", ");
            else
                first_loop = false;

            printf("%s", marker_name.c_str());
        }
        printf("\n");
    }
    catch (const std::runtime_error &e)
    {
        printf("MissionDefinitionReader::read_file: Checking second level of json failed: Values of key 'markers' are incorrect.\n");

        // Forward exception
        throw e;
    }

    // Check successfull

    if (dry_run) // Dry run ends here
        return;

    // TODO continue here: store result in internal data structure for future use
}
