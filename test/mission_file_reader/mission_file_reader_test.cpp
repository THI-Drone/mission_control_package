#include <gtest/gtest.h>

#include <array>
#include <chrono>
#include <cinttypes>
#include <stdexcept>
#include <vector>

#include "common_package/commands.hpp"
#include "mission_definition_file.hpp"
#include "rclcpp/executors.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/node.hpp"

using namespace mission_file_lib;

TEST(mission_control_package, file_read_test) {
    {
        // Test correct file path
        MissionDefinitionReader mdr;
        ASSERT_NO_THROW(
            mdr.read_file("../../src/mission_control_package/test/"
                          "mission_file_reader/test_assets/mdf_correct.json",
                          true));
    }

    {
        // Test incorrect file path
        MissionDefinitionReader mdr;
        ASSERT_THROW(mdr.read_file("../../src/mission_control_package/test/"
                                   "mission_file_reader/test_assets/abc.json",
                                   true),
                     std::runtime_error);
    }
}

TEST(mission_control_package, file_format_test) {
    {
        // Test correct file formatting
        MissionDefinitionReader mdr;
        ASSERT_NO_THROW(
            mdr.read_file("../../src/mission_control_package/test/"
                          "mission_file_reader/test_assets/mdf_correct.json",
                          true));
    }

    {
        // Test empty file
        MissionDefinitionReader mdr;
        ASSERT_THROW(
            mdr.read_file("../../src/mission_control_package/test/"
                          "mission_file_reader/test_assets/mdf_empty.json",
                          true),
            std::runtime_error);
    }

    {
        // Test incorrect json file
        MissionDefinitionReader mdr;
        ASSERT_THROW(
            mdr.read_file("../../src/mission_control_package/test/"
                          "mission_file_reader/test_assets/mdf_incorrect.json",
                          true),
            std::runtime_error);
    }

    {
        // Test incorrect json file missing the "markers" key
        MissionDefinitionReader mdr;
        ASSERT_THROW(
            mdr.read_file("../../src/mission_control_package/test/"
                          "mission_file_reader/test_assets/mdf_no_markers.json",
                          true),
            std::runtime_error);
    }

    {
        // Test incorrect json file missing the "geofence" key
        MissionDefinitionReader mdr;
        ASSERT_THROW(mdr.read_file(
                         "../../src/mission_control_package/test/"
                         "mission_file_reader/test_assets/mdf_no_geofence.json",
                         true),
                     std::runtime_error);
    }

    {
        // Test incorrect json file with "min_cruise_height_cm" higher than
        // "max_height_cm"
        MissionDefinitionReader mdr;
        ASSERT_THROW(mdr.read_file("../../src/mission_control_package/test/"
                                   "mission_file_reader/test_assets/"
                                   "mdf_invalid_safety_values_1.json",
                                   true),
                     std::runtime_error);
    }

    {
        // Test incorrect json file with "min_cruise_height_cm" higher than
        // hardcoded max height
        MissionDefinitionReader mdr;
        ASSERT_THROW(mdr.read_file("../../src/mission_control_package/test/"
                                   "mission_file_reader/test_assets/"
                                   "mdf_invalid_safety_values_2.json",
                                   true),
                     std::runtime_error);
    }

    {
        // Test incorrect json file with no geofence points specified
        MissionDefinitionReader mdr;
        ASSERT_THROW(
            mdr.read_file(
                "../../src/mission_control_package/test/"
                "mission_file_reader/test_assets/mdf_no_geofence_points.json",
                true),
            std::runtime_error);
    }

    {
        // Test incorrect json file with too less (2) geofence points specified
        MissionDefinitionReader mdr;
        ASSERT_THROW(mdr.read_file("../../src/mission_control_package/test/"
                                   "mission_file_reader/test_assets/"
                                   "mdf_too_less_geofence_points.json",
                                   true),
                     std::runtime_error);
    }

    {
        // Test incorrect json file with a duplicate geofence point
        MissionDefinitionReader mdr;
        ASSERT_THROW(mdr.read_file("../../src/mission_control_package/test/"
                                   "mission_file_reader/test_assets/"
                                   "mdf_duplicate_geofence_point.json",
                                   true),
                     std::runtime_error);
    }

    {
        // Test incorrect json file with no marker specified
        MissionDefinitionReader mdr;
        ASSERT_THROW(mdr.read_file("../../src/mission_control_package/test/"
                                   "mission_file_reader/test_assets/"
                                   "mdf_empty_markers.json",
                                   true),
                     std::runtime_error);
    }

    {
        // Test incorrect json file with a marker without an array
        MissionDefinitionReader mdr;
        ASSERT_THROW(mdr.read_file("../../src/mission_control_package/test/"
                                   "mission_file_reader/test_assets/"
                                   "mdf_invalid_marker_1.json",
                                   true),
                     std::runtime_error);
    }

    {
        // Test incorrect json file with a marker with an empty array
        MissionDefinitionReader mdr;
        ASSERT_THROW(mdr.read_file("../../src/mission_control_package/test/"
                                   "mission_file_reader/test_assets/"
                                   "mdf_invalid_marker_2.json",
                                   true),
                     std::runtime_error);
    }

    {
        // Test incorrect json file without an end_mission command
        MissionDefinitionReader mdr;
        ASSERT_THROW(mdr.read_file("../../src/mission_control_package/test/"
                                   "mission_file_reader/test_assets/"
                                   "mdf_no_end_mission.json",
                                   true),
                     std::runtime_error);
    }

    {
        // Test incorrect json file with a waypoint outside of the geofence
        MissionDefinitionReader mdr;
        ASSERT_THROW(mdr.read_file("../../src/mission_control_package/test/"
                                   "mission_file_reader/test_assets/"
                                   "mdf_invalid_waypoint.json",
                                   true),
                     std::runtime_error);
    }

    {
        // Test incorrect json file without an "init" marker
        MissionDefinitionReader mdr;
        ASSERT_THROW(mdr.read_file("../../src/mission_control_package/test/"
                                   "mission_file_reader/test_assets/"
                                   "mdf_no_init_marker.json",
                                   true),
                     std::runtime_error);
    }

    {
        // Test correct json file with only an "init" marker
        MissionDefinitionReader mdr;
        ASSERT_NO_THROW(
            mdr.read_file("../../src/mission_control_package/test/"
                          "mission_file_reader/test_assets/"
                          "mdf_correct_init_only.json",
                          true));
    }

    {
        // Test incorrect json file with two "detect_marker" commands in a
        // marker
        MissionDefinitionReader mdr;
        ASSERT_THROW(mdr.read_file("../../src/mission_control_package/test/"
                                   "mission_file_reader/test_assets/"
                                   "mdf_too_many_detect_marker.json",
                                   true),
                     std::runtime_error);
    }
}

TEST(mission_control_package, file_save_test) {
    // Dry-run mode
    {
        // Check that the geofence points were read and stored correctly
        MissionDefinitionReader mdr;
        ASSERT_NO_THROW(
            mdr.read_file("../../src/mission_control_package/test/"
                          "mission_file_reader/test_assets/"
                          "mdf_correct.json",
                          true));

        safety safety_settings = mdr.get_safety_settings();

        const std::vector<std::array<double, 2>> geofence_points_raw = {
            {48.768466, 11.336380},
            {48.768175, 11.337411},
            {48.768187, 11.336225},
            {48.767764, 11.337214}};
        ASSERT_EQ(safety_settings.get_geofence_points(), geofence_points_raw);

        // Check the other safety settings
        ASSERT_EQ(safety_settings.max_height_cm,
                  common_lib::MAX_FLIGHT_HEIGHT_CM);
        ASSERT_EQ(safety_settings.min_cruise_height_cm,
                  common_lib::MIN_CRUISE_HEIGHT_CM);
        ASSERT_EQ(safety_settings.max_horizontal_speed_mps,
                  common_lib::MAX_HORIZONTAL_SPEED_MPS);
        ASSERT_EQ(safety_settings.max_vertical_speed_mps,
                  common_lib::MAX_VERTICAL_SPEED_MPS);
        ASSERT_EQ(safety_settings.min_soc_percent, common_lib::MIN_SOC_PERCENT);

        // Checking markers
        ASSERT_THROW(mdr.get_marker_commands("init"), std::runtime_error);
        ASSERT_THROW(mdr.get_marker_commands("1"), std::runtime_error);
        ASSERT_THROW(mdr.get_marker_commands("2"), std::runtime_error);
    }

    // Normal mode
    {
        // Check that the geofence points were read and stored correctly
        MissionDefinitionReader mdr;
        ASSERT_NO_THROW(
            mdr.read_file("../../src/mission_control_package/test/"
                          "mission_file_reader/test_assets/"
                          "mdf_correct.json",
                          false));

        safety safety_settings = mdr.get_safety_settings();

        const std::vector<std::array<double, 2>> geofence_points_raw = {
            {48.768466, 11.336380},
            {48.768175, 11.337411},
            {48.768187, 11.336225},
            {48.767764, 11.337214}};
        ASSERT_EQ(safety_settings.get_geofence_points(), geofence_points_raw);

        // Check the other safety settings
        ASSERT_EQ(safety_settings.max_height_cm, 5000);
        ASSERT_EQ(safety_settings.min_cruise_height_cm, 500);
        ASSERT_EQ(safety_settings.max_horizontal_speed_mps, 10.0);
        ASSERT_EQ(safety_settings.max_vertical_speed_mps, 2.5);
        ASSERT_EQ(safety_settings.min_soc_percent, 30);

        // Checking markers
        {
            std::vector<command> cmds = mdr.get_marker_commands("init");

            ASSERT_EQ(cmds.size(), 3);

            {
                const size_t index = 0;
                ASSERT_EQ(cmds.at(index).type, "waypoint");

                const nlohmann::ordered_json json_data = {
                    {"target_coordinate_lat", 48.768158},
                    {"target_coordinate_lon", 11.336879},
                    {"pre_wait_time_ms", 0},
                    {"post_wait_time_ms", 100},
                    {"cruise_height_cm", 2000},
                    {"target_height_cm", 500},
                    {"horizontal_speed_mps", 3.0},
                    {"vertical_speed_mps", 1.0}};

                ASSERT_EQ(cmds.at(index).data, json_data);
            }

            {
                const size_t index = 1;
                ASSERT_EQ(cmds.at(index).type, "waypoint");

                const nlohmann::ordered_json json_data = {
                    {"target_coordinate_lat", 48.768175},
                    {"target_coordinate_lon", 11.336760},
                    {"pre_wait_time_ms", 100},
                    {"post_wait_time_ms", 200},
                    {"cruise_height_cm", 2000},
                    {"target_height_cm", 500},
                    {"horizontal_speed_mps", 3.0},
                    {"vertical_speed_mps", 1.0}};

                ASSERT_EQ(cmds.at(index).data, json_data);
            }

            {
                const size_t index = 2;
                ASSERT_EQ(cmds.at(index).type, "detect_marker");

                const nlohmann::ordered_json json_data = {
                    {"timeout_ms", 30000}};

                ASSERT_EQ(cmds.at(index).data, json_data);
            }
        }

        {
            std::vector<command> cmds = mdr.get_marker_commands("1");

            ASSERT_EQ(cmds.size(), 2);

            {
                const size_t index = 0;
                ASSERT_EQ(cmds.at(index).type, "waypoint");

                const nlohmann::ordered_json json_data = {
                    {"target_coordinate_lat", 48.768004},
                    {"target_coordinate_lon", 11.337075},
                    {"pre_wait_time_ms", 0},
                    {"post_wait_time_ms", 0},
                    {"cruise_height_cm", 2000},
                    {"target_height_cm", 500},
                    {"horizontal_speed_mps", 3.0},
                    {"vertical_speed_mps", 1.0}};

                ASSERT_EQ(cmds.at(index).data, json_data);
            }

            {
                const size_t index = 1;
                ASSERT_EQ(cmds.at(index).type, "detect_marker");

                const nlohmann::ordered_json json_data = {
                    {"timeout_ms", 30000}};

                ASSERT_EQ(cmds.at(index).data, json_data);
            }
        }

        {
            std::vector<command> cmds = mdr.get_marker_commands("2");

            ASSERT_EQ(cmds.size(), 1);

            const size_t index = 0;
            ASSERT_EQ(cmds.at(index).type, "end_mission");
            ASSERT_EQ(cmds.at(index).data, nlohmann::ordered_json::parse("{}"));
        }
    }
}
