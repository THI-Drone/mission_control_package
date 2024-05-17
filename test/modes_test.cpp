#include <gtest/gtest.h>

#include <chrono>
#include <cinttypes>
#include <nlohmann/json.hpp>

#include "common_package/topic_names.hpp"
#include "mission_control.hpp"
#include "rclcpp/executors.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/node_options.hpp"

/**
 * @brief Test case for the mode_prepare_mission function in the
 * mission_control_package.
 *
 * This test verifies that the mode_prepare_mission function executes without
 * throwing any exceptions.
 */
TEST(mission_control_package, mode_prepare_mission_test) {
    rclcpp::NodeOptions options;
    options.append_parameter_override(
        "MDF_FILE_PATH",
        "../../src/mission_control_package/test/mission_file_reader/"
        "test_assets/mdf_correct.json");

    MissionControl mission_control(options);

    mission_control.active_node_id = "abc";

    ASSERT_NO_THROW(mission_control.mode_prepare_mission());

    ASSERT_EQ("", mission_control.get_active_node_id());
    ASSERT_EQ(MissionControl::MissionState_t::selfcheck,
              mission_control.get_mission_state());
}

TEST(mission_control_package, mode_decision_maker_test) {
    rclcpp::NodeOptions default_options;
    default_options.append_parameter_override(
        "MDF_FILE_PATH",
        "../../src/mission_control_package/test/mission_file_reader/"
        "test_assets/mdf_correct.json");

    // Test with active marker 'init'
    {
        MissionControl mission_control(default_options);

        // Read mission file
        mission_control.mode_prepare_mission();

        mission_control.set_mission_state(MissionControl::decision_maker);
        ASSERT_EQ("init", mission_control.get_active_marker_name());
        ASSERT_EQ(0, mission_control.current_command_id);

        // First round
        mission_control.set_mission_state(MissionControl::decision_maker);
        ASSERT_NO_THROW(mission_control.mode_decision_maker());
        ASSERT_EQ(0, mission_control.current_command_id);
        ASSERT_EQ(MissionControl::fly_to_waypoint,
                  mission_control.get_mission_state());

        // Second round
        mission_control.set_mission_state(MissionControl::decision_maker);
        ASSERT_NO_THROW(mission_control.mode_decision_maker());
        ASSERT_EQ(1, mission_control.current_command_id);
        ASSERT_EQ(MissionControl::fly_to_waypoint,
                  mission_control.get_mission_state());

        // Third round
        mission_control.set_mission_state(MissionControl::decision_maker);
        ASSERT_NO_THROW(mission_control.mode_decision_maker());
        ASSERT_EQ(2, mission_control.current_command_id);
        ASSERT_EQ(MissionControl::detect_marker,
                  mission_control.get_mission_state());

        // Forth round -> this should fail as no more commands are left
        mission_control.set_mission_state(MissionControl::decision_maker);
        ASSERT_DEATH({ mission_control.mode_decision_maker(); }, ".*");
    }

    // Test marker change
    {
        MissionControl mission_control(default_options);

        // Read mission file
        mission_control.mode_prepare_mission();

        mission_control.set_mission_state(MissionControl::decision_maker);
        ASSERT_EQ("init", mission_control.get_active_marker_name());
        ASSERT_EQ(0, mission_control.current_command_id);

        // First round of marker 'init'
        mission_control.set_mission_state(MissionControl::decision_maker);
        ASSERT_NO_THROW(mission_control.mode_decision_maker());
        ASSERT_EQ(0, mission_control.current_command_id);
        ASSERT_EQ(MissionControl::fly_to_waypoint,
                  mission_control.get_mission_state());

        // Second round of marker 'init'
        mission_control.set_mission_state(MissionControl::decision_maker);
        ASSERT_NO_THROW(mission_control.mode_decision_maker());
        ASSERT_EQ(1, mission_control.current_command_id);
        ASSERT_EQ(MissionControl::fly_to_waypoint,
                  mission_control.get_mission_state());

        // Third round of marker 'init'
        mission_control.set_mission_state(MissionControl::decision_maker);
        ASSERT_NO_THROW(mission_control.mode_decision_maker());
        ASSERT_EQ(2, mission_control.current_command_id);
        ASSERT_EQ(MissionControl::detect_marker,
                  mission_control.get_mission_state());

        // Change to active marker '1'
        mission_control.set_mission_state(MissionControl::decision_maker);
        mission_control.set_active_marker_name("1");
        ASSERT_EQ("1", mission_control.get_active_marker_name());

        // First round of marker '1'
        mission_control.set_mission_state(MissionControl::decision_maker);
        ASSERT_NO_THROW(mission_control.mode_decision_maker());
        ASSERT_EQ(0, mission_control.current_command_id);
        ASSERT_EQ(MissionControl::fly_to_waypoint,
                  mission_control.get_mission_state());

        // Change to active marker '2'
        mission_control.set_mission_state(MissionControl::decision_maker);
        mission_control.set_active_marker_name("2");
        ASSERT_EQ("2", mission_control.get_active_marker_name());

        // Second round of marker '1', as one command is still left
        mission_control.set_mission_state(MissionControl::decision_maker);
        ASSERT_NO_THROW(mission_control.mode_decision_maker());
        ASSERT_EQ(1, mission_control.current_command_id);
        ASSERT_EQ(MissionControl::detect_marker,
                  mission_control.get_mission_state());

        // First round of marker '2' -> end_mission command which means that
        // this will exit
        mission_control.set_mission_state(MissionControl::decision_maker);
        ASSERT_DEATH({ mission_control.mode_decision_maker(); }, ".*");
    }

    // Test with empty command list
    {
        MissionControl mission_control(default_options);

        // Read mission file
        mission_control.mode_prepare_mission();

        // First round
        mission_control.set_mission_state(MissionControl::decision_maker);
        ASSERT_NO_THROW(mission_control.mode_decision_maker());
        ASSERT_EQ(0, mission_control.current_command_id);
        ASSERT_EQ(MissionControl::fly_to_waypoint,
                  mission_control.get_mission_state());

        // Delete all commands
        mission_control.commands.clear();

        // This should fail because command list is empty
        mission_control.set_mission_state(MissionControl::decision_maker);
        ASSERT_DEATH({ mission_control.mode_decision_maker(); }, ".*");
    }
}
