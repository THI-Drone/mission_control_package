#include <gtest/gtest.h>

#include <chrono>
#include <cinttypes>
#include <nlohmann/json.hpp>

#include "common_package/common_node.hpp"
#include "common_package/node_names.hpp"
#include "common_package/topic_names.hpp"
#include "mission_control.hpp"
#include "rclcpp/executors.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/node_options.hpp"

// Message includes
#include "interfaces/msg/flight_mode.hpp"
#include "interfaces/msg/landed_state.hpp"
#include "interfaces/msg/uav_command.hpp"
#include "interfaces/msg/waypoint.hpp"

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

TEST(mission_control_package, initiate_takeoff_test) {
    rclcpp::NodeOptions default_options;
    default_options.append_parameter_override(
        "MDF_FILE_PATH",
        "../../src/mission_control_package/test/"
        "mission_file_reader/test_assets/mdf_correct.json");

    // Fully working takeoff procedure
    {
        rclcpp::executors::SingleThreadedExecutor executor;

        // To make sure that a control message is only received once for each
        // active state
        bool control_true_flag = false;
        bool control_false_flag = false;

        // To make sure that the takeoff command was received and only received
        // once
        bool takeoff_command_received = false;

        std::shared_ptr<MissionControl> mission_control_node =
            std::make_shared<MissionControl>(default_options);

        // Manually set variables
        mission_control_node->current_flight_mode =
            interfaces::msg::FlightMode::HOLD;
        mission_control_node->current_landed_state =
            interfaces::msg::LandedState::ON_GROUND;
        mission_control_node->current_position.values_set = true;

        // Create demo nodes
        class OpenCommonNode : public common_lib::CommonNode {
           public:
            OpenCommonNode(const std::string &id) : CommonNode(id) {}

            void activate_wrapper() { this->activate(); }
        };

        std::shared_ptr<common_lib::CommonNode> qr_code_scanner_node =
            std::make_shared<common_lib::CommonNode>(
                common_lib::node_names::QRCODE_SCANNER);

        std::shared_ptr<OpenCommonNode> fcc_bridge =
            std::make_shared<OpenCommonNode>(
                common_lib::node_names::FCC_BRIDGE);
        fcc_bridge->activate_wrapper();

        std::shared_ptr<common_lib::CommonNode> waypoint_node =
            std::make_shared<common_lib::CommonNode>(
                common_lib::node_names::WAYPOINT);

        // Create test node
        std::shared_ptr<rclcpp::Node> test_node =
            std::make_shared<rclcpp::Node>("test_node");

        mission_control_node->set_mission_state(MissionControl::takeoff);

        // Subscribe test node to Control topic
        rclcpp::Subscription<interfaces::msg::Control>::SharedPtr control_sub =
            test_node->create_subscription<interfaces::msg::Control>(
                common_lib::topic_names::Control, 10,
                [test_node, &control_false_flag, &control_true_flag,
                 mission_control_node,
                 &executor](interfaces::msg::Control::ConstSharedPtr msg) {
                    RCLCPP_DEBUG(test_node->get_logger(),
                                 "Received control message: Active: %d",
                                 msg->active);

                    ASSERT_STREQ(common_lib::node_names::MISSION_CONTROL,
                                 msg->target_id.c_str());

                    if (msg->active) {
                        ASSERT_TRUE(mission_control_node->get_active());
                        ASSERT_STREQ(
                            mission_control_node->get_name(),
                            mission_control_node->get_active_node_id().c_str());

                        ASSERT_FALSE(control_true_flag);
                        control_true_flag = true;
                    } else {
                        ASSERT_FALSE(mission_control_node->get_active());
                        ASSERT_EQ("",
                                  mission_control_node->get_active_node_id());

                        ASSERT_FALSE(control_false_flag);
                        control_false_flag = true;

                        executor.cancel();
                    }
                });

        // Subscribe test node to UAVCommand topic
        rclcpp::Subscription<interfaces::msg::UAVCommand>::SharedPtr
            uav_command_sub =
                test_node->create_subscription<interfaces::msg::UAVCommand>(
                    common_lib::topic_names::UAVCommand, 10,
                    [test_node, mission_control_node,
                     &takeoff_command_received](
                        interfaces::msg::UAVCommand::ConstSharedPtr msg) {
                        RCLCPP_DEBUG(test_node->get_logger(),
                                     "Received uav command message");

                        ASSERT_FALSE(takeoff_command_received);
                        takeoff_command_received = true;

                        ASSERT_STREQ(common_lib::node_names::MISSION_CONTROL,
                                     msg->sender_id.c_str());

                        ASSERT_EQ(0.0, msg->waypoint.latitude_deg);
                        ASSERT_EQ(0.0, msg->waypoint.longitude_deg);
                        ASSERT_EQ(2.0, msg->waypoint.relative_altitude_m);
                        ASSERT_EQ(interfaces::msg::UAVCommand::TAKE_OFF,
                                  msg->type);

                        // Set mission progress
                        mission_control_node->mission_progress = 1.0;
                    });

        executor.add_node(mission_control_node);
        executor.add_node(test_node);

        executor.add_node(qr_code_scanner_node);
        executor.add_node(fcc_bridge);
        executor.add_node(waypoint_node);

        executor.spin();

        ASSERT_EQ(MissionControl::decision_maker,
                  mission_control_node->get_mission_state());
        ASSERT_TRUE(control_true_flag);
        ASSERT_TRUE(control_false_flag);
        ASSERT_TRUE(takeoff_command_received);
    }

    // Mission Abort, because flight mode is 'UNKNOWN'
    {
        rclcpp::executors::SingleThreadedExecutor executor;

        std::shared_ptr<MissionControl> mission_control_node =
            std::make_shared<MissionControl>(default_options);

        // Manually set variables
        mission_control_node->current_flight_mode =
            interfaces::msg::FlightMode::UNKNOWN;
        mission_control_node->current_landed_state =
            interfaces::msg::LandedState::ON_GROUND;
        mission_control_node->current_position.values_set = true;

        // Create demo nodes
        class OpenCommonNode : public common_lib::CommonNode {
           public:
            OpenCommonNode(const std::string &id) : CommonNode(id) {}

            void activate_wrapper() { this->activate(); }
        };

        std::shared_ptr<common_lib::CommonNode> qr_code_scanner_node =
            std::make_shared<common_lib::CommonNode>(
                common_lib::node_names::QRCODE_SCANNER);

        std::shared_ptr<OpenCommonNode> fcc_bridge =
            std::make_shared<OpenCommonNode>(
                common_lib::node_names::FCC_BRIDGE);
        fcc_bridge->activate_wrapper();

        std::shared_ptr<common_lib::CommonNode> waypoint_node =
            std::make_shared<common_lib::CommonNode>(
                common_lib::node_names::WAYPOINT);

        mission_control_node->set_mission_state(MissionControl::takeoff);

        executor.add_node(mission_control_node);

        executor.add_node(qr_code_scanner_node);
        executor.add_node(fcc_bridge);
        executor.add_node(waypoint_node);

        ASSERT_DEATH({ executor.spin(); }, ".*");
    }

    // Mission Abort, because landed state is 'UNKNOWN'
    {
        rclcpp::executors::SingleThreadedExecutor executor;

        std::shared_ptr<MissionControl> mission_control_node =
            std::make_shared<MissionControl>(default_options);

        // Manually set variables
        mission_control_node->current_flight_mode =
            interfaces::msg::FlightMode::HOLD;
        mission_control_node->current_landed_state =
            interfaces::msg::LandedState::UNKNOWN;
        mission_control_node->current_position.values_set = true;

        // Create demo nodes
        class OpenCommonNode : public common_lib::CommonNode {
           public:
            OpenCommonNode(const std::string &id) : CommonNode(id) {}

            void activate_wrapper() { this->activate(); }
        };

        std::shared_ptr<common_lib::CommonNode> qr_code_scanner_node =
            std::make_shared<common_lib::CommonNode>(
                common_lib::node_names::QRCODE_SCANNER);

        std::shared_ptr<OpenCommonNode> fcc_bridge =
            std::make_shared<OpenCommonNode>(
                common_lib::node_names::FCC_BRIDGE);
        fcc_bridge->activate_wrapper();

        std::shared_ptr<common_lib::CommonNode> waypoint_node =
            std::make_shared<common_lib::CommonNode>(
                common_lib::node_names::WAYPOINT);

        mission_control_node->set_mission_state(MissionControl::takeoff);

        executor.add_node(mission_control_node);

        executor.add_node(qr_code_scanner_node);
        executor.add_node(fcc_bridge);
        executor.add_node(waypoint_node);

        ASSERT_DEATH({ executor.spin(); }, ".*");
    }

    // Mission Abort, because current position is unknown
    {
        rclcpp::executors::SingleThreadedExecutor executor;

        std::shared_ptr<MissionControl> mission_control_node =
            std::make_shared<MissionControl>(default_options);

        // Manually set variables
        mission_control_node->current_flight_mode =
            interfaces::msg::FlightMode::HOLD;
        mission_control_node->current_landed_state =
            interfaces::msg::LandedState::ON_GROUND;
        mission_control_node->current_position.values_set = false;

        // Create demo nodes
        class OpenCommonNode : public common_lib::CommonNode {
           public:
            OpenCommonNode(const std::string &id) : CommonNode(id) {}

            void activate_wrapper() { this->activate(); }
        };

        std::shared_ptr<common_lib::CommonNode> qr_code_scanner_node =
            std::make_shared<common_lib::CommonNode>(
                common_lib::node_names::QRCODE_SCANNER);

        std::shared_ptr<OpenCommonNode> fcc_bridge =
            std::make_shared<OpenCommonNode>(
                common_lib::node_names::FCC_BRIDGE);
        fcc_bridge->activate_wrapper();

        std::shared_ptr<common_lib::CommonNode> waypoint_node =
            std::make_shared<common_lib::CommonNode>(
                common_lib::node_names::WAYPOINT);

        mission_control_node->set_mission_state(MissionControl::takeoff);

        executor.add_node(mission_control_node);

        executor.add_node(qr_code_scanner_node);
        executor.add_node(fcc_bridge);
        executor.add_node(waypoint_node);

        ASSERT_DEATH({ executor.spin(); }, ".*");
    }
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
