#include <gtest/gtest.h>

#include <chrono>
#include <cinttypes>
#include <nlohmann/json.hpp>

#include "common_package/commands.hpp"
#include "common_package/common_node.hpp"
#include "common_package/node_names.hpp"
#include "common_package/topic_names.hpp"
#include "mission_control.hpp"
#include "mission_definition_file.hpp"
#include "rclcpp/executors.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/node_options.hpp"

// Message includes
#include "interfaces/msg/flight_mode.hpp"
#include "interfaces/msg/landed_state.hpp"
#include "interfaces/msg/safety_limits.hpp"
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

/**
 * @brief Test case for the `mission_control_package` mode_self_check_test.
 *
 * This test case verifies the behavior of the `mode_self_check` function in the
 * `mission_control_package`. It tests the self check procedure under different
 * conditions, checks the correctness of safety settings, and verifies the
 * mission abort behavior due to a timeout.
 */
TEST(mission_control_package, mode_self_check_test) {
    rclcpp::NodeOptions default_options;
    default_options.append_parameter_override(
        "MDF_FILE_PATH",
        "../../src/mission_control_package/test/mission_file_reader/"
        "test_assets/mdf_correct.json");

    const uint32_t max_wait_time =
        (30 /* [s] */ * 1000) / MissionControl::event_loop_time_delta_ms;

    // Test fully working self check procedure
    {
        MissionControl mission_control(default_options);
        for (size_t i = 0; i < max_wait_time * 0.1; i++) {
            ASSERT_NO_THROW(mission_control.mode_self_check());
        }

        mission_control.heartbeat_received_all = true;
        mission_control.drone_health_ok = false;

        for (size_t i = 0; i < max_wait_time * 0.1; i++) {
            ASSERT_NO_THROW(mission_control.mode_self_check());
        }

        mission_control.heartbeat_received_all = false;
        mission_control.drone_health_ok = true;

        for (size_t i = 0; i < max_wait_time * 0.1; i++) {
            ASSERT_NO_THROW(mission_control.mode_self_check());
        }

        mission_control.heartbeat_received_all = false;
        mission_control.drone_health_ok = false;

        for (size_t i = 0; i < max_wait_time * 0.1; i++) {
            ASSERT_NO_THROW(mission_control.mode_self_check());
        }

        mission_control.heartbeat_received_all = true;
        mission_control.drone_health_ok = true;

        ASSERT_NO_THROW(mission_control.mode_self_check());
        ASSERT_EQ(MissionControl::check_drone_configuration,
                  mission_control.get_mission_state());
    }

    // Check that safety settings are sent correctly
    {
        rclcpp::executors::SingleThreadedExecutor executor;

        // To make sure that the safety settings message is only received
        // exactly once
        bool safety_settings_received_flag = false;

        std::shared_ptr<MissionControl> mission_control_node =
            std::make_shared<MissionControl>(default_options);

        // Manually set variable
        mission_control_node->drone_health_ok = true;

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

        // Subscribe test node to SafetyLimits topic
        rclcpp::Subscription<
            interfaces::msg::SafetyLimits>::SharedPtr safety_limits_sub =
            test_node->create_subscription<interfaces::msg::SafetyLimits>(
                common_lib::topic_names::SafetyLimits, 10,
                [test_node, &safety_settings_received_flag,
                 mission_control_node,
                 &executor](interfaces::msg::SafetyLimits::ConstSharedPtr msg) {
                    RCLCPP_DEBUG(test_node->get_logger(),
                                 "Received safety limits message");

                    ASSERT_FALSE(safety_settings_received_flag);
                    safety_settings_received_flag = true;

                    ASSERT_STREQ(mission_control_node->get_name(),
                                 msg->sender_id.c_str());

                    mission_file_lib::safety safety_settings =
                        mission_control_node->mission_definition_reader
                            .get_safety_settings();

                    {
                        const float max_speed_m_s =
                            std::max(safety_settings.max_horizontal_speed_mps,
                                     safety_settings.max_vertical_speed_mps);
                        ASSERT_EQ(max_speed_m_s, msg->max_speed_m_s);
                    }

                    {
                        const float max_height_m =
                            safety_settings.max_height_cm / 100.0;
                        ASSERT_EQ(max_height_m, msg->max_height_m);
                    }

                    {
                        const float min_soc = safety_settings.min_soc_percent;
                        ASSERT_EQ(min_soc, msg->min_soc);
                    }

                    {
                        std::vector<interfaces::msg::Waypoint> geofence_points;

                        for (const auto &p :
                             safety_settings.get_geofence_points()) {
                            interfaces::msg::Waypoint geofence_point;
                            geofence_point.latitude_deg = p.at(0);
                            geofence_point.longitude_deg = p.at(1);
                            geofence_point.relative_altitude_m =
                                interfaces::msg::Waypoint::INVALID_ALTITUDE;

                            geofence_points.push_back(geofence_point);
                        }

                        ASSERT_EQ(geofence_points, msg->geofence_points);
                    }

                    executor.cancel();
                });

        executor.add_node(mission_control_node);
        executor.add_node(test_node);

        executor.add_node(qr_code_scanner_node);
        executor.add_node(fcc_bridge);
        executor.add_node(waypoint_node);

        executor.spin();

        ASSERT_TRUE(safety_settings_received_flag);
    }

    // Mission Abort because of a timeout
    {
        MissionControl mission_control(default_options);
        for (size_t i = 0; i < max_wait_time - 1; i++) {
            ASSERT_NO_THROW(mission_control.mode_self_check());
        }

        ASSERT_DEATH({ mission_control.mode_self_check(); }, ".*");
    }
}

/**
 * @brief Test case for the `mission_control_package` module's
 * `initiate_takeoff_test` function.
 *
 * This test case verifies the behavior of the `initiate_takeoff_test` function
 * in the `mission_control_package` module. It tests the takeoff procedure by
 * simulating various scenarios and checking the expected outcomes.
 */
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

/**
 * @brief Test case for the mission_control_package mode_decision_maker_test.
 *
 * This test case verifies the behavior of the mode_decision_maker function in
 * the mission_control_package. It tests the mode_decision_maker function with
 * different scenarios, including active marker 'init', marker change, and empty
 * command list.
 */
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

/**
 * @brief Test case for the mission_control_package.
 *
 * This test case verifies the behavior of the `mode_fly_to_waypoint_test`
 * function in the `mission_control_package`. It tests the fly to waypoint
 * procedure by simulating different scenarios such as a fully working
 * procedure, a mission abort due to a command with the wrong type, and a
 * mission abort due to a timeout.
 */
TEST(mission_control_package, mode_fly_to_waypoint_test) {
    rclcpp::NodeOptions default_options;
    default_options.append_parameter_override(
        "MDF_FILE_PATH",
        "../../src/mission_control_package/test/"
        "mission_file_reader/test_assets/mdf_correct.json");

    // Fully working fly to waypoint procedure
    {
        rclcpp::executors::SingleThreadedExecutor executor;

        std::shared_ptr<MissionControl> mission_control_node =
            std::make_shared<MissionControl>(default_options);

        // To make sure that a control message is only received once for each
        // active state
        bool control_true_flag = false;
        bool control_false_flag = false;

        // Create demo nodes
        class OpenCommonNode : public common_lib::CommonNode {
           public:
            OpenCommonNode(const std::string &id) : CommonNode(id) {}

            void activate_wrapper() { this->activate(); }
            void deactivate_wrapper() { this->deactivate(); }
            void job_finished_wrapper() { this->job_finished(); }
        };

        std::shared_ptr<common_lib::CommonNode> qr_code_scanner_node =
            std::make_shared<common_lib::CommonNode>(
                common_lib::node_names::QRCODE_SCANNER);

        std::shared_ptr<OpenCommonNode> fcc_bridge =
            std::make_shared<OpenCommonNode>(
                common_lib::node_names::FCC_BRIDGE);
        fcc_bridge->activate_wrapper();

        // Create test node
        std::shared_ptr<OpenCommonNode> waypoint_node =
            std::make_shared<OpenCommonNode>(common_lib::node_names::WAYPOINT);

        ASSERT_NO_THROW(mission_control_node->mode_prepare_mission());
        ASSERT_NO_THROW(mission_control_node->mode_decision_maker());
        ASSERT_EQ(MissionControl::fly_to_waypoint,
                  mission_control_node->get_mission_state());

        // Subscribe test node to Control topic
        rclcpp::Subscription<interfaces::msg::Control>::SharedPtr control_sub =
            waypoint_node->create_subscription<interfaces::msg::Control>(
                common_lib::topic_names::Control, 10,
                [waypoint_node, mission_control_node, &control_true_flag,
                 &control_false_flag,
                 &executor](interfaces::msg::Control::ConstSharedPtr msg) {
                    RCLCPP_DEBUG(waypoint_node->get_logger(),
                                 "Received control message: Active: %d",
                                 msg->active);

                    ASSERT_STREQ(waypoint_node->get_name(),
                                 msg->target_id.c_str());

                    if (msg->active) {
                        ASSERT_FALSE(waypoint_node->get_active());
                        waypoint_node->activate_wrapper();

                        ASSERT_FALSE(control_true_flag);
                        control_true_flag = true;

                        // Check that payload is formatted correctly
                        ASSERT_NO_THROW(
                            common_lib::CommandDefinitions::
                                parse_check_json_str(
                                    msg->payload,
                                    common_lib::CommandDefinitions::
                                        get_waypoint_command_definition()));

                        // Indicate that job is finished
                        waypoint_node->job_finished_wrapper();
                    } else {
                        // Active should already be false as it is deactivated
                        // in job_finished function
                        ASSERT_FALSE(waypoint_node->get_active());

                        ASSERT_FALSE(control_false_flag);
                        control_false_flag = true;

                        executor.cancel();
                    }
                });

        executor.add_node(mission_control_node);
        executor.add_node(waypoint_node);

        executor.add_node(qr_code_scanner_node);
        executor.add_node(fcc_bridge);

        executor.spin();

        // Activate one more time for cleanup
        mission_control_node->mode_fly_to_waypoint();

        ASSERT_EQ(MissionControl::decision_maker,
                  mission_control_node->get_mission_state());
        ASSERT_TRUE(control_true_flag);
        ASSERT_TRUE(control_false_flag);
    }

    // Mission Abort because command has the wrong type
    {
        rclcpp::executors::SingleThreadedExecutor executor;

        std::shared_ptr<MissionControl> mission_control_node =
            std::make_shared<MissionControl>(default_options);

        ASSERT_NO_THROW(mission_control_node->mode_prepare_mission());
        ASSERT_NO_THROW(mission_control_node->mode_decision_maker());

        mission_control_node->commands[mission_control_node->current_command_id]
            .type = "abc";

        ASSERT_DEATH({ mission_control_node->mode_fly_to_waypoint(); }, ".*");
    }

    // Mission Abort because of a timeout
    {
        rclcpp::executors::SingleThreadedExecutor executor;

        std::shared_ptr<MissionControl> mission_control_node =
            std::make_shared<MissionControl>(default_options);

        ASSERT_NO_THROW(mission_control_node->mode_prepare_mission());
        ASSERT_NO_THROW(mission_control_node->mode_decision_maker());

        ASSERT_NO_THROW(mission_control_node->mode_fly_to_waypoint());

        // Simulate that wait time is over
        mission_control_node->wait_time_finished_ok = true;

        ASSERT_DEATH({ mission_control_node->mode_fly_to_waypoint(); }, ".*");
    }
}

/**
 * @brief Test case for the mission_control_package mode_detect_marker_test.
 *
 * This test case verifies the behavior of the mode_detect_marker function in
 * the mission_control_package. It tests the following scenarios:
 * 1. Fully working detect marker procedure.
 * 2. Mission abort because the command has the wrong type.
 * 3. Mission abort because of a timeout.
 */
TEST(mission_control_package, mode_detect_marker_test) {
    rclcpp::NodeOptions default_options;
    default_options.append_parameter_override(
        "MDF_FILE_PATH",
        "../../src/mission_control_package/test/"
        "mission_file_reader/test_assets/mdf_correct.json");

    // Fully working detect marker procedure
    {
        rclcpp::executors::SingleThreadedExecutor executor;

        std::shared_ptr<MissionControl> mission_control_node =
            std::make_shared<MissionControl>(default_options);

        // To make sure that a control message is only received once for each
        // active state
        bool control_true_flag = false;
        bool control_false_flag = false;

        // Create demo nodes
        class OpenCommonNode : public common_lib::CommonNode {
           public:
            OpenCommonNode(const std::string &id) : CommonNode(id) {}

            void activate_wrapper() { this->activate(); }
            void deactivate_wrapper() { this->deactivate(); }
            void job_finished_wrapper() { this->job_finished(); }
            void job_finished_wrapper(const uint8_t error_code,
                                      const nlohmann::json &payload) {
                this->job_finished(error_code, payload);
            }
        };

        std::shared_ptr<common_lib::CommonNode> waypoint_node =
            std::make_shared<common_lib::CommonNode>(
                common_lib::node_names::WAYPOINT);

        std::shared_ptr<OpenCommonNode> fcc_bridge =
            std::make_shared<OpenCommonNode>(
                common_lib::node_names::FCC_BRIDGE);
        fcc_bridge->activate_wrapper();

        // Create test node
        std::shared_ptr<OpenCommonNode> qr_code_scanner_node =
            std::make_shared<OpenCommonNode>(
                common_lib::node_names::QRCODE_SCANNER);

        ASSERT_NO_THROW(mission_control_node->mode_prepare_mission());
        for (size_t i = 0; i < 3; i++)
            ASSERT_NO_THROW(mission_control_node->mode_decision_maker());

        ASSERT_EQ(MissionControl::detect_marker,
                  mission_control_node->get_mission_state());

        // Subscribe test node to Control topic
        rclcpp::Subscription<interfaces::msg::Control>::SharedPtr control_sub =
            qr_code_scanner_node->create_subscription<interfaces::msg::Control>(
                common_lib::topic_names::Control, 10,
                [qr_code_scanner_node, mission_control_node, &control_true_flag,
                 &control_false_flag,
                 &executor](interfaces::msg::Control::ConstSharedPtr msg) {
                    RCLCPP_DEBUG(qr_code_scanner_node->get_logger(),
                                 "Received control message: Active: %d",
                                 msg->active);

                    ASSERT_STREQ(qr_code_scanner_node->get_name(),
                                 msg->target_id.c_str());

                    if (msg->active) {
                        ASSERT_FALSE(qr_code_scanner_node->get_active());
                        qr_code_scanner_node->activate_wrapper();

                        ASSERT_FALSE(control_true_flag);
                        control_true_flag = true;

                        // Check that payload is formatted correctly
                        ASSERT_NO_THROW(
                            common_lib::CommandDefinitions::parse_check_json_str(
                                msg->payload,
                                common_lib::CommandDefinitions::
                                    get_detect_marker_command_definition()));

                        // Indicate that job is finished
                        qr_code_scanner_node->job_finished_wrapper(
                            EXIT_SUCCESS, {{"marker", "1"}});
                    } else {
                        // Active should already be false as it is deactivated
                        // in job_finished function
                        ASSERT_FALSE(qr_code_scanner_node->get_active());

                        ASSERT_FALSE(control_false_flag);
                        control_false_flag = true;

                        executor.cancel();
                    }
                });

        executor.add_node(mission_control_node);
        executor.add_node(waypoint_node);

        executor.add_node(qr_code_scanner_node);
        executor.add_node(fcc_bridge);

        executor.spin();

        // Activate one more time for cleanup
        mission_control_node->mode_detect_marker();

        ASSERT_EQ(MissionControl::decision_maker,
                  mission_control_node->get_mission_state());
        ASSERT_TRUE(control_true_flag);
        ASSERT_TRUE(control_false_flag);
    }

    // Mission Abort because command has the wrong type
    {
        rclcpp::executors::SingleThreadedExecutor executor;

        std::shared_ptr<MissionControl> mission_control_node =
            std::make_shared<MissionControl>(default_options);

        ASSERT_NO_THROW(mission_control_node->mode_prepare_mission());
        for (size_t i = 0; i < 3; i++)
            ASSERT_NO_THROW(mission_control_node->mode_decision_maker());

        mission_control_node->commands[mission_control_node->current_command_id]
            .type = "abc";

        ASSERT_DEATH({ mission_control_node->mode_detect_marker(); }, ".*");
    }

    // Mission Abort because of a timeout
    {
        rclcpp::executors::SingleThreadedExecutor executor;

        std::shared_ptr<MissionControl> mission_control_node =
            std::make_shared<MissionControl>(default_options);

        ASSERT_NO_THROW(mission_control_node->mode_prepare_mission());
        for (size_t i = 0; i < 3; i++)
            ASSERT_NO_THROW(mission_control_node->mode_decision_maker());

        ASSERT_NO_THROW(mission_control_node->mode_detect_marker());

        // Simulate that wait time is over
        mission_control_node->wait_time_finished_ok = true;

        ASSERT_DEATH({ mission_control_node->mode_detect_marker(); }, ".*");
    }
}
