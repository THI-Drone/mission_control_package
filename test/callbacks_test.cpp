#include <gtest/gtest.h>

#include <chrono>
#include <cinttypes>
#include <nlohmann/json.hpp>
#include <unordered_set>

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
#include "structs.hpp"

// Message includes
#include "interfaces/msg/flight_mode.hpp"
#include "interfaces/msg/flight_state.hpp"
#include "interfaces/msg/gps_position.hpp"
#include "interfaces/msg/heartbeat.hpp"
#include "interfaces/msg/job_finished.hpp"
#include "interfaces/msg/landed_state.hpp"
#include "interfaces/msg/mission_progress.hpp"
#include "interfaces/msg/mission_start.hpp"
#include "interfaces/msg/uav_command.hpp"
#include "interfaces/msg/uav_health.hpp"
#include "interfaces/msg/uav_waypoint_command.hpp"

/**
 * @brief Test case for the `mission_control_package` module's
 * `job_finished_callback` function.
 *
 * This test verifies the behavior of the `job_finished_callback` function in
 * different scenarios:
 * - Successfull JobFinished message from an active node
 * - Successfull JobFinished message from an inactive node
 * - Failed JobFinished message from an active node
 * - Failed JobFinished message from an inactive node
 *
 * The test sets up a mission control node and a test node, and publishes
 * JobFinished messages to the mission control node. It checks that the mission
 * control node handles the messages correctly and performs the necessary
 * actions.
 */
TEST(mission_control_package, job_finished_callback_test) {
    rclcpp::NodeOptions default_options;
    default_options.append_parameter_override(
        "MDF_FILE_PATH",
        "../../src/mission_control_package/test/mission_file_reader/"
        "test_assets/mdf_correct.json");

    // Successfull JobFinished message from an active node
    {
        rclcpp::executors::SingleThreadedExecutor executor;

        bool job_finished = false;
        bool control_json_received = false;

        std::shared_ptr<MissionControl> mission_control_node =
            std::make_shared<MissionControl>(default_options);

        mission_control_node->set_active_node_id("test_node");

        // Create test node
        std::shared_ptr<rclcpp::Node> test_node =
            std::make_shared<rclcpp::Node>("test_node");

        const auto message_publisher =
            test_node->create_publisher<interfaces::msg::JobFinished>(
                common_lib::topic_names::JobFinished, 10);

        rclcpp::TimerBase::SharedPtr trigger_timer =
            test_node->create_wall_timer(
                std::chrono::milliseconds(10),
                [test_node, &message_publisher]() {
                    RCLCPP_DEBUG(test_node->get_logger(), "Publishing message");

                    // Create and publish message
                    interfaces::msg::JobFinished msg;
                    msg.sender_id = test_node->get_name();
                    msg.error_code = EXIT_SUCCESS;
                    msg.payload = nlohmann::json{{"abc", 123}}.dump();

                    message_publisher->publish(msg);
                });

        rclcpp::TimerBase::SharedPtr end_timer =
            mission_control_node->create_wall_timer(
                std::chrono::milliseconds(10),
                [mission_control_node, &job_finished, &control_json_received,
                 &executor]() {
                    // All checks executed, stop test
                    if (control_json_received && job_finished) {
                        executor.cancel();
                    }

                    // Check that job finished
                    if (!mission_control_node->get_job_finished_successfully())
                        return;

                    // Check that job finished only once
                    ASSERT_FALSE(job_finished);

                    RCLCPP_DEBUG(mission_control_node->get_logger(),
                                 "JobFinished message detected");

                    ASSERT_EQ(nlohmann::json({{"abc", 123}}),
                              mission_control_node->get_job_finished_payload());

                    ASSERT_EQ("", mission_control_node->get_active_node_id());

                    job_finished = true;
                });

        rclcpp::Subscription<interfaces::msg::Control>::SharedPtr control_sub =
            test_node->create_subscription<interfaces::msg::Control>(
                common_lib::topic_names::Control, 10,
                [test_node, &control_json_received](
                    interfaces::msg::Control::ConstSharedPtr msg) {
                    RCLCPP_DEBUG(test_node->get_logger(),
                                 "Received control message");

                    // Check that control message was only received once
                    ASSERT_FALSE(control_json_received);

                    // Check contents of control message
                    ASSERT_EQ(test_node->get_name(), msg->target_id);
                    ASSERT_EQ(false, msg->active);
                    ASSERT_EQ(nlohmann::json(),
                              nlohmann::json::parse(msg->payload));

                    control_json_received = true;
                });

        executor.add_node(mission_control_node);
        executor.add_node(test_node);

        executor.spin();

        ASSERT_TRUE(control_json_received);
        ASSERT_TRUE(job_finished);
    }

    // Successfull JobFinished message from an inactive node
    {
        rclcpp::executors::SingleThreadedExecutor executor;

        bool control_json_received = false;

        std::shared_ptr<MissionControl> mission_control_node =
            std::make_shared<MissionControl>(default_options);

        mission_control_node->set_active_node_id("");

        // Create test node
        std::shared_ptr<rclcpp::Node> test_node =
            std::make_shared<rclcpp::Node>("test_node");

        const auto message_publisher =
            test_node->create_publisher<interfaces::msg::JobFinished>(
                common_lib::topic_names::JobFinished, 10);

        rclcpp::TimerBase::SharedPtr trigger_timer =
            test_node->create_wall_timer(
                std::chrono::milliseconds(10),
                [test_node, &message_publisher]() {
                    RCLCPP_DEBUG(test_node->get_logger(), "Publishing message");

                    // Create and publish message
                    interfaces::msg::JobFinished msg;
                    msg.sender_id = test_node->get_name();
                    msg.error_code = EXIT_SUCCESS;
                    msg.payload = nlohmann::json{{"abc", 123}}.dump();

                    message_publisher->publish(msg);
                });

        rclcpp::TimerBase::SharedPtr check_timer =
            mission_control_node->create_wall_timer(
                std::chrono::milliseconds(10),
                [mission_control_node, &control_json_received]() {
                    // Check that job didn't finish
                    ASSERT_FALSE(
                        mission_control_node->get_job_finished_successfully());
                    ASSERT_EQ(nlohmann::json(),
                              mission_control_node->get_job_finished_payload());
                });

        rclcpp::TimerBase::SharedPtr end_timer =
            mission_control_node->create_wall_timer(
                std::chrono::milliseconds(500),
                [mission_control_node, &executor]() {
                    RCLCPP_DEBUG(mission_control_node->get_logger(),
                                 "Ending executor");

                    executor.cancel();
                });

        rclcpp::Subscription<interfaces::msg::Control>::SharedPtr control_sub =
            test_node->create_subscription<interfaces::msg::Control>(
                common_lib::topic_names::Control, 10,
                [test_node, &control_json_received](
                    interfaces::msg::Control::ConstSharedPtr msg) {
                    // This shouldn't be called

                    RCLCPP_ERROR(test_node->get_logger(),
                                 "Received control message: '%s'",
                                 msg->target_id.c_str());

                    control_json_received = true;
                });

        executor.add_node(mission_control_node);
        executor.add_node(test_node);

        executor.spin();

        ASSERT_FALSE(control_json_received);
    }

    // Failed JobFinished message from an active node
    {
        rclcpp::executors::SingleThreadedExecutor executor;

        std::shared_ptr<MissionControl> mission_control_node =
            std::make_shared<MissionControl>(default_options);

        mission_control_node->set_active_node_id("test_node");

        // Create test node
        std::shared_ptr<rclcpp::Node> test_node =
            std::make_shared<rclcpp::Node>("test_node");

        const auto message_publisher =
            test_node->create_publisher<interfaces::msg::JobFinished>(
                common_lib::topic_names::JobFinished, 10);

        rclcpp::TimerBase::SharedPtr trigger_timer =
            test_node->create_wall_timer(
                std::chrono::milliseconds(10),
                [test_node, &message_publisher]() {
                    RCLCPP_DEBUG(test_node->get_logger(), "Publishing message");

                    // Create and publish message
                    interfaces::msg::JobFinished msg;
                    msg.sender_id = test_node->get_name();
                    msg.error_code = EXIT_FAILURE;
                    msg.payload = nlohmann::json{{"abc", 123}}.dump();

                    message_publisher->publish(msg);
                });

        executor.add_node(mission_control_node);
        executor.add_node(test_node);

        ASSERT_DEATH({ executor.spin(); }, ".*");
    }

    // Failed JobFinished message from an inactive node
    {
        rclcpp::executors::SingleThreadedExecutor executor;

        std::shared_ptr<MissionControl> mission_control_node =
            std::make_shared<MissionControl>(default_options);

        mission_control_node->set_active_node_id("");

        // Create test node
        std::shared_ptr<rclcpp::Node> test_node =
            std::make_shared<rclcpp::Node>("test_node");

        const auto message_publisher =
            test_node->create_publisher<interfaces::msg::JobFinished>(
                common_lib::topic_names::JobFinished, 10);

        rclcpp::TimerBase::SharedPtr trigger_timer =
            test_node->create_wall_timer(
                std::chrono::milliseconds(10),
                [test_node, &message_publisher]() {
                    RCLCPP_DEBUG(test_node->get_logger(), "Publishing message");

                    // Create and publish message
                    interfaces::msg::JobFinished msg;
                    msg.sender_id = test_node->get_name();
                    msg.error_code = EXIT_FAILURE;
                    msg.payload = nlohmann::json{{"abc", 123}}.dump();

                    message_publisher->publish(msg);
                });

        executor.add_node(mission_control_node);
        executor.add_node(test_node);

        ASSERT_DEATH({ executor.spin(); }, ".*");
    }

    // JobFinished message with malformed json in payload
    {
        rclcpp::executors::SingleThreadedExecutor executor;

        std::shared_ptr<MissionControl> mission_control_node =
            std::make_shared<MissionControl>(default_options);

        // Create test node
        std::shared_ptr<rclcpp::Node> test_node =
            std::make_shared<rclcpp::Node>("test_node");

        const auto message_publisher =
            test_node->create_publisher<interfaces::msg::JobFinished>(
                common_lib::topic_names::JobFinished, 10);

        rclcpp::TimerBase::SharedPtr trigger_timer =
            test_node->create_wall_timer(
                std::chrono::milliseconds(10),
                [test_node, &message_publisher]() {
                    RCLCPP_DEBUG(test_node->get_logger(), "Publishing message");

                    // Create and publish message
                    interfaces::msg::JobFinished msg;
                    msg.sender_id = test_node->get_name();
                    msg.error_code = EXIT_SUCCESS;
                    msg.payload = "this is not a json";

                    message_publisher->publish(msg);
                });

        executor.add_node(mission_control_node);
        executor.add_node(test_node);

        ASSERT_DEATH({ executor.spin(); }, ".*");
    }
}

/**
 * @brief Test case for the mission_control_package.
 *
 * This test case verifies the behavior of the mission start functionality in
 * the mission_control_package. It includes multiple test scenarios such as
 * successful mission start, mission start while not in 'armed' state, mission
 * start with an unknown node id, and mission start with an unauthorized node
 * id.
 */
TEST(mission_control_package, mission_start_callback_test) {
    rclcpp::NodeOptions default_options;
    default_options.append_parameter_override(
        "MDF_FILE_PATH",
        "../../src/mission_control_package/test/mission_file_reader/"
        "test_assets/mdf_correct.json");

    // Test successfull Mission Start
    {
        rclcpp::executors::SingleThreadedExecutor executor;

        std::shared_ptr<MissionControl> mission_control_node =
            std::make_shared<MissionControl>(default_options);

        // Deactivate event loop so it doesn't mess with our test
        mission_control_node->event_loop_active = false;

        // Set mission state to 'armed'
        mission_control_node->set_mission_state(MissionControl::armed);

        // Create test node
        std::shared_ptr<rclcpp::Node> test_node =
            std::make_shared<rclcpp::Node>(common_lib::node_names::FCC_BRIDGE);

        const auto message_publisher =
            test_node->create_publisher<interfaces::msg::MissionStart>(
                common_lib::topic_names::MissionStart, 10);

        rclcpp::TimerBase::SharedPtr trigger_timer =
            test_node->create_wall_timer(
                std::chrono::milliseconds(10),
                [test_node, &message_publisher]() {
                    RCLCPP_DEBUG(test_node->get_logger(), "Publishing message");

                    // Create and publish message
                    interfaces::msg::MissionStart msg;
                    msg.sender_id = test_node->get_name();

                    message_publisher->publish(msg);
                });

        rclcpp::TimerBase::SharedPtr check_timer =
            mission_control_node->create_wall_timer(
                std::chrono::milliseconds(100),
                [mission_control_node, &executor]() {
                    // Check that mission started
                    ASSERT_EQ("init",
                              mission_control_node->get_active_marker_name());
                    ASSERT_EQ(MissionControl::takeoff,
                              mission_control_node->get_mission_state());

                    executor.cancel();
                });

        executor.add_node(mission_control_node);
        executor.add_node(test_node);

        executor.spin();
    }

    // Test Mission Start while not in 'armed' state
    {
        rclcpp::executors::SingleThreadedExecutor executor;

        std::shared_ptr<MissionControl> mission_control_node =
            std::make_shared<MissionControl>(default_options);

        // Deactivate event loop so it doesn't mess with our test
        mission_control_node->event_loop_active = false;

        // Set mission state to 'prepare_mission'
        mission_control_node->set_mission_state(
            MissionControl::prepare_mission);

        // Create test node
        std::shared_ptr<rclcpp::Node> test_node =
            std::make_shared<rclcpp::Node>(common_lib::node_names::FCC_BRIDGE);

        const auto message_publisher =
            test_node->create_publisher<interfaces::msg::MissionStart>(
                common_lib::topic_names::MissionStart, 10);

        rclcpp::TimerBase::SharedPtr trigger_timer =
            test_node->create_wall_timer(
                std::chrono::milliseconds(10),
                [test_node, &message_publisher]() {
                    RCLCPP_DEBUG(test_node->get_logger(), "Publishing message");

                    // Create and publish message
                    interfaces::msg::MissionStart msg;
                    msg.sender_id = test_node->get_name();

                    message_publisher->publish(msg);
                });

        rclcpp::TimerBase::SharedPtr check_timer =
            mission_control_node->create_wall_timer(
                std::chrono::milliseconds(100),
                [mission_control_node, &executor]() {
                    // Check that mission didn't start
                    ASSERT_EQ(MissionControl::prepare_mission,
                              mission_control_node->get_mission_state());

                    executor.cancel();
                });

        executor.add_node(mission_control_node);
        executor.add_node(test_node);

        executor.spin();
    }

    // Test Mission Start with unknown node id
    {
        rclcpp::executors::SingleThreadedExecutor executor;

        std::shared_ptr<MissionControl> mission_control_node =
            std::make_shared<MissionControl>(default_options);

        // Deactivate event loop so it doesn't mess with our test
        mission_control_node->event_loop_active = false;

        // Set mission state to 'armed'
        mission_control_node->set_mission_state(MissionControl::armed);

        // Create test node
        std::shared_ptr<rclcpp::Node> test_node =
            std::make_shared<rclcpp::Node>("test_node");

        const auto message_publisher =
            test_node->create_publisher<interfaces::msg::MissionStart>(
                common_lib::topic_names::MissionStart, 10);

        rclcpp::TimerBase::SharedPtr trigger_timer =
            test_node->create_wall_timer(
                std::chrono::milliseconds(10),
                [test_node, &message_publisher]() {
                    RCLCPP_DEBUG(test_node->get_logger(), "Publishing message");

                    // Create and publish message
                    interfaces::msg::MissionStart msg;
                    msg.sender_id = test_node->get_name();

                    message_publisher->publish(msg);
                });

        rclcpp::TimerBase::SharedPtr check_timer =
            mission_control_node->create_wall_timer(
                std::chrono::milliseconds(100),
                [mission_control_node, &executor]() {
                    // Check that mission didn't start
                    ASSERT_EQ(MissionControl::armed,
                              mission_control_node->get_mission_state());

                    executor.cancel();
                });

        executor.add_node(mission_control_node);
        executor.add_node(test_node);

        executor.spin();
    }

    // Test Mission Start with unauthorized node id
    {
        rclcpp::executors::SingleThreadedExecutor executor;

        std::shared_ptr<MissionControl> mission_control_node =
            std::make_shared<MissionControl>(default_options);

        // Deactivate event loop so it doesn't mess with our test
        mission_control_node->event_loop_active = false;

        // Set mission state to 'armed'
        mission_control_node->set_mission_state(MissionControl::armed);

        // Create test node
        std::shared_ptr<rclcpp::Node> test_node =
            std::make_shared<rclcpp::Node>(common_lib::node_names::WAYPOINT);

        const auto message_publisher =
            test_node->create_publisher<interfaces::msg::MissionStart>(
                common_lib::topic_names::MissionStart, 10);

        rclcpp::TimerBase::SharedPtr trigger_timer =
            test_node->create_wall_timer(
                std::chrono::milliseconds(10),
                [test_node, &message_publisher]() {
                    RCLCPP_DEBUG(test_node->get_logger(), "Publishing message");

                    // Create and publish message
                    interfaces::msg::MissionStart msg;
                    msg.sender_id = test_node->get_name();

                    message_publisher->publish(msg);
                });

        rclcpp::TimerBase::SharedPtr check_timer =
            mission_control_node->create_wall_timer(
                std::chrono::milliseconds(100),
                [mission_control_node, &executor]() {
                    // Check that mission didn't start
                    ASSERT_EQ(MissionControl::armed,
                              mission_control_node->get_mission_state());

                    executor.cancel();
                });

        executor.add_node(mission_control_node);
        executor.add_node(test_node);

        executor.spin();
    }
}

/**
 * @brief Test case for the `mission_control_package` module's
 * `waypoint_command_callback` function.
 *
 * This test case verifies the behavior of the `waypoint_command_callback`
 * function in the `MissionControl` class. It tests various scenarios such as
 * successful waypoint command check, probation period, too old timestamp,
 * inactive sender node, and mission not running.
 */
TEST(mission_control_package, waypoint_command_callback_test) {
    rclcpp::NodeOptions default_options;
    default_options.append_parameter_override(
        "MDF_FILE_PATH",
        "../../src/mission_control_package/test/mission_file_reader/"
        "test_assets/mdf_correct.json");

    // Test successfull waypoint command check
    {
        rclcpp::executors::SingleThreadedExecutor executor;

        std::shared_ptr<MissionControl> mission_control_node =
            std::make_shared<MissionControl>(default_options);

        // Deactivate event loop so it doesn't mess with our test
        mission_control_node->event_loop_active = false;

        // Set mission state to 'fly_to_waypoint'
        mission_control_node->set_mission_state(
            MissionControl::fly_to_waypoint);

        // Set active node id
        mission_control_node->set_active_node_id(
            common_lib::node_names::WAYPOINT);

        // Create test node
        std::shared_ptr<rclcpp::Node> test_node =
            std::make_shared<rclcpp::Node>(common_lib::node_names::WAYPOINT);

        const auto message_publisher =
            test_node->create_publisher<interfaces::msg::UAVWaypointCommand>(
                common_lib::topic_names::UAVWaypointCommand, 10);

        rclcpp::TimerBase::SharedPtr trigger_timer =
            test_node->create_wall_timer(
                std::chrono::milliseconds(10),
                [test_node, &message_publisher]() {
                    RCLCPP_DEBUG(test_node->get_logger(), "Publishing message");

                    // Create and publish message
                    interfaces::msg::UAVWaypointCommand msg;
                    msg.sender_id = test_node->get_name();
                    msg.time_stamp = test_node->now();

                    message_publisher->publish(msg);
                });

        rclcpp::TimerBase::SharedPtr end_timer =
            mission_control_node->create_wall_timer(
                std::chrono::milliseconds(100),
                [mission_control_node, &executor]() {
                    RCLCPP_DEBUG(mission_control_node->get_logger(),
                                 "Shutting down");

                    executor.cancel();
                });

        executor.add_node(mission_control_node);
        executor.add_node(test_node);

        executor.spin();
    }

    // Test successfull waypoint command check during probation period
    {
        rclcpp::executors::SingleThreadedExecutor executor;

        std::shared_ptr<MissionControl> mission_control_node =
            std::make_shared<MissionControl>(default_options);

        // Deactivate event loop so it doesn't mess with our test
        mission_control_node->event_loop_active = false;

        // Set mission state to 'fly_to_waypoint'
        mission_control_node->set_mission_state(
            MissionControl::fly_to_waypoint);

        // Set last active node id and thereby start probation period
        mission_control_node->set_active_node_id(
            common_lib::node_names::WAYPOINT);
        mission_control_node->clear_active_node_id();

        // Create test node
        std::shared_ptr<rclcpp::Node> test_node =
            std::make_shared<rclcpp::Node>(common_lib::node_names::WAYPOINT);

        const auto message_publisher =
            test_node->create_publisher<interfaces::msg::UAVWaypointCommand>(
                common_lib::topic_names::UAVWaypointCommand, 10);

        rclcpp::TimerBase::SharedPtr trigger_timer =
            test_node->create_wall_timer(
                std::chrono::milliseconds(10),
                [test_node, &message_publisher]() {
                    RCLCPP_DEBUG(test_node->get_logger(), "Publishing message");

                    // Create and publish message
                    interfaces::msg::UAVWaypointCommand msg;
                    msg.sender_id = test_node->get_name();
                    msg.time_stamp = test_node->now();

                    message_publisher->publish(msg);
                });

        rclcpp::TimerBase::SharedPtr end_timer =
            mission_control_node->create_wall_timer(
                std::chrono::milliseconds(
                    MissionControl::probation_period_length_ms - 10),
                [mission_control_node, &executor]() {
                    RCLCPP_DEBUG(mission_control_node->get_logger(),
                                 "Shutting down");

                    executor.cancel();
                });

        executor.add_node(mission_control_node);
        executor.add_node(test_node);

        executor.spin();
    }

    // Test successfull waypoint command check with too old timestamp while node
    // is not active
    {
        rclcpp::executors::SingleThreadedExecutor executor;

        std::shared_ptr<MissionControl> mission_control_node =
            std::make_shared<MissionControl>(default_options);

        // Deactivate event loop so it doesn't mess with our test
        mission_control_node->event_loop_active = false;

        // Set mission state to 'fly_to_waypoint'
        mission_control_node->set_mission_state(
            MissionControl::fly_to_waypoint);

        // Create test node
        std::shared_ptr<rclcpp::Node> test_node =
            std::make_shared<rclcpp::Node>(common_lib::node_names::WAYPOINT);

        const auto message_publisher =
            test_node->create_publisher<interfaces::msg::UAVWaypointCommand>(
                common_lib::topic_names::UAVWaypointCommand, 10);

        rclcpp::TimerBase::SharedPtr trigger_timer =
            test_node->create_wall_timer(
                std::chrono::milliseconds(10),
                [test_node, &message_publisher]() {
                    RCLCPP_DEBUG(test_node->get_logger(), "Publishing message");

                    // Create and publish message
                    interfaces::msg::UAVWaypointCommand msg;
                    msg.sender_id = test_node->get_name();

                    message_publisher->publish(msg);
                });

        rclcpp::TimerBase::SharedPtr end_timer =
            mission_control_node->create_wall_timer(
                std::chrono::milliseconds(
                    MissionControl::probation_period_length_ms - 10),
                [mission_control_node, &executor]() {
                    RCLCPP_DEBUG(mission_control_node->get_logger(),
                                 "Shutting down");

                    executor.cancel();
                });

        executor.add_node(mission_control_node);
        executor.add_node(test_node);

        executor.spin();
    }

    // Test failed waypoint command check with inactive sender node
    {
        rclcpp::executors::SingleThreadedExecutor executor;

        std::shared_ptr<MissionControl> mission_control_node =
            std::make_shared<MissionControl>(default_options);

        // Deactivate event loop so it doesn't mess with our test
        mission_control_node->event_loop_active = false;

        // Set mission state to 'fly_to_waypoint'
        mission_control_node->set_mission_state(
            MissionControl::fly_to_waypoint);

        // Create test node
        std::shared_ptr<rclcpp::Node> test_node =
            std::make_shared<rclcpp::Node>(common_lib::node_names::WAYPOINT);

        const auto message_publisher =
            test_node->create_publisher<interfaces::msg::UAVWaypointCommand>(
                common_lib::topic_names::UAVWaypointCommand, 10);

        rclcpp::TimerBase::SharedPtr trigger_timer =
            test_node->create_wall_timer(
                std::chrono::milliseconds(10),
                [test_node, &message_publisher]() {
                    RCLCPP_DEBUG(test_node->get_logger(), "Publishing message");

                    // Create and publish message
                    interfaces::msg::UAVWaypointCommand msg;
                    msg.sender_id = test_node->get_name();
                    msg.time_stamp = test_node->now();

                    message_publisher->publish(msg);
                });

        executor.add_node(mission_control_node);
        executor.add_node(test_node);

        ASSERT_DEATH({ executor.spin(); }, ".*");
    }

    // Test failed waypoint command check because the mission is not running
    {
        // All the states that indicate a not running mission
        std::unordered_set<MissionControl::MissionState_t> states = {
            MissionControl::prepare_mission, MissionControl::selfcheck,
            MissionControl::check_drone_configuration, MissionControl::armed};

        for (const auto &state : states) {
            rclcpp::executors::SingleThreadedExecutor executor;

            std::shared_ptr<MissionControl> mission_control_node =
                std::make_shared<MissionControl>(default_options);

            // Deactivate event loop so it doesn't mess with our test
            mission_control_node->event_loop_active = false;

            // Set mission state
            mission_control_node->set_mission_state(state);
            RCLCPP_DEBUG(mission_control_node->get_logger(),
                         "Testing state: '%s'",
                         mission_control_node->get_mission_state_str());

            // Set active node id
            mission_control_node->set_active_node_id(
                common_lib::node_names::WAYPOINT);

            // Create test node
            std::shared_ptr<rclcpp::Node> test_node =
                std::make_shared<rclcpp::Node>(
                    common_lib::node_names::WAYPOINT);

            const auto message_publisher =
                test_node
                    ->create_publisher<interfaces::msg::UAVWaypointCommand>(
                        common_lib::topic_names::UAVWaypointCommand, 10);

            rclcpp::TimerBase::SharedPtr trigger_timer =
                test_node->create_wall_timer(
                    std::chrono::milliseconds(10),
                    [test_node, &message_publisher]() {
                        RCLCPP_DEBUG(test_node->get_logger(),
                                     "Publishing message");

                        // Create and publish message
                        interfaces::msg::UAVWaypointCommand msg;
                        msg.sender_id = test_node->get_name();
                        msg.time_stamp = test_node->now();

                        message_publisher->publish(msg);
                    });

            executor.add_node(mission_control_node);
            executor.add_node(test_node);

            ASSERT_DEATH({ executor.spin(); }, ".*");
        }
    }
}

/**
 * @brief Test case for the mission_control_package.
 *
 * This test case verifies the behavior of the command_callback function in the
 * mission_control_package. It consists of two sub-tests:
 * 1. Test successfull command sent by Mission Control: This sub-test creates a
 * mission_control_node and publishes a UAVCommand message. It verifies that the
 * message is published successfully.
 * 2. Test unauthorized command sent by another node: This sub-test creates a
 * mission_control_node and a test_node. The test_node publishes a UAVCommand
 * message with a different sender_id. It verifies that the mission_control_node
 * does not process the unauthorized command and shuts down the test.
 */
TEST(mission_control_package, command_callback_test) {
    rclcpp::NodeOptions default_options;
    default_options.append_parameter_override(
        "MDF_FILE_PATH",
        "../../src/mission_control_package/test/mission_file_reader/"
        "test_assets/mdf_correct.json");

    // Test successfull command sent by Mission Control
    {
        rclcpp::executors::SingleThreadedExecutor executor;

        std::shared_ptr<MissionControl> mission_control_node =
            std::make_shared<MissionControl>(default_options);

        // Deactivate event loop so it doesn't mess with our test
        mission_control_node->event_loop_active = false;

        const auto message_publisher =
            mission_control_node->create_publisher<interfaces::msg::UAVCommand>(
                common_lib::topic_names::UAVCommand, 10);

        rclcpp::TimerBase::SharedPtr trigger_timer =
            mission_control_node->create_wall_timer(
                std::chrono::milliseconds(10),
                [mission_control_node, &message_publisher]() {
                    RCLCPP_DEBUG(mission_control_node->get_logger(),
                                 "Publishing message");

                    // Create and publish message
                    interfaces::msg::UAVCommand msg;
                    msg.sender_id = mission_control_node->get_name();
                    msg.time_stamp = mission_control_node->now();

                    message_publisher->publish(msg);
                });

        rclcpp::TimerBase::SharedPtr end_timer =
            mission_control_node->create_wall_timer(
                std::chrono::milliseconds(100),
                [mission_control_node, &executor]() {
                    RCLCPP_DEBUG(mission_control_node->get_logger(),
                                 "Shutting down test");

                    executor.cancel();
                });

        executor.add_node(mission_control_node);
        executor.spin();
    }

    // Test unauthorized command sent by another node
    {
        rclcpp::executors::SingleThreadedExecutor executor;

        std::shared_ptr<MissionControl> mission_control_node =
            std::make_shared<MissionControl>(default_options);

        // Deactivate event loop so it doesn't mess with our test
        mission_control_node->event_loop_active = false;

        // Create test node
        std::shared_ptr<rclcpp::Node> test_node =
            std::make_shared<rclcpp::Node>(common_lib::node_names::WAYPOINT);

        const auto message_publisher =
            test_node->create_publisher<interfaces::msg::UAVCommand>(
                common_lib::topic_names::UAVCommand, 10);

        rclcpp::TimerBase::SharedPtr trigger_timer =
            test_node->create_wall_timer(
                std::chrono::milliseconds(10),
                [test_node, &message_publisher]() {
                    RCLCPP_DEBUG(test_node->get_logger(), "Publishing message");

                    // Create and publish message
                    interfaces::msg::UAVCommand msg;
                    msg.sender_id = test_node->get_name();
                    msg.time_stamp = test_node->now();

                    message_publisher->publish(msg);
                });

        executor.add_node(mission_control_node);
        executor.add_node(test_node);

        ASSERT_DEATH({ executor.spin(); }, ".*");
    }
}

/**
 * @brief Test case for the `mission_control_package` heartbeat callback.
 *
 * This test case verifies the behavior of the heartbeat callback function in
 * the `mission_control_package`. It tests various scenarios including unknown
 * sender ID being ignored, probation period, and FCC bridge active states.
 */
TEST(mission_control_package, heartbeat_callback_test) {
    rclcpp::NodeOptions default_options;
    default_options.append_parameter_override(
        "MDF_FILE_PATH",
        "../../src/mission_control_package/test/mission_file_reader/"
        "test_assets/mdf_correct.json");

    // NOTE: The correct behavior of this function if a valid heartbeat is
    // received, is already tested in this test as a sidetaks. Therefore, this
    // path is not explicitly tested here (just the failure cases).

    // Test unknown sender id being ignored
    {
        rclcpp::executors::SingleThreadedExecutor executor;

        std::shared_ptr<MissionControl> mission_control_node =
            std::make_shared<MissionControl>(default_options);

        // Deactivate event loop so it doesn't mess with our test
        mission_control_node->event_loop_active = false;

        // Create test node
        std::shared_ptr<rclcpp::Node> test_node =
            std::make_shared<rclcpp::Node>("test_node");

        const auto message_publisher =
            test_node->create_publisher<interfaces::msg::Heartbeat>(
                common_lib::topic_names::Heartbeat, 10);

        rclcpp::TimerBase::SharedPtr trigger_timer =
            test_node->create_wall_timer(
                std::chrono::milliseconds(10),
                [test_node, &message_publisher]() {
                    RCLCPP_DEBUG(test_node->get_logger(), "Publishing message");

                    // Create and publish message
                    interfaces::msg::Heartbeat msg;
                    msg.sender_id = test_node->get_name();
                    msg.active = false;
                    msg.time_stamp = test_node->now();
                    msg.tick = 1;

                    message_publisher->publish(msg);
                });

        rclcpp::TimerBase::SharedPtr end_timer =
            mission_control_node->create_wall_timer(
                std::chrono::milliseconds(100),
                [mission_control_node, &message_publisher, &executor]() {
                    RCLCPP_DEBUG(mission_control_node->get_logger(),
                                 "Ending test");

                    executor.cancel();
                });

        executor.add_node(mission_control_node);
        executor.add_node(test_node);

        executor.spin();
    }

    // Test probation period
    {// Test with node being active even though it shouldn't be
     {rclcpp::executors::SingleThreadedExecutor executor;

    std::shared_ptr<MissionControl> mission_control_node =
        std::make_shared<MissionControl>(default_options);

    // Deactivate event loop so it doesn't mess with our test
    mission_control_node->event_loop_active = false;

    // Set last active node id and trigger probation period
    mission_control_node->set_active_node_id(common_lib::node_names::WAYPOINT);
    mission_control_node->clear_active_node_id();

    size_t heartbeat_counter = 0;

    // Create test node
    std::shared_ptr<rclcpp::Node> test_node =
        std::make_shared<rclcpp::Node>(common_lib::node_names::WAYPOINT);

    uint32_t test_node_heartbeat_tick = 0;

    const auto message_publisher =
        test_node->create_publisher<interfaces::msg::Heartbeat>(
            common_lib::topic_names::Heartbeat, 10);

    rclcpp::TimerBase::SharedPtr trigger_timer = test_node->create_wall_timer(
        std::chrono::milliseconds(10),
        [test_node, mission_control_node, &message_publisher,
         &test_node_heartbeat_tick, &heartbeat_counter]() {
            RCLCPP_DEBUG(test_node->get_logger(), "Publishing message");

            heartbeat_counter++;

            // Check that heartbeat was accepted by mission control
            // (except for the first run, as no heartbeat was received
            // before)
            if (heartbeat_counter >= 2) {
                auto &hb = mission_control_node
                               ->node_map[common_lib::node_names::WAYPOINT]
                               .hb_payload;

                ASSERT_TRUE(hb.received);

                hb.received = false;
            }

            // Create and publish message
            interfaces::msg::Heartbeat msg;
            msg.sender_id = test_node->get_name();
            msg.active = true;
            msg.time_stamp = test_node->now();
            msg.tick = ++test_node_heartbeat_tick;

            message_publisher->publish(msg);
        });

    rclcpp::TimerBase::SharedPtr end_timer =
        mission_control_node->create_wall_timer(
            std::chrono::milliseconds(
                MissionControl::probation_period_length_ms - 5),
            [mission_control_node, &message_publisher, &executor]() {
                RCLCPP_DEBUG(mission_control_node->get_logger(),
                             "Ending test before probation period ends");

                executor.cancel();
            });

    executor.add_node(mission_control_node);
    executor.add_node(test_node);

    executor.spin();
}

// Test with node being inactive even though it shouldn't be
{
    rclcpp::executors::SingleThreadedExecutor executor;

    std::shared_ptr<MissionControl> mission_control_node =
        std::make_shared<MissionControl>(default_options);

    // Deactivate event loop so it doesn't mess with our test
    mission_control_node->event_loop_active = false;

    // Set active node id and trigger probation period
    mission_control_node->set_active_node_id(common_lib::node_names::WAYPOINT);

    size_t heartbeat_counter = 0;

    // Create test node
    std::shared_ptr<rclcpp::Node> test_node =
        std::make_shared<rclcpp::Node>(common_lib::node_names::WAYPOINT);

    uint32_t test_node_heartbeat_tick = 0;

    const auto message_publisher =
        test_node->create_publisher<interfaces::msg::Heartbeat>(
            common_lib::topic_names::Heartbeat, 10);

    rclcpp::TimerBase::SharedPtr trigger_timer = test_node->create_wall_timer(
        std::chrono::milliseconds(10),
        [test_node, mission_control_node, &message_publisher,
         &test_node_heartbeat_tick, &heartbeat_counter]() {
            RCLCPP_DEBUG(test_node->get_logger(), "Publishing message");

            heartbeat_counter++;

            // Check that heartbeat was accepted by mission control
            // (except for the first run, as no heartbeat was received
            // before)
            if (heartbeat_counter >= 2) {
                auto &hb = mission_control_node
                               ->node_map[common_lib::node_names::WAYPOINT]
                               .hb_payload;

                ASSERT_TRUE(hb.received);

                hb.received = false;
            }

            // Create and publish message
            interfaces::msg::Heartbeat msg;
            msg.sender_id = test_node->get_name();
            msg.active = false;
            msg.time_stamp = test_node->now();
            msg.tick = ++test_node_heartbeat_tick;

            message_publisher->publish(msg);
        });

    rclcpp::TimerBase::SharedPtr end_timer =
        mission_control_node->create_wall_timer(
            std::chrono::milliseconds(
                MissionControl::probation_period_length_ms - 5),
            [mission_control_node, &message_publisher, &executor]() {
                RCLCPP_DEBUG(mission_control_node->get_logger(),
                             "Ending test before probation period ends");

                executor.cancel();
            });

    executor.add_node(mission_control_node);
    executor.add_node(test_node);

    executor.spin();
}
}

// Test fcc bridge active states
{// Test 'selfcheck' and 'prepare_mission' states, where fcc bridge
 // doesn't need to be active
 {std::unordered_set<MissionControl::MissionState_t> states = {
      MissionControl::selfcheck, MissionControl::prepare_mission};

for (const auto &state : states) {
    rclcpp::executors::SingleThreadedExecutor executor;

    std::shared_ptr<MissionControl> mission_control_node =
        std::make_shared<MissionControl>(default_options);

    // Deactivate event loop so it doesn't mess with our test
    mission_control_node->event_loop_active = false;

    // Set mission state
    mission_control_node->set_mission_state(state);

    // Create test node
    std::shared_ptr<rclcpp::Node> test_node =
        std::make_shared<rclcpp::Node>(common_lib::node_names::FCC_BRIDGE);

    uint32_t test_node_tick = 0;

    const auto message_publisher =
        test_node->create_publisher<interfaces::msg::Heartbeat>(
            common_lib::topic_names::Heartbeat, 10);

    rclcpp::TimerBase::SharedPtr trigger_timer = test_node->create_wall_timer(
        std::chrono::milliseconds(10),
        [test_node, &message_publisher, &test_node_tick]() {
            RCLCPP_DEBUG(test_node->get_logger(), "Publishing message");

            // Create and publish message
            interfaces::msg::Heartbeat msg;
            msg.sender_id = test_node->get_name();
            msg.active = false;
            msg.time_stamp = test_node->now();
            msg.tick = ++test_node_tick;

            message_publisher->publish(msg);
        });

    rclcpp::TimerBase::SharedPtr end_timer =
        mission_control_node->create_wall_timer(
            std::chrono::milliseconds(100),
            [mission_control_node, &message_publisher, &executor]() {
                RCLCPP_DEBUG(mission_control_node->get_logger(), "Ending test");

                executor.cancel();
            });

    executor.add_node(mission_control_node);
    executor.add_node(test_node);

    executor.spin();
}
}

// Test the other states, where fcc bridge
// must be active
{
    std::unordered_set<MissionControl::MissionState_t> states = {
        MissionControl::check_drone_configuration,
        MissionControl::armed,
        MissionControl::takeoff,
        MissionControl::decision_maker,
        MissionControl::fly_to_waypoint,
        MissionControl::detect_marker};

    for (const auto &state : states) {
        rclcpp::executors::SingleThreadedExecutor executor;

        std::shared_ptr<MissionControl> mission_control_node =
            std::make_shared<MissionControl>(default_options);

        // Deactivate event loop so it doesn't mess with our test
        mission_control_node->event_loop_active = false;

        // Set mission state
        mission_control_node->set_mission_state(state);

        // Create test node
        std::shared_ptr<rclcpp::Node> test_node =
            std::make_shared<rclcpp::Node>(common_lib::node_names::FCC_BRIDGE);

        uint32_t test_node_tick = 0;

        const auto message_publisher =
            test_node->create_publisher<interfaces::msg::Heartbeat>(
                common_lib::topic_names::Heartbeat, 10);

        rclcpp::TimerBase::SharedPtr trigger_timer =
            test_node->create_wall_timer(
                std::chrono::milliseconds(10),
                [test_node, &message_publisher, &test_node_tick]() {
                    RCLCPP_DEBUG(test_node->get_logger(), "Publishing message");

                    // Create and publish message
                    interfaces::msg::Heartbeat msg;
                    msg.sender_id = test_node->get_name();
                    msg.active = false;
                    msg.time_stamp = test_node->now();
                    msg.tick = ++test_node_tick;

                    message_publisher->publish(msg);
                });

        executor.add_node(mission_control_node);
        executor.add_node(test_node);

        ASSERT_DEATH({ executor.spin(); }, ".*");
    }
}
}

// Test invalid tick
{// Test always the same tick
 {rclcpp::executors::SingleThreadedExecutor executor;

size_t heartbeat_counter = 0;

std::shared_ptr<MissionControl> mission_control_node =
    std::make_shared<MissionControl>(default_options);

// Deactivate event loop so it doesn't mess with our test
mission_control_node->event_loop_active = false;

// Create test node
std::shared_ptr<rclcpp::Node> test_node =
    std::make_shared<rclcpp::Node>(common_lib::node_names::WAYPOINT);

const auto message_publisher =
    test_node->create_publisher<interfaces::msg::Heartbeat>(
        common_lib::topic_names::Heartbeat, 10);

rclcpp::TimerBase::SharedPtr trigger_timer = test_node->create_wall_timer(
    std::chrono::milliseconds(10),
    [test_node, &message_publisher, &heartbeat_counter,
     mission_control_node]() {
        RCLCPP_DEBUG(test_node->get_logger(), "Publishing message");

        heartbeat_counter++;

        if (heartbeat_counter == 2) {
            // Check if this is the second heartbeat message, as the
            // first one will actually go trough and result in the
            // received flag being true, which is therefore reset to
            // false
            mission_control_node->node_map[common_lib::node_names::WAYPOINT]
                .hb_payload.received = false;
        }

        // Create and publish message
        interfaces::msg::Heartbeat msg;
        msg.sender_id = test_node->get_name();
        msg.active = false;
        msg.time_stamp = test_node->now();
        msg.tick = 1;

        message_publisher->publish(msg);
    });

rclcpp::TimerBase::SharedPtr end_timer =
    mission_control_node->create_wall_timer(
        std::chrono::milliseconds(100), [mission_control_node, &executor]() {
            RCLCPP_DEBUG(mission_control_node->get_logger(),
                         "Checking mission control state and shutting "
                         "down test");

            const auto &hb =
                mission_control_node->node_map[common_lib::node_names::WAYPOINT]
                    .hb_payload;

            ASSERT_FALSE(hb.received);
            ASSERT_EQ(1u, hb.tick);
            ASSERT_FALSE(hb.active);

            executor.cancel();
        });

executor.add_node(mission_control_node);
executor.add_node(test_node);

executor.spin();
}

// Test decrementing tick
{
    rclcpp::executors::SingleThreadedExecutor executor;

    size_t heartbeat_counter = 0;

    std::shared_ptr<MissionControl> mission_control_node =
        std::make_shared<MissionControl>(default_options);

    // Deactivate event loop so it doesn't mess with our test
    mission_control_node->event_loop_active = false;

    // Create test node
    std::shared_ptr<rclcpp::Node> test_node =
        std::make_shared<rclcpp::Node>(common_lib::node_names::WAYPOINT);

    uint32_t test_node_heartbeat_tick = UINT32_MAX;

    const auto message_publisher =
        test_node->create_publisher<interfaces::msg::Heartbeat>(
            common_lib::topic_names::Heartbeat, 10);

    rclcpp::TimerBase::SharedPtr trigger_timer = test_node->create_wall_timer(
        std::chrono::milliseconds(10),
        [test_node, &message_publisher, &test_node_heartbeat_tick,
         &heartbeat_counter, mission_control_node]() {
            RCLCPP_DEBUG(test_node->get_logger(), "Publishing message");

            heartbeat_counter++;

            if (heartbeat_counter == 2) {
                // Check if this is the second heartbeat message, as the
                // first one will actually go trough and result in the
                // received flag being true, which is therefore reset to
                // false
                mission_control_node->node_map[common_lib::node_names::WAYPOINT]
                    .hb_payload.received = false;
            }

            // Create and publish message
            interfaces::msg::Heartbeat msg;
            msg.sender_id = test_node->get_name();
            msg.active = false;
            msg.time_stamp = test_node->now();
            msg.tick = test_node_heartbeat_tick--;

            message_publisher->publish(msg);
        });

    rclcpp::TimerBase::SharedPtr end_timer =
        mission_control_node->create_wall_timer(
            std::chrono::milliseconds(100),
            [mission_control_node, &executor]() {
                RCLCPP_DEBUG(mission_control_node->get_logger(),
                             "Checking mission control state and shutting "
                             "down test");

                const auto &hb =
                    mission_control_node
                        ->node_map[common_lib::node_names::WAYPOINT]
                        .hb_payload;

                ASSERT_FALSE(hb.received);
                ASSERT_EQ(UINT32_MAX, hb.tick);
                ASSERT_FALSE(hb.active);

                executor.cancel();
            });

    executor.add_node(mission_control_node);
    executor.add_node(test_node);

    executor.spin();
}

// Test tick overflow
{
    rclcpp::executors::SingleThreadedExecutor executor;

    size_t heartbeat_counter = 0;

    std::shared_ptr<MissionControl> mission_control_node =
        std::make_shared<MissionControl>(default_options);

    // Deactivate event loop so it doesn't mess with our test
    mission_control_node->event_loop_active = false;

    // Create test node
    std::shared_ptr<rclcpp::Node> test_node =
        std::make_shared<rclcpp::Node>(common_lib::node_names::WAYPOINT);

    uint32_t test_node_heartbeat_tick = UINT32_MAX - 2;

    const auto message_publisher =
        test_node->create_publisher<interfaces::msg::Heartbeat>(
            common_lib::topic_names::Heartbeat, 10);

    rclcpp::TimerBase::SharedPtr trigger_timer = test_node->create_wall_timer(
        std::chrono::milliseconds(100),
        [test_node, &message_publisher, &test_node_heartbeat_tick,
         &heartbeat_counter, mission_control_node]() {
            heartbeat_counter++;

            // Check that heartbeat was accepted by mission control
            // (except for the first run, as no heartbeat was received
            // before)
            if (heartbeat_counter >= 2) {
                auto &hb = mission_control_node
                               ->node_map[common_lib::node_names::WAYPOINT]
                               .hb_payload;

                ASSERT_TRUE(hb.received);

                hb.received = false;
            }

            // Create and publish message
            interfaces::msg::Heartbeat msg;
            msg.sender_id = test_node->get_name();
            msg.active = false;
            msg.time_stamp = test_node->now();
            msg.tick = ++test_node_heartbeat_tick;

            RCLCPP_ERROR(test_node->get_logger(),
                         "Publishing message with tick: %u", msg.tick);
            message_publisher->publish(msg);
        });

    rclcpp::TimerBase::SharedPtr end_timer =
        mission_control_node->create_wall_timer(
            std::chrono::milliseconds(500),
            [mission_control_node, &executor]() {
                RCLCPP_DEBUG(mission_control_node->get_logger(),
                             "Shutting down test");

                executor.cancel();
            });

    executor.add_node(mission_control_node);
    executor.add_node(test_node);

    executor.spin();
}
}

// Test too old timestamp
{
    rclcpp::executors::SingleThreadedExecutor executor;

    std::shared_ptr<MissionControl> mission_control_node =
        std::make_shared<MissionControl>(default_options);

    // Deactivate event loop so it doesn't mess with our test
    mission_control_node->event_loop_active = false;

    // Create test node
    std::shared_ptr<rclcpp::Node> test_node =
        std::make_shared<rclcpp::Node>(common_lib::node_names::WAYPOINT);

    rclcpp::Time old_timestamp = test_node->now();

    const auto message_publisher =
        test_node->create_publisher<interfaces::msg::Heartbeat>(
            common_lib::topic_names::Heartbeat, 10);

    rclcpp::TimerBase::SharedPtr trigger_timer = test_node->create_wall_timer(
        std::chrono::milliseconds(
            MissionControl::heartbeat_max_timestamp_age_ms + 10),
        [test_node, &message_publisher, &old_timestamp]() {
            RCLCPP_DEBUG(test_node->get_logger(),
                         "Publishing heartbeat message");
            // Create and publish message
            interfaces::msg::Heartbeat msg;
            msg.sender_id = test_node->get_name();
            msg.active = false;
            msg.time_stamp = old_timestamp;
            msg.tick = 1;

            message_publisher->publish(msg);
        });

    executor.add_node(mission_control_node);
    executor.add_node(test_node);

    ASSERT_DEATH({ executor.spin(); }, ".*");
}
}

/**
 * @brief Test case for the `heartbeat_timer_callback` function in the
 * `mission_control_package`.
 *
 * This test case verifies the behavior of the `heartbeat_timer_callback`
 * function in the `mission_control_package`. It tests various scenarios related
 * to heartbeat reception and mission state.
 */
TEST(mission_control_package, heartbeat_timer_callback_test) {
    rclcpp::NodeOptions default_options;
    default_options.append_parameter_override(
        "MDF_FILE_PATH",
        "../../src/mission_control_package/test/mission_file_reader/"
        "test_assets/mdf_correct.json");

    // Test valid heartbeats
    {
        MissionControl mission_control_node(default_options);

        ASSERT_FALSE(mission_control_node.heartbeat_received_all);

        for (auto &node : mission_control_node.node_map) {
            node.second.hb_payload.received = true;

            if (node.second.is_fcc_bridge) {
                node.second.hb_payload.active = true;
            }
        }

        mission_control_node.heartbeat_timer_callback();

        ASSERT_TRUE(mission_control_node.heartbeat_received_all);

        // Check that heartbeat received flags are turned off again
        for (auto &node : mission_control_node.node_map) {
            ASSERT_FALSE(node.second.hb_payload.received);
        }
    }

    // Test fcc bridge not active in selfcheck state
    {
        MissionControl mission_control_node(default_options);

        mission_control_node.heartbeat_received_all = true;

        for (auto &node : mission_control_node.node_map) {
            node.second.hb_payload.received = true;
        }

        mission_control_node.heartbeat_timer_callback();

        ASSERT_FALSE(mission_control_node.heartbeat_received_all);

        // Check that heartbeat received flags are turned off again
        for (auto &node : mission_control_node.node_map) {
            ASSERT_FALSE(node.second.hb_payload.received);
        }
    }

    // Test fcc bridge not active in takeoff state
    {
        MissionControl mission_control_node(default_options);
        mission_control_node.set_mission_state(MissionControl::takeoff);

        mission_control_node.heartbeat_received_all = true;

        for (auto &node : mission_control_node.node_map) {
            node.second.hb_payload.received = true;
        }

        ASSERT_DEATH({ mission_control_node.heartbeat_timer_callback(); },
                     ".*");
    }

    // Test waypoint node heartbeat not received while in selfcheck state
    {
        MissionControl mission_control_node(default_options);

        mission_control_node.heartbeat_received_all = true;

        for (auto &node : mission_control_node.node_map) {
            node.second.hb_payload.received = true;

            if (node.second.is_fcc_bridge) {
                node.second.hb_payload.active = true;
            }
        }

        mission_control_node.node_map[common_lib::node_names::WAYPOINT]
            .hb_payload.received = false;

        mission_control_node.heartbeat_timer_callback();

        ASSERT_FALSE(mission_control_node.heartbeat_received_all);

        // Check that heartbeat received flags are turned off again
        for (auto &node : mission_control_node.node_map) {
            ASSERT_FALSE(node.second.hb_payload.received);
        }
    }

    // Test waypoint node heartbeat not received while in takeoff state
    {
        MissionControl mission_control_node(default_options);
        mission_control_node.set_mission_state(MissionControl::takeoff);

        mission_control_node.heartbeat_received_all = true;

        for (auto &node : mission_control_node.node_map) {
            node.second.hb_payload.received = true;

            if (node.second.is_fcc_bridge) {
                node.second.hb_payload.active = true;
            }
        }

        mission_control_node.node_map[common_lib::node_names::WAYPOINT]
            .hb_payload.received = false;

        ASSERT_DEATH({ mission_control_node.heartbeat_timer_callback(); },
                     ".*");
    }

    // Test all heartbeats not received while in selfcheck state
    {
        MissionControl mission_control_node(default_options);
        mission_control_node.heartbeat_received_all = true;

        mission_control_node.heartbeat_timer_callback();

        ASSERT_FALSE(mission_control_node.heartbeat_received_all);
    }

    // Test all heartbeats not received while in takeoff state
    {
        MissionControl mission_control_node(default_options);
        mission_control_node.set_mission_state(MissionControl::takeoff);

        ASSERT_DEATH({ mission_control_node.heartbeat_timer_callback(); },
                     ".*");
    }
}

/**
 * @brief Test case for the `mission_control_package` position callback.
 *
 * This test case verifies the behavior of the position callback in the
 * `mission_control_package`. It tests the handling of valid position messages,
 * position messages with too old timestamps while not being 'active', and
 * position messages with too old timestamps while being 'active'.
 */
TEST(mission_control_package, position_callback_test) {
    class OpenMissionControl : public MissionControl {
       public:
        OpenMissionControl(const rclcpp::NodeOptions &options)
            : MissionControl(options) {}

        void open_deactivate() { deactivate(); }

        void open_activate() { activate(); }
    };

    rclcpp::NodeOptions default_options;
    default_options.append_parameter_override(
        "MDF_FILE_PATH",
        "../../src/mission_control_package/test/mission_file_reader/"
        "test_assets/mdf_correct.json");

    // Test valid position message
    {
        rclcpp::executors::SingleThreadedExecutor executor;

        std::shared_ptr<MissionControl> mission_control_node =
            std::make_shared<MissionControl>(default_options);

        // Deactivate event loop so it doesn't mess with our test
        mission_control_node->event_loop_active = false;

        // Create test node
        std::shared_ptr<rclcpp::Node> test_node =
            std::make_shared<rclcpp::Node>("test_node");

        const auto message_publisher =
            test_node->create_publisher<interfaces::msg::GPSPosition>(
                common_lib::topic_names::GPSPosition, 10);

        rclcpp::TimerBase::SharedPtr trigger_timer =
            test_node->create_wall_timer(
                std::chrono::milliseconds(10),
                [test_node, &message_publisher]() {
                    RCLCPP_DEBUG(test_node->get_logger(), "Publishing message");

                    // Create and publish message
                    interfaces::msg::GPSPosition msg;
                    msg.sender_id = test_node->get_name();
                    msg.time_stamp = test_node->now();
                    msg.latitude_deg = 1.234;
                    msg.longitude_deg = 4.321;
                    msg.relative_altitude_m = 10.1;
                    msg.fix_type = interfaces::msg::GPSPosition::FIX_3D;

                    message_publisher->publish(msg);
                });

        rclcpp::TimerBase::SharedPtr end_timer =
            mission_control_node->create_wall_timer(
                std::chrono::milliseconds(60),
                [mission_control_node, &message_publisher, &executor]() {
                    RCLCPP_DEBUG(mission_control_node->get_logger(),
                                 "Ending test");

                    const Position &pos =
                        mission_control_node->current_position;

                    ASSERT_EQ(1.234, pos.coordinate_lat);
                    ASSERT_EQ(4.321, pos.coordinate_lon);
                    ASSERT_EQ(1010, pos.height_cm);

                    executor.cancel();
                });

        executor.add_node(mission_control_node);
        executor.add_node(test_node);

        executor.spin();
    }

    // Test position message with too old timestamp while not being 'active'
    {
        rclcpp::executors::SingleThreadedExecutor executor;

        std::shared_ptr<OpenMissionControl> mission_control_node =
            std::make_shared<OpenMissionControl>(default_options);

        // Deactivate event loop so it doesn't mess with our test
        mission_control_node->event_loop_active = false;

        // Deactivate Mission Control
        mission_control_node->open_deactivate();

        // Create test node
        std::shared_ptr<rclcpp::Node> test_node =
            std::make_shared<rclcpp::Node>("test_node");

        const auto message_publisher =
            test_node->create_publisher<interfaces::msg::GPSPosition>(
                common_lib::topic_names::GPSPosition, 10);

        rclcpp::TimerBase::SharedPtr trigger_timer =
            test_node->create_wall_timer(
                std::chrono::milliseconds(10),
                [test_node, &message_publisher]() {
                    RCLCPP_DEBUG(test_node->get_logger(), "Publishing message");

                    // Create and publish message
                    interfaces::msg::GPSPosition msg;
                    msg.sender_id = test_node->get_name();
                    msg.latitude_deg = 1.234;
                    msg.longitude_deg = 4.321;
                    msg.relative_altitude_m = 10.1;
                    msg.fix_type = interfaces::msg::GPSPosition::FIX_3D;

                    message_publisher->publish(msg);
                });

        rclcpp::TimerBase::SharedPtr end_timer =
            mission_control_node->create_wall_timer(
                std::chrono::milliseconds(60),
                [mission_control_node, &message_publisher, &executor]() {
                    RCLCPP_DEBUG(mission_control_node->get_logger(),
                                 "Ending test");

                    const Position &pos =
                        mission_control_node->current_position;

                    ASSERT_EQ(1.234, pos.coordinate_lat);
                    ASSERT_EQ(4.321, pos.coordinate_lon);
                    ASSERT_EQ(1010, pos.height_cm);

                    executor.cancel();
                });

        executor.add_node(mission_control_node);
        executor.add_node(test_node);

        executor.spin();
    }

    // Test position message with too old timestamp while being 'active'
    {
        rclcpp::executors::SingleThreadedExecutor executor;

        std::shared_ptr<OpenMissionControl> mission_control_node =
            std::make_shared<OpenMissionControl>(default_options);

        // Deactivate event loop so it doesn't mess with our test
        mission_control_node->event_loop_active = false;

        // Deactivate Mission Control
        mission_control_node->open_activate();

        // Create test node
        std::shared_ptr<rclcpp::Node> test_node =
            std::make_shared<rclcpp::Node>("test_node");

        const auto message_publisher =
            test_node->create_publisher<interfaces::msg::GPSPosition>(
                common_lib::topic_names::GPSPosition, 10);

        rclcpp::TimerBase::SharedPtr trigger_timer =
            test_node->create_wall_timer(
                std::chrono::milliseconds(10),
                [test_node, &message_publisher]() {
                    RCLCPP_DEBUG(test_node->get_logger(), "Publishing message");

                    // Create and publish message
                    interfaces::msg::GPSPosition msg;
                    msg.sender_id = test_node->get_name();
                    msg.latitude_deg = 1.234;
                    msg.longitude_deg = 4.321;
                    msg.relative_altitude_m = 10.1;
                    msg.fix_type = interfaces::msg::GPSPosition::FIX_3D;

                    message_publisher->publish(msg);
                });

        executor.add_node(mission_control_node);
        executor.add_node(test_node);

        ASSERT_DEATH({ executor.spin(); }, ".*");
    }
}

/**
 * @brief Test case for the mission_progress_callback_test.
 *
 * This test case verifies the behavior of the mission_progress_callback_test
 * function. It tests the mission progress message with a valid timestamp and an
 * invalid timestamp. The test creates a mission control node and a test node,
 * and publishes mission progress messages. It checks if the mission progress
 * counter is updated correctly and asserts that the mission progress is
 * increasing.
 */
TEST(mission_control_package, mission_progress_callback_test) {
    rclcpp::NodeOptions default_options;
    default_options.append_parameter_override(
        "MDF_FILE_PATH",
        "../../src/mission_control_package/test/mission_file_reader/"
        "test_assets/mdf_correct.json");

    // Test valid mission progress message
    {
        rclcpp::executors::SingleThreadedExecutor executor;

        std::shared_ptr<MissionControl> mission_control_node =
            std::make_shared<MissionControl>(default_options);

        // Deactivate event loop so it doesn't mess with our test
        mission_control_node->event_loop_active = false;

        // Explicitly set mission progress to 0
        mission_control_node->mission_progress = 0.0;

        float mission_progress_counter = 0.0;
        float last_mission_progress_counter = 0.0;

        // Create test node
        std::shared_ptr<rclcpp::Node> test_node =
            std::make_shared<rclcpp::Node>("test_node");

        const auto message_publisher =
            test_node->create_publisher<interfaces::msg::MissionProgress>(
                common_lib::topic_names::MissionProgress, 10);

        rclcpp::TimerBase::SharedPtr trigger_timer =
            test_node->create_wall_timer(
                std::chrono::milliseconds(50),
                [test_node, &message_publisher, &mission_progress_counter]() {
                    RCLCPP_DEBUG(test_node->get_logger(), "Publishing message");

                    if (mission_progress_counter < 1.0) {
                        mission_progress_counter += 0.2;
                    }

                    // Create and publish message
                    interfaces::msg::MissionProgress msg;
                    msg.sender_id = test_node->get_name();
                    msg.time_stamp = test_node->now();
                    msg.progress = mission_progress_counter;

                    message_publisher->publish(msg);
                });

        rclcpp::TimerBase::SharedPtr check_timer =
            mission_control_node->create_wall_timer(
                std::chrono::milliseconds(10),
                [mission_control_node, &last_mission_progress_counter]() {
                    RCLCPP_DEBUG(mission_control_node->get_logger(),
                                 "Checking progress counter");

                    ASSERT_GE(mission_control_node->mission_progress,
                              last_mission_progress_counter);

                    last_mission_progress_counter =
                        mission_control_node->mission_progress;
                });

        rclcpp::TimerBase::SharedPtr end_timer =
            mission_control_node->create_wall_timer(
                std::chrono::milliseconds(300),
                [mission_control_node, &executor]() {
                    RCLCPP_DEBUG(mission_control_node->get_logger(),
                                 "Ending test");

                    executor.cancel();
                });

        executor.add_node(mission_control_node);
        executor.add_node(test_node);

        executor.spin();
    }

    // Test mission progress message with invalid timestamp
    {
        rclcpp::executors::SingleThreadedExecutor executor;

        std::shared_ptr<MissionControl> mission_control_node =
            std::make_shared<MissionControl>(default_options);

        // Deactivate event loop so it doesn't mess with our test
        mission_control_node->event_loop_active = false;

        // Explicitly set mission progress to 0
        mission_control_node->mission_progress = 0.0;

        float mission_progress_counter = 0.0;
        float last_mission_progress_counter = 0.0;

        // Create test node
        std::shared_ptr<rclcpp::Node> test_node =
            std::make_shared<rclcpp::Node>("test_node");

        const auto message_publisher =
            test_node->create_publisher<interfaces::msg::MissionProgress>(
                common_lib::topic_names::MissionProgress, 10);

        rclcpp::TimerBase::SharedPtr trigger_timer =
            test_node->create_wall_timer(
                std::chrono::milliseconds(50),
                [test_node, &message_publisher, &mission_progress_counter]() {
                    RCLCPP_DEBUG(test_node->get_logger(), "Publishing message");

                    if (mission_progress_counter < 1.0) {
                        mission_progress_counter += 0.2;
                    }

                    // Create and publish message
                    interfaces::msg::MissionProgress msg;
                    msg.sender_id = test_node->get_name();
                    msg.progress = mission_progress_counter;

                    message_publisher->publish(msg);
                });

        rclcpp::TimerBase::SharedPtr check_timer =
            mission_control_node->create_wall_timer(
                std::chrono::milliseconds(10),
                [mission_control_node, &last_mission_progress_counter]() {
                    RCLCPP_DEBUG(mission_control_node->get_logger(),
                                 "Checking progress counter");

                    ASSERT_GE(mission_control_node->mission_progress,
                              last_mission_progress_counter);

                    last_mission_progress_counter =
                        mission_control_node->mission_progress;
                });

        rclcpp::TimerBase::SharedPtr end_timer =
            mission_control_node->create_wall_timer(
                std::chrono::milliseconds(300),
                [mission_control_node, &executor]() {
                    RCLCPP_DEBUG(mission_control_node->get_logger(),
                                 "Ending test");

                    executor.cancel();
                });

        executor.add_node(mission_control_node);
        executor.add_node(test_node);

        executor.spin();
    }
}

/**
 * @brief Test case for the flight_state_callback_test.
 *
 * This test case verifies the behavior of the flight_state_callback_test
 * function. It tests the handling of valid and invalid flight state messages by
 * the MissionControl class. The test creates a test node that publishes flight
 * state messages and checks if the MissionControl class correctly handles the
 * messages and updates its internal state variables.
 */
TEST(mission_control_package, flight_state_callback_test) {
    rclcpp::NodeOptions default_options;
    default_options.append_parameter_override(
        "MDF_FILE_PATH",
        "../../src/mission_control_package/test/mission_file_reader/"
        "test_assets/mdf_correct.json");

    // Test valid flight state message
    {
        rclcpp::executors::SingleThreadedExecutor executor;

        std::shared_ptr<MissionControl> mission_control_node =
            std::make_shared<MissionControl>(default_options);

        // Deactivate event loop so it doesn't mess with our test
        mission_control_node->event_loop_active = false;

        // Create test node
        std::shared_ptr<rclcpp::Node> test_node =
            std::make_shared<rclcpp::Node>("test_node");

        const auto message_publisher =
            test_node->create_publisher<interfaces::msg::FlightState>(
                common_lib::topic_names::FlightState, 10);

        rclcpp::TimerBase::SharedPtr trigger_timer =
            test_node->create_wall_timer(
                std::chrono::milliseconds(10),
                [test_node, &message_publisher]() {
                    RCLCPP_DEBUG(test_node->get_logger(), "Publishing message");

                    // Create and publish message
                    interfaces::msg::FlightState msg;
                    msg.sender_id = test_node->get_name();
                    msg.time_stamp = test_node->now();

                    interfaces::msg::FlightMode flight_mode_msg;
                    flight_mode_msg.mode = interfaces::msg::FlightMode::HOLD;

                    interfaces::msg::LandedState landed_state_msg;
                    landed_state_msg.state =
                        interfaces::msg::LandedState::ON_GROUND;

                    msg.mode = flight_mode_msg;
                    msg.state = landed_state_msg;

                    msg.armed = false;

                    message_publisher->publish(msg);
                });

        rclcpp::TimerBase::SharedPtr end_timer =
            mission_control_node->create_wall_timer(
                std::chrono::milliseconds(60),
                [mission_control_node, &executor]() {
                    RCLCPP_DEBUG(mission_control_node->get_logger(),
                                 "Ending test");

                    ASSERT_EQ(interfaces::msg::FlightMode::HOLD,
                              mission_control_node->current_flight_mode);

                    ASSERT_EQ(interfaces::msg::LandedState::ON_GROUND,
                              mission_control_node->current_landed_state);

                    executor.cancel();
                });

        executor.add_node(mission_control_node);
        executor.add_node(test_node);

        executor.spin();
    }

    // Test flight state message with invalid timestamp
    {
        rclcpp::executors::SingleThreadedExecutor executor;

        std::shared_ptr<MissionControl> mission_control_node =
            std::make_shared<MissionControl>(default_options);

        // Deactivate event loop so it doesn't mess with our test
        mission_control_node->event_loop_active = false;

        // Create test node
        std::shared_ptr<rclcpp::Node> test_node =
            std::make_shared<rclcpp::Node>("test_node");

        const auto message_publisher =
            test_node->create_publisher<interfaces::msg::FlightState>(
                common_lib::topic_names::FlightState, 10);

        rclcpp::TimerBase::SharedPtr trigger_timer =
            test_node->create_wall_timer(
                std::chrono::milliseconds(10),
                [test_node, &message_publisher]() {
                    RCLCPP_DEBUG(test_node->get_logger(), "Publishing message");

                    // Create and publish message
                    interfaces::msg::FlightState msg;
                    msg.sender_id = test_node->get_name();

                    interfaces::msg::FlightMode flight_mode_msg;
                    flight_mode_msg.mode = interfaces::msg::FlightMode::HOLD;

                    interfaces::msg::LandedState landed_state_msg;
                    landed_state_msg.state =
                        interfaces::msg::LandedState::ON_GROUND;

                    msg.mode = flight_mode_msg;
                    msg.state = landed_state_msg;

                    msg.armed = false;

                    message_publisher->publish(msg);
                });

        rclcpp::TimerBase::SharedPtr end_timer =
            mission_control_node->create_wall_timer(
                std::chrono::milliseconds(60),
                [mission_control_node, &executor]() {
                    RCLCPP_DEBUG(mission_control_node->get_logger(),
                                 "Ending test");

                    ASSERT_EQ(interfaces::msg::FlightMode::UNKNOWN,
                              mission_control_node->current_flight_mode);

                    ASSERT_EQ(interfaces::msg::LandedState::UNKNOWN,
                              mission_control_node->current_landed_state);

                    executor.cancel();
                });

        executor.add_node(mission_control_node);
        executor.add_node(test_node);

        executor.spin();
    }
}

/**
 * @brief Test case for the `mission_control_package` health callback.
 *
 * This test case verifies the behavior of the health callback in the
 * `mission_control_package`. It tests various scenarios by publishing different
 * health messages and checking the resulting drone health status.
 */
TEST(mission_control_package, health_callback_test) {
    rclcpp::NodeOptions default_options;
    default_options.append_parameter_override(
        "MDF_FILE_PATH",
        "../../src/mission_control_package/test/mission_file_reader/"
        "test_assets/mdf_correct.json");

    // Test valid health message with all flags set to true
    {
        rclcpp::executors::SingleThreadedExecutor executor;

        std::shared_ptr<MissionControl> mission_control_node =
            std::make_shared<MissionControl>(default_options);

        // Deactivate event loop so it doesn't mess with our test
        mission_control_node->event_loop_active = false;

        // Check that drone health is not ok on init
        ASSERT_FALSE(mission_control_node->drone_health_ok);

        // Create test node
        std::shared_ptr<rclcpp::Node> test_node =
            std::make_shared<rclcpp::Node>("test_node");

        const auto message_publisher =
            test_node->create_publisher<interfaces::msg::UAVHealth>(
                common_lib::topic_names::UAVHealth, 10);

        rclcpp::TimerBase::SharedPtr trigger_timer =
            test_node->create_wall_timer(
                std::chrono::milliseconds(10),
                [test_node, &message_publisher]() {
                    RCLCPP_DEBUG(test_node->get_logger(), "Publishing message");

                    // Create and publish message
                    interfaces::msg::UAVHealth msg;
                    msg.sender_id = test_node->get_name();
                    msg.time_stamp = test_node->now();
                    msg.is_gyrometer_calibration_ok = true;
                    msg.is_accelerometer_calibration_ok = true;
                    msg.is_magnetometer_calibration_ok = true;
                    msg.is_local_position_ok = true;
                    msg.is_global_position_ok = true;
                    msg.is_home_position_ok = true;
                    msg.is_armable = true;

                    message_publisher->publish(msg);
                });

        rclcpp::TimerBase::SharedPtr end_timer =
            mission_control_node->create_wall_timer(
                std::chrono::milliseconds(60),
                [mission_control_node, &executor]() {
                    RCLCPP_DEBUG(mission_control_node->get_logger(),
                                 "Ending test");

                    ASSERT_TRUE(mission_control_node->drone_health_ok);

                    executor.cancel();
                });

        executor.add_node(mission_control_node);
        executor.add_node(test_node);

        executor.spin();
    }

    // Test health message with invalid timestamp
    {
        rclcpp::executors::SingleThreadedExecutor executor;

        std::shared_ptr<MissionControl> mission_control_node =
            std::make_shared<MissionControl>(default_options);

        // Deactivate event loop so it doesn't mess with our test
        mission_control_node->event_loop_active = false;

        // Check that drone health is not ok on init
        ASSERT_FALSE(mission_control_node->drone_health_ok);

        // Create test node
        std::shared_ptr<rclcpp::Node> test_node =
            std::make_shared<rclcpp::Node>("test_node");

        const auto message_publisher =
            test_node->create_publisher<interfaces::msg::UAVHealth>(
                common_lib::topic_names::UAVHealth, 10);

        rclcpp::TimerBase::SharedPtr trigger_timer =
            test_node->create_wall_timer(
                std::chrono::milliseconds(10),
                [test_node, &message_publisher]() {
                    RCLCPP_DEBUG(test_node->get_logger(), "Publishing message");

                    // Create and publish message
                    interfaces::msg::UAVHealth msg;
                    msg.sender_id = test_node->get_name();
                    msg.is_gyrometer_calibration_ok = true;
                    msg.is_accelerometer_calibration_ok = true;
                    msg.is_magnetometer_calibration_ok = true;
                    msg.is_local_position_ok = true;
                    msg.is_global_position_ok = true;
                    msg.is_home_position_ok = true;
                    msg.is_armable = true;

                    message_publisher->publish(msg);
                });

        rclcpp::TimerBase::SharedPtr end_timer =
            mission_control_node->create_wall_timer(
                std::chrono::milliseconds(60),
                [mission_control_node, &executor]() {
                    RCLCPP_DEBUG(mission_control_node->get_logger(),
                                 "Ending test");

                    ASSERT_FALSE(mission_control_node->drone_health_ok);

                    executor.cancel();
                });

        executor.add_node(mission_control_node);
        executor.add_node(test_node);

        executor.spin();
    }

    // Test health message with one flag set to false and all set to false
    for (size_t unavail_flag = 0; unavail_flag < 8; unavail_flag++) {
        rclcpp::executors::SingleThreadedExecutor executor;

        std::shared_ptr<MissionControl> mission_control_node =
            std::make_shared<MissionControl>(default_options);

        // Deactivate event loop so it doesn't mess with our test
        mission_control_node->event_loop_active = false;

        // Check that drone health is not ok on init
        ASSERT_FALSE(mission_control_node->drone_health_ok);

        // Create test node
        std::shared_ptr<rclcpp::Node> test_node =
            std::make_shared<rclcpp::Node>("test_node");

        const auto message_publisher =
            test_node->create_publisher<interfaces::msg::UAVHealth>(
                common_lib::topic_names::UAVHealth, 10);

        rclcpp::TimerBase::SharedPtr trigger_timer =
            test_node->create_wall_timer(
                std::chrono::milliseconds(10),
                [test_node, &message_publisher, &unavail_flag]() {
                    RCLCPP_DEBUG(test_node->get_logger(), "Publishing message");

                    // Create and publish message
                    interfaces::msg::UAVHealth msg;
                    msg.sender_id = test_node->get_name();
                    msg.time_stamp = test_node->now();
                    msg.is_gyrometer_calibration_ok = true;
                    msg.is_accelerometer_calibration_ok = true;
                    msg.is_magnetometer_calibration_ok = true;
                    msg.is_local_position_ok = true;
                    msg.is_global_position_ok = true;
                    msg.is_home_position_ok = true;
                    msg.is_armable = true;

                    switch (unavail_flag) {
                        case 0:
                            msg.is_gyrometer_calibration_ok = false;
                            break;
                        case 1:
                            msg.is_accelerometer_calibration_ok = false;
                            break;
                        case 2:
                            msg.is_magnetometer_calibration_ok = false;
                            break;
                        case 3:
                            msg.is_local_position_ok = false;
                            break;
                        case 4:
                            msg.is_global_position_ok = false;
                            break;
                        case 5:
                            msg.is_home_position_ok = false;
                            break;
                        case 6:
                            msg.is_armable = false;
                            break;
                        case 7:
                            msg.is_gyrometer_calibration_ok = false;
                            msg.is_accelerometer_calibration_ok = false;
                            msg.is_magnetometer_calibration_ok = false;
                            msg.is_local_position_ok = false;
                            msg.is_global_position_ok = false;
                            msg.is_home_position_ok = false;
                            msg.is_armable = false;
                            break;
                    }

                    message_publisher->publish(msg);
                });

        rclcpp::TimerBase::SharedPtr end_timer =
            mission_control_node->create_wall_timer(
                std::chrono::milliseconds(60),
                [mission_control_node, &executor]() {
                    RCLCPP_DEBUG(mission_control_node->get_logger(),
                                 "Ending test");

                    ASSERT_FALSE(mission_control_node->drone_health_ok);

                    executor.cancel();
                });

        executor.add_node(mission_control_node);
        executor.add_node(test_node);

        executor.spin();
    }
}
