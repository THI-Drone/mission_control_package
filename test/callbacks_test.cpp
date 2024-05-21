#include <gtest/gtest.h>

#include <chrono>
#include <cinttypes>
#include <nlohmann/json.hpp>
#include <optional>

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
#include "interfaces/msg/job_finished.hpp"

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
