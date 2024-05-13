#include "mission_control.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <cinttypes>
#include <nlohmann/json.hpp>

#include "common_package/topic_names.hpp"
#include "rclcpp/executors.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/node_options.hpp"

// Message includes
#include "interfaces/msg/control.hpp"
#include "interfaces/msg/mission_finished.hpp"

TEST(mission_control_package, constructor_test) {
    // Check constructor with default file
    {
        std::shared_ptr<MissionControl> mission_control = nullptr;
        ASSERT_NO_THROW(mission_control = std::make_shared<MissionControl>());

        ASSERT_EQ(mission_control->mdf_file_path,
                  "src/mission_control_package/assets/mission_test.json");
    }

    // Pass node options with invalid parameter
    {
        rclcpp::NodeOptions options;
        options.append_parameter_override("MDF_FILE_PATH", 234);

        ASSERT_DEATH(
            { MissionControl mission_control = MissionControl(options); },
            ".*");
    }

    // Pass node options with non-existent file path
    {
        const std::string file_path = "xyz.json";

        rclcpp::executors::SingleThreadedExecutor executor;

        rclcpp::NodeOptions options;
        options.append_parameter_override("MDF_FILE_PATH", file_path);

        std::shared_ptr<MissionControl> mission_control =
            std::make_shared<MissionControl>(options);
        ASSERT_EQ(mission_control->mdf_file_path, file_path);

        executor.add_node(mission_control);

        ASSERT_DEATH({ executor.spin(); }, ".*");
    }
}

TEST(mission_control_package, send_control_test) {
    // Check if "active = false" and target is active_node_id
    {
        const std::string target_id = "test";
        const bool active = false;
        const std::string payload = "{}";

        rclcpp::executors::SingleThreadedExecutor executor;

        std::shared_ptr<MissionControl> mission_control_node =
            std::make_shared<MissionControl>();

        // Set some values to check that they were resetted in the process
        mission_control_node->active_node_id = target_id;
        mission_control_node->mission_progress = 0.5;

        rclcpp::Node::SharedPtr test_node =
            std::make_shared<rclcpp::Node>("test");

        rclcpp::Subscription<interfaces::msg::Control>::SharedPtr control_sub =
            test_node->create_subscription<interfaces::msg::Control>(
                common_lib::topic_names::Control, 10,
                [&target_id, active, &payload, &executor,
                 test_node](interfaces::msg::Control::ConstSharedPtr msg) {
                    RCLCPP_DEBUG(test_node->get_logger(),
                                 "Received control message");

                    ASSERT_EQ(msg->target_id, target_id);
                    ASSERT_EQ(msg->active, active);
                    ASSERT_EQ(msg->payload, payload);

                    executor.cancel();
                });

        rclcpp::TimerBase::SharedPtr end_timer =
            mission_control_node->create_wall_timer(
                std::chrono::milliseconds(10),
                [&target_id, active, &payload, mission_control_node]() {
                    RCLCPP_DEBUG(mission_control_node->get_logger(),
                                 "Executing control function");

                    mission_control_node->send_control(target_id, active,
                                                       payload);

                    ASSERT_EQ(mission_control_node->get_active_node_id(), "");
                    ASSERT_EQ(mission_control_node->mission_progress, 0.0);
                });

        executor.add_node(mission_control_node);
        executor.add_node(test_node);

        executor.spin();
    }

    // Check if "active = false" and target isn't active_node_id
    {
        const std::string target_id = "test";
        const bool active = false;
        const std::string payload = "{}";

        rclcpp::executors::SingleThreadedExecutor executor;

        std::shared_ptr<MissionControl> mission_control_node =
            std::make_shared<MissionControl>();

        // Set some values to check that they were resetted in the process
        mission_control_node->active_node_id = "abc";
        mission_control_node->mission_progress = 0.5;

        rclcpp::Node::SharedPtr test_node =
            std::make_shared<rclcpp::Node>("test");

        rclcpp::Subscription<interfaces::msg::Control>::SharedPtr control_sub =
            test_node->create_subscription<interfaces::msg::Control>(
                common_lib::topic_names::Control, 10,
                [&target_id, active, &payload, &executor,
                 test_node](interfaces::msg::Control::ConstSharedPtr msg) {
                    RCLCPP_DEBUG(test_node->get_logger(),
                                 "Received control message");

                    ASSERT_EQ(msg->target_id, target_id);
                    ASSERT_EQ(msg->active, active);
                    ASSERT_EQ(msg->payload, payload);

                    executor.cancel();
                });

        rclcpp::TimerBase::SharedPtr end_timer =
            mission_control_node->create_wall_timer(
                std::chrono::milliseconds(10),
                [&target_id, active, &payload, mission_control_node]() {
                    RCLCPP_DEBUG(mission_control_node->get_logger(),
                                 "Executing control function");

                    mission_control_node->send_control(target_id, active,
                                                       payload);

                    ASSERT_EQ(mission_control_node->get_active_node_id(),
                              "abc");
                    ASSERT_EQ(mission_control_node->mission_progress, 0.0);
                });

        executor.add_node(mission_control_node);
        executor.add_node(test_node);

        executor.spin();
    }

    // Check if "active = true"
    {
        const std::string target_id = "test";
        const bool active = true;
        const std::string payload = "{}";

        rclcpp::executors::SingleThreadedExecutor executor;

        std::shared_ptr<MissionControl> mission_control_node =
            std::make_shared<MissionControl>();

        // Set some values to check that they were resetted in the process
        mission_control_node->active_node_id = "abc";
        mission_control_node->mission_progress = 0.5;

        rclcpp::Node::SharedPtr test_node =
            std::make_shared<rclcpp::Node>("test");

        rclcpp::Subscription<interfaces::msg::Control>::SharedPtr control_sub =
            test_node->create_subscription<interfaces::msg::Control>(
                common_lib::topic_names::Control, 10,
                [&target_id, active, &payload, &executor,
                 test_node](interfaces::msg::Control::ConstSharedPtr msg) {
                    RCLCPP_DEBUG(test_node->get_logger(),
                                 "Received control message");

                    ASSERT_EQ(msg->target_id, target_id);
                    ASSERT_EQ(msg->active, active);
                    ASSERT_EQ(msg->payload, payload);

                    executor.cancel();
                });

        rclcpp::TimerBase::SharedPtr end_timer =
            mission_control_node->create_wall_timer(
                std::chrono::milliseconds(10),
                [&target_id, active, &payload, mission_control_node]() {
                    RCLCPP_DEBUG(mission_control_node->get_logger(),
                                 "Executing control function");

                    mission_control_node->send_control(target_id, active,
                                                       payload);

                    ASSERT_EQ(mission_control_node->get_active_node_id(),
                              target_id);
                    ASSERT_EQ(mission_control_node->mission_progress, 0.0);
                });

        executor.add_node(mission_control_node);
        executor.add_node(test_node);

        executor.spin();
    }
}

TEST(mission_control_package, send_control_json_test) {
    // Check with valid json
    {
        const std::string target_id = "test";
        const bool active = false;
        const nlohmann::json payload_json = {{"abc", 123}};
        const std::string payload_string = "{\"abc\":123}";

        rclcpp::executors::SingleThreadedExecutor executor;

        std::shared_ptr<MissionControl> mission_control_node =
            std::make_shared<MissionControl>();

        rclcpp::Node::SharedPtr test_node =
            std::make_shared<rclcpp::Node>("test");

        rclcpp::Subscription<interfaces::msg::Control>::SharedPtr control_sub =
            test_node->create_subscription<interfaces::msg::Control>(
                common_lib::topic_names::Control, 10,
                [&target_id, active, &payload_string, &executor,
                 test_node](interfaces::msg::Control::ConstSharedPtr msg) {
                    RCLCPP_DEBUG(test_node->get_logger(),
                                 "Received control message");

                    ASSERT_EQ(msg->target_id, target_id);
                    ASSERT_EQ(msg->active, active);
                    ASSERT_EQ(msg->payload, payload_string);

                    executor.cancel();
                });

        rclcpp::TimerBase::SharedPtr end_timer =
            mission_control_node->create_wall_timer(
                std::chrono::milliseconds(10),
                [&target_id, active, &payload_json, mission_control_node]() {
                    RCLCPP_DEBUG(mission_control_node->get_logger(),
                                 "Executing control function");

                    mission_control_node->send_control_json(target_id, active,
                                                            payload_json);
                });

        executor.add_node(mission_control_node);
        executor.add_node(test_node);

        executor.spin();
    }

    // Check with invalid json
    {
        const std::string target_id = "test";
        const bool active = false;
        const nlohmann::json payload_json = "ä\xA9ü";

        rclcpp::executors::SingleThreadedExecutor executor;

        std::shared_ptr<MissionControl> mission_control_node =
            std::make_shared<MissionControl>();

        rclcpp::TimerBase::SharedPtr end_timer =
            mission_control_node->create_wall_timer(
                std::chrono::milliseconds(10),
                [&target_id, active, &payload_json, mission_control_node]() {
                    RCLCPP_DEBUG(mission_control_node->get_logger(),
                                 "Executing control function");

                    mission_control_node->send_control_json(target_id, active,
                                                            payload_json);

                    ASSERT_EQ(mission_control_node->get_active_node_id(), "");
                    ASSERT_EQ(mission_control_node->mission_progress, 0.0);
                });

        executor.add_node(mission_control_node);

        ASSERT_DEATH({ executor.spin(); }, ".*");
    }
}

TEST(mission_control_package, mission_abort_test) {
    // Without any active node
    {
        rclcpp::executors::SingleThreadedExecutor executor;

        std::shared_ptr<MissionControl> mission_control_node =
            std::make_shared<MissionControl>();

        rclcpp::TimerBase::SharedPtr end_timer =
            mission_control_node->create_wall_timer(
                std::chrono::milliseconds(10), [mission_control_node]() {
                    RCLCPP_DEBUG(mission_control_node->get_logger(),
                                 "Executing mission_abort function");

                    mission_control_node->mission_abort("Planned Failure");
                });

        executor.add_node(mission_control_node);

        ASSERT_DEATH({ executor.spin(); }, ".*");
    }

    // With an active node
    {
        rclcpp::executors::SingleThreadedExecutor executor;

        std::shared_ptr<MissionControl> mission_control_node =
            std::make_shared<MissionControl>();

        mission_control_node->active_node_id = "abc";

        rclcpp::TimerBase::SharedPtr end_timer =
            mission_control_node->create_wall_timer(
                std::chrono::milliseconds(10), [mission_control_node]() {
                    RCLCPP_DEBUG(mission_control_node->get_logger(),
                                 "Executing mission_abort function");

                    mission_control_node->mission_abort("Planned Failure");
                });

        executor.add_node(mission_control_node);

        ASSERT_DEATH({ executor.spin(); }, ".*");
    }
}

TEST(mission_control_package, mission_finished_test) {
    // Without any active node
    {
        rclcpp::executors::SingleThreadedExecutor executor;

        std::shared_ptr<MissionControl> mission_control_node =
            std::make_shared<MissionControl>();

        rclcpp::TimerBase::SharedPtr end_timer =
            mission_control_node->create_wall_timer(
                std::chrono::milliseconds(10), [mission_control_node]() {
                    RCLCPP_DEBUG(mission_control_node->get_logger(),
                                 "Executing mission_finished function");

                    mission_control_node->mission_finished();
                });

        executor.add_node(mission_control_node);

        ASSERT_DEATH({ executor.spin(); }, ".*");
    }

    // With an active node
    {
        rclcpp::executors::SingleThreadedExecutor executor;

        std::shared_ptr<MissionControl> mission_control_node =
            std::make_shared<MissionControl>();

        mission_control_node->active_node_id = "abc";

        rclcpp::TimerBase::SharedPtr end_timer =
            mission_control_node->create_wall_timer(
                std::chrono::milliseconds(10), [mission_control_node]() {
                    RCLCPP_DEBUG(mission_control_node->get_logger(),
                                 "Executing mission_finished function");

                    mission_control_node->mission_finished();
                });

        executor.add_node(mission_control_node);

        ASSERT_DEATH({ executor.spin(); }, ".*");
    }
}
