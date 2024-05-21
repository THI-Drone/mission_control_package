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

/**
 * @brief Test case for the mission_control_package.
 *
 * This test case checks the behavior of the constructor in the MissionControl
 * class. It verifies the following scenarios:
 * 1. Constructor with default file: It checks if the constructor correctly
 * initializes the MissionControl object with the default file path.
 * 2. Constructor with invalid parameter: It checks if the constructor throws an
 * exception when an invalid parameter is passed.
 * 3. Constructor with non-existent file path: It checks if the constructor
 * correctly initializes the MissionControl object with a non-existent file path
 * and if the executor throws an exception when the node is added and spun.
 */
TEST(mission_control_package, constructor_test) {
    // Check constructor with default file
    {
        std::shared_ptr<MissionControl> mission_control = nullptr;

        rclcpp::NodeOptions options;
        options.append_parameter_override("MDF_FILE_PATH", "DEFAULT");

        ASSERT_NO_THROW(mission_control =
                            std::make_shared<MissionControl>(options));

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

/**
 * @brief Test case for the `mission_control_package` module's
 * `send_control_test` function.
 *
 * This test case verifies the behavior of the `send_control_test` function in
 * the `mission_control_package` module. It checks different scenarios where the
 * `active` flag is set to true or false, and the target node ID matches or
 * doesn't match the active node ID.
 */
TEST(mission_control_package, send_control_test) {
    rclcpp::NodeOptions default_options;
    default_options.append_parameter_override(
        "MDF_FILE_PATH",
        "../../src/mission_control_package/test/mission_file_reader/"
        "test_assets/mdf_correct.json");

    // Check if "active = false" and target is active_node_id
    {
        const std::string target_id = "test";
        const bool active = false;
        const std::string payload = "{}";

        rclcpp::executors::SingleThreadedExecutor executor;

        std::shared_ptr<MissionControl> mission_control_node =
            std::make_shared<MissionControl>(default_options);

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
            std::make_shared<MissionControl>(default_options);

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
            std::make_shared<MissionControl>(default_options);

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

/**
 * @brief Test case for the `mission_control_package` module's
 * `send_control_json_test` function.
 *
 * This test case checks the behavior of the `send_control_json` function in the
 * `MissionControl` class. It verifies the handling of both valid and invalid
 * JSON payloads.
 */
TEST(mission_control_package, send_control_json_test) {
    rclcpp::NodeOptions default_options;
    default_options.append_parameter_override(
        "MDF_FILE_PATH",
        "../../src/mission_control_package/test/mission_file_reader/"
        "test_assets/mdf_correct.json");

    // Check with valid json
    {
        const std::string target_id = "test";
        const bool active = false;
        const nlohmann::json payload_json = {{"abc", 123}};
        const std::string payload_string = "{\"abc\":123}";

        rclcpp::executors::SingleThreadedExecutor executor;

        std::shared_ptr<MissionControl> mission_control_node =
            std::make_shared<MissionControl>(default_options);

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
            std::make_shared<MissionControl>(default_options);

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

/**
 * @brief Test case for the mission_abort_test.
 *
 * This test case verifies the behavior of the mission_abort function in
 * different scenarios. It tests the behavior when there is no active node, when
 * there is an active node, when there is a last active node, and when there is
 * both an active node and a last active node. The test case creates a
 * `MissionControl` node with the given `default_options` and sets the necessary
 * parameters. It then creates a timer that calls the `mission_abort` function
 * with a specific message. The test case uses a `SingleThreadedExecutor` to
 * spin the node and verifies that the node terminates with an expected death
 * message.
 */
TEST(mission_control_package, mission_abort_test) {
    rclcpp::NodeOptions default_options;
    default_options.append_parameter_override(
        "MDF_FILE_PATH",
        "../../src/mission_control_package/test/mission_file_reader/"
        "test_assets/mdf_correct.json");

    // Without any active node
    {
        rclcpp::executors::SingleThreadedExecutor executor;

        std::shared_ptr<MissionControl> mission_control_node =
            std::make_shared<MissionControl>(default_options);

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
            std::make_shared<MissionControl>(default_options);

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

    // With a last active node
    {
        rclcpp::executors::SingleThreadedExecutor executor;

        std::shared_ptr<MissionControl> mission_control_node =
            std::make_shared<MissionControl>(default_options);

        mission_control_node->active_node_id = "";
        mission_control_node->last_active_node_id = "abc";

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

    // With an active node and a last active node
    {
        rclcpp::executors::SingleThreadedExecutor executor;

        std::shared_ptr<MissionControl> mission_control_node =
            std::make_shared<MissionControl>(default_options);

        mission_control_node->active_node_id = "abc";
        mission_control_node->last_active_node_id = "def";

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

/**
 * @brief Test case for the mission_finished_test.
 *
 * This test case verifies the behavior of the mission_finished_test function in
 * the mission_control_package. It tests two scenarios: one without any active
 * node and one with an active node. The test uses the ASSERT_DEATH macro to
 * check if the expected behavior is triggered.
 */
TEST(mission_control_package, mission_finished_test) {
    rclcpp::NodeOptions default_options;
    default_options.append_parameter_override(
        "MDF_FILE_PATH",
        "../../src/mission_control_package/test/mission_file_reader/"
        "test_assets/mdf_correct.json");

    // Without any active node
    {
        rclcpp::executors::SingleThreadedExecutor executor;

        std::shared_ptr<MissionControl> mission_control_node =
            std::make_shared<MissionControl>(default_options);

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
            std::make_shared<MissionControl>(default_options);

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

/**
 * @brief Test case for the mission_control_package.
 *
 * This test case verifies the behavior of the event loop in the mission control
 * package. It creates a custom mission control node and sets different mission
 * states to test the execution of various functions within the event loop.
 */
TEST(mission_control_package, event_loop_test) {
    rclcpp::NodeOptions default_options;
    default_options.append_parameter_override(
        "MDF_FILE_PATH",
        "../../src/mission_control_package/test/"
        "mission_file_reader/test_assets/mdf_correct.json");

    class CustomMissionControl : public MissionControl {
       public:
        size_t mode_prepare_mission_counter = 0;
        size_t mode_self_check_counter = 0;
        size_t mode_check_drone_configuration_counter = 0;
        size_t initiate_takeoff_counter = 0;
        size_t mode_decision_maker_counter = 0;
        size_t mode_fly_to_waypoint_counter = 0;
        size_t mode_detect_marker_counter = 0;

       public:
        CustomMissionControl(
            const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
            : MissionControl(options) {}

        void mode_prepare_mission() override {
            RCLCPP_DEBUG(this->get_logger(), "Custom function '%s' called",
                         __func__);

            mode_prepare_mission_counter++;
        }

        void mode_self_check() override {
            RCLCPP_DEBUG(this->get_logger(), "Custom function '%s' called",
                         __func__);

            mode_self_check_counter++;
        }

        void mode_check_drone_configuration() override {
            RCLCPP_DEBUG(this->get_logger(), "Custom function '%s' called",
                         __func__);

            mode_check_drone_configuration_counter++;
        }

        void initiate_takeoff() override {
            RCLCPP_DEBUG(this->get_logger(), "Custom function '%s' called",
                         __func__);

            initiate_takeoff_counter++;
        }

        void mode_decision_maker() override {
            RCLCPP_DEBUG(this->get_logger(), "Custom function '%s' called",
                         __func__);

            mode_decision_maker_counter++;
        }

        void mode_fly_to_waypoint() override {
            RCLCPP_DEBUG(this->get_logger(), "Custom function '%s' called",
                         __func__);

            mode_fly_to_waypoint_counter++;
        }

        void mode_detect_marker() override {
            RCLCPP_DEBUG(this->get_logger(), "Custom function '%s' called",
                         __func__);

            mode_detect_marker_counter++;
        }
    };

    // Check state 'prepare_mission'
    {
        rclcpp::executors::SingleThreadedExecutor executor;

        std::shared_ptr<CustomMissionControl> mission_control_node =
            std::make_shared<CustomMissionControl>(default_options);

        mission_control_node->set_mission_state(
            MissionControl::prepare_mission);

        const uint32_t timer_executions = 5;

        rclcpp::TimerBase::SharedPtr end_timer =
            mission_control_node->create_wall_timer(
                std::chrono::milliseconds(
                    mission_control_node->event_loop_time_delta_ms *
                    timer_executions),
                [mission_control_node, &executor, timer_executions]() {
                    RCLCPP_DEBUG(mission_control_node->get_logger(),
                                 "Ending event loop test");

                    // Check that counts are correct
                    ASSERT_EQ(
                        timer_executions,
                        mission_control_node->mode_prepare_mission_counter);

                    ASSERT_EQ(0, mission_control_node->mode_self_check_counter);
                    ASSERT_EQ(0, mission_control_node
                                     ->mode_check_drone_configuration_counter);
                    ASSERT_EQ(0,
                              mission_control_node->initiate_takeoff_counter);
                    ASSERT_EQ(
                        0, mission_control_node->mode_decision_maker_counter);
                    ASSERT_EQ(
                        0, mission_control_node->mode_fly_to_waypoint_counter);
                    ASSERT_EQ(0,
                              mission_control_node->mode_detect_marker_counter);

                    executor.cancel();
                });

        executor.add_node(mission_control_node);
        executor.spin();
    }

    // Check state 'selfcheck'
    {
        rclcpp::executors::SingleThreadedExecutor executor;

        std::shared_ptr<CustomMissionControl> mission_control_node =
            std::make_shared<CustomMissionControl>(default_options);

        mission_control_node->set_mission_state(MissionControl::selfcheck);

        const uint32_t timer_executions = 5;

        rclcpp::TimerBase::SharedPtr end_timer =
            mission_control_node->create_wall_timer(
                std::chrono::milliseconds(
                    mission_control_node->event_loop_time_delta_ms *
                    timer_executions),
                [mission_control_node, &executor, timer_executions]() {
                    RCLCPP_DEBUG(mission_control_node->get_logger(),
                                 "Ending event loop test");

                    // Check that counts are correct
                    ASSERT_EQ(timer_executions,
                              mission_control_node->mode_self_check_counter);

                    ASSERT_EQ(
                        0, mission_control_node->mode_prepare_mission_counter);
                    ASSERT_EQ(0, mission_control_node
                                     ->mode_check_drone_configuration_counter);
                    ASSERT_EQ(0,
                              mission_control_node->initiate_takeoff_counter);
                    ASSERT_EQ(
                        0, mission_control_node->mode_decision_maker_counter);
                    ASSERT_EQ(
                        0, mission_control_node->mode_fly_to_waypoint_counter);
                    ASSERT_EQ(0,
                              mission_control_node->mode_detect_marker_counter);

                    executor.cancel();
                });

        executor.add_node(mission_control_node);
        executor.spin();
    }

    // Check state 'check_drone_configuration'
    {
        rclcpp::executors::SingleThreadedExecutor executor;

        std::shared_ptr<CustomMissionControl> mission_control_node =
            std::make_shared<CustomMissionControl>(default_options);

        mission_control_node->set_mission_state(
            MissionControl::check_drone_configuration);

        const uint32_t timer_executions = 5;

        rclcpp::TimerBase::SharedPtr end_timer =
            mission_control_node->create_wall_timer(
                std::chrono::milliseconds(
                    mission_control_node->event_loop_time_delta_ms *
                    timer_executions),
                [mission_control_node, &executor, timer_executions]() {
                    RCLCPP_DEBUG(mission_control_node->get_logger(),
                                 "Ending event loop test");

                    // Check that counts are correct
                    ASSERT_EQ(timer_executions,
                              mission_control_node
                                  ->mode_check_drone_configuration_counter);

                    ASSERT_EQ(
                        0, mission_control_node->mode_prepare_mission_counter);
                    ASSERT_EQ(0, mission_control_node->mode_self_check_counter);
                    ASSERT_EQ(0,
                              mission_control_node->initiate_takeoff_counter);
                    ASSERT_EQ(
                        0, mission_control_node->mode_decision_maker_counter);
                    ASSERT_EQ(
                        0, mission_control_node->mode_fly_to_waypoint_counter);
                    ASSERT_EQ(0,
                              mission_control_node->mode_detect_marker_counter);

                    executor.cancel();
                });

        executor.add_node(mission_control_node);
        executor.spin();
    }

    // Check state 'armed'
    {
        rclcpp::executors::SingleThreadedExecutor executor;

        std::shared_ptr<CustomMissionControl> mission_control_node =
            std::make_shared<CustomMissionControl>(default_options);

        mission_control_node->set_mission_state(MissionControl::armed);

        const uint32_t timer_executions = 5;

        rclcpp::TimerBase::SharedPtr end_timer =
            mission_control_node->create_wall_timer(
                std::chrono::milliseconds(
                    mission_control_node->event_loop_time_delta_ms *
                    timer_executions),
                [mission_control_node, &executor, timer_executions]() {
                    RCLCPP_DEBUG(mission_control_node->get_logger(),
                                 "Ending event loop test");

                    // Check that counts are correct
                    ASSERT_EQ(
                        0, mission_control_node->mode_prepare_mission_counter);
                    ASSERT_EQ(0, mission_control_node->mode_self_check_counter);
                    ASSERT_EQ(0, mission_control_node
                                     ->mode_check_drone_configuration_counter);
                    ASSERT_EQ(0,
                              mission_control_node->initiate_takeoff_counter);
                    ASSERT_EQ(
                        0, mission_control_node->mode_decision_maker_counter);
                    ASSERT_EQ(
                        0, mission_control_node->mode_fly_to_waypoint_counter);
                    ASSERT_EQ(0,
                              mission_control_node->mode_detect_marker_counter);

                    executor.cancel();
                });

        executor.add_node(mission_control_node);
        executor.spin();
    }

    // Check state 'takeoff'
    {
        rclcpp::executors::SingleThreadedExecutor executor;

        std::shared_ptr<CustomMissionControl> mission_control_node =
            std::make_shared<CustomMissionControl>(default_options);

        mission_control_node->set_mission_state(MissionControl::takeoff);

        const uint32_t timer_executions = 5;

        rclcpp::TimerBase::SharedPtr end_timer =
            mission_control_node->create_wall_timer(
                std::chrono::milliseconds(
                    mission_control_node->event_loop_time_delta_ms *
                    timer_executions),
                [mission_control_node, &executor, timer_executions]() {
                    RCLCPP_DEBUG(mission_control_node->get_logger(),
                                 "Ending event loop test");

                    // Check that counts are correct
                    ASSERT_EQ(timer_executions,
                              mission_control_node->initiate_takeoff_counter);

                    ASSERT_EQ(
                        0, mission_control_node->mode_prepare_mission_counter);
                    ASSERT_EQ(0, mission_control_node->mode_self_check_counter);
                    ASSERT_EQ(0, mission_control_node
                                     ->mode_check_drone_configuration_counter);
                    ASSERT_EQ(
                        0, mission_control_node->mode_decision_maker_counter);
                    ASSERT_EQ(
                        0, mission_control_node->mode_fly_to_waypoint_counter);
                    ASSERT_EQ(0,
                              mission_control_node->mode_detect_marker_counter);

                    executor.cancel();
                });

        executor.add_node(mission_control_node);
        executor.spin();
    }

    // Check state 'decision_maker'
    {
        rclcpp::executors::SingleThreadedExecutor executor;

        std::shared_ptr<CustomMissionControl> mission_control_node =
            std::make_shared<CustomMissionControl>(default_options);

        mission_control_node->set_mission_state(MissionControl::decision_maker);

        const uint32_t timer_executions = 5;

        rclcpp::TimerBase::SharedPtr end_timer =
            mission_control_node->create_wall_timer(
                std::chrono::milliseconds(
                    mission_control_node->event_loop_time_delta_ms *
                    timer_executions),
                [mission_control_node, &executor, timer_executions]() {
                    RCLCPP_DEBUG(mission_control_node->get_logger(),
                                 "Ending event loop test");

                    // Check that counts are correct
                    ASSERT_EQ(
                        timer_executions,
                        mission_control_node->mode_decision_maker_counter);

                    ASSERT_EQ(
                        0, mission_control_node->mode_prepare_mission_counter);
                    ASSERT_EQ(0, mission_control_node->mode_self_check_counter);
                    ASSERT_EQ(0, mission_control_node
                                     ->mode_check_drone_configuration_counter);
                    ASSERT_EQ(0,
                              mission_control_node->initiate_takeoff_counter);
                    ASSERT_EQ(
                        0, mission_control_node->mode_fly_to_waypoint_counter);
                    ASSERT_EQ(0,
                              mission_control_node->mode_detect_marker_counter);

                    executor.cancel();
                });

        executor.add_node(mission_control_node);
        executor.spin();
    }

    // Check state 'fly_to_waypoint'
    {
        rclcpp::executors::SingleThreadedExecutor executor;

        std::shared_ptr<CustomMissionControl> mission_control_node =
            std::make_shared<CustomMissionControl>(default_options);

        mission_control_node->set_mission_state(
            MissionControl::fly_to_waypoint);

        const uint32_t timer_executions = 5;

        rclcpp::TimerBase::SharedPtr end_timer =
            mission_control_node->create_wall_timer(
                std::chrono::milliseconds(
                    mission_control_node->event_loop_time_delta_ms *
                    timer_executions),
                [mission_control_node, &executor, timer_executions]() {
                    RCLCPP_DEBUG(mission_control_node->get_logger(),
                                 "Ending event loop test");

                    // Check that counts are correct
                    ASSERT_EQ(
                        timer_executions,
                        mission_control_node->mode_fly_to_waypoint_counter);

                    ASSERT_EQ(
                        0, mission_control_node->mode_prepare_mission_counter);
                    ASSERT_EQ(0, mission_control_node->mode_self_check_counter);
                    ASSERT_EQ(0, mission_control_node
                                     ->mode_check_drone_configuration_counter);
                    ASSERT_EQ(0,
                              mission_control_node->initiate_takeoff_counter);
                    ASSERT_EQ(
                        0, mission_control_node->mode_decision_maker_counter);
                    ASSERT_EQ(0,
                              mission_control_node->mode_detect_marker_counter);

                    executor.cancel();
                });

        executor.add_node(mission_control_node);
        executor.spin();
    }

    // Check state 'detect_marker'
    {
        rclcpp::executors::SingleThreadedExecutor executor;

        std::shared_ptr<CustomMissionControl> mission_control_node =
            std::make_shared<CustomMissionControl>(default_options);

        mission_control_node->set_mission_state(MissionControl::detect_marker);

        const uint32_t timer_executions = 5;

        rclcpp::TimerBase::SharedPtr end_timer =
            mission_control_node->create_wall_timer(
                std::chrono::milliseconds(
                    mission_control_node->event_loop_time_delta_ms *
                    timer_executions),
                [mission_control_node, &executor, timer_executions]() {
                    RCLCPP_DEBUG(mission_control_node->get_logger(),
                                 "Ending event loop test");

                    // Check that counts are correct
                    ASSERT_EQ(timer_executions,
                              mission_control_node->mode_detect_marker_counter);

                    ASSERT_EQ(
                        0, mission_control_node->mode_prepare_mission_counter);
                    ASSERT_EQ(0, mission_control_node->mode_self_check_counter);
                    ASSERT_EQ(0, mission_control_node
                                     ->mode_check_drone_configuration_counter);
                    ASSERT_EQ(0,
                              mission_control_node->initiate_takeoff_counter);
                    ASSERT_EQ(
                        0, mission_control_node->mode_decision_maker_counter);
                    ASSERT_EQ(
                        0, mission_control_node->mode_fly_to_waypoint_counter);

                    executor.cancel();
                });

        executor.add_node(mission_control_node);
        executor.spin();
    }

    // Check invalid state
    {
        rclcpp::executors::SingleThreadedExecutor executor;

        std::shared_ptr<CustomMissionControl> mission_control_node =
            std::make_shared<CustomMissionControl>(default_options);

        mission_control_node->mission_state =
            (MissionControl::MissionState_t)-1;

        executor.add_node(mission_control_node);

        ASSERT_DEATH({ executor.spin(); }, ".*");
    }
}
