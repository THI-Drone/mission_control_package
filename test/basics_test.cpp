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

TEST(mission_control_package, set_mission_state_test) {
    // Set new mission state
    {
        MissionControl mc = MissionControl();
        ASSERT_EQ(mc.get_mission_state(), MissionControl::prepare_mission);
        mc.job_finished_successfully = true;
        mc.state_first_loop = false;
        mc.mission_progress = 1.0;

        mc.set_mission_state(MissionControl::decision_maker);

        ASSERT_FALSE(mc.job_finished_successfully);
        ASSERT_TRUE(mc.state_first_loop);
        ASSERT_EQ(mc.mission_progress, 0.0);
        ASSERT_EQ(mc.get_mission_state(), MissionControl::decision_maker);
    }

    // Set already existing mission state
    {
        MissionControl mc = MissionControl();
        ASSERT_EQ(mc.get_mission_state(), MissionControl::prepare_mission);
        mc.job_finished_successfully = true;
        mc.state_first_loop = false;
        mc.mission_progress = 1.0;

        mc.set_mission_state(MissionControl::prepare_mission);

        ASSERT_TRUE(mc.job_finished_successfully);
        ASSERT_FALSE(mc.state_first_loop);
        ASSERT_EQ(mc.mission_progress, 1.0);
        ASSERT_EQ(mc.get_mission_state(), MissionControl::prepare_mission);
    }
}

TEST(mission_control_package, set_active_node_id_test) {
    // Test with no previous active node id
    {
        MissionControl mc = MissionControl();
        ASSERT_FALSE(mc.probation_period);
        ASSERT_EQ(mc.get_active_node_id(), "");
        ASSERT_EQ(mc.get_last_active_node_id(), "");

        mc.set_active_node_id("abc");

        ASSERT_TRUE(mc.probation_period);
        ASSERT_EQ(mc.get_active_node_id(), "abc");
        ASSERT_EQ(mc.get_last_active_node_id(), "");
    }

    // Test with previous active node id
    {
        MissionControl mc = MissionControl();
        mc.active_node_id = "previous";

        ASSERT_FALSE(mc.probation_period);
        ASSERT_EQ(mc.get_active_node_id(), "previous");
        ASSERT_EQ(mc.get_last_active_node_id(), "");

        mc.set_active_node_id("abc");

        ASSERT_TRUE(mc.probation_period);
        ASSERT_EQ(mc.get_active_node_id(), "abc");
        ASSERT_EQ(mc.get_last_active_node_id(), "previous");
    }
}

TEST(mission_control_package, clear_active_node_id_test) {
    // Test with no previous active node id
    {
        MissionControl mc = MissionControl();
        ASSERT_FALSE(mc.probation_period);
        ASSERT_EQ(mc.get_active_node_id(), "");
        ASSERT_EQ(mc.get_last_active_node_id(), "");

        mc.clear_active_node_id();

        ASSERT_TRUE(mc.probation_period);
        ASSERT_EQ(mc.get_active_node_id(), "");
        ASSERT_EQ(mc.get_last_active_node_id(), "");
    }

    // Test with previous active node id
    {
        MissionControl mc = MissionControl();
        mc.active_node_id = "abc";

        ASSERT_FALSE(mc.probation_period);
        ASSERT_EQ(mc.get_active_node_id(), "abc");
        ASSERT_EQ(mc.get_last_active_node_id(), "");

        mc.clear_active_node_id();

        ASSERT_TRUE(mc.probation_period);
        ASSERT_EQ(mc.get_active_node_id(), "");
        ASSERT_EQ(mc.get_last_active_node_id(), "abc");
    }
}

TEST(mission_control_package, set_active_marker_name_test) {
    // Test with no explicitly set previous marker name
    {
        MissionControl mc = MissionControl();
        ASSERT_EQ(mc.get_active_marker_name(), "init");

        mc.set_active_marker_name("abc");

        ASSERT_EQ(mc.get_active_marker_name(), "abc");
    }

    // Test with previous marker name
    {
        MissionControl mc = MissionControl();
        mc.active_marker_name = "previous";

        ASSERT_EQ(mc.get_active_marker_name(), "previous");

        mc.set_active_marker_name("abc");

        ASSERT_EQ(mc.get_active_marker_name(), "abc");
    }
}

TEST(mission_control_package, probation_period_test) {
    // Check using the 'set_active_node_id' function
    {
        rclcpp::executors::SingleThreadedExecutor executor;

        std::shared_ptr<MissionControl> mission_control_node =
            std::make_shared<MissionControl>();

        mission_control_node->event_loop_active =
            false;  // Deactivate event loop so that an internal state doesn't
                    // mess with our tested variables
        mission_control_node->active_node_id = "previous";

        // Time offset after which the probation period is triggered
        const uint32_t time_offset_ms = 100;

        rclcpp::TimerBase::SharedPtr probation_period_trigger_timer;
        probation_period_trigger_timer =
            mission_control_node->create_wall_timer(
                std::chrono::milliseconds(time_offset_ms),
                [mission_control_node, probation_period_trigger_timer]() {
                    RCLCPP_ERROR(mission_control_node->get_logger(),
                                 "Setting active node id and thereby "
                                 "triggering probation period");

                    if (probation_period_trigger_timer)
                        probation_period_trigger_timer->cancel();

                    mission_control_node->set_active_node_id("abc");
                });

        rclcpp::TimerBase::SharedPtr probation_period_check_active_timer;
        probation_period_check_active_timer =
            mission_control_node->create_wall_timer(
                std::chrono::milliseconds(
                    time_offset_ms +
                    mission_control_node->probation_period_length_ms - 10),
                [mission_control_node, probation_period_check_active_timer]() {
                    RCLCPP_ERROR(
                        mission_control_node->get_logger(),
                        "Checking that probation period is currently active");

                    if (probation_period_check_active_timer)
                        probation_period_check_active_timer->cancel();

                    ASSERT_TRUE(mission_control_node->probation_period);
                    ASSERT_EQ(mission_control_node->get_active_node_id(),
                              "abc");
                    ASSERT_EQ(mission_control_node->get_last_active_node_id(),
                              "previous");
                });

        rclcpp::TimerBase::SharedPtr probation_period_check_inactive_timer =
            mission_control_node->create_wall_timer(
                std::chrono::milliseconds(
                    time_offset_ms +
                    mission_control_node->probation_period_length_ms + 10),
                [mission_control_node, &executor]() {
                    RCLCPP_ERROR(mission_control_node->get_logger(),
                                 "Checking that probation period is currently "
                                 "not active");

                    ASSERT_FALSE(mission_control_node->probation_period);
                    ASSERT_EQ(mission_control_node->get_active_node_id(),
                              "abc");
                    ASSERT_EQ(mission_control_node->get_last_active_node_id(),
                              "");

                    executor.cancel();
                });

        executor.add_node(mission_control_node);
        executor.spin();
    }

    // Check using the 'clear_active_node_id' function
    {
        rclcpp::executors::SingleThreadedExecutor executor;

        std::shared_ptr<MissionControl> mission_control_node =
            std::make_shared<MissionControl>();

        mission_control_node->event_loop_active =
            false;  // Deactivate event loop so that an internal state doesn't
                    // mess with our tested variables
        mission_control_node->active_node_id = "previous";

        // Time offset after which the probation period is triggered
        const uint32_t time_offset_ms = 100;

        rclcpp::TimerBase::SharedPtr probation_period_trigger_timer;
        probation_period_trigger_timer =
            mission_control_node->create_wall_timer(
                std::chrono::milliseconds(time_offset_ms),
                [mission_control_node, probation_period_trigger_timer]() {
                    RCLCPP_ERROR(mission_control_node->get_logger(),
                                 "Clearing active node id and thereby "
                                 "triggering probation period");

                    if (probation_period_trigger_timer)
                        probation_period_trigger_timer->cancel();

                    mission_control_node->clear_active_node_id();
                });

        rclcpp::TimerBase::SharedPtr probation_period_check_active_timer;
        probation_period_check_active_timer =
            mission_control_node->create_wall_timer(
                std::chrono::milliseconds(
                    time_offset_ms +
                    mission_control_node->probation_period_length_ms - 10),
                [mission_control_node, probation_period_check_active_timer]() {
                    RCLCPP_ERROR(
                        mission_control_node->get_logger(),
                        "Checking that probation period is currently active");

                    if (probation_period_check_active_timer)
                        probation_period_check_active_timer->cancel();

                    ASSERT_TRUE(mission_control_node->probation_period);
                    ASSERT_EQ(mission_control_node->get_active_node_id(), "");
                    ASSERT_EQ(mission_control_node->get_last_active_node_id(),
                              "previous");
                });

        rclcpp::TimerBase::SharedPtr probation_period_check_inactive_timer =
            mission_control_node->create_wall_timer(
                std::chrono::milliseconds(
                    time_offset_ms +
                    mission_control_node->probation_period_length_ms + 10),
                [mission_control_node, &executor]() {
                    RCLCPP_ERROR(mission_control_node->get_logger(),
                                 "Checking that probation period is currently "
                                 "not active");

                    ASSERT_FALSE(mission_control_node->probation_period);
                    ASSERT_EQ(mission_control_node->get_active_node_id(), "");
                    ASSERT_EQ(mission_control_node->get_last_active_node_id(),
                              "");

                    executor.cancel();
                });

        executor.add_node(mission_control_node);
        executor.spin();
    }
}

TEST(mission_control_package, get_state_first_loop_test) {
    // Test with 'state_first_loop' false
    {
        MissionControl mc = MissionControl();
        mc.state_first_loop = false;

        ASSERT_FALSE(mc.get_state_first_loop());
    }

    // Test with 'state_first_loop' true
    {
        MissionControl mc = MissionControl();
        mc.state_first_loop = true;

        ASSERT_TRUE(mc.get_state_first_loop());

        ASSERT_FALSE(mc.get_state_first_loop());
        ASSERT_FALSE(mc.state_first_loop);
    }
}

TEST(mission_control_package, get_job_finished_successfully_test) {
    // Test with 'job_finished_successfully' set to false
    {
        MissionControl mc = MissionControl();
        mc.job_finished_successfully = false;

        ASSERT_FALSE(mc.get_job_finished_successfully());
    }

    // Test with 'job_finished_successfully' set to true
    {
        MissionControl mc = MissionControl();
        mc.job_finished_successfully = true;

        ASSERT_TRUE(mc.get_job_finished_successfully());

        ASSERT_FALSE(mc.get_job_finished_successfully());
        ASSERT_FALSE(mc.job_finished_successfully);
    }
}

TEST(mission_control_package, get_job_finished_payload_test) {
    // Test with 'get_job_finished_payload' not set
    {
        MissionControl mc = MissionControl();

        ASSERT_EQ(mc.get_job_finished_payload(), nlohmann::json());
        ASSERT_EQ(mc.get_job_finished_payload(), nlohmann::json());
    }

    // Test with 'get_job_finished_payload' set
    {
        MissionControl mc = MissionControl();
        mc.job_finished_payload = {{"abc", 123}};

        ASSERT_EQ(mc.get_job_finished_payload(),
                  nlohmann::json({{"abc", 123}}));
        ASSERT_EQ(mc.get_job_finished_payload(), nlohmann::json());
    }
}

TEST(mission_control_package, current_mission_finished_test) {
    // Test with 'mission_progress' set to 0.0
    {
        MissionControl mc = MissionControl();
        mc.mission_progress = 0.0f;

        ASSERT_FALSE(mc.current_mission_finished());
        ASSERT_EQ(mc.mission_progress, 0.0f);
    }

    // Test with 'mission_progress' set to 0.5
    {
        MissionControl mc = MissionControl();
        mc.mission_progress = 0.5f;

        ASSERT_FALSE(mc.current_mission_finished());
        ASSERT_EQ(mc.mission_progress, 0.5f);
    }

    // Test with 'mission_progress' set to 1.0
    {
        MissionControl mc = MissionControl();
        mc.mission_progress = 1.0f;

        ASSERT_TRUE(mc.current_mission_finished());
        ASSERT_EQ(mc.mission_progress, 0.0f);
    }

    // Test with 'mission_progress' set to 5.123
    {
        MissionControl mc = MissionControl();
        mc.mission_progress = 5.123f;

        ASSERT_TRUE(mc.current_mission_finished());
        ASSERT_EQ(mc.mission_progress, 0.0f);
    }

    // Test with 'mission_progress' set to -1.23
    {
        MissionControl mc = MissionControl();
        mc.mission_progress = -1.23f;

        ASSERT_FALSE(mc.current_mission_finished());
        ASSERT_EQ(mc.mission_progress, -1.23f);
    }
}
