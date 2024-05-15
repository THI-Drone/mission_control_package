#include <gtest/gtest.h>

#include <chrono>
#include <cinttypes>
#include <nlohmann/json.hpp>
#include <optional>

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
                [mission_control_node, &probation_period_trigger_timer]() {
                    RCLCPP_DEBUG(mission_control_node->get_logger(),
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
                [mission_control_node, &probation_period_check_active_timer]() {
                    RCLCPP_DEBUG(
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
                    RCLCPP_DEBUG(mission_control_node->get_logger(),
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
                [mission_control_node, &probation_period_trigger_timer]() {
                    RCLCPP_DEBUG(mission_control_node->get_logger(),
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
                [mission_control_node, &probation_period_check_active_timer]() {
                    RCLCPP_DEBUG(
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
                    RCLCPP_DEBUG(mission_control_node->get_logger(),
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

TEST(mission_control_package, wait_time_test) {
    // Check the wait time funtionality with a wait time of 100 ms
    {
        rclcpp::executors::SingleThreadedExecutor executor;

        std::shared_ptr<MissionControl> mission_control_node =
            std::make_shared<MissionControl>();

        mission_control_node->event_loop_active =
            false;  // Deactivate event loop so that an internal state doesn't
                    // mess with the test

        // Time offset that is accepted
        const double time_offset_s = 10.0 / 1000.0;

        // Duration that should be waited for
        const uint32_t wait_time_ms = 100;

        // Timestamp when the wait time was started
        rclcpp::Time start_timestamp;

        rclcpp::TimerBase::SharedPtr wait_check_timer;
        wait_check_timer = mission_control_node->create_wall_timer(
            std::chrono::milliseconds(1),
            [&executor, mission_control_node, &wait_check_timer, time_offset_s,
             wait_time_ms, &start_timestamp]() {
                if (mission_control_node->wait_time_finished()) {
                    rclcpp::Time end_timestamp = mission_control_node->now();

                    if (wait_check_timer) wait_check_timer->cancel();

                    const double abs_time_dif =
                        std::abs((start_timestamp - end_timestamp).seconds()) -
                        (wait_time_ms / 1000.0);

                    RCLCPP_DEBUG(mission_control_node->get_logger(),
                                 "Wait time finished: Time dif: %f",
                                 abs_time_dif);

                    ASSERT_TRUE(abs_time_dif <= time_offset_s);
                    ASSERT_FALSE(mission_control_node->wait_time_finished_ok);

                    executor.cancel();
                } else {
                    ASSERT_FALSE(mission_control_node->wait_time_finished_ok);
                }
            });

        rclcpp::TimerBase::SharedPtr start_wait_time_timer;
        start_wait_time_timer = mission_control_node->create_wall_timer(
            std::chrono::milliseconds(10),
            [mission_control_node, &start_wait_time_timer, wait_time_ms,
             &start_timestamp]() {
                RCLCPP_DEBUG(mission_control_node->get_logger(),
                             "Starting wait time");

                if (start_wait_time_timer) start_wait_time_timer->cancel();

                mission_control_node->init_wait(wait_time_ms);
                start_timestamp = mission_control_node->now();
            });

        executor.add_node(mission_control_node);
        executor.spin();
    }

    // Check the wait time funtionality with a wait time of 1 ms
    {
        rclcpp::executors::SingleThreadedExecutor executor;

        std::shared_ptr<MissionControl> mission_control_node =
            std::make_shared<MissionControl>();

        mission_control_node->event_loop_active =
            false;  // Deactivate event loop so that an internal state doesn't
                    // mess with the test

        // Time offset that is accepted
        const double time_offset_s = 10.0 / 1000.0;

        // Duration that should be waited for
        const uint32_t wait_time_ms = 1;

        // Timestamp when the wait time was started
        rclcpp::Time start_timestamp;

        rclcpp::TimerBase::SharedPtr wait_check_timer;
        wait_check_timer = mission_control_node->create_wall_timer(
            std::chrono::milliseconds(1),
            [&executor, mission_control_node, &wait_check_timer, time_offset_s,
             wait_time_ms, &start_timestamp]() {
                if (mission_control_node->wait_time_finished()) {
                    rclcpp::Time end_timestamp = mission_control_node->now();

                    if (wait_check_timer) wait_check_timer->cancel();

                    const double abs_time_dif =
                        std::abs((start_timestamp - end_timestamp).seconds()) -
                        (wait_time_ms / 1000.0);

                    RCLCPP_DEBUG(mission_control_node->get_logger(),
                                 "Wait time finished: Time dif: %f",
                                 abs_time_dif);

                    ASSERT_TRUE(abs_time_dif <= time_offset_s);
                    ASSERT_FALSE(mission_control_node->wait_time_finished_ok);

                    executor.cancel();
                } else {
                    ASSERT_FALSE(mission_control_node->wait_time_finished_ok);
                }
            });

        rclcpp::TimerBase::SharedPtr start_wait_time_timer;
        start_wait_time_timer = mission_control_node->create_wall_timer(
            std::chrono::milliseconds(10),
            [mission_control_node, &start_wait_time_timer, wait_time_ms,
             &start_timestamp]() {
                RCLCPP_DEBUG(mission_control_node->get_logger(),
                             "Starting wait time");

                if (start_wait_time_timer) start_wait_time_timer->cancel();

                mission_control_node->init_wait(wait_time_ms);
                start_timestamp = mission_control_node->now();
            });

        executor.add_node(mission_control_node);
        executor.spin();
    }

    // Check the wait time funtionality with a wait time of 0 ms
    {
        rclcpp::executors::SingleThreadedExecutor executor;

        std::shared_ptr<MissionControl> mission_control_node =
            std::make_shared<MissionControl>();

        mission_control_node->event_loop_active =
            false;  // Deactivate event loop so that an internal state doesn't
                    // mess with the test

        // Time offset that is accepted
        const double time_offset_s = 10.0 / 1000.0;

        // Duration that should be waited for
        const uint32_t wait_time_ms = 0;

        // Timestamp when the wait time was started
        rclcpp::Time start_timestamp;

        rclcpp::TimerBase::SharedPtr wait_check_timer;
        wait_check_timer = mission_control_node->create_wall_timer(
            std::chrono::milliseconds(1),
            [&executor, mission_control_node, &wait_check_timer, time_offset_s,
             wait_time_ms, &start_timestamp]() {
                if (mission_control_node->wait_time_finished()) {
                    rclcpp::Time end_timestamp = mission_control_node->now();

                    if (wait_check_timer) wait_check_timer->cancel();

                    const double abs_time_dif =
                        std::abs((start_timestamp - end_timestamp).seconds()) -
                        (wait_time_ms / 1000.0);

                    RCLCPP_DEBUG(mission_control_node->get_logger(),
                                 "Wait time finished: Time dif: %f",
                                 abs_time_dif);

                    ASSERT_TRUE(abs_time_dif <= time_offset_s);
                    ASSERT_FALSE(mission_control_node->wait_time_finished_ok);

                    executor.cancel();
                } else {
                    ASSERT_FALSE(mission_control_node->wait_time_finished_ok);
                }
            });

        rclcpp::TimerBase::SharedPtr start_wait_time_timer;
        start_wait_time_timer = mission_control_node->create_wall_timer(
            std::chrono::milliseconds(10),
            [mission_control_node, &start_wait_time_timer, wait_time_ms,
             &start_timestamp]() {
                RCLCPP_DEBUG(mission_control_node->get_logger(),
                             "Starting wait time");

                if (start_wait_time_timer) start_wait_time_timer->cancel();

                mission_control_node->init_wait(wait_time_ms);
                start_timestamp = mission_control_node->now();
            });

        executor.add_node(mission_control_node);
        executor.spin();
    }

    // Check cancelling the wait time
    {
        rclcpp::executors::SingleThreadedExecutor executor;

        std::shared_ptr<MissionControl> mission_control_node =
            std::make_shared<MissionControl>();

        mission_control_node->event_loop_active =
            false;  // Deactivate event loop so that an internal state doesn't
                    // mess with the test

        // Duration that should be waited for
        const uint32_t wait_time_ms = 100;

        // Timestamp when the wait time was started
        std::optional<rclcpp::Time> start_timestamp;

        rclcpp::TimerBase::SharedPtr wait_check_timer;
        wait_check_timer = mission_control_node->create_wall_timer(
            std::chrono::milliseconds(1),
            [&executor, mission_control_node, &wait_check_timer, wait_time_ms,
             &start_timestamp]() {
                ASSERT_FALSE(mission_control_node->wait_time_finished());
                rclcpp::Time cur_timestamp = mission_control_node->now();

                // Check if wait time has not started yet
                if (!start_timestamp.has_value()) return;

                if ((cur_timestamp - start_timestamp.value()).seconds() >
                    ((wait_time_ms * 1.5) / 1000.0)) {
                    RCLCPP_DEBUG(mission_control_node->get_logger(),
                                 "Finished waiting for wait time");

                    executor.cancel();
                }
            });

        rclcpp::TimerBase::SharedPtr cancel_wait_time_timer;
        cancel_wait_time_timer = mission_control_node->create_wall_timer(
            std::chrono::milliseconds((uint32_t)(wait_time_ms * 0.5)),
            [mission_control_node, &cancel_wait_time_timer, wait_time_ms]() {
                RCLCPP_DEBUG(mission_control_node->get_logger(),
                             "Cancelling wait time");

                if (cancel_wait_time_timer) cancel_wait_time_timer->cancel();

                ASSERT_FALSE(mission_control_node->wait_time_finished_ok);
                mission_control_node->cancel_wait();
                ASSERT_FALSE(mission_control_node->wait_time_finished_ok);
            });

        rclcpp::TimerBase::SharedPtr start_wait_time_timer;
        start_wait_time_timer = mission_control_node->create_wall_timer(
            std::chrono::milliseconds(10),
            [mission_control_node, &start_wait_time_timer, wait_time_ms,
             &start_timestamp]() {
                RCLCPP_DEBUG(mission_control_node->get_logger(),
                             "Starting wait time");

                if (start_wait_time_timer) start_wait_time_timer->cancel();

                mission_control_node->init_wait(wait_time_ms);
                start_timestamp = mission_control_node->now();
            });

        executor.add_node(mission_control_node);
        executor.spin();
    }
}
