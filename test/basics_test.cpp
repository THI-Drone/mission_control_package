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
