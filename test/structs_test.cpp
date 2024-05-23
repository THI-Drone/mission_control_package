#include "structs.hpp"

#include <gtest/gtest.h>

#include <array>
#include <stdexcept>

/**
 * @brief Test case for the `HeartbeatPayload` struct in the
 * `mission_control_package`.
 *
 * This test verifies the initial state of the `HeartbeatPayload` object.
 */
TEST(mission_control_package, HeartbeatPayload_test) {
    HeartbeatPayload hb = HeartbeatPayload();

    ASSERT_FALSE(hb.received);
    ASSERT_EQ(0, hb.tick);
    ASSERT_FALSE(hb.active);
}

/**
 * @brief Test case for the `mission_control_package` module's `ros_node_test`.
 *
 * This test case verifies the behavior of the `RosNode` class constructors.
 */
TEST(mission_control_package, ros_node_test) {
    // Test default constructor
    {
        RosNode rn = RosNode();

        ASSERT_FALSE(rn.can_start_mission);
        ASSERT_FALSE(rn.is_fcc_bridge);
    }

    // Test custom constructor
    {
        {
            RosNode rn = RosNode(false, false);

            ASSERT_FALSE(rn.can_start_mission);
            ASSERT_FALSE(rn.is_fcc_bridge);
        }

        {
            RosNode rn = RosNode(true, false);

            ASSERT_TRUE(rn.can_start_mission);
            ASSERT_FALSE(rn.is_fcc_bridge);
        }

        {
            RosNode rn = RosNode(false, true);

            ASSERT_FALSE(rn.can_start_mission);
            ASSERT_TRUE(rn.is_fcc_bridge);
        }

        {
            RosNode rn = RosNode(true, true);

            ASSERT_TRUE(rn.can_start_mission);
            ASSERT_TRUE(rn.is_fcc_bridge);
        }
    }
}

TEST(mission_control_package, position_test) {
    // Test default constructor
    {
        Position ps = Position();

        ASSERT_FALSE(ps.values_set);
        ASSERT_EQ(0.0, ps.coordinate_lat);
        ASSERT_EQ(0.0, ps.coordinate_lon);
        ASSERT_EQ(0u, ps.height_cm);
    }

    // Test custom constructor
    {
        Position ps = Position(1.23, 4.56, 78);

        ASSERT_TRUE(ps.values_set);
        ASSERT_EQ(1.23, ps.coordinate_lat);
        ASSERT_EQ(4.56, ps.coordinate_lon);
        ASSERT_EQ(78u, ps.height_cm);
    }

    // Test 'set_position' function
    {
        Position ps = Position();
        ASSERT_FALSE(ps.values_set);

        ps.set_position(1.23, 4.56, 78);

        ASSERT_TRUE(ps.values_set);
        ASSERT_EQ(1.23, ps.coordinate_lat);
        ASSERT_EQ(4.56, ps.coordinate_lon);
        ASSERT_EQ(78u, ps.height_cm);
    }

    // Test 'get_position_array' function
    {
        // Test with coords set
        {
            Position ps = Position(1.23, 4.56, 78);

            std::array<double, 2> coords;
            ASSERT_NO_THROW(coords = ps.get_position_array());

            std::array<double, 2> expected_coords = {1.23, 4.56};
            ASSERT_EQ(expected_coords, coords);
        }

        // Test with coords not set
        {
            Position ps = Position();

            ASSERT_THROW(ps.get_position_array(), std::runtime_error);
        }
    }
}
