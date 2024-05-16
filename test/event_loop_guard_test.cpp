#include "event_loop_guard.hpp"

#include <gtest/gtest.h>

#include <cinttypes>

/**
 * @brief Test case for the `mission_control_package` module's
 * `event_loop_guard_test`.
 *
 * This test case verifies the behavior of the `EventLoopGuard` class by testing
 * different scenarios where the flag is initialized with different values and
 * the new state is either true or false.
 *
 * The test checks if the flag is updated correctly based on the
 * `EventLoopGuard` object's constructor arguments, and asserts the expected
 * flag value after the `EventLoopGuard` object goes out of scope.
 */
TEST(mission_control_package, event_loop_guard_test) {
    // Check with flag initialized with false and new state is true
    {
        bool flag = false;

        {
            EventLoopGuard elg = EventLoopGuard(&flag, true);
            ASSERT_TRUE(flag);
        }

        ASSERT_FALSE(flag);
    }

    // Check with flag initialized with true and new state is false
    {
        bool flag = true;

        {
            EventLoopGuard elg = EventLoopGuard(&flag, false);
            ASSERT_FALSE(flag);
        }

        ASSERT_TRUE(flag);
    }

    // Check with flag initialized with true and new state is true
    {
        bool flag = true;

        {
            EventLoopGuard elg = EventLoopGuard(&flag, true);
            ASSERT_TRUE(flag);
        }

        ASSERT_TRUE(flag);
    }

    // Check with flag initialized with false and new state is false
    {
        bool flag = false;

        {
            EventLoopGuard elg = EventLoopGuard(&flag, false);
            ASSERT_FALSE(flag);
        }

        ASSERT_FALSE(flag);
    }
}
