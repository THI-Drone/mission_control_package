#include "event_loop_guard.hpp"

#include <gtest/gtest.h>

#include <cinttypes>

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
