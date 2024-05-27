#include <gtest/gtest.h>

#include "rclcpp/utilities.hpp"

static constexpr size_t TEST_DOMAIN_ID = 2;

int main(int argc, char **argv) {
    rclcpp::InitOptions init_options;
    init_options.set_domain_id(TEST_DOMAIN_ID);
    rclcpp::init(0, nullptr, init_options);

    testing::InitGoogleTest(&argc, argv);
    const int ret = RUN_ALL_TESTS();

    rclcpp::shutdown();
    return ret;
}
