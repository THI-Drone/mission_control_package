#include "mission_control.hpp"

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MissionControl>(rclcpp::NodeOptions()));
    rclcpp::shutdown();
    return 0;
}
