#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "ekf_ros_wrapper.h"
#include "ekf.h"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    // Create EKF based navigation system.
    std::string config = "/home/gunaco/ros_ws/src/autorccar_outdoor/autorccar_navigation/config.yaml";

    EKF nav(config);
    
    auto node = std::make_shared<EKFWrapper>(&nav);
    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}
