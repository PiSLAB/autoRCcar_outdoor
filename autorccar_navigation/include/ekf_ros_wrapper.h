#ifndef EKF_ROS_WRAPPER_H
#define EKF_ROS_WRAPPER_H

#include <chrono>
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "autorccar_interfaces/msg/imu.hpp"
#include "autorccar_interfaces/msg/gnss.hpp"
#include "autorccar_interfaces/msg/nav_state.hpp"

#include "ekf.h"

class EKFWrapper: public rclcpp::Node
{
    public:
        EKFWrapper(EKF* pEKF);
    
    private:
        rclcpp::Subscription<autorccar_interfaces::msg::Imu>::SharedPtr imu_subscriber;
        rclcpp::Subscription<autorccar_interfaces::msg::Gnss>::SharedPtr gps_subscriber;
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr yaw_subscriber;
        rclcpp::Publisher<autorccar_interfaces::msg::NavState>::SharedPtr state_publisher;
        
        EKF* mpEKF;
        NavSol* mpSol;

        void ImuCallback(const autorccar_interfaces::msg::Imu & msg);
        void GpsCallback(const autorccar_interfaces::msg::Gnss & msg);
        void InitYawCallback(const std_msgs::msg::Float32 &msg);
        void PublishNavSol();
};

#endif
