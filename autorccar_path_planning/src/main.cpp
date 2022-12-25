#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "Glob_def.h"
#include "autorccar_interfaces/msg/nav_state.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

Bezier bc;

class PathPlanning : public rclcpp::Node {
private:

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<autorccar_interfaces::msg::NavState>::SharedPtr sub_nav;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_start_goal;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_path;

    void navi_sub_callback(const autorccar_interfaces::msg::NavState & msg) const
    {
        // RCLCPP_INFO(this->get_logger(), "I heard: %d", msg.timestamp.sec);
    }

    void gcs_start_goal_callback(const std_msgs::msg::Float64MultiArray & msg) const
    {
        double start[2], goal[2];

        start[0] = msg.data.at(0);
        start[1] = msg.data.at(1);
        goal[0] = msg.data.at(2);
        goal[1] = msg.data.at(3);

        RCLCPP_INFO(this->get_logger(), "start: %lf, %lf", start[0], start[1]);
        RCLCPP_INFO(this->get_logger(), "goal: %lf, %lf", goal[0], goal[1]);

        bc.set_start_point(start[0], start[1]);
        bc.set_goal_point(goal[0], goal[1]);

        bc.update_bezier_curve();

        auto message = std_msgs::msg::Float64MultiArray();
        message.data.push_back(bc.B0[X]);
        message.data.push_back(bc.B0[Y]);
        message.data.push_back(bc.B1[X]);
        message.data.push_back(bc.B1[Y]);
        message.data.push_back(bc.B2[X]);
        message.data.push_back(bc.B2[Y]);
        message.data.push_back(bc.B3[X]);
        message.data.push_back(bc.B3[Y]);
        message.data.push_back(bc.B4[X]);
        message.data.push_back(bc.B4[Y]);

        // RCLCPP_INFO(this->get_logger(), "Send message");

        pub_path->publish(message);

    }

public:
    PathPlanning() : Node("path_planning"){
        sub_nav = this->create_subscription<autorccar_interfaces::msg::NavState>(
        "nav_topic", 10, std::bind(&PathPlanning::navi_sub_callback, this, _1));

        sub_start_goal = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        "start_goal_topic", 10, std::bind(&PathPlanning::gcs_start_goal_callback, this, _1));

        pub_path = this->create_publisher<std_msgs::msg::Float64MultiArray>("path_topic", 10);
    }
};



int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);

    bc.set_start_point(B0_INIT_X, B0_INIT_X);
    bc.set_start_velocity(B0_INIT_G, B0_INIT_V);
    bc.set_start_curvature(B0_INIT_C);
    bc.set_goal_point(B4_INIT_X, B4_INIT_Y);
    bc.set_goal_velocity(B4_INIT_G, B4_INIT_V);
    bc.set_goal_curvature(B4_INIT_C);

    bc.update_bezier_curve();

    rclcpp::spin(std::make_shared<PathPlanning>());
    rclcpp::shutdown();
    
    return 0;
}
