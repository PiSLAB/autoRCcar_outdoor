#include <memory>

#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "autorccar_interfaces/msg/gnss.hpp"
#include "autorccar_interfaces/msg/imu.hpp"

using namespace std;
using std::placeholders::_1;

class SensorSubs : public rclcpp::Node
{
  public:
    SensorSubs()
    : Node("SensorSubs")
    {
      subscription_imu = this->create_subscription<autorccar_interfaces::msg::Imu>(
      "IMU", 1000, std::bind(&SensorSubs::topic_callback_imu, this, _1));
      
      subscription_gnss = this->create_subscription<autorccar_interfaces::msg::Gnss>(
      "GNSS", 1000, std::bind(&SensorSubs::topic_callback_gnss, this, _1));
    }

  private:
    void topic_callback_imu(const autorccar_interfaces::msg::Imu & msg) const
    {
	RCLCPP_INFO_STREAM(this->get_logger(), "IMU: '" << msg.linear_acceleration.x << "'");
	RCLCPP_INFO_STREAM(this->get_logger(), "IMU: '" << msg.linear_acceleration.y << "'"); 
	RCLCPP_INFO_STREAM(this->get_logger(), "IMU: '" << msg.linear_acceleration.z << "'"); 
	RCLCPP_INFO_STREAM(this->get_logger(), "IMU: '" << msg.angular_velocity.x << "'"); 
	RCLCPP_INFO_STREAM(this->get_logger(), "IMU: '" << msg.angular_velocity.y << "'"); 
	RCLCPP_INFO_STREAM(this->get_logger(), "IMU: '" << msg.angular_velocity.z << "'"); 
	RCLCPP_INFO_STREAM(this->get_logger(), "IMU: '" << msg.timestamp.sec << "'");
	RCLCPP_INFO_STREAM(this->get_logger(), "IMU: '" << msg.timestamp.nanosec << "'");
	cout << "                        "<< endl; 
    }
    void topic_callback_gnss(const autorccar_interfaces::msg::Gnss & msg) const
    {
	RCLCPP_INFO_STREAM(this->get_logger(), "GNSS: '" << msg.position_ecef.x << "'");
	RCLCPP_INFO_STREAM(this->get_logger(), "GNSS: '" << msg.position_ecef.y << "'"); 
	RCLCPP_INFO_STREAM(this->get_logger(), "GNSS: '" << msg.position_ecef.z << "'"); 
	RCLCPP_INFO_STREAM(this->get_logger(), "GNSS: '" << msg.velocity_ecef.x << "'"); 
	RCLCPP_INFO_STREAM(this->get_logger(), "GNSS: '" << msg.velocity_ecef.y << "'"); 
	RCLCPP_INFO_STREAM(this->get_logger(), "GNSS: '" << msg.velocity_ecef.z << "'"); 
	RCLCPP_INFO_STREAM(this->get_logger(), "GNSS: '" << msg.timestamp.sec << "'");
	RCLCPP_INFO_STREAM(this->get_logger(), "GNSS: '" << msg.timestamp.nanosec << "'");
	cout << "                        "<< endl; 
    }
    
    rclcpp::Subscription<autorccar_interfaces::msg::Imu>::SharedPtr subscription_imu;
    rclcpp::Subscription<autorccar_interfaces::msg::Gnss>::SharedPtr subscription_gnss;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SensorSubs>());
  rclcpp::shutdown();
  return 0;
}

