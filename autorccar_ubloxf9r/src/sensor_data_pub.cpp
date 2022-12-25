// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <gtest/gtest.h>

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <math.h>

#include <iostream>

#include <boost/iostreams/stream.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/asio.hpp>
#include <boost/circular_buffer.hpp>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time_source.hpp"
#include "std_msgs/msg/string.hpp"
#include "autorccar_interfaces/msg/gnss.hpp"
#include "autorccar_interfaces/msg/imu.hpp"

#include "ublox_decode.cpp"

using namespace std;
using boost::asio::serial_port_base;

//using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

//Global Variable//
extern IMU imu;
extern GNSS_ECEF ECEF;

char Packet_Flag = 0;
char Packet_Flag1 = 0;
char Packet_Flag2 = 0;
char Packet_Flag3 = 0;
//Global Variable//

class IMU_Pub : public rclcpp::Node
{
	public:
		IMU_Pub()
			: Node("IMU_Pub")
		{
			publisher_ = this->create_publisher<autorccar_interfaces::msg::Imu>("IMU", 1000);
			timer_ = this->create_wall_timer(
					0.1ms, std::bind(&IMU_Pub::timer_callback, this));	
		}

	private:
		void timer_callback()
		{
			if(Packet_Flag1 == 1)
			{
				Packet_Flag1 = 0;
				auto message = autorccar_interfaces::msg::Imu();
				message.linear_acceleration.x = imu.AccX;
				message.linear_acceleration.y = imu.AccY;
				message.linear_acceleration.z = imu.AccZ;
				message.angular_velocity.x = imu.GyroX;
				message.angular_velocity.y = imu.GyroY;
				message.angular_velocity.z = imu.GyroZ;
				message.timestamp = now();
				/*
				RCLCPP_INFO_STREAM(this->get_logger(), "Publishing: '" << message.linear_acceleration.x << "'");
				RCLCPP_INFO_STREAM(this->get_logger(), "Publishing: '" << message.linear_acceleration.y << "'"); 
				RCLCPP_INFO_STREAM(this->get_logger(), "Publishing: '" << message.linear_acceleration.z << "'"); 
				RCLCPP_INFO_STREAM(this->get_logger(), "Publishing: '" << message.angular_velocity.x << "'"); 
				RCLCPP_INFO_STREAM(this->get_logger(), "Publishing: '" << message.angular_velocity.y << "'"); 
				RCLCPP_INFO_STREAM(this->get_logger(), "Publishing: '" << message.angular_velocity.z << "'"); 
				RCLCPP_INFO_STREAM(this->get_logger(), "Publishing: '" << message.timestamp.sec << "'");
				RCLCPP_INFO_STREAM(this->get_logger(), "Publishing: '" << message.timestamp.nanosec << "'");
				cout << "                        "<< endl;
				*/ 
				publisher_->publish(message);
			}
		}
		rclcpp::TimerBase::SharedPtr timer_;
		rclcpp::Publisher<autorccar_interfaces::msg::Imu>::SharedPtr publisher_;
};

class GNSS_Pub : public rclcpp::Node
{
	public:
		GNSS_Pub()
			: Node("GNSS_Pub")
		{
			publisher_ = this->create_publisher<autorccar_interfaces::msg::Gnss>("GNSS", 1000);
			timer_ = this->create_wall_timer(
					0.1ms, std::bind(&GNSS_Pub::timer_callback, this));
		}

	private:
		void timer_callback()
		{	
			if(Packet_Flag2 == 1 && Packet_Flag3 == 1)
			{
				Packet_Flag2 = 0;
				Packet_Flag3 = 0;			
				auto message = autorccar_interfaces::msg::Gnss();
				message.position_ecef.x = ECEF.PosX;
				message.position_ecef.y = ECEF.PosY;
				message.position_ecef.z = ECEF.PosZ;
				message.velocity_ecef.x = ECEF.VelX;
				message.velocity_ecef.y = ECEF.VelY;
				message.velocity_ecef.z = ECEF.VelZ;
				message.timestamp = now();
				RCLCPP_INFO_STREAM(this->get_logger(), "Publishing: '" << message.position_ecef.x << "'");
				RCLCPP_INFO_STREAM(this->get_logger(), "Publishing: '" << message.position_ecef.y << "'"); 
				RCLCPP_INFO_STREAM(this->get_logger(), "Publishing: '" << message.position_ecef.z << "'"); 
				RCLCPP_INFO_STREAM(this->get_logger(), "Publishing: '" << message.velocity_ecef.x << "'"); 
				RCLCPP_INFO_STREAM(this->get_logger(), "Publishing: '" << message.velocity_ecef.y << "'"); 
				RCLCPP_INFO_STREAM(this->get_logger(), "Publishing: '" << message.velocity_ecef.z << "'"); 
				RCLCPP_INFO_STREAM(this->get_logger(), "Publishing: '" << message.timestamp.sec << "'");
				RCLCPP_INFO_STREAM(this->get_logger(), "Publishing: '" << message.timestamp.nanosec << "'");
				cout << "                        "<< endl; 
				publisher_->publish(message);				
			}
		}
		rclcpp::TimerBase::SharedPtr timer_;
		rclcpp::Publisher<autorccar_interfaces::msg::Gnss>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{ 
	std::function<void(const boost::system::error_code&, size_t)> readSerial;
	boost::asio::io_service ioService;
	auto port = boost::asio::serial_port(ioService);

	boost::system::error_code ec;
	port.open("/dev/ttyACM0", ec);

	auto opt_baud = boost::asio::serial_port_base::baud_rate(115200);
	auto opt_parity = serial_port_base::parity(serial_port_base::parity::none);
	auto opt_csize = serial_port_base::character_size(8);
	auto opt_flow = serial_port_base::flow_control(serial_port_base::flow_control::none);
	auto opt_stop = serial_port_base::stop_bits(serial_port_base::stop_bits::one);

	port.set_option(opt_baud);
	port.set_option(opt_parity);
	port.set_option(opt_csize);
	port.set_option(opt_flow);
	port.set_option(opt_stop);    

	unsigned int readBuffer[2048];

	readSerial = [&port, &readSerial, &readBuffer](const boost::system::error_code& ec, size_t bytes_transferred)
	{
		Packet_Flag = Ublox_8_DEQUEUE(readBuffer, bytes_transferred);
		switch (Packet_Flag)
   			{
       			case 1:
       				 Packet_Flag1 = 1;
            			break;
       			case 2:
       				 Packet_Flag2 = 1;
            			break;
       			case 3:
       				 Packet_Flag3 = 1;
            			break;           				
			}
		port.async_read_some(boost::asio::buffer(readBuffer, 512), readSerial);
	};	
	port.async_read_some(boost::asio::buffer(readBuffer, 512), readSerial);
	
	auto Decode_Thread = std::thread([&ioService](){ioService.run();});
	Decode_Thread.detach();

	//별도의 Thread로 시작됨
	//rcl에도 ros node multithread 관련 lib이 있음.
	//하지만 이 코드에서는 std에서 생성하는 통신 및 디코더 thread1
	// + rcl lib에서 관리하는 node pub 관련 thread2 가 실행 됨. 
	rclcpp::init(argc, argv);
	rclcpp::executors::MultiThreadedExecutor executor;
	auto IMU = std::make_shared<IMU_Pub>();
	auto GNSS = std::make_shared<GNSS_Pub>();	
	executor.add_node(IMU);
	executor.add_node(GNSS);
	executor.spin();

	rclcpp::shutdown();
}
