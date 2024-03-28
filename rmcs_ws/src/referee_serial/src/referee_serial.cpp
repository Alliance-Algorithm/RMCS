#include "geometry_msgs/msg/vector3.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/byte.hpp"
#include "std_msgs/msg/float32.hpp"
#include <cstdint>
#include <iostream>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <serial/serial.h>
#include <std_msgs/msg/detail/bool__struct.hpp>
#include <std_msgs/msg/detail/float32_multi_array__struct.hpp>
#include <string>
/*
 * 串口数据格式
 * 帧头(5字节) + cmd id(2字节) + 数据(n字节) + CRC16(2字节)
 * 帧头: SOF(1字节) + 数据长度(2字节) + 包序号(1字节) + CRC8(1字节)
 * 数据: data(n字节)
 * 帧尾: CRC16(2字节)
 */

typedef struct frame_header_t
{
	uint16_t data_lenght;
	uint16_t notcare;
	uint16_t cmdid;
} FrameHeader;

// 串口初始化
std::unique_ptr<serial::Serial> referee_serial = std::make_unique<serial::Serial>("/dev/sentry_referee_serial", 115200U, serial::Timeout::simpleTimeout(50U),
                   serial::eightbits, serial::parity_none, serial::stopbits_one,
                   serial::flowcontrol_none);

class RefereeTransport : public rclcpp::Node
{
public:
	RefereeTransport(std::string name, bool red) : rclcpp::Node(name)
	{
		isred = red;
		// gain_point_pub_ = this->create_publisher<std_msgs::msg::Bool>(
		//     header + "gain_point_enable", 10);
		lasted_time_pub_ = this->create_publisher<std_msgs::msg::Float32>(
			header + "lasted_time", 10);
		hp_pub_ = this->create_publisher<std_msgs::msg::Float32>(header + "hp", 10);
		bullet_count_pub_ = this->create_publisher<std_msgs::msg::Float32>(
			header + "bullet_count", 10);
		attckid =
			this->create_publisher<std_msgs::msg::Byte>("referee/attack_id", 10);
	};
	void Tranport(const uint16_t cmdid, const char *data)
	{
		std::cout << std::hex << (cmdid) << "\n";
		switch (cmdid)
		{
		case 0x0001:
		{
			auto msg = std_msgs::msg::Float32();
			if ((data[0] & 0xf0) == 0x40)
			{
				msg.data = 0;
			}
			else
			{
				msg.data = (float)(*(uint16_t *)(&data[1]));
			}

			lasted_time_pub_->publish(msg);
			break;
		}
		case 0x0201:
		{
			auto msg = std_msgs::msg::Float32();
			msg.data = (float)(*(uint16_t *)&data[2]);
			hp_pub_->publish(msg);
			break;
		}
		case 0x0207:
		{
			bullet_count_++;
			auto msg = std_msgs::msg::Float32();
			msg.data = (float)bullet_count_;
			hp_pub_->publish(msg);
			break;
		}
		case 0x0206:
		{
			auto msg = std_msgs::msg::Byte();
			msg.data = data[0] & 0x0f;
			if ((data[0] & 0x0f) != 0)
				attckid->publish(msg);
		}
		default:
			break;
		}
	}

private:
	const std::string header = "/watcher/decision_maker/rmul/";
	//   rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr
	//       gain_point_pub_; // I do not findout how
	rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr lasted_time_pub_;
	rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr hp_pub_;
	rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr bullet_count_pub_;
	rclcpp::Publisher<std_msgs::msg::Byte>::SharedPtr attckid;
	bool isred;
	int bullet_count_ = 0;

	//   rclcpp::Publisher<ge>::SharedPtr total_resume_pub_;
};

// // 串口发送数据
int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	std::shared_ptr<RefereeTransport> referee =
		std::make_shared<RefereeTransport>("referee_transport", argv[0]);

	

	while (true)
	{
		try
		{
			std::string head = referee_serial->read(1);
			while ((uint8_t)head[0] != 0xA5)
			{
				head = referee_serial->read();
			}

			std::string temp = referee_serial->read(sizeof(frame_header_t));
			std::cout << temp << std::endl;
			std::string data = referee_serial->read(((FrameHeader *)(temp.c_str()))->data_lenght + 2ul);
			// std::cout << std::dec << temp << "\n";
			referee->Tranport(((FrameHeader *)(temp.c_str()))->cmdid, data.c_str());
		}
		catch (const std::exception &e)
		{
			try {
			referee_serial = std::make_unique<serial::Serial>("/dev/sentry_referee_serial", 115200U, serial::Timeout::simpleTimeout(50U),
                   serial::eightbits, serial::parity_none, serial::stopbits_one,
                   serial::flowcontrol_none);
			} catch (const serial::IOException &se) {
				std::cerr << se.what() << std::endl;
				continue;
			}
			
		}
	}
	return 0;
}