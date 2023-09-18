// C library headers
#include <stdio.h>
#include <string.h>
//#include <iostream>
// Linux headers
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/twist.hpp"


#include <string>
#include <vector>
#include <sstream>
#include <array>

using std::placeholders::_1;
class Rover : public rclcpp::Node
{
  private:
    bool debug_f = 0;

    int serial_port;
    struct termios tty;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_full;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr vo_pose;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_cmd_vel;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subscription_follow;

    std::array<double, 3> m_position;
    std::array<double, 3> m_rotation;
//    std::array<double, 3> m_positionGoal_arr;
//    std::array<double, 3> m_rotationGoal_arr;

    void topic_callback(const std_msgs::msg::String &msg) const
    {
      RCLCPP_INFO(this->get_logger(), "Received: '%s'", msg.data.c_str());
      std::string command = msg.data.c_str();
      std::string prefix = {char(45), char(230)};
      command.insert(0, prefix);
      command += '\n';
      RCLCPP_INFO(this->get_logger(), "Sending to ttyUSB0: '%s' with size of ", command.c_str());
      std::cout << sizeof(command.c_str()) << '\n';
//      msg.data += std::string("\n");
      if(!debug_f)	write(serial_port, command.c_str(), command.size());
    }
    void topic_callback_with_parsing(const std_msgs::msg::String &msg)
    {
      RCLCPP_INFO(this->get_logger(), "Received: '%s'", msg.data.c_str());
      std::string command = msg.data.c_str();
      std::string prefix = {char(45), char(230)};
      std::vector<int> values;
      std::stringstream ss(command);
      int value;
      while(ss >> value)
      {
        values.push_back(value);
        if(ss.peek() == ' ')	ss.ignore();
	if(values.size() == 4)	break;
      }
      std::string duration_str = command.substr(command.find_last_of(' ') + 1, command.size() - 2);
      std::vector<double> duration_vctr;
      duration_vctr.push_back(std::stod(duration_str));
//	std::cout << "command after ss " << command << '\n';
      command.clear();
      command += prefix;
      for(int it : values)	command.push_back(char(it));
      std::string cursedFuckingShit = doublesToString(duration_vctr);
      
      
//      command.push_back('\n');
      std::cout << "bytes: ";
      for(uint8_t it : values)	std::cout << int(it) << ' ';
      std::cout << '\n';
      RCLCPP_INFO(this->get_logger(), "Sending to ttyUSB0: '%s' with size of %s", command.c_str(), std::to_string(command.size()).c_str());
      //if(!debug_f)	write(serial_port, command.c_str(), command.size());
      
      sendString(command, cursedFuckingShit);
    }
    void topic_callback_getRPY(const geometry_msgs::msg::PoseStamped &msg)
    {
      RCLCPP_INFO(this->get_logger(), "[Pose] Received something from slam node");

      std::array<double, 4> q;
      
      q[0] = msg.pose.orientation.x;
      q[1] = msg.pose.orientation.y;
      q[2] = msg.pose.orientation.z;
      q[3] = msg.pose.orientation.w;
      
      m_rotation[0] = std::atan2(2.0 * (q[0] * q[1] + q[3] * q[2]), pow(q[3], 2) - pow(q[0], 2) - pow(q[1], 2) + pow(q[2], 2));//roll //not sure if it's working
      m_rotation[1] = std::asin(-2.0 * (q[0] * q[2] - q[3] * q[1]));//pitch
      m_rotation[2] = std::atan2(2.0 * (q[0] * q[1] + q[3] * q[2]), pow(q[3], 2) + pow(q[0], 2) - pow(q[1], 2) - pow(q[2], 2));//yaw

      m_position[0] = msg.pose.position.x;
      m_position[1] = msg.pose.position.y;
      m_position[2] = msg.pose.position.z;
      
      std::array<double, 4> data;
      data[0] = m_rotation[2];
      for(uint8_t i = 0; i < 3; ++i)
      {
	data[i + 1] = m_position[i];
      }
      std::string command_str;
      for(uint8_t i = 0; i < 4; ++i)
      {
	float num = static_cast<float>(data[i]);
	char arr[4];
	memcpy(arr, &num, 4);
	command_str += std::string(arr, 4);
      }
      for(double it : data)	std::cout << it << '\n';
      std::string prefix = {char(45), char(231)};
      
      sendString(prefix, command_str);
    }
    void cmd_vel_callback(const geometry_msgs::msg::Twist &msg)
    {
      RCLCPP_INFO(this->get_logger(), "Received cmd_vel data: \n");
      std::vector<double> double_vctr;
      double_vctr.push_back(msg.linear.x);
      double_vctr.push_back(msg.angular.z);
      for(double it : double_vctr)	std::cout << it << '\n';
      std::string prefix = {char(45), char(232)};
      sendString(prefix, doublesToString(double_vctr));
      
    }
    void follow_callback(const std_msgs::msg::Bool &msg)  
    {
      char byte = (!msg.data) ? 0 : 1;
      std::string prefix = {char(45), char(233)};
      std::string data;
      data += byte;
      sendString(prefix, data);
    }
    std::string doublesToString(const std::vector<double> &double_vctr) const
    {
      std::string command_str;
      for(uint8_t i = 0; i < double_vctr.size(); ++i)
      {
	float float_ = static_cast<float>(double_vctr[i]);
	char buff[4];
	memcpy(buff, &float_, 4);
	command_str += std::string(buff, 4);
      }
      return command_str;
    }
    void sendString(const std::string &prefix, const std::string &data)
    {
      std::string command = prefix + data + '\n';
      if(!debug_f)	write(serial_port, command.c_str(), command.size());
    }

  public:
    Rover() : Node("rover")
    {
      subscription_ = this->create_subscription<std_msgs::msg::String>("navigation_topic", 10, std::bind(&Rover::topic_callback, this, _1));
      subscription_full = this->create_subscription<std_msgs::msg::String>("hr_topic", 10, std::bind(&Rover::topic_callback_with_parsing, this, _1));
//      vo_pose = this->create_subscription<geometry_msgs::msg::PoseStamped>("/visual_slam/tracking/vo_pose", 10, std::bind(&Rover::topic_callback_getRPY, this, _1));
      subscription_cmd_vel = this->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", 10, std::bind(&Rover::cmd_vel_callback, this, _1));
      subscription_follow = this->create_subscription<std_msgs::msg::Bool>("/follow_flag", 10, std::bind(&Rover::follow_callback, this, _1));
      this->declare_parameter("debug", "Nope");
      std::string debugStatus_str= this->get_parameter("debug").as_string();
      debug_f = ((debugStatus_str == "Yez") ? 1 : 0);
      if(debug_f)	RCLCPP_INFO(this->get_logger(), "[!] Node spinnig in debug mode, serial is unavailable");

      this->declare_parameter("tty", "/dev/ttyUSB0");
      std::string tty_str = this->get_parameter("tty").as_string();
      serial_port = open(tty_str.c_str(), O_RDWR);
      if(serial_port < 0 && !debug_f)
      {
	RCLCPP_INFO(this->get_logger(), "[-] Can't open serial port, exiting with errno '%i'", errno);
	rclcpp::shutdown();
      }
      else	RCLCPP_INFO(this->get_logger(), "[+]");

      if(tcgetattr(serial_port, &tty) != 0 && !debug_f) 
      {
	RCLCPP_INFO(this->get_logger(), "[-] Tcgetattr error, exiting with errno '%i'", errno);
	rclcpp::shutdown();
      }
      else	RCLCPP_INFO(this->get_logger(), "[+]");

      tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
      tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
      tty.c_cflag &= ~CSIZE; // Clear all bits that set the data size 
      tty.c_cflag |= CS8; // 8 bits per byte (most common)
      tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
      tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

      tty.c_lflag &= ~ICANON;
      tty.c_lflag &= ~ECHO; // Disable echo
      tty.c_lflag &= ~ECHOE; // Disable erasure
      tty.c_lflag &= ~ECHONL; // Disable new-line echo
      tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
      tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
      tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes

      tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
      tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
  // tty.c_oflag &= ~OXTABS; // Prevent conversion of tabs to spaces (NOT PRESENT ON LINUX)
  // tty.c_oflag &= ~ONOEOT; // Prevent removal of C-d chars (0x004) in output (NOT PRESENT ON LINUX)

      tty.c_cc[VTIME] = 10;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
      tty.c_cc[VMIN] = 0;

  // Set in/out baud rate to be 9600
      cfsetispeed(&tty, B9600);
      cfsetospeed(&tty, B9600);

  // Save tty settings, also checking for error
      if(tcsetattr(serial_port, TCSANOW, &tty) != 0 && !debug_f) 
      {
	RCLCPP_INFO(this->get_logger(), "[-] Tcgetattr error, exiting with errno '%i'", errno);
	rclcpp::shutdown();
      }
      else	RCLCPP_INFO(this->get_logger(), "[+]");

    }
};  
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Rover>());
  rclcpp::shutdown();
  

  return 0;
}
