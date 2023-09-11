// C library headers
#include <stdio.h>
#include <string.h>

// Linux headers
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
using std::placeholders::_1;
class Rover : public rclcpp::Node
{
  private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    void topic_callback(const std_msgs::msg::String &msg) const
    {
      RCLCPP_INFO(this->get_logger(), "Reeived: '%s'", msg.data.c_str());
      write(serial_port, msg.data.c_str(), sizeof(msg.data.c_str()));
    }
    int serial_port;
    struct termios tty;
  public:
    Rover() : Node("rover")
    {
      serial_port = open("/dev/ttyUSB0", O_RDWR);
      if(serial_port < 0)
      {
	RCLCPP_INFO(this->get_logger(), "[-] Can't open serial port, exiting with errno '%i'", errno);
	rclcpp::shutdown();
      }
      else	RCLCPP_INFO(this->get_logger(), "[+]");

      if(tcgetattr(serial_port, &tty) != 0) 
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
      if (tcsetattr(serial_port, TCSANOW, &tty) != 0) 
      {
	RCLCPP_INFO(this->get_logger(), "[-] Tcgetattr error, exiting with errno '%i'", errno);
	rclcpp::shutdown();
      }
      else	RCLCPP_INFO(this->get_logger(), "[+]");

      subscription_ = this->create_subscription<std_msgs::msg::String>("navigation_topic", 10, std::bind(&Rover::topic_callback, this, _1));
    }
};  
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Rover>());
  rclcpp::shutdown();
  

  return 0;
}
