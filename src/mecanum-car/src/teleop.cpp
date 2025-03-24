#include "interfaces/msg/chassis_speeds.hpp"
#include <csignal>
#include <rclcpp/rclcpp.hpp>
#include <termios.h>
#include <unistd.h>

using namespace std::chrono_literals;

/**
 * A node that runs on the operator controller (eg: my laptop) and sends key
 * commands from the terminal to the guidance node as ChassisSpeeds messages.
 *
 * Right now, the ChassisSpeeds values are not scaled to any real velocity, and
 * cannot be diagonal.
 */
class Teleop : rclcpp::Node {
public:
  Teleop();
  void keyboard_loop();

private:
  rclcpp::Publisher<interfaces::msg::ChassisSpeeds>::SharedPtr output_publisher;
};

// TODO: Deal with termios in a separate class
int fd = 0;
struct termios cooked, raw;

Teleop::Teleop() : rclcpp::Node("teleop") {
  output_publisher = this->create_publisher<interfaces::msg::ChassisSpeeds>(
      "teleop_speeds", 10);
}

void Teleop::keyboard_loop() {
  char c;
  bool should_resend = false;
  interfaces::msg::ChassisSpeeds speeds;

  // TODO: Deal with termios in a separate class
  tcgetattr(fd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &= ~(ICANON | ECHO);
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(fd, TCSANOW, &raw);

  RCLCPP_INFO(this->get_logger(), "Reading from keyboard");

  while (true) {
    if (::read(fd, &c, 1) < 0) {
      RCLCPP_ERROR(this->get_logger(), "read error");
      exit(-1);
    }

    switch (c) {
    case 'w':
      RCLCPP_INFO(this->get_logger(), "forward");
      speeds.v_x = 1.0;
      should_resend = true;
      break;
    case 'a':
      RCLCPP_INFO(this->get_logger(), "left");
      speeds.v_y = -1.0;
      should_resend = true;
      break;
    case 's':
      RCLCPP_INFO(this->get_logger(), "backward");
      speeds.v_x = -1.0;
      should_resend = true;
      break;
    case 'd':
      RCLCPP_INFO(this->get_logger(), "right");
      speeds.v_y = 1.0;
      should_resend = true;
      break;
    }

    if (should_resend) {
      output_publisher->publish(speeds);

      should_resend = false;
      speeds.v_x = 0.0;
      speeds.v_y = 0.0;
    }
  }
}

void quit(int sig) {
  // In order to reset the terminal correctly, we must handle interrupts on
  // our own.
  (void)sig;

  puts("Now exiting");

  tcsetattr(fd, TCSANOW, &cooked);
  rclcpp::shutdown();
  exit(0);
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  Teleop teleop;

  signal(SIGINT, quit);

  teleop.keyboard_loop();

  return 0;
}