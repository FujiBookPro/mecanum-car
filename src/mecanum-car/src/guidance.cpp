#include <memory>
#include <string>

#include "interfaces/msg/chassis_speeds.hpp"
#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;

/**
 * A node that runs on the robot controller (eg: Raspberry PI) and sends control
 * commands to the drivetrain control node.
 *
 * Currently, this node just relays ChassisSpeeds from a teleop node, but it
 * could also be made to read from a path file, or navigate autonomously.
 */
class Guidance : public rclcpp::Node {
public:
  Guidance();
  void handle_teleop(const interfaces::msg::ChassisSpeeds &);

private:
  rclcpp::Publisher<interfaces::msg::ChassisSpeeds>::SharedPtr
      drivetrain_publisher_;
  rclcpp::Subscription<interfaces::msg::ChassisSpeeds>::SharedPtr
      teleop_subscription_;

  rclcpp::TimerBase::SharedPtr timer_;

  interfaces::msg::ChassisSpeeds latest_teleop_speeds_;
  rclcpp::Clock::SharedPtr update_clock;
  rclcpp::Time last_update;
};

Guidance::Guidance() : Node("guidance") {
  using interfaces::msg::ChassisSpeeds;

  update_clock = this->get_clock();
  last_update = update_clock->now();

  drivetrain_publisher_ =
      this->create_publisher<ChassisSpeeds>("drive_control_request", 10);

  teleop_subscription_ = this->create_subscription<ChassisSpeeds>(
      "teleop_speeds", 10,
      [this](const ChassisSpeeds &msg) { handle_teleop(msg); });

  auto timer_callback = [this]() -> void {
    if (update_clock->now() > last_update + 100ms) {
      latest_teleop_speeds_ = ChassisSpeeds();
    }

    drivetrain_publisher_->publish(latest_teleop_speeds_);
  };

  timer_ = this->create_wall_timer(50ms, timer_callback);
}

void Guidance::handle_teleop(const interfaces::msg::ChassisSpeeds &msg) {
  latest_teleop_speeds_ = msg;
  last_update = update_clock->now();

  RCLCPP_INFO_STREAM(this->get_logger(),
                     "Recieved v_x: " << msg.v_x << " v_y: " << msg.v_y
                                      << " omega: " << msg.omega);
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Guidance>());
  rclcpp::shutdown();
  return 0;
}