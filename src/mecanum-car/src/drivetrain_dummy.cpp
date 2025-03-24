#include "interfaces/msg/chassis_speeds.hpp"
#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;

/**
 * A very basic simulated drivetrain to confirm that other parts of the code
 * work. Prints a position estimate taken by integrating the commanded velocity.
 */
class DummyDrivetrain : public rclcpp::Node {
public:
  DummyDrivetrain();

private:
  void handle_timer();

  rclcpp::Subscription<interfaces::msg::ChassisSpeeds>::SharedPtr
      guidance_subscription_;

  rclcpp::TimerBase::SharedPtr timer_;

  interfaces::msg::ChassisSpeeds lastest_speeds_;
  double x_{0.0};
  double y_{0.0};
  double theta_{0.0};

  constexpr const static std::chrono::duration<long, std::milli> DELTA_TIME =
      20ms;
};

DummyDrivetrain::DummyDrivetrain() : Node("drivetrain") {
  using interfaces::msg::ChassisSpeeds;

  auto guidance_callback = [this](const ChassisSpeeds &msg) {
    lastest_speeds_ = msg;
  };

  guidance_subscription_ = this->create_subscription<ChassisSpeeds>(
      "drive_control_request", 10, guidance_callback);

  timer_ =
      this->create_wall_timer(DELTA_TIME, [this] { this->handle_timer(); });
}

void DummyDrivetrain::handle_timer() {
  double dt_seconds = static_cast<double>(DELTA_TIME.count()) / 1000.0;

  x_ += lastest_speeds_.v_x * dt_seconds;
  y_ += lastest_speeds_.v_y * dt_seconds;
  theta_ += lastest_speeds_.omega * dt_seconds;

  RCLCPP_INFO_STREAM(this->get_logger(),
                     "Position: " << x_ << ", " << y_
                                  << " Rotation: " << theta_);
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DummyDrivetrain>());
  rclcpp::shutdown();
  return 0;
}