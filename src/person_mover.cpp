#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <cmath>

class PersonMover : public rclcpp::Node {
public:
  PersonMover() : rclcpp::Node("person_mover"), t_(0.0) {
    pub_ = this->create_publisher<geometry_msgs::msg::Twist>("person_cmd_vel", 10);
    timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
      std::bind(&PersonMover::onTimer, this));
    RCLCPP_INFO(this->get_logger(), "Person mover initialized (faster movement)");
  }

private:
  void onTimer() {
    // Faster circular motion: increased forward + faster yaw
    double linear = 0.3; // m/s (빠르게)
    double angular = 0.4; // rad/s (빠르게)
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = linear;
    cmd.angular.z = angular;
    pub_->publish(cmd);
    t_ += 0.1;
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  double t_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PersonMover>());
  rclcpp::shutdown();
  return 0;
}
