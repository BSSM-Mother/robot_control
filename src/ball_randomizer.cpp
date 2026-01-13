#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <random>
#include <cmath>

class BallRandomizer : public rclcpp::Node {
public:
  BallRandomizer() : Node("ball_randomizer") {
    // person_cmd_vel 퍼블리셔
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("person_cmd_vel", 10);
    
    // 타이머 (100ms마다 위치/속도 업데이트)
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&BallRandomizer::timer_callback, this)
    );
    
    // 랜덤 생성기
    rng_.seed(std::chrono::system_clock::now().time_since_epoch().count());
    new_direction();
    
    RCLCPP_INFO(this->get_logger(), "⚽ Ball randomizer started - moving ball randomly");
  }

private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::mt19937 rng_;
  int time_counter_ = 0;
  double current_linear_ = 0.3;
  double current_angular_ = 0.0;

  void new_direction() {
    std::uniform_real_distribution<double> linear_dist(-0.6, 0.6);
    std::uniform_real_distribution<double> angular_dist(-1.2, 1.2);
    
    current_linear_ = linear_dist(rng_);
    current_angular_ = angular_dist(rng_);
    
    RCLCPP_INFO(this->get_logger(), "🎯 Ball new direction: linear=%.2f, angular=%.2f", 
                current_linear_, current_angular_);
  }

  void timer_callback() {
    auto cmd = geometry_msgs::msg::Twist();
    
    time_counter_++;
    
    // 1초마다 방향 변경 (10 * 100ms = 1000ms)
    if (time_counter_ >= 10) {
      time_counter_ = 0;
      new_direction();
    }
    
    cmd.linear.x = current_linear_;
    cmd.angular.z = current_angular_;
    cmd_vel_pub_->publish(cmd);
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BallRandomizer>());
  rclcpp::shutdown();
  return 0;
}
