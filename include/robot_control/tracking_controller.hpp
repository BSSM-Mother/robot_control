#ifndef ROBOT_CONTROL__TRACKING_CONTROLLER_HPP_
#define ROBOT_CONTROL__TRACKING_CONTROLLER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "std_msgs/msg/bool.hpp"

class TrackingController : public rclcpp::Node {
public:
  TrackingController();
  
private:
  void personPositionCallback(const geometry_msgs::msg::Point::SharedPtr msg);
  void timerCallback();
  void followModeCallback(const std_msgs::msg::Bool::SharedPtr msg);
  
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr person_pos_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr follow_mode_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::TimerBase::SharedPtr search_timer_;
  
  // PID 제어 파라미터
  float kp_linear_;
  float ki_linear_;
  float kd_linear_;
  
  float kp_angular_;
  float ki_angular_;
  float kd_angular_;
  
  // 적분/미분을 위한 변수
  float prev_error_x_;
  float prev_error_y_;
  float integral_error_x_;
  float integral_error_y_;
  
  // 추적 상태
  bool person_detected_;
  bool follow_mode_;  // MQTT로 제어 (true = 따라가기 ON)
  rclcpp::Time last_update_time_;
  rclcpp::Time last_detection_time_;
  static constexpr float SEARCH_TIMEOUT_ = 2.0;  // 2초 이상 감지 안되면 회전
};

#endif  // ROBOT_CONTROL__TRACKING_CONTROLLER_HPP_
