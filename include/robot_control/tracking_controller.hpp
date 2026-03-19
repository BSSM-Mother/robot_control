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
  geometry_msgs::msg::Twist last_tracking_cmd_;   // 마지막 추적 명령 (timerCallback에서 재발행)
  static constexpr float SEARCH_TIMEOUT_ = 5.0f;  // 추적 중 5초 감지 없으면 검색 모드
  static constexpr float MIN_CONFIDENCE_ = 0.1f;  // 이 미만이면 감지 무시 (conf_threshold와 동일)

  // 검색 모드: ROTATE → OBSERVE → ROTATE ...
  //   ROTATE  : 회전
  //   OBSERVE : 정지 후 감지 대기 — 감지되면 즉시 TRACK, 타임아웃 시 다시 ROTATE
  enum class SearchPhase { ROTATE, OBSERVE };
  SearchPhase  search_phase_;
  rclcpp::Time search_phase_start_;
  bool         was_searching_;                       // 추적→검색 전환 감지용

  static constexpr float SEARCH_ANGULAR_    = 0.5f;  // 검색 회전 각속도 (rad/s)
  static constexpr float SEARCH_ROTATE_SEC_ = 0.5f;  // 회전 시간 [s] — 실측 후 튜닝
  static constexpr float OBSERVE_SEC_       = 2.5f;  // 감지 대기 시간 [s]
  static constexpr float SETTLE_TIME_       = 0.3f;  // 회전 직후 안정화 (감지 무시) [s]
  rclcpp::Time ignore_detection_until_;
};

#endif  // ROBOT_CONTROL__TRACKING_CONTROLLER_HPP_
