#include "robot_control/tracking_controller.hpp"
#include "std_msgs/msg/bool.hpp"

TrackingController::TrackingController()
  : rclcpp::Node("tracking_controller"),
    kp_linear_(0.6), ki_linear_(0.02), kd_linear_(0.08),
    kp_angular_(0.8), ki_angular_(0.03), kd_angular_(0.15),
    prev_error_x_(0.0), prev_error_y_(0.0),
    integral_error_x_(0.0), integral_error_y_(0.0),
    person_detected_(false),
    follow_mode_(true) {

  // 구독/발행 설정
  person_pos_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
    "person_position", 10,
    std::bind(&TrackingController::personPositionCallback, this, std::placeholders::_1)
  );

  follow_mode_sub_ = this->create_subscription<std_msgs::msg::Bool>(
    "/robot/follow_mode", 10,
    std::bind(&TrackingController::followModeCallback, this, std::placeholders::_1)
  );

  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel_raw", 10);

  // 검색 타이머 설정 (10Hz로 주기적으로 체크)
  search_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&TrackingController::timerCallback, this)
  );

  last_update_time_ = this->now();
  last_detection_time_ = this->now();

  RCLCPP_INFO(this->get_logger(), "Tracking Controller initialized");
}

void TrackingController::followModeCallback(const std_msgs::msg::Bool::SharedPtr msg) {
  follow_mode_ = msg->data;
  RCLCPP_INFO(this->get_logger(), "Follow mode: %s", follow_mode_ ? "ON" : "OFF");

  // 모드 OFF 시 즉시 정지
  if (!follow_mode_) {
    auto cmd = geometry_msgs::msg::Twist();
    cmd.linear.x = 0.0;
    cmd.angular.z = 0.0;
    cmd_vel_pub_->publish(cmd);
  }
}

void TrackingController::personPositionCallback(const geometry_msgs::msg::Point::SharedPtr msg) {
  // follow_mode가 꺼져 있으면 감지 무시
  if (!follow_mode_) {
    return;
  }

  float confidence = msg->z;

  // 신뢰도가 너무 낮으면 감지로 인정하지 않음
  // → last_detection_time_ 갱신 안 함 → 검색 모드(회전)가 정상 동작
  if (confidence < MIN_CONFIDENCE_) {
    return;
  }

  person_detected_ = true;
  last_detection_time_ = this->now();

  // 현재 시간 계산
  auto current_time = this->now();
  double dt = (current_time - last_update_time_).seconds();
  if (dt < 0.001) dt = 0.033;  // 약 30Hz
  last_update_time_ = current_time;

  // 오차 계산
  float error_x = msg->x;  // 좌우 오차
  float error_y = msg->y;  // 상하 오차 (거리)

  // 적분 계산
  integral_error_x_ += error_x * dt;
  integral_error_y_ += error_y * dt;

  // 적분 항의 포화 방지
  integral_error_x_ = std::max(-1.0f, std::min(1.0f, integral_error_x_));
  integral_error_y_ = std::max(-1.0f, std::min(1.0f, integral_error_y_));

  // 미분 계산
  float deriv_x = (error_x - prev_error_x_) / dt;

  // PID 제어
  float angular_vel = kp_angular_ * error_x + ki_angular_ * integral_error_x_ + kd_angular_ * deriv_x;
  // 거리 조절: error_y = 음수(공 작음/멀다) → 양수 속도(앞으로)
  //                     양수(공 큼/가까움) → 음수 속도(뒤로)
  float linear_vel = -kp_linear_ * error_y * confidence;

  // 속도 제한 (거리 유지 제어가 작동하도록 최소속도 강제 제거)
  linear_vel = std::max(-0.8f, std::min(0.8f, linear_vel));
  angular_vel = std::max(-1.5f, std::min(1.5f, angular_vel));

  // 속도 명령 발행
  auto cmd = geometry_msgs::msg::Twist();
  cmd.linear.x = -linear_vel;  // 부호 반전: 뒤로 가던 문제 해결
  cmd.angular.z = angular_vel;
  last_tracking_cmd_ = cmd;  // timerCallback 재발행용 저장
  cmd_vel_pub_->publish(cmd);

  // 상태 업데이트
  prev_error_x_ = error_x;
  prev_error_y_ = error_y;

  RCLCPP_DEBUG(this->get_logger(),
    "Person at (%.2f, %.2f) | Linear: %.2f, Angular: %.2f | Confidence: %.2f",
    error_x, error_y, linear_vel, angular_vel, confidence);
}

void TrackingController::timerCallback() {
  // follow_mode가 꺼져 있으면 회전 검색 안 함
  if (!follow_mode_) {
    return;
  }

  // 마지막 감지 이후 경과 시간 확인
  double time_since_detection = (this->now() - last_detection_time_).seconds();

  if (time_since_detection > SEARCH_TIMEOUT_) {
    // 감지 없음 → 천천히 제자리 회전
    person_detected_ = false;
    auto cmd = geometry_msgs::msg::Twist();
    cmd.linear.x = 0.0;
    cmd.angular.z = 0.2;
    cmd_vel_pub_->publish(cmd);
  } else {
    // 감지 중 → 마지막 추적 명령 재발행 (wheel_controller timeout 방지)
    cmd_vel_pub_->publish(last_tracking_cmd_);
  }
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrackingController>());
  rclcpp::shutdown();
  return 0;
}
