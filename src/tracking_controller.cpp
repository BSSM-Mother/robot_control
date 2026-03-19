#include "robot_control/tracking_controller.hpp"
#include "std_msgs/msg/bool.hpp"

TrackingController::TrackingController()
  : rclcpp::Node("tracking_controller"),
    kp_linear_(0.6), ki_linear_(0.02), kd_linear_(0.08),
    kp_angular_(0.5), ki_angular_(0.01), kd_angular_(0.05),
    prev_error_x_(0.0), prev_error_y_(0.0),
    integral_error_x_(0.0), integral_error_y_(0.0),
    person_detected_(false),
    follow_mode_(true),
    search_phase_(SearchPhase::ROTATE) {

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

  // 타이머 설정 (30Hz — 감지→명령 딜레이 최소화)
  search_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(33),
    std::bind(&TrackingController::timerCallback, this)
  );

  last_update_time_    = this->now();
  last_detection_time_ = this->now();
  search_phase_start_  = this->now();

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

  person_detected_    = true;
  last_detection_time_ = this->now();
  // 다음에 검색 모드 진입 시 ROTATE부터 시작하도록 리셋
  search_phase_       = SearchPhase::ROTATE;
  search_phase_start_ = this->now();

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
  linear_vel = std::max(-0.5f, std::min(0.5f, linear_vel));
  angular_vel = std::max(-0.6f, std::min(0.6f, angular_vel));

  // 상태만 업데이트 (발행은 timerCallback에서만)
  auto cmd = geometry_msgs::msg::Twist();
  cmd.linear.x = linear_vel;    // ey<0(멀다)→앞으로, ey>0(가깝다)→뒤로
  cmd.angular.z = -angular_vel; // ex>0(오른쪽)→오른 회전, ex<0(왼쪽)→왼 회전
  last_tracking_cmd_ = cmd;

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
    person_detected_ = false;

    const auto   now          = this->now();
    const double phase_elapsed = (now - search_phase_start_).seconds();
    auto cmd = geometry_msgs::msg::Twist();

    if (search_phase_ == SearchPhase::ROTATE) {
      if (phase_elapsed >= SEARCH_ROTATE_SEC_) {
        // 회전 완료 → 대기 페이즈로 전환
        search_phase_      = SearchPhase::WAIT;
        search_phase_start_ = now;
        // 정지 명령 (cmd 기본값 0)
        RCLCPP_INFO(this->get_logger(), "[SEARCH] 대기 시작 (%.2fs 회전 완료)", phase_elapsed);
      } else {
        cmd.angular.z = SEARCH_ANGULAR_;
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
          "[SEARCH] 회전 중 %.2f/%.2fs", phase_elapsed, SEARCH_ROTATE_SEC_);
      }
    } else {  // WAIT
      if (phase_elapsed >= SEARCH_WAIT_SEC_) {
        // 대기 완료 → 회전 페이즈로 전환
        search_phase_      = SearchPhase::ROTATE;
        search_phase_start_ = now;
        cmd.angular.z      = SEARCH_ANGULAR_;
        RCLCPP_INFO(this->get_logger(), "[SEARCH] 회전 시작");
      }
      // else: 대기 중 → cmd 그대로 0 (정지)
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
        "[SEARCH] 대기 중 %.2f/%.2fs", phase_elapsed, SEARCH_WAIT_SEC_);
    }

    cmd_vel_pub_->publish(cmd);
  } else {
    cmd_vel_pub_->publish(last_tracking_cmd_);
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
      "[TRACK] lin=%.2f ang=%.2f (%.1fs ago)",
      last_tracking_cmd_.linear.x, last_tracking_cmd_.angular.z, time_since_detection);
  }
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrackingController>());
  rclcpp::shutdown();
  return 0;
}
