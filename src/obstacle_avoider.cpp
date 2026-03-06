#include "robot_control/obstacle_avoider.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

// ─────────────────────────────────────────────────────────────
// 아키텍처
//   tracking_controller ──► /cmd_vel_raw ──► ObstacleAvoider ──► /cmd_vel
//                                            ▲
//                                /scan ──────┘
//
// 로봇이 원판형이므로 이동 방향 기준으로 섹터를 설정해
// 최근접 거리를 구한 뒤 선속도/각속도를 감쇠 또는 차단한다.
//
// 거리 단계:
//   stop_dist = robot_radius + safety_margin  → 선속도 0 (긴급 정지)
//   slow_dist = stop_dist + slowdown_zone     → 선속도 비례 감쇠
// ─────────────────────────────────────────────────────────────

ObstacleAvoider::ObstacleAvoider()
: rclcpp::Node("obstacle_avoider"),
  last_scan_time_(this->now()),
  last_cmd_time_(this->now()),
  robot_radius_(this->declare_parameter<double>("robot_radius", 0.07)),
  safety_margin_(this->declare_parameter<double>("safety_margin", 0.03)),
  slowdown_zone_(this->declare_parameter<double>("slowdown_zone", 0.25)),
  forward_half_angle_deg_(this->declare_parameter<double>("forward_half_angle_deg", 35.0)),
  side_start_deg_(this->declare_parameter<double>("side_start_deg", 40.0)),
  side_end_deg_(this->declare_parameter<double>("side_end_deg", 140.0)),
  scan_timeout_sec_(this->declare_parameter<double>("scan_timeout_sec", 1.0))
{
  // 라이다 구독 (/scan — RPLIDAR C1 기본 토픽명)
  scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "scan", rclcpp::SensorDataQoS(),
    std::bind(&ObstacleAvoider::scanCallback, this, std::placeholders::_1));

  // 추적 제어기 원본 cmd 구독
  cmd_raw_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel_raw", 10,
    std::bind(&ObstacleAvoider::cmdVelRawCallback, this, std::placeholders::_1));

  // 보정된 cmd 발행
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

  // watchdog: cmd_vel_raw 가 오래 안 오면 정지 명령 발행
  watchdog_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(200),
    std::bind(&ObstacleAvoider::timerCallback, this));

  RCLCPP_INFO(this->get_logger(),
    "ObstacleAvoider ready — stop=%.2fm  slow=%.2fm  fwd_half=%.0fdeg",
    robot_radius_ + safety_margin_,
    robot_radius_ + safety_margin_ + slowdown_zone_,
    forward_half_angle_deg_);
}

// ── 라이다 콜백 ──────────────────────────────────────────────

void ObstacleAvoider::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  latest_scan_ = msg;
  last_scan_time_ = this->now();
}

// ── 원본 cmd 수신 → 장애물 보정 → 발행 ────────────────────────

void ObstacleAvoider::cmdVelRawCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  last_cmd_time_ = this->now();
  latest_cmd_raw_ = *msg;

  geometry_msgs::msg::Twist out = *msg;  // 기본: 그대로 통과

  // 라이다 데이터 없거나 오래된 경우: 그대로 통과 (라이다 미연결 상태 허용)
  if (!latest_scan_ ||
    (this->now() - last_scan_time_).seconds() > scan_timeout_sec_)
  {
    cmd_vel_pub_->publish(out);
    return;
  }

  const auto & scan = *latest_scan_;
  const float stop_dist = static_cast<float>(robot_radius_ + safety_margin_);
  const float slow_dist = static_cast<float>(robot_radius_ + safety_margin_ + slowdown_zone_);
  const float half_fwd  = static_cast<float>(forward_half_angle_deg_ * M_PI / 180.0);
  const float s_start   = static_cast<float>(side_start_deg_ * M_PI / 180.0);
  const float s_end     = static_cast<float>(side_end_deg_ * M_PI / 180.0);

  // ── 1. 전방/후방 선속도 체크 ────────────────────────────
  if (std::abs(out.linear.x) > 1e-4) {
    // 전진이면 정면(0 rad), 후진이면 후방(π rad) 섹터
    float center = (out.linear.x > 0) ? 0.0f : static_cast<float>(M_PI);
    float min_dist = getMinDistInSector(scan, center - half_fwd, center + half_fwd);

    if (min_dist < stop_dist) {
      // 긴급 정지
      out.linear.x = 0.0;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 500,
        "[Obstacle] 장애물 %.2fm — 정지 (stop=%.2fm)", min_dist, stop_dist);
    } else if (min_dist < slow_dist) {
      // 비례 감속: stop_dist → 1.0, slow_dist → 0.0 사이로 스케일
      float scale = (min_dist - stop_dist) / static_cast<float>(slowdown_zone_);
      out.linear.x *= scale;
      RCLCPP_DEBUG(get_logger(),
        "[Obstacle] 감속: dist=%.2f  scale=%.2f", min_dist, scale);
    }
  }

  // ── 2. 측면 각속도 체크 ─────────────────────────────────
  // angular.z > 0 → 왼쪽 회전 → 왼쪽(양각) 섹터 확인
  // angular.z < 0 → 오른쪽 회전 → 오른쪽(음각) 섹터 확인
  if (std::abs(out.angular.z) > 1e-4) {
    float sign = (out.angular.z > 0) ? 1.0f : -1.0f;
    float ang_a = sign * s_start;
    float ang_b = sign * s_end;
    if (ang_a > ang_b) {std::swap(ang_a, ang_b);}

    float min_dist = getMinDistInSector(scan, ang_a, ang_b);

    if (min_dist < stop_dist) {
      out.angular.z = 0.0;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 500,
        "[Obstacle] 측면 장애물 %.2fm — 회전 차단", min_dist);
    } else if (min_dist < slow_dist) {
      float scale = (min_dist - stop_dist) / static_cast<float>(slowdown_zone_);
      out.angular.z *= scale;
    }
  }

  cmd_vel_pub_->publish(out);
}

// ── watchdog 타이머 ──────────────────────────────────────────
// tracking_controller 가 멈추면 마지막 raw cmd 도 안 오므로
// 새로운 처리는 하지 않고 단순히 오래된 cmd 가 있으면 재발행.
// (tracking_controller 자체도 timeout 시 정지 명령을 보내므로 중복 안전장치)

void ObstacleAvoider::timerCallback()
{
  constexpr double CMD_TIMEOUT_SEC = 0.5;
  if ((this->now() - last_cmd_time_).seconds() > CMD_TIMEOUT_SEC) {
    // 상위 노드에서 명령이 안 왔으면 정지
    cmd_vel_pub_->publish(geometry_msgs::msg::Twist{});
  }
}

// ── 섹터 최소 거리 ───────────────────────────────────────────

float ObstacleAvoider::getMinDistInSector(
  const sensor_msgs::msg::LaserScan & scan,
  float angle_min_rad,
  float angle_max_rad) const
{
  // 각도를 [-π, π] 로 정규화
  auto norm = [](float a) -> float {
    while (a > static_cast<float>(M_PI)) {a -= 2.0f * static_cast<float>(M_PI);}
    while (a < -static_cast<float>(M_PI)) {a += 2.0f * static_cast<float>(M_PI);}
    return a;
  };
  angle_min_rad = norm(angle_min_rad);
  angle_max_rad = norm(angle_max_rad);

  float min_dist = std::numeric_limits<float>::infinity();

  for (size_t i = 0; i < scan.ranges.size(); ++i) {
    float r = scan.ranges[i];
    // 유효하지 않은 측정값 제외
    if (!std::isfinite(r) || r < scan.range_min || r > scan.range_max) {
      continue;
    }

    float angle = norm(scan.angle_min + static_cast<float>(i) * scan.angle_increment);

    // 섹터 포함 여부 (wraparound 고려)
    bool in_sector;
    if (angle_min_rad <= angle_max_rad) {
      in_sector = (angle >= angle_min_rad && angle <= angle_max_rad);
    } else {
      // 예: 후방 섹터 155° ~ -155° (180° 중심, wraparound)
      in_sector = (angle >= angle_min_rad || angle <= angle_max_rad);
    }

    if (in_sector) {
      min_dist = std::min(min_dist, r);
    }
  }

  return min_dist;
}

// ── main ─────────────────────────────────────────────────────

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObstacleAvoider>());
  rclcpp::shutdown();
  return 0;
}
