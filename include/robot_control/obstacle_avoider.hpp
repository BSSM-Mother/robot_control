#ifndef ROBOT_CONTROL__OBSTACLE_AVOIDER_HPP_
#define ROBOT_CONTROL__OBSTACLE_AVOIDER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

// ─────────────────────────────────────────────────────────────
// ObstacleAvoider
//
// tracking_controller  ──► cmd_vel_raw ──► [ObstacleAvoider] ──► cmd_vel
//                                        ▲
//                          /scan (LiDAR) ┘
//
// 로봇은 원판 형태.
// 이동 방향 섹터의 최근접 거리로 선속도/각속도를 보정한다.
// ─────────────────────────────────────────────────────────────

class ObstacleAvoider : public rclcpp::Node
{
public:
  ObstacleAvoider();

private:
  // 콜백
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void cmdVelRawCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void timerCallback();

  // 지정 각도 범위(rad) 내 최소 거리 반환. 유효하지 않은 값(inf/nan/0) 제외.
  float getMinDistInSector(
    const sensor_msgs::msg::LaserScan & scan,
    float angle_min_rad,
    float angle_max_rad) const;

  // 0~2π 범위로 정규화
  float normalizeAngle(float angle) const;

  // 라이다 데이터
  sensor_msgs::msg::LaserScan::SharedPtr latest_scan_;
  rclcpp::Time last_scan_time_;

  // 수신된 원본 cmd
  geometry_msgs::msg::Twist latest_cmd_raw_;
  rclcpp::Time last_cmd_time_;

  // ROS
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_raw_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;

  // ── 파라미터 ──
  // 로봇 반지름 (m)
  double robot_radius_;
  // 로봇 외곽에서 얼마나 더 여유를 둘지 (m)
  double safety_margin_;
  // stop_dist  = robot_radius_ + safety_margin_      → 이 안쪽이면 선속도 0
  // slow_dist  = stop_dist + slowdown_zone_          → 이 안쪽이면 선속도 감쇠
  double slowdown_zone_;

  // 전방/후방 확인 각도 (±deg, 전체 원호 = 2×deg)
  double forward_half_angle_deg_;
  // 측면 확인 각도 범위: 양쪽 side_start_deg ~ side_end_deg
  double side_start_deg_;
  double side_end_deg_;

  // 라이다 데이터가 이 시간(초) 이상 안 오면 원본 cmd 그대로 통과
  double scan_timeout_sec_;
};

#endif  // ROBOT_CONTROL__OBSTACLE_AVOIDER_HPP_
