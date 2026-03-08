# robot_control - 고수준 제어 로직

**로봇의 자율 제어 및 행동 로직을 담당하는 패키지**

## 📌 개요

사람 추적, 장애물 회피, 움직임 감지 등 고수준의 제어 로직을 구현합니다. 비전 데이터와 센서 정보를 기반으로 로봇의 속도 명령을 생성합니다.

## 🎯 주요 기능

- ✅ **사람 추적** (Tracking Controller): PID 제어로 사람을 따라다님
- ✅ **장애물 회피** (Obstacle Avoider): LIDAR 데이터로 충돌 방지
- ✅ **움직임 응답** (Person Mover): 사람의 움직임에 로봇 반응
- ✅ **테스트 유틸** (Ball Randomizer): 시뮬레이션용

## 🚀 포함된 노드

### 1. **tracking_controller** - 사람 추적 제어
```
입력: person_position (geometry_msgs/Point)
출력: cmd_vel_raw (geometry_msgs/Twist)
```

**기능**:
- PID 제어로 부드러운 추적
- 신뢰도 기반 속도 조절
- 일정 시간 미감지 시 제자리 회전 (검색 모드)

**파라미터**:
```yaml
tracking_controller:
  ros__parameters:
    kp_linear: 0.6         # 선속도 비례 이득
    ki_linear: 0.02        # 선속도 적분 이득
    kd_linear: 0.08        # 선속도 미분 이득
    kp_angular: 0.8        # 각속도 비례 이득
    ki_angular: 0.03       # 각속도 적분 이득
    kd_angular: 0.15       # 각속도 미분 이득
    search_timeout: 5.0    # 검색 시작 시간 (초)
```

### 2. **obstacle_avoider** - 장애물 회피
```
입력: cmd_vel_raw (tracking_controller), scan (LIDAR)
출력: cmd_vel (geometry_msgs/Twist → robot_base)
```

**기능**:
- `cmd_vel_raw`를 받아 LIDAR 데이터와 비교
- 전방 장애물 거리에 따라 속도 보정 (감속/정지)
- 보정된 명령을 `cmd_vel`로 발행 (필터 역할)

### 3. **person_mover** - 움직임 응답 제어
```
입력: person_position (geometry_msgs/Point)
출력: cmd_vel_raw (geometry_msgs/Twist)
```

**기능**:
- 사람의 움직임에 따라 로봇 이동
- 관성 기반 제어로 자연스러운 움직임

### 4. **ball_randomizer** - 테스트 유틸
```
출력: ball_position (geometry_msgs/Point)
```

**기능**:
- 시뮬레이션용 공 위치 랜덤 생성
- 추적 제어 테스트 시 사용

## 📡 토픽 아키텍처

```
┌──────────────────┐
│ robot_perception │ ← 카메라에서 사람 감지
│ person_detector  │
└────────┬─────────┘
         │ person_position (Point)
         ▼
┌──────────────────────────────────────┐
│    robot_control (제어 로직)          │
│  ┌────────────────────────────────┐  │
│  │ tracking_controller (추적)      │  │
│  │ obstacle_avoider (회피 필터)   │  │
│  │ person_mover (움직임 응답)     │  │
│  └────────────────────────────────┘  │
└────────┬─────────────────────────────┘
         │ cmd_vel (보정된 속도 명령)
         ▼
┌──────────────────┐
│   robot_base     │
│ wheel_controller │ ← 모터 제어
└──────────────────┘
```

## ⚙️ 구독/발행 토픽

### 구독 토픽
| 토픽 | 타입 | 출처 | 설명 |
|------|------|------|------|
| `person_position` | `geometry_msgs/Point` | robot_perception | 사람 위치 좌표 |
| `scan` | `sensor_msgs/LaserScan` | sllidar_ros2 | LIDAR 거리 데이터 |
| `/robot/follow_mode` | `std_msgs/Bool` | 외부 | 추적 모드 ON/OFF |

### 발행 토픽
| 토픽 | 타입 | 대상 | 설명 |
|------|------|------|------|
| `cmd_vel` | `geometry_msgs/Twist` | robot_base | 보정된 속도 명령 (장애물 회피 적용) |

## 💾 빌드

```bash
colcon build --packages-select robot_control
```

## 🚀 실행

개별 노드 실행:
```bash
# 사람 추적
ros2 run robot_control tracking_controller

# 장애물 회피
ros2 run robot_control obstacle_avoider

# 움직임 응답
ros2 run robot_control person_mover
```

또는 런치 파일로 전체 실행:
```bash
ros2 launch robot_launch robot.launch.py
```

## 📊 속도 명령 범위

```
linear.x (선속도): -0.8 ~ 0.8 m/s
- 음수: 후진
- 0: 정지
- 양수: 전진

angular.z (각속도): -1.5 ~ 1.5 rad/s
- 음수: 시계방향 회전
- 0: 직진
- 양수: 반시계방향 회전
```

## 🔧 튜닝 가이드

### PID 파라미터 조정

**추적이 너무 느린 경우**:
- `kp_linear`, `kp_angular` 증가
- `ki` 값 미세 조정

**추적이 떨리거나 불안정한 경우**:
- `kd` 값 증가 (제동 효과)
- `kp` 값 감소

**과도한 오버슛**:
- `kd` 값 증가
- `kp` 값 감소

## 📝 소스 파일

```
src/robot_control/src/
├── tracking_controller.cpp    # 사람 추적
├── obstacle_avoider.cpp       # 장애물 회피
├── person_mover.cpp           # 움직임 응답
└── ball_randomizer.cpp        # 테스트 유틸
```

## 🔗 의존성 및 연관 패키지

- **입력 출처**:
  - `robot_perception` (사람 위치)
  - `sllidar_ros2` (LIDAR 데이터)
- **출력 대상**: `robot_base` (모터 제어)
- **의존성**: rclcpp, geometry_msgs, sensor_msgs, std_msgs, tf2

## ⚠️ 주의사항

- `follow_mode` 토픽이 False일 때는 추적 작동 안 함 (안전 모드)
- 센서 데이터 손실 시 자동 정지
- 급격한 속도 변화는 모터 손상 야기 가능 (제한됨)
