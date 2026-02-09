# vertical_action

라인 입구 출발 위치 기록 및 시나리오 제어용 ROS2 패키지.

## 코드1: 라인 위치 기록 (record_line_positions_node)

로봇을 수동으로 각 라인 입구 출발 지점에 두고, 현재 위치·방향을 YAML에 1번부터 저장한다.  
저장 형식은 Nav2 Goal과 동일한 `geometry_msgs/PoseStamped` (position + orientation quaternion).

### 실행

```bash
ros2 run vertical_action record_line_positions_node
```

- **k** : 현재 pose를 다음 번호(1, 2, 3, …)로 저장
- **s** 또는 **Ctrl+C** : 종료

### 저장 위치

- `src/vertical_action/data/line_positions.yaml` (소스 패키지 기준)

### 파라미터

| 파라미터      | 기본값                     | 설명                    |
|---------------|----------------------------|-------------------------|
| `pose_topic`  | `/liorf/mapping/odometry`  | Pose 구독 토픽          |
| `pose_type`   | `odom`                     | `odom` / `amcl` / `pose_stamped` |

### 토픽

- 구독: `pose_topic` (Odometry / PoseWithCovarianceStamped / PoseStamped)
- 구독: `/record_line_position` (`std_msgs/Empty`) — 한 번 publish할 때마다 현재 위치 저장 (터미널 없이 사용 시)

### YAML 형식 예시

```yaml
line_1:
  frame_id: "odom"
  position: { x: -0.21, y: -0.02, z: 0.0 }
  orientation: { x: 0.0, y: 0.02, z: -0.005, w: -0.999 }
line_2:
  ...
```

---

## 코드3: 수동 액션 명령 (manual_action_command_node)

사람이 육안으로 확인한 뒤 `/action` 또는 `/action_check`로 수동 명령을 보낸다.

### 실행

**대화형** (터미널에서 키 입력):

```bash
ros2 run vertical_action manual_action_command_node
```

- **n** : `/action`에 `entering_next` (다음 단계로)
- **e** : `/action`에 `entering_end` (시나리오 종료)
- **c** : `/action_check`에 `entering_check` (진입 확인 수신 알림)
- **f** : `/action_check`에 `goal_finish` (목표 완료 알림)
- **r** : `/action_check`에 `goal_return_finish` (복귀 완료 알림)
- **q** 또는 **Ctrl+C** : 노드 종료

**한 번만 전송 후 종료**:

```bash
ros2 run vertical_action manual_action_command_node entering_next
ros2 run vertical_action manual_action_command_node entering_end
ros2 run vertical_action manual_action_command_node entering_check
ros2 run vertical_action manual_action_command_node goal_finish
ros2 run vertical_action manual_action_command_node goal_return_finish
```

### 토픽

- 발행: `/action` (`std_msgs/String`) — `entering_next`, `entering_end`
- 발행: `/action_check` (`std_msgs/String`) — `entering_check`, `goal_finish`, `goal_return_finish`

---

## (예정) 코드2

- **코드2**: 진입 라인 입력 → Nav2 이동 → `/action`·`/action_check` 시나리오 제어 (런치로 실행 예정)

## 빌드

```bash
cd /path/to/vertical_action
colcon build --packages-select vertical_action
source install/setup.bash
```
