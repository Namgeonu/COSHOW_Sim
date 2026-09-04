# aideck_aruco_ros

`multi_aruco_udp_viewer.py`(udp-link-diagnostic)를 ROS 2 노드로 옮긴 패키지입니다.
터미널에 `print` 하던 결과를 ROS 토픽으로 publish 합니다.

**원본 스크립트는 전혀 건드리지 않았습니다.** UDP 프로토콜 처리 코드(`udp_stream.py`)는
원본을 1:1로 옮겼습니다.

## COSHOW 연동으로 추가된 것 (2026-09-04)

행동트리(BT)가 마커의 **실제 지면 좌표**를 받을 수 있도록 세 가지를 더했습니다.

1. **드론 pose 구독** — crazyswarm2 가 발행하는 `/<name>/pose` 를 드론마다 구독합니다.
2. **역투영** — 마커 픽셀과 드론 pose(위치+자세)로 마커가 지면 어디에 있는지 역산합니다.
   AI-Deck(Himax HM01B0) 규격 324x244 · 수평 화각 87° 를 상수로 씁니다.
   324x244 는 정사각 센서의 세로 크롭이라 **수직 화각은 87° 가 아니라 약 71°** 이고,
   코드가 그 관계를 자동으로 계산합니다.
3. **`/<name>/marker_detections` 발행** — `coshow_interfaces/MarkerDetections` 타입.

대신 픽셀 정보만 담던 아래 둘은 **제거**했습니다. 위 토픽이 같은 내용을 포함하고,
`marker_center` 는 "가장 큰 마커" 를 고르는데 BT 는 target id 로 골라야 해서
쓸 수 없었습니다.

* `<name>/markers` (`vision_msgs/Detection2DArray`)
* `<name>/marker_center` (`geometry_msgs/PointStamped`)

### 알려진 한계 — 렌즈 왜곡

규격표의 대각 화각 115° 는 87°x87° 정사각을 핀홀로 계산한 값(107°)과 맞지 않습니다.
광학 왜곡이 있다는 뜻이고, 역투영은 왜곡 없는 핀홀을 가정하므로 **화면 가장자리에서
오차가 커집니다.** 화면 중앙 근처는 신뢰할 만합니다. 실측 후 문제가 크면
`cv2.calibrateCamera` 로 왜곡 계수를 구해 보정하는 것이 다음 단계입니다.

### 빌드 의존성

`coshow_interfaces` 패키지가 필요합니다. 이 워크스페이스의 `src/` 에 함께 넣어
두었으므로 `colcon build` 한 번이면 같이 빌드됩니다.

## 빌드 / 실행

```sh
cd ~/AIDeck_ws/aideck_ros2_ws
colcon build --symlink-install
source install/setup.bash

# 1) launch (config/drones.yaml 사용)
ros2 launch aideck_aruco_ros aideck_aruco.launch.py

# 2) ros2 run
ros2 run aideck_aruco_ros aideck_aruco_node --ros-args \
    -p drones:="['drone1,192.168.1.125,5001','drone2,192.168.1.126,5002']"

# 3) 빌드 없이 그냥 파이썬으로 (원본처럼)
python3 aideck_aruco_ros/aideck_aruco_node.py --ros-args \
    -p drones:="['drone1,192.168.1.125,5001']"
```

`drones` 항목 포맷은 원본 `--drone` 과 **완전히 동일**합니다:
`NAME,DECK_IP,LISTEN_PORT[,DECK_PORT]` (DECK_PORT 기본 5000).

## 토픽 (드론 이름이 `drone1`, namespace 가 `/aideck` 인 경우)

| 토픽 | 타입 | 내용 |
|---|---|---|
| `/aideck/drone1/image_raw` | `sensor_msgs/Image` | 원본 프레임 (`mono8`) |
| `/aideck/drone1/image_annotated` | `sensor_msgs/Image` | 마커 그려진 프레임 (`bgr8`) |
| `/aideck/drone1/image_annotated/compressed` | `sensor_msgs/CompressedImage` | JPEG (`publish_compressed:=true` 일 때만) |
| `/aideck/drone1/markers_text` | `std_msgs/String` | 터미널 확인용 짧은 한 줄 요약 |
| `/aideck/drone1/fps` | `std_msgs/Float32` | 1 Hz |
| `/aideck/drone1/stream_ok` | `std_msgs/Bool` | 스트림 stall 시 false, 1 Hz |

여기에 더해, **네임스페이스를 타지 않는 전역 이름**으로 하나 더 나갑니다:

| 토픽 | 타입 | 내용 |
|---|---|---|
| `/drone1/marker_detections` | `coshow_interfaces/MarkerDetections` | 마커별 픽셀 좌표 **+ 지면 월드 좌표** |

`MarkerDetections` 의 필드:

* `drone` = 드론 이름, `drone_pose` = 검출 시점의 드론 pose
* `markers[i].id / cx / cy / size_px` = 마커 ID 와 픽셀 정보
* `markers[i].world_x / world_y` = **역투영한 마커의 실제 지면 좌표 (map 프레임)**
  * 드론 pose 를 아직 못 받았거나 역투영에 실패하면 `0` 으로 남습니다.
  * 소비하는 쪽에서 `0` 인 프레임은 버리고 다음 프레임을 쓰면 됩니다.

이 토픽만 전역 이름인 이유는 crazyswarm2 (`/drone1/pose`) 와 COSHOW 행동트리가
쓰는 이름에 맞추기 위해서입니다. launch 의 `namespace` 를 바꿔도 이 토픽 이름은
변하지 않습니다.

터미널에서 사람이 보기에는 `markers` 가 너무 길기 때문에 보통 아래 토픽을 확인하세요:

```sh
ros2 topic echo /aideck/drone1/markers_text
ros2 topic echo /aideck/drone2/markers_text
```

예시:

```text
data: frame=123 count=2 fps=3.04 | id=2 x=102.0 y=75.8 size=62.0 theta=-3.11 | id=4 x=211.5 y=119.0 size=48.2 theta=0.02
```

### QoS 주의

이미지 토픽 기본 QoS 는 `sensor_data`(= **best effort**)입니다. 카메라 토픽 표준이고
RViz2 / rqt_image_view 는 잘 붙지만, `ros2 topic echo` 는 기본이 reliable 이라
아무것도 안 보입니다. 확인하려면:

```sh
ros2 run rqt_image_view rqt_image_view        # 권장
# 또는 파라미터로 reliable 로 바꾸기
ros2 run aideck_aruco_ros aideck_aruco_node --ros-args -p image_qos:=reliable -p drones:="[...]"
```

`markers` / `marker_center` / `fps` / `stream_ok` 는 reliable 이라 `ros2 topic echo` 로 바로 보입니다.

## 파라미터

`config/drones.yaml` 참고. 원본 argparse 와 이름/기본값을 맞췄습니다.

| 파라미터 | 기본값 | 원본 대응 |
|---|---|---|
| `drones` | (필수) | `--drone` (여러 개면 리스트) |
| `dictionary` | `DICT_4X4_50` | `--dictionary` |
| `target_id` | `-1` (전체) | `--target-id` |
| `detect_every` | `1` | `--detect-every` |
| `probe_period` | `1.0` | `--probe-period` |
| `timeout` | `0.2` | `--timeout` |
| `stall_timeout` | `3.0` | `--stall-timeout` |
| `display` | `false` | `--no-display` 의 반대 |
| `display_scale` | `1.6` | `--display-scale` |
| `publish_raw` / `publish_annotated` | `true` | (신규) |
| `publish_compressed` / `jpeg_quality` | `false` / `80` | (신규) |
| `image_qos` | `sensor_data` | (신규) `reliable` 가능 |
| `frame_id_template` | `{name}_camera` | (신규) |
| `log_detections` | `false` | 원본의 `Found ...` 줄을 rosout 으로 |
| `publish_empty_text` | `false` | `markers_text` 에 marker 없음도 publish 할지 |
| `poll_rate` | `100.0` | (신규) publish 폴링 주기 |
| `queue_depth` | `4` | (신규) 프레임 큐 깊이 |
| `rcvbuf_bytes` | `0` (OS 기본) | (신규) `SO_RCVBUF` |

`display:=true` 로 하면 원본과 똑같은 OpenCV 창도 같이 뜹니다 (`q`/ESC 로 종료).

## 원본과 달라진 점

1. **`display_scale` 은 화면에만 적용**됩니다. 원본은 1.6배 확대된 이미지를
   보관했지만, 여기서는 publish 되는 `image_annotated` 는 원본 해상도(324x244)
   그대로입니다. 다운스트림에서 픽셀 좌표를 그대로 쓸 수 있게 하려는 의도입니다.
2. `print` → ROS 로거(rosout). `STREAM WAIT` / `STREAM STALL` / `STREAM RECOVERED`
   메시지는 그대로 남아 있습니다.
3. 손상된 image header (`size == 0` 또는 비정상적으로 큰 값) 를 걸러냅니다.
4. 이전 프레임이 끝나기 전에 새 헤더가 오면 `partial_frames_dropped` 로 셉니다
   (종료 시 요약 출력). 원본은 조용히 버렸습니다.
5. SIGTERM 도 처리해서 어떤 방식으로 죽어도 `BYE` 가 나갑니다.

## cv_bridge 를 안 쓰는 이유

이 PC 는 numpy 2.2.6 인데 Humble 의 `cv_bridge` 는 numpy 1.x 로 빌드돼 있어서
`CvBridge().cv2_to_imgmsg()` 가 `KeyError: 16` 으로 죽습니다. 그래서
`sensor_msgs/Image` 를 직접 채웁니다 (cv_bridge 가 내부에서 하는 일과 동일).

## 알아둘 점

* AI-Deck(ESP32) 는 **`FER` 패킷의 출발지 IP:PORT 하나만** 스트림 대상으로
  기억합니다 (`aideck-esp-firmware-udp/main/wifi.c`). 따라서 **원본 뷰어와 이
  노드를 같은 드론에 동시에 띄우면 안 됩니다.** 서로 대상을 뺏습니다.
  같은 listen 포트를 쓰면 `Address already in use` 로 바로 죽으니 금방 알 수 있고,
  다른 포트를 쓰면 조용히 서로 스트림을 뺏어갑니다.
* `camera_info` 는 publish 하지 않습니다. AI-Deck 카메라 intrinsic 캘리브레이션이
  없어서, 가짜 값을 내보내면 `aruco_ros` 같은 pose 추정 노드가 틀린 결과를 냅니다.
  마커의 3D pose 가 필요하면 캘리브레이션부터 하셔야 합니다.
