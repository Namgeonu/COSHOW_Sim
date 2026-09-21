# 관람자 대시보드 — 로컬 실행 안내 (실기체 담당용)

이 폴더 하나면 대시보드가 돈다. 별도 저장소를 받을 필요 없이, 이 폴더가 들어 있는
COSHOW 체크아웃에서 바로 실행한다. 대시보드는 ROS 토픽만 구독해 화면을 그리는
관측 전용이다 (로봇을 직접 조종하지 않음. 비상 착륙/취소만 역할 기체에 보낼 수 있음).

## 1. 필요한 것

- Ubuntu 22.04 + ROS 2 Humble
- 메시지 패키지가 있는 워크스페이스(overlay)를 source 할 수 있어야 한다:
  `crazyflie_interfaces`, `coshow_interfaces`, `nav2_msgs`,
  `std_msgs`, `sensor_msgs`, `nav_msgs`, `geometry_msgs`
  (실기 운용 시 쓰는 crazyswarm2 워크스페이스 + coshow_interfaces overlay 면 충분)
- 파이썬 패키지 (apt):
  ```bash
  sudo apt-get install -y python3-aiohttp python3-yaml iputils-ping
  ```

## 2. 실행

```bash
# (1) ROS overlay source — 둘 중 하나
source ~/<your_ws>/install/setup.bash            # 메시지 패키지가 있는 워크스페이스
#   또는 아래처럼 run.sh 에 경로만 알려줘도 된다:
#   export COSHOW_ROS_SETUP=~/<your_ws>/install/setup.bash

# (2) 도메인을 BT/시뮬/리모와 동일하게 맞춘다 (전부 같은 값이어야 토픽이 보인다)
export ROS_DOMAIN_ID=33

# (3) 실행
cd ~/COSHOW/dashboard
./run.sh
```

브라우저: **http://localhost:8080/static/visitor.html**

`run.sh` 는 ROS 환경을 스스로 찾는다(우선순위: `COSHOW_ROS_SETUP` → `~/COSHOW` 시뮬
레이아웃 → 이미 source 된 환경 → `/opt/ros/humble`). 실기 노트북에서는 (1)처럼 자기
워크스페이스를 미리 source 하거나 `COSHOW_ROS_SETUP` 만 지정하면 된다.

## 3. 실행 전 점검 (토픽/타입이 실제로 붙는지)

```bash
cd ~/COSHOW/dashboard
python3 server.py --config config/dashboard.yaml --check-config
```
각 토픽/서비스/액션/노드의 pass/fail 을 표로 보여준다. 아직 안 뜬 노드는 fail 로 나온다.

## 4. 로봇 없이 화면만 미리 보기 (목업)

```bash
cd ~/COSHOW/dashboard
python3 server.py --config config/dashboard.yaml --mock
```
가짜 데이터로 UI 흐름(신호색·캐러셀·팝업·카메라 자리)을 확인할 수 있다.

## 5. 화면이 "대기"에서 안 넘어갈 때

대시보드가 미션 화면(신호색·후레쉬·캐러셀·팝업)을 그리려면 두 가지가 모두 필요하다.

1. **BT 노드가 떠 있어야 한다** — 대시보드가 `space_bt_bridge` 노드를 감지하면 active 로 전환.
2. **`/coshow/mission_state` 토픽이 와야 한다** — BT 가 발행한다 (config 의 `mission_state_topic`).

둘 중 하나라도 없으면 "대기" 화면이 유지된다. 확인:
```bash
ros2 node list | grep space_bt_bridge
ros2 topic echo /coshow/mission_state --once
```

`config/dashboard.yaml` 의 로스터(`robots.drones`, `robots.limos`)와 토픽 템플릿이
실제 기체 이름과 맞는지도 확인할 것. 도메인(`ROS_DOMAIN_ID`)이 다른 노드와 어긋나면
아무 토픽도 안 보인다.
