# COSHOW 중앙 제어 BT (py_bt_ros 시나리오)

중앙 컴퓨터에서 프로세스 1개로 도는 PA-BT(백체이닝) 트리. 블랙보드에 토픽으로 현재 상황을 기록하고,
조건 노드가 그것을 읽어 "미션 완료? → 타겟 마커 인식? → 미션 마커 인식?" 순으로 달성되지 않은
조건의 서브트리를 실행한다.

## 설치

```bash
cd ~/COSHOW
git clone https://github.com/inmo-jang/py_bt_ros.git bt      # (또는 submodule)
cp -r <이 폴더> bt/scenarios/coshow
pip install pygame pyyaml
```
`modules/` 는 손대지 않는다. 시나리오 폴더만 추가하는 것이 이 프레임워크의 확장 방식.

## 실행 (순서)

```bash
# 1) Webots: coshow_integrated.wbt 열기
# 2) 드라이버 (드론 4 + limo_a, limo_b + Nav2) — limo_b 도 반드시 포함 (BT 가 두 LIMO pose 를 모두 요구)
source ~/COSHOW/setup_env.sh
ros2 launch ~/COSHOW/coshow_all_drivers.launch.py robots:=limo_a,limo_b
# 3) 드론 서버 + 검출 노드
python3 ~/COSHOW/nodes/coshow_server.py
python3 ~/COSHOW/nodes/aruco_detector_node.py --ros-args -p drone:=cf_a   # cf_b~d 도 (카메라 확장 후)
# 4) BT
cd ~/COSHOW/bt && source ~/COSHOW/setup_env.sh
python3 main.py --config scenarios/coshow/configs/coshow_sim.yaml
```

## 파일

| 파일 | 역할 |
|---|---|
| `configs/coshow_sim.yaml` | 좌표(P0, base), 구역·레인, 마커 ID 매핑, 허용오차, 타이머 |
| `bt_nodes.py` | 블랙보드 갱신 노드 1 + 조건 10 + 액션 11 |
| `default_bt.xml` | 루트 트리 (줄기 3개 + Phase 2 탐색) |
| `phase1_observe.xml` | limo_a→P0, cf_a 이륙→P0 상공→검출 대기 |
| `phase3_rescue.xml` | 재검출 확정 → 나머지 복귀 ∥ limo_b 출동 → 10s → 전원 복귀 |

## 블랙보드 키
`bt_nodes.py` 상단 docstring 참조. 주요: `pose[robot]`, `mission_marker`, `target_id`, `target_marker`(=`P_N`),
`target_confirmed`, `finder`, `drone_arrived/limo_arrived[robot]={goal,t}`, `rescue_done_t`, `cmd[robot]`.

## 프레임워크 규칙과 맞물린 설계 결정 (수정 시 주의)

1. **서비스 액션은 반드시 `ReactiveFallback(조건, 액션)` 안에.** 베이스 `ActionWithROSService` 는 응답 후
   `_sent` 를 리셋해 Reactive 구조에서 매 tick 재호출된다. `_DroneService` 는 run 을 오버라이드해
   "목표 서명이 바뀔 때만" 호출하고 항상 RUNNING 을 반환한다. 조건이 참이 되면 halt → 다음 목표에 재사용.
2. **`ActionWithROSAction._on_running` 은 반드시 Status 를 반환.** 기본값 None 이면 부모가 FAILURE 로 오판한다.
3. **Reactive 제어 노드는 halt 를 자식에 전파하지 않는다** (Parallel 만 전파). leaf 내부 상태는
   사이클 경계에서 `ResetCycle` 이 `agent.halt_tree()` 로 깊이 초기화한다.
4. **도착 판정 기준**: LIMO 는 Nav2 액션 SUCCEEDED 만 (pose 미사용, `IsLimoAt`/`_limo_home`).
   드론은 서비스라 완료 보고가 없으므로 `IsDroneAt` 이 pose 로 감독. ABORTED 는 `LimoNavigateTo` 가 3s 후 재전송.
5. 시각은 BT 프로세스의 `time.monotonic()` 으로 통일 (`/clock` 없음). 메시지 stamp 는 비교에 쓰지 않는다.
6. `UpdateBlackboard` 는 검출 토픽을 자체 콜백으로 처리한다(연속 3프레임 카운트가 10Hz tick 보다 빠를 수 있어서).
   블랙보드는 이 노드만 쓴다(단일 작성자). 조건 노드는 읽기만, 액션은 `cmd/arrived` 흔적만 기록.

## 오프라인 검증
`bt/test_harness.py` (가짜 ROS 로 트리 전체를 tick). 2사이클 완주, 명령 재호출 0, 게이팅(발신자·ID 범위·도착 이후) 확인됨.
```bash
cd ~/COSHOW/bt && python3 test_harness.py
```

## 실물 전환 시 바꿀 것
- `SetLed`: `/{robot}/led` stub → crazyswarm2 파라미터 쓰기
- LIMO pose 토픽: `limos.*.pose_topic` (odom → 실물 localization 토픽)
- 좌표/구역/허용오차: yaml 만
