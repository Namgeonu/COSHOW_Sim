# COSHOW 리허설 브랜치 — 팀 공유 노트

`rehearsal` 브랜치 기준. 최근 변경 사항과 팀원이 알아야 할 운용 정보를 정리했다.
(BT·검출·대시보드·리모 담당 모두 대상)

---

## 1. BT가 두 버전이다 — 2대용 / 1대용

리모 대수에 따라 **config만 바꿔** 실행한다. Python(bt_nodes.py)은 두 버전이 공유하며 무수정.

### 2대용 (기본, limo_a + limo_b)
| 항목 | 파일 |
|---|---|
| config | `bt/scenarios/coshow/configs/coshow_rehearsal.yaml` (실기) / `coshow_rehearsal_sim.yaml` (시뮬) |
| 트리 | `bt/scenarios/coshow/rehearsal_bt.xml` |
| phase1 서브트리 | `bt/scenarios/coshow/phase1_observe.xml` |
| 리모 | limo_a(운반), limo_b(구조) |

```bash
cd ~/COSHOW/bt
python3 main.py --config scenarios/coshow/configs/coshow_rehearsal.yaml
```

### 1대용 (fallback, limo_b 한 대가 운반+구조 겸임)
두 대로 하다 문제가 생기면 이걸로 전환한다. **limo_b 한 대**가 네 명령을 순서대로 수행:
관측점 전달 → 복귀 → P_N 구조 출동 → 최종 복귀.

| 항목 | 파일 |
|---|---|
| config | `coshow_rehearsal_1limo.yaml` (실기) / `coshow_rehearsal_1limo_sim.yaml` (시뮬) |
| 트리 | `bt/scenarios/coshow/rehearsal_1limo_bt.xml` |
| phase1 서브트리 | `bt/scenarios/coshow/phase1_observe_1limo.xml` |
| 리모 | limo_b 하나만 (config `limos` 에 limo_b 만 있음) |

```bash
cd ~/COSHOW/bt
python3 main.py --config scenarios/coshow/configs/coshow_rehearsal_1limo.yaml
```

- 실제 리모 1대는 **limo_b** 이름으로 띄운다 (`/limo_b/navigate_to_pose` 액션 제공).
- 2대용의 최종 복귀에 있던 limo_a·limo_b 두 브랜치를 limo_b 하나로 병합했다 (동시 목표 충돌 방지).
- 통합 시뮬 검증 완료 (전 국면 정상 완료).

---

## 2. flag로 켜고 끄는 것들

### (a) 사전점검 10초 게이트 — CLI 플래그
`main.py` 실행 시:
```bash
python3 main.py --config <config>                 # config preflight.gate 값을 따름
python3 main.py --config <config> --no-preflight  # 10초 점검 건너뜀
python3 main.py --config <config> --preflight      # 10초 점검 강제 실행
```
- 두 플래그 상호배타. 안 주면 config를 따른다.
- **주의**: `--no-preflight`는 시작 전 **10초 대화형 게이트만** 끈다. 트리 안의 `PreflightReady`(별도 안전 관문)는 `preflight.required`가 따로 제어하며, 리허설은 `required: true`라 여전히 켜져 있다.

### (b) 처음부터 고장난 기체가 있을 때
- 10초 게이트를 **켜고** 문제 기체를 y로 승인 → 그 기체가 DEGRADED로 처리되어 **즉시 착륙·재할당**. 이게 시작 시점 고장을 다루는 유일한 자동 경로다.
- 게이트를 끄면 그 고장 기체가 `PreflightReady`(또는 pose 게이트)를 막아 **BT가 이륙조차 안 한다**. 시작 시점에 4대가 다 정상일 때만 게이트를 꺼라.
- **비행 중** 고장은 게이트와 무관하게 항상 자동 착륙·재할당된다.

### (c) 리모 pose를 시작 게이트에 요구할지 — config 플래그
`coshow_rehearsal*.yaml`:
```yaml
require_limo_pose: false   # 기본: 시작 게이트가 드론 pose만 확인 (리모 odom 불요)
                            # true 로 바꾸면 limo_a/limo_b odom 수신도 시작 조건에 포함
```
- BT는 리모 pose 내용을 안 쓴다(도착은 Nav2 액션 결과로 판정). 그래서 기본 false.

---

## 3. BT가 리모/드론/검출에 요구하는 인터페이스 (담당자 확인용)

### 리모 (하드웨어 담당)
- BT가 쓰는 것은 **`/limo_a/navigate_to_pose`, `/limo_b/navigate_to_pose` 액션**(nav2_msgs/NavigateToPose)뿐. (1대용은 `/limo_b/...`만.)
- 중앙 PC `ros2 action list`에 이 이름들이 떠야 하고, `ros2 action info`에 **서버 1개**여야 한다.
- 리모 Nav2를 `/limo_a`·`/limo_b` 네임스페이스로 띄우면 `fleet_manager` 없이도 BT가 그대로 동작한다. 두 리모가 맨이름(`/navigate_to_pose`)으로 충돌하면 `tools/ns_relay_node.py` + `tools/fleet_manager_node.py`로 네임스페이스를 붙인다.
- **도착 판정 = Nav2 액션이 SUCCEEDED 반환.** BT는 위치를 재지 않는다. 도착 정밀도는 리모 Nav2 goal_checker가 결정.
- `/limo_x/odom`, `/limo_x/amcl_pose`는 BT 필수 아님(대시보드·모니터용). 시작 게이트에서 리모 제외됨.

### 검출 (검출 담당)
- 마커 위치 = **화면상 마커 크기 기반 역투영**(지붕 높이 마커 대응). 시뮬 검출 노드는 반영됨. **실기 aideck 노드(`ros2_ws/.../aideck_aruco_node.py`)는 아직 미커밋** — 지붕 마커 쓰려면 커밋 필요.
- 실제 출력 마커 한 변이 **0.2 m**여야 정확(`_MARKER_SIZE_M`). 다르면 위치가 비례해서 틀어진다.
- 마커 확정 = 최근 `confirm_window_sec`(3초) 안에 `confirm_frames`(3장) 이상 + 윈도우 안에서 **드론이 가장 위에 있던 프레임** 채택(방식 B).

### 대시보드 (대시보드 담당)
- 위치: `~/COSHOW/dashboard`, 실행 `./run.sh` → http://localhost:8080/static/visitor.html
- 미션 서사(신호색·후레쉬·캐러셀·팝업·상태칩) = **`/coshow/mission_state`** 하나로 그림. BT의 UpdateBlackboard가 발행(직렬화만, 행동 결정 없음).
- 카메라 영상 = 검출 노드가 `/aideck/<드론>/image_annotated/compressed`(CompressedImage, format jpeg) 발행. 토픽 이름·타입·드론 이름이 대시보드 config와 정확히 일치해야 함.
- 모든 노드가 **같은 `ROS_DOMAIN_ID`** 여야 한다.

---

## 4. 이번 세션 주요 변경 요약

- **BT → `/coshow/mission_state` 발행**: 대시보드용 미션 상태 직렬화(UpdateBlackboard).
- **마커 위치 방식 B**: 윈도우 안에서 드론이 가장 위였던 프레임 채택(크기기반 오차 최소화).
- **구역 재할당 로직 교체**: 실시간 pose 거리 대신 **고정 구역 순번(인덱스) 인접 배정**. 연속 블록이라 한 드론이 남의 구역을 가로지르지 않음. 래치 제거(순간 pose 끊김에 안 흔들림).
- **탐색 범위**: x축 왕복이 **[0, +1.5]** (미션기 진입 충돌 회피). y는 각 구역 중앙 고정.
- **리모 복귀 위치**: limo_a (1.5, 1.5), limo_b (1.5, -1.5). (1대용 limo_b는 (1.5,-1.5).)
- **이동 duration**: goto_short 10s, **goto_long 20s**(15→20), **capture 10s**(7→10), takeoff/land 8s.
- **구조 대기**: 리모 P_N 도착 후 **10초**(`rescue_sec`).
- **시작 게이트에서 리모 제외**(`require_limo_pose: false`).
- **사전점검 CLI 플래그** `--preflight` / `--no-preflight`.
- **대시보드**: 실행 필수 파일만 남기고 정리, 배터리 바 스케일(≥4.0V 초록3 / 3.7~4.0 노랑2 / ≤3.7 빨강1), 3D 맵 바닥마커·드론궤적 제거, handover 팝업 제거, 캐러셀 발견 시 "구출" 단계로.
- **검출 노드 카메라 발행** 추가.

---

## 5. 시뮬 전체 실행 (드론 4 + 리모 + 검출 + 대시보드)

모든 터미널 **같은 `ROS_DOMAIN_ID`** (예: 30 또는 33)로.
```bash
# 공통: export ROS_DOMAIN_ID=30 ; source ~/COSHOW/setup_env.sh
webots ~/COSHOW/worlds/coshow_rehearsal.wbt          # T1
ros2 launch ~/COSHOW/rehearsal_drivers.launch.py     # T2
python3 ~/COSHOW/coshow_server.py                    # T3
python3 ~/COSHOW/tools/fake_limo.py                  # T4 (가짜 리모)
# T5: 검출 4대 (cd ~/COSHOW; nodes/aruco_detector_node.py --ros-args -p drone:=cf230 ...)
cd ~/COSHOW/bt && python3 main.py --config scenarios/coshow/configs/coshow_rehearsal_sim.yaml   # T6
cd ~/COSHOW/dashboard && ./run.sh                    # T7
```
1대용 시뮬은 T6를 `coshow_rehearsal_1limo_sim.yaml`로.

---

## 6. 알아둘 점 / 미결

- **실기 검출 노드(aideck_aruco_node.py)의 크기기반 역투영은 아직 미커밋.** 지붕 마커를 실기에서 쓰려면 커밋 필요.
- 저장소에 건물/월드 실험(`worlds/coshow_integrated.wbt`, `protos/buildings/`, `controllers/roof_probe/` 등)이 **미커밋 WIP**로 남아 있음 — 이번 리허설 변경과 무관.
- `PreflightReady`는 `preflight.required: true`(리허설)면 항상 켜져 있다. `--no-preflight`로는 안 꺼진다.
- 리모 라이다 높이가 COSHOW는 -0.120(착륙 드론 감지용), 별도 `~/LIMO_webots`는 공식값 -0.034 — 목적이 달라 값이 다름.
