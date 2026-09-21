# Stage A · Webots 관찰 전용

담당: 협업자. 환경: Ubuntu 22.04, ROS Humble, Webots `coshow_rehearsal.wbt`, 기존 드라이버·fake_limo·시뮬 검출 노드 4개. 다른 노드와 같은 `ROS_DOMAIN_ID`·RMW 환경에서 실행한다. 실기체, AI Deck, preflight는 사용하지 않는다.

먼저 `~/COSHOW`에서 `source setup_env.sh`를 실행하고, 다음 한 줄로 설정 결과를 남긴다.

```bash
python3 dashboard/server.py --check-config 2>&1 | tee dashboard/logs/stage_a_check_config.log
```

| 검사 대상 | 기대 결과 | 이 단계에서 허용되는 예외 |
|---|---|---|
| 역할 드론 pose 4개 | PASS, 수신 신선도 < `freshness_s.pose` | 없음 |
| 역할 LIMO odom 2개 | PASS, 수신 신선도 < `freshness_s.odom` | 스페어 LIMO odom은 미기동 가능 |
| marker_detections 4개 | PASS | 없음 |
| 주석 JPEG·fps·stream_ok 각 4개 | PASS, BEST_EFFORT 수신, ≥ 5 fps | 처음 프레임 이전에는 대기 |
| mission_state | 외부 BT 실행 후 PASS | BT 실행 전에는 발행자 없음 |
| status·arm·crazyflie_server·preflight·실기체 aideck 노드 | FAIL 가능 | 시뮬에 이 인터페이스가 없기 때문 |
| 타입 미확인 LIMO 상태 | 명시적 null이면 SKIP | 임의 타입 추측 금지 |
| 실제 하드웨어 URI/IP·실기체 preflight 플래그 | 하드웨어용 정적 검사는 FAIL 가능 | 관찰 화면을 막지는 않음 |

`--check-config`의 전체 종료 코드가 1이어도 위에 명시한 예외만 있으면 관찰을 진행한다. 필수 pose·검출·영상의 이름/타입/QoS 실패를 예외로 넘기지 않는다.

1. 기존 절차대로 월드·드라이버·fake_limo를 시작한다. 검출 노드마다 `--ros-args -p publish_images:=true`를 적용하고 기존 display 설정은 유지한다.
2. 협업자가 기존 BT YAML의 `preflight.required: false`를 확인한다. 대시보드는 YAML을 자동 수정하지 않는다.
3. 별도 터미널에서 `bash dashboard/run.sh`, 이어 `bash dashboard/kiosk.sh`를 실행한다.
4. 다음 기록기를 켠 후 기존 절차대로 BT를 터미널에서 시작하고 관측점에 미션 마커를 둔다.

```bash
python3 dashboard/tests/stage_probe.py --fleet --record dashboard/logs/stage_a.jsonl --dump-poses dashboard/logs/stage_a_poses.csv
```

**점검 시작·데모 시작·비상 착륙·리셋 버튼을 사용하지 않는다.** 관리자에는 외부 BT 관찰 상태, run.state는 IDLE이며 신선한 외부 mission이 참관자 내러티브를 구동한다. Webots에는 status/arm 등 실행 주기 검증에 필요한 인터페이스가 없다.

| 통과 항목 | 기록 |
|---|---|
| observe → handover → search → capture → rescue_dispatch → rescue → return → done 8단계 순서 | 전이 시각 8개, 화면 캡처 |
| 드론 4대·LIMO 2대가 Webots와 같은 0.2 m 셀, 높이도 일치 | 동일 시점 Webots/트윈 화면 + pose CSV |
| 카메라 4타일 실제 시뮬 주석 영상, ≥ 5 fps, stream_ok=true | fps/수신 fps 표 + 관심 id 배지 |
| 발견 위치·finder·발광 표시 일치 | 발견 후 `stage_a_capture.png` |
| 30분 뒤 브라우저/GPU 메모리 정체 | Chrome 작업 관리자 시작/30분 값과 캡처 |

Chrome 작업 관리자(`Shift+Esc`)에서 GPU 메모리 열을 켜고 같은 브라우저
프로세스의 시작/30분 값을 기록한다. Mac의 heap·DOM·렌더러 자원 개수 결과는
GPU 메모리 바이트 측정을 대신하지 않는다.

| GPU 메모리 기록 | 값·스크린샷 |
|---|---|
| 행사 머신·브라우저 버전·GPU | ______ |
| 시작 시각·GPU 메모리 | ______ / ______ bytes |
| 시작 스크린샷 | `dashboard/REPORTS/img/stage_a_gpu_start.png` |
| 30분 시각·GPU 메모리 | ______ / ______ bytes |
| 30분 스크린샷 | `dashboard/REPORTS/img/stage_a_gpu_30min.png` |
| 변화·정체 여부·관측 한계 | ______ |

스크린샷은 `dashboard/REPORTS/img/stage_a_*.png`, 서버 로그와 관리자 이벤트는 `dashboard/logs/`에 보관한다. 좌표 차이·누락은 토픽 이름, 타입, QoS, pose/frame age로 기록한다. 시뮬에서 estop이나 실제 착륙 안전을 통과했다고 기록하지 않는다.

사후 요약 한 줄:

```bash
python3 dashboard/tests/analyze_stage.py dashboard/logs/stage_a.jsonl > dashboard/logs/stage_a_analysis.txt
```

이미 기록 파일이 있으면 프로브가 타임스탬프 이름으로 전환한다. 위 분석 명령의
JSONL 경로를 이번 실행의 `OUTPUT` 경로로 바꾼다. JSONL·CSV 중 한 경로가
잘못되면 새로 만든 빈 파일은 남기지 않는다.

판정: 날짜 ______ 운영자 ______ 필수 항목 PASS/FAIL ______ 남은 예외 ______
