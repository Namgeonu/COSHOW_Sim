# Stage C · LIMO 4대 연결

담당: 협업자. LIMO 4대 Jetson의 기존 limo_ros2·Nav2를 같은 WiFi와 `ROS_DOMAIN_ID`에서 실행한다. 드론은 연결하지 않는다. 단계 B의 미해결 수신 문제를 먼저 정리한다.

`~/COSHOW`에서 `source setup_env.sh` 후 다음 한 줄로 시작한다.

```bash
python3 dashboard/server.py --check-config 2>&1 | tee dashboard/logs/stage_c_check_config.log
```

| 검사 대상 | 기대 결과 |
|---|---|
| odom | 역할 2 + 스페어 2, 총 4개 PASS; age < `freshness_s.odom` |
| Nav2 액션 서버 | 역할 2개 PASS; 스페어에는 제어 클라이언트 없음 |
| ping | 인벤토리 LIMO 4개 IP의 결과 표시 |
| LIMO 상태·배터리 | 실제 타입 확인 후 PASS; 정보가 없으면 명시적 null/SKIP·정보 없음 |
| 드론 pose/status/카메라/서비스 | 드론 미연결 FAIL 허용 |
| preflight·mission | 실행하지 않으므로 발행자 없음 허용 |

1. `dashboard/config/dashboard.yaml`의 `fleet.limos` namespace/IP를 실제 Jetson과 맞춘다.
2. 상태 토픽 타입을 그래프에서 조회한다. 확인된 토픽만 `types.topics.limo_status_template`에 기록한다. 전압 필드와 단위도 Jetson 담당자에게 확인하며 임의 타입·임계값을 넣지 않는다.

```bash
ros2 topic list -t
```

실제 이름을 확인한 뒤 `ros2 topic info -v <확인한_상태_토픽>` 및 `ros2 interface show <확인한_타입>` 결과를 기록한다. 전압 필드가 현재 어댑터와 맞지 않으면 타입·필드 근거를 전달하고 해당 정보는 미확인으로 유지한다.

3. 읽기 전용 플릿/pose 기록기를 켠다. 전체 인벤토리는 `--fleet` 진단 표로 확인하며 기본 관리자 화면의 역할 6개 구성과 구분한다.

```bash
python3 dashboard/tests/stage_probe.py --fleet --record dashboard/logs/stage_c.jsonl --dump-poses dashboard/logs/stage_c_poses.csv
```

기존 기록이 있으면 타임스탬프 이름으로 전환하며 실제 저장 경로를 `OUTPUT`으로
표시한다. JSONL·CSV 중 한 경로가 잘못되면 새로 만든 빈 파일도 제거한다.

4. 역할 LIMO 2대를 안전한 수동 조작 모드에서 천천히 움직여 3D 위치·방향이 따라오는지 확인한다. 기록기는 Nav2 goal이나 취소 명령을 보내지 않는다.
5. 스페어 LIMO를 역할로 교체할 때는 Jetson에서 기존 네임스페이스/노드를 정상 종료한 후 새 역할 네임스페이스로 재시작한다. 대시보드 인벤토리 namespace도 실제 값과 맞춘다. 같은 역할 namespace를 두 Jetson이 동시에 사용하지 않도록 확인한다.
6. `--check-config`를 다시 실행하고 역할 2개의 Nav2 준비 및 4개 odom 신선도가 유지되는지 확인한다.

통과 기준: 4행 실제 값, 역할 2개 Nav2 준비, 위치·방향 추종, 실제 LIMO 상태 타입 및 전압 임계값의 설정 근거. 임계값을 아직 결정하지 못했다면 배터리 정보는 미확인으로 표시하고 열린 항목으로 남긴다.

증거: `dashboard/REPORTS/img/stage_c_*.png`, 설정 diff, 타입 조회 결과, JSONL/pose CSV. 미사용 드론 인터페이스의 FAIL을 LIMO 실패와 구분한다.

판정: 날짜 ______ 운영자 ______ 상태 타입 ______ 전압 필드/단위 ______ PASS/FAIL ______
