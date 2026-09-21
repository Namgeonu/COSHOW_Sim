# Stage B · 프로펠러 없는 실기체 벤치

담당: 협업자·사용자. Crazyradio 3개, Crazyflie 10대, AI Deck WiFi를 준비하고 **10대 모두 프로펠러를 분리한다.** LIMO는 연결하지 않는다. Lighthouse가 없으면 pose 검사는 보류하며 status/카메라 수신을 우선 확인한다. Stage A 기록을 검토한 후 진행한다.

`~/COSHOW`에서 `source setup_env.sh` 후 설정 결과를 남긴다.

```bash
python3 dashboard/server.py --check-config 2>&1 | tee dashboard/logs/stage_b_check_config.log
```

| 검사 대상 | 기대 결과 |
|---|---|
| 역할 4 + 스페어 6 status | 10개 PASS, age < 1 s, 전압·RSSI·armed=false |
| 역할 pose 4개 | Lighthouse가 있으면 PASS; 없으면 미확인 사유 기록 |
| 역할 카메라·fps·stream_ok 각 4개 | 실제 AI Deck 주석 영상 ≥ 5 fps |
| land/arm 서비스 | 역할 4대에서 발견됨. 기록기는 호출하지 않음 |
| crazyflie_server·aideck 노드 | PASS |
| AI Deck ping | 10대 IP 확인, 갱신 결과 표시 |
| LIMO odom/Nav2/상태 | 미연결 FAIL·null 타입 SKIP 허용 |
| preflight·mission | 실행 전 발행자 없음 허용 |
| 스페어 pose/카메라 | 제어·역할 카메라 대상 아님; status만으로 행이 채워짐 |

1. 관리자 고급 플릿 인벤토리에서 10대의 실제 URI/IP를 확인한다. 기본 화면에는 역할 6개만 표시하며, 전체 물리 기체 점검은 고급 관리와 `--fleet` 진단 표로 한다. 라디오당 최대 5대 제한을 지키며 3개 라디오에 배분한다.
2. 역할 드론 4개를 배정하고 저장한다. IDLE에서 스택 기동 후 `logs/crazyflie_server.log`의 10대 연결을 확인한다.
3. `--fleet`의 14행 진단 표를 5초마다 기록한다. LIMO 4행의 미연결 상태도 함께 남긴다.

```bash
python3 dashboard/tests/stage_probe.py --fleet --record dashboard/logs/stage_b.jsonl
```

기존 기록이 있으면 타임스탬프 이름으로 전환하며 실제 저장 경로를 `OUTPUT`으로
표시한다. 사후 분석에는 이번 실행의 `OUTPUT` JSONL 경로를 사용한다.

4. 배터리 하나를 이미 낮은 전압인 기체로 교체해 임계값 경고/차단을 확인한다. 시험을 위해 배터리를 과방전시키지 않는다.
5. 아래 로스터 교체를 한 번 수행한다. 원래 YAML을 손으로 덮어쓰지 않는다.

| 로스터 교체 절차 | 확인·캡처 |
|---|---|
| IDLE이고 기체 armed=false 확인 | 변경 전 플릿 표 |
| 해당 역할 드롭다운에서 이전 기체를 해제하고 새 물리 기체 선택 | 중복 배정 없음 |
| 로스터 저장 | 역할/물리 ID 대응, roster_hash 변경 |
| 스택 재기동 | generated 파일 적용, applied_hash 일치 |
| 새 기체가 해당 역할의 status·카메라를 발행 | 새 기체 값 확인 |
| 이전 기체는 스페어 행으로 이동 | status만 표시, 스페어 무장 경고 없음 |

기동이 일부만 성공하면 먼저 뜬 스택은 유지되고 적용 해시는 null이 된다.
실패 사유를 해결한 뒤 IDLE의 **스택 재기동**으로 복구한다. 대시보드만 종료해도
스택과 `dashboard/run/`의 PID·소유권·적용 기록은 남는다. 다시 실행하면 신원이
일치하는 스택을 재입양하므로 중복 기동하지 않는다. 벤치 작업을 끝내고 스택까지
끄려면 IDLE에서 **스택 정지**를 누른다. 리셋은 스택 정지 명령이 아니다.

Stage B의 필수 수용은 수신·표시·로스터 교체다. preflight는 stage 3 통과 직후 자동으로 stage 4에서 무장하므로, **버튼을 누른 뒤 사람이 stage 4 직전에 맞춰 중단하는 방법은 사용하지 않는다.** 기존의 검증된 차단 조건이 stage 4 진입을 막는 환경에서만 단계 1–3을 관찰할 수 있다. 그런 조건이 없다면 preflight/무장 확인은 Stage D로 넘긴다. Lighthouse가 없어 stage 1에서 실패한 경우는 그 사유를 기록한다.

통과 기준: 10대 상태 값이 1초 이내 갱신, 역할 4타일 실제 주석 영상, 파일 수동 편집 없는 로스터 교체, 라디오 수 제한 통과, 스페어 무장 경고 없음. 캡처는 `dashboard/REPORTS/img/stage_b_*.png`, 로그/JSONL은 `dashboard/logs/`에 보관한다.

판정: 날짜 ______ 운영자 ______ 라디오별 기체 수 ______ 로스터 전/후 ______ PASS/FAIL ______
