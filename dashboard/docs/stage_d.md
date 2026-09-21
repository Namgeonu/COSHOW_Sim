# Stage D · 실기체 통합 데모 운영 체크리스트

담당: 협업자·사용자·운영자. Stage B+C가 끝난 환경, Lighthouse, 실제 필드·마커 배치를 사용한다. 협업자가 기존 BT YAML의 `preflight.required: true`, `emergency_land_on_exit: true`, `bt_visualiser.enabled: false`를 확인한다. 이 단계의 실행은 사람이 담당한다.

시작 한 줄 (`~/COSHOW`, `source setup_env.sh` 후):

```bash
python3 dashboard/server.py --check-config 2>&1 | tee dashboard/logs/stage_d_check_config.log
```

역할의 필수 인터페이스/설정에 FAIL이 없어야 한다. 명시적 null 타입은 SKIP이며 미확인 상태로 남긴다. 스페어 미기입은 경고로 기록하되, 역할 기체와 스페어의 무장 여부를 직접 확인한다.

1. 스택 기동 → 관리자 체크리스트 확인 → 점검 시작 → preflight 4단계와 실제 무장 확인 → 데모 시작을 순서대로 수행한다. BT·preflight를 별도 터미널에서 중복 실행하지 않는다.
2. 기록기를 켜고 두 화면을 동시에 녹화하거나 30초마다 캡처한다.

```bash
python3 dashboard/tests/stage_probe.py --fleet --record dashboard/logs/stage_d.jsonl --dump-poses dashboard/logs/stage_d_poses.csv
```

회차의 기존 기록을 덮어쓰지 않는다. 파일이 있으면 타임스탬프 이름으로 전환하며
실제 경로를 `OUTPUT`으로 표시한다. 아래 사후 분석에는 이번 `OUTPUT` JSONL
경로를 사용한다. 두 출력 중 하나를 열지 못하면 새로 만든 빈 파일도 제거한다.

| 회차 | 수행 | 결과·시간 기록 |
|---|---|---|
| 1 | 정상 완주: observe → handover → search → capture → rescue_dispatch → rescue → return → done | 시작 ____ DONE ____ 8단계/화면/카메라 ____ |
| 2 | capture에서 비상 착륙을 한 번 누름 | capture ____ LANDING ____ ABORTED ____ |
| 3 | 리셋·재점검 후 정상 완주 | IDLE ____ READY ____ DONE ____ |

회차 2에서는 LANDING 잠금 상태에서 estop/reset을 반복하지 않는다. 다음 순서를 관리자 이벤트와 실제 동작으로 대조한다.

| 비상 착륙 순서 | 확인 |
|---|---|
| LANDING 진입 → 해당 BT PID에 SIGINT 정확히 1회 | 시각 ____ 횟수 ____ |
| 약 0.5초 뒤 역할 드론 land ×4 및 역할 LIMO 전체 goal 취소 ×2 | 전송/응답 ____ |
| 드론 착륙·LIMO 정지 | 육안/영상 ____ |
| 착륙 지속 시간 + 2초까지 BT 종료 대기, 필요할 때만 SIGKILL | 종료 시각 ____ |
| preflight SIGINT | 시각 ____ |
| 신선한 pose와 landed_z 조건을 만족한 드론만 arm=false | 기체별 확인 ____ |
| ABORTED → 리셋 → IDLE → 재점검 READY | reset→READY ____초, <300초 |

착륙 미확인 경고가 남은 기체는 자동으로 무장 해제됐다고 간주하지 않는다. 운영자가 육안 확인 후 기존 수동 절차로 처리한다. BT의 요청된 SIGINT 종료 코드가 -6이어도 순서·착륙 증거로 판정하며, 종료 코드만으로 시퀀스 성공/실패를 정하지 않는다.

최소 한 번은 IDLE에서 스페어로 로스터를 교체하고 스택을 재기동한 뒤 다음 회차를 실행한다. 실제 착륙장에서는 역할 변경을 비행 중 수행하지 않는다.

회차 사이 IDLE에서 대시보드를 재시작할 때도 드론 서버·카메라 스택은 계속
실행된다. 실행 중 정상 서버 종료라면 BT·preflight의 기존 착륙 시퀀스를 먼저
마친다. 스택 PID와 `dashboard/run/stack.applied.json` 등 소유권 기록을 보존하며,
다음 대시보드에서
재입양과 적용 해시 복원을 확인한다. 적용 기록이 없거나 일부 기동만 성공했다면
해시는 null이고 시작은 차단되어야 한다. IDLE에서 **스택 재기동**으로 복구한다.

하루 운영 종료 시에는 데모 종료·착륙 확인·리셋 후 IDLE에서 **스택 정지**를
누른다. 이 명시적 정지나 재기동에서만 두 소유 스택에 병렬 SIGINT를 보내고
필요하면 10초 뒤 SIGKILL로 마무리한다. 대시보드 종료와 리셋은 스택을 끄지 않는다.

| 스택 운영 확인 | 기록 |
|---|---|
| 대시보드 종료 후 스택 PID 생존·기록 보존 | ______ |
| 대시보드 재시작 후 재입양·적용 해시 | ______ |
| 부분 기동 실패 시 살아 있는 스택 유지·해시 null·실행 차단 | ______ |
| 최종 IDLE 스택 정지 시 소유 스택만 종료 | ______ |

운영 중 해당 상황이 발생한 항목만 기록한다.

종료 후 core 파일 확인 한 줄:

```bash
find ~/COSHOW/bt -maxdepth 1 -type f -name 'core*' -printf '%TY-%Tm-%Td %TH:%TM:%TS %p\n'
```

실행 전/후 결과를 비교하여 새 core 파일이 없고 apport 창도 없음을 확인한다. 서버·BT·preflight 로그 및 관리자 마지막 이벤트 30건을 보존한다.

사후 분석 한 줄:

```bash
python3 dashboard/tests/analyze_stage.py dashboard/logs/stage_d.jsonl > dashboard/logs/stage_d_analysis.txt
```

최종 수용: **정상 완주 2회 + capture estop 1회 + reset→READY 5분 이내 + 새 core/apport 없음**. 분석표는 수신/이벤트 시각을 정리할 뿐 실제 착륙·정지를 대신 확인하지 않는다. 누락은 이름·타입·QoS·신선도·단계·시각과 함께 기록한다.

날짜 ______ 운영자 ______ 물리 필드/소프트웨어 버전 ______ PASS/FAIL ______ 남은 항목 ______
