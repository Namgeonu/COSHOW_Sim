# M8 후속 수정 — 2026-09-14

## 한 것

기준은 `dashboard`의 `4a9d021`과 v2.6 최종 리뷰 §2이다.
[인계 원문 SHA256](evidence/M8_sources.log).
[2026-09-14 스택 결정](evidence/M8_stack_shutdown_decision.md)을 반영해 필수 후속을 마쳤다.
A3의 자동 병렬 종료 요구는 철회됐고, 서버 종료·부분 기동 실패 시 스택을 유지한다.
본 보고서·코드·증거 전체가 요청한 **M8 후속 한 커밋**의 범위다.
리드 재확인 뒤 사람이 Stage A를 시작한다. Stage A–D 및 실기체 운용 통과를 의미하지 않는다.

| 항목 | 수정 위치와 동작 | 회귀·재실행 증거 |
|---|---|---|
| A1 | [server.py](../server.py) `reconfigure/_reconfigure`: 실행기 가드 → 새 ROSIO 시작 → 실행기 참조 교체 → 구 IO 정리. 새 시작 실패는 기존 IO 유지, 구 IO 정리 실패도 새 제어 경로 유지. 요청 취소 중 교체는 서버 소유 작업으로 끝낸다. [runner.py](../runner.py) `_require_idle`, [fleet.py](../fleet.py) `_operation/save_roster`: busy/파일 변경 전 가드, 실패 시 설정·로스터·생성 파일 복원 | [원인 재현 7건](evidence/M8_safety_red.log), [서버 회귀 13개](evidence/M8_server_green.log), [교체 중 취소 재현](evidence/M8_reconfigure_cancel_red.log), [반복 취소 red](evidence/M8_reconfigure_repeated_cancel_red.log)·[성공 유지/실패 롤백 green](evidence/M8_reconfigure_repeated_cancel_green.log), [실제 DDS](evidence/M8_reconfigure_dds.log): SIGINT 거부 preflight → reset IDLE → 저장409 → 기존 IO 유지 → 다음 preflight/estop land4·cancel2 |
| A2 | [server.py](../server.py) `shutdown`: 첫 await 전에 SIGINT·SIGTERM을 경고 기록 후 무시하는 핸들러로 교체 | [실제 DDS 종료 7종](evidence/M8_server_shutdown.log): +1초 SIGINT와 SIGTERM 각각 재전송, BT SIGINT는 1회, 직접 land4·cancel2·조건부 disarm 유지 |
| A3·종료 결정 | [fleet.py](../fleet.py) `close`: 신호 없이 감시/소유권만 해제, PID·argv·신원·applied 영수증 보존. `stop_stack/restart_stack`만 IDLE에서 두 자식을 병렬 정지. [server.py](../server.py) 로컬 stop API·mock 상태, [fleet-ui.js](../static/js/fleet-ui.js) 관리자 ‘스택 정지’ 버튼 | [결정 후 플릿·체크리스트106개](evidence/M8_stack_decision_green.log), [DDS 종료7종](evidence/M8_server_shutdown.log): 스택2개 생존·신원/영수증 불변, BT/preflight 정리, EXIT_BOUND=6초 유지. [정지 UI](evidence/M8_fleet_ui.log), [정지 후 화면](img/M8_stack_stopped.png) |
| A4 | [verify_m5_server_shutdown.py](../tests/verify_m5_server_shutdown.py) `bt-crash`: 실제 자식 `os._exit(1)` | [DDS 로그](evidence/M8_server_shutdown.log): land4·cancel2, BT 신호0, arm0, `last_error=BT exited rc=1`. 첫 축약 시퀀스 확인 후 더미 preflight는 자발 종료시켜 서버 정리와 구분 |
| A5 | [runner.py](../runner.py) `_control`: ROSIO의 1초 경계보다 긴 1.5초 방어 상한, 예외는 `repr` | [안전 회귀](evidence/M8_safety_green.log), [최신 서버 회귀](evidence/M8_server_green.log): 1.03초 응답 인정·빈 TimeoutError 사유 제거 |
| A6 | [runner.py](../runner.py) `_watch`: DONE에서도 소유 preflight 사망 감시, 살아 있는 BT에 전체 순서 | [test_m8_safety.py](../tests/test_m8_safety.py), [서버 회귀](evidence/M8_server_green.log) |
| B1 | [dashboard.yaml](../config/dashboard.yaml) `nodes.aideck=/aideck/aideck_aruco_node` | [플릿 DDS](evidence/M8_fleet_dds.log): 실제 namespaced 노드를 external로 인식, 중복 spawn 거부·`global.aideck` 통과 |
| B2 | [fleet.py](../fleet.py) `generate_config`: 미입력 URI 스페어 제외+경고, 역할 URI/IP는 오류 유지. [server.py](../server.py) `--check-config`에서 생성 실패도 FAIL/exit1 | [생성 CLI 회귀](evidence/M8_config_check_green.log), [경고 재현](evidence/M8_config_warning_red.log), [동봉 설정 첫 기동](evidence/M8_bundled_check_config.log): 역할4 생성·스페어6 제외 경고 |
| B3 | [config.py](../config.py) `_inventory`: 템플릿의 enabled 값과 무관하게 역할 이름↔URI 대응 | [플릿100개](evidence/M8_fleet_green.log), [동봉 설정](evidence/M8_bundled_check_config.log) |
| B4 | [fleet.py](../fleet.py) `selected_type`: 템플릿 역할 타입 유지, 역할 타입 부재/스페어는 `fleet.robot_type`. 선택값이 robot_types에 없으면 오류 | [플릿 회귀](evidence/M8_fleet_green.log), [설정 회귀 red](evidence/M8_fleet_config_red.log). cf21 기본값은 YAML에만 둔다 |
| B5 | [dashboard.yaml](../config/dashboard.yaml) `aideck_template` 명시, 코드 경로 폴백 제거, [README](../README.md) 문서화 | [플릿 회귀](evidence/M8_fleet_green.log): 누락 설정 거부 |
| B6 | [admin-model.js](../static/js/admin-model.js), [admin.js](../static/js/admin.js), [admin.html](../static/admin.html): 역할 외 체크 중 warning만 실패 목록 아래 “여유 기체 주의”로 표시 | [브라우저 회귀](evidence/M8_frontend_browser_green.log), [주의 화면](img/M8_spare_warnings.png). 기본 역할 표6행·전체 인벤토리 접힘 유지 |
| B7 | [server.py](../server.py) `fleet_action`, [fleet.py](../fleet.py) `save_roster`: expected_hash 키 필수. 현재 해시가 null인 고장 설정 복구에는 명시적 null을 비교 허용 | [HTTP·UI](evidence/M8_fleet_ui.log): 누락409·READY409, [플릿 회귀](evidence/M8_fleet_green.log) |
| C1 | [glass.css](../static/css/glass.css): low 모드 내러티브/소개는 남색 .78, 나머지는 흰색 .08, backdrop-filter 해제 | [브라우저 계산 스타일](evidence/M8_frontend_browser_green.log), [low 화면](img/M8_fx_low.png) |
| C2 | [twin-model.js](../static/js/twin-model.js): pose 신선도만으로 fresh 판정, waiting과 offline 분리 | [Node 회귀](evidence/M8_final_node.log): waiting에서 fresh pose opacity1·고도 스템 유지 |
| C3 | [admin.css](../static/css/admin.css), [admin.js](../static/js/admin.js): 실제 배너 높이로 sticky 제어 바 offset | [브라우저 bounds](evidence/M8_frontend_browser_green.log): 배너 bottom44, 제어 top44, estop top50. [스크롤 화면](img/M8_admin_disconnected_scrolled.png) |
| C4 | [admin.js](../static/js/admin.js): 연결 끊기면 pending·ACK 타이머 제거, “서버 연결 없음 — 명령을 보낼 수 없습니다” | [브라우저 회귀](evidence/M8_frontend_browser_green.log): ACK timeout 전후 끊김·1초 후 문구 유지·페이지 재로드 없는 재접속 |
| D1 | [stage_probe.py](../tests/stage_probe.py): 기존 출력 파일은 timestamp 경로 사용, 두 파일 예약 실패 시 새 빈 파일 롤백, 오류는 짧은 안내. Stage 문서에 실제 출력 경로 사용 안내 | [킷20개](evidence/M8_install_kit_green.log), [red4건](evidence/M8_install_kit_red.log) |
| D2 | 배포본 [install.sh](../install.sh)를 build/install 없는 새 Ubuntu/일반 사용자에서 실행 | [clean 로그](evidence/M8_install_clean.log), [검증 조건·명령·해시](evidence/M8_install_manifest.log). 구 `M7_install_clean.log`는 [_superseded](evidence/M7_install_clean_superseded.log)로 보존 |
| D3 | 원본 JSONL/pose CSV를 재현 가능한 gzip으로 evidence에 동봉. [M7 manifest](evidence/M7_stage_manifest.log)에 원본↔분석 사본↔압축 파일 대응 | [JSONL 전체](evidence/M7_stage_full.jsonl.gz), [CSV 전체](evidence/M7_poses_full.csv.gz), [압축 해제 SHA256 대조](evidence/M8_install_archive.log). 기존 미압축 logs/는 저장소 미포함 |
| D4 | [kiosk.sh](../kiosk.sh) dry-run X 검사 생략. 실제 실행의 비-X11 거부 유지, [README](../README.md) Xorg 재로그인 절차·[M7 보충](M7.md) | [DISPLAY 없음·Wayland dry-run exit0](evidence/M8_install_kiosk_no_x.log) |
| D5 | [install.sh](../install.sh) setup_env source 전에 ~/COSHOW 경로 검사·명확한 오류. README 1단계 blocking 전제 | [잘못된 clone exit2](evidence/M8_install_wrong_clone.log), [외부 overlay 없는 merged 설치](evidence/M8_install_merged.log). 보호 setup_env.sh는 변경하지 않음 |
| 스택 고아 | [fleet.py](../fleet.py) `recover_stacks`: pid·argv·시작 신원 영수증 일치 시 재입양, applied_hash 복원, 없으면 재기동 필요. mismatch external·죽은 pid 삭제·살아 있는 pid 또는 graph로 spawn 차단. [checklist.py](../checklist.py) 고아 행에 스택 포함 | [재입양 회귀](evidence/M8_fleet_green.log), [DDS](evidence/M8_fleet_dds.log), [shebang 명령 red](evidence/M8_fleet_shebang_red.log). 부분 spawn 실패/반복 요청 취소에도 이미 생성한 자식 유지·applied_hash=null·실패 사유 기록. [반복 취소 재현](evidence/M8_stack_repeated_cancel_red.log). PID 기록 실패도 [실제 자식 생존 회귀](evidence/M8_decision_server_green.log)로 확인 |
| GPU 기록 | [Stage A](../docs/stage_a.md)에 Chrome 작업 관리자 GPU 메모리 시작/30분 스크린샷·바이트 기록란 추가 | v2.6 §1.2 결정 반영. 실제 GPU 측정은 사람 수행, M4 자원 정체 증거는 그대로 보존 |

## 증거

- [최종 검증 소스·자산 SHA256](evidence/M8_final_source_sha256.log): 보고서 자체를 제외한
  101개 파일로 검증 대상과 인계본을 대조할 수 있다.
- 최신 작업본 Python 3.10.12 / ROS Humble 전체: **469 passed**.
  [전체 명령 결과·의존/오프라인/범위 감사](evidence/M8_final_python.log).
  `docker exec coshow-m4-images-test bash -lc 'source /opt/ros/humble/setup.bash && source /ws_build/install/setup.bash && cd /ws && python3 -m pytest dashboard/tests -q && python3 dashboard/tests/verify_m2_constraints.py && python3 dashboard/tests/verify_m1_scope.py'`
  이 컨테이너는 기존 이미지 실행 테스트용 OpenCV4.10 wheel 포함. 배포 환경은 아래 별도 설치로 확인했다.
- 기본 Docker 하네스: **462 passed, 7 skipped**.
  [원형 하네스 전체 결과](evidence/M8_harness_python.log).
  `bash dashboard/docker/run.sh python3 -m pytest dashboard/tests -q`.
  OpenCV 미설치 시 시뮬 이미지 실행7개 skip이며 관련 AST 검사는 실행된다.
- Node **51 passed**: `node --test dashboard/tests/test_*.mjs`.
  [전체 로그](evidence/M8_final_node.log). verify/measure 스크립트는 test 글롭에 넣지 않는다.
- 실제 DDS: `ROS_LOCALHOST_ONLY=1`, 종료 프로브 domain90, 어댑터 재구성 domain91,
  플릿 프로브 domain92. 더미 child·더미 ROS 노드/서비스만 사용했다.
  [A1](evidence/M8_reconfigure_dds.log), [종료7종](evidence/M8_server_shutdown.log),
  [플릿/재입양/명시적 재기동](evidence/M8_fleet_dds.log), [생성 파일 diff](evidence/M8_fleet_generated_diff.log).
  위 ROS 컨테이너의 같은 source/cd 환경에서 각각
  `ROS_DOMAIN_ID=90 ROS_LOCALHOST_ONLY=1 python3 dashboard/tests/verify_m5_server_shutdown.py`,
  `ROS_DOMAIN_ID=91 ROS_LOCALHOST_ONLY=1 python3 dashboard/tests/verify_m8_reconfigure.py`,
  `ROS_DOMAIN_ID=92 ROS_LOCALHOST_ONLY=1 python3 dashboard/tests/verify_m6_fleet.py`를 실행했다.
  동봉 설정은 `ROS_DOMAIN_ID=91 ROS_LOCALHOST_ONLY=1 python3 dashboard/tests/verify_m8_bundled_config.py`로 재현한다.
  종료7종은 설정 land.duration=.25초의 테스트이며 EXIT_BOUND=6초. 실제 BT 기본8초의 종료 시간으로 해석하지 않는다. 스택은 서버 종료 시 유지되므로
  6초 상한에 스택의 SIGINT 대기 시간을 더하지 않는다. 반복 신호 전 1초 대기도
  최초 SIGINT부터의 총 6초에 포함하도록 프로브의 남은 시간을 계산한다. 명시적 스택 정지/재기동의
  별도 10초 대기·SIGKILL 경계는 플릿 DDS 프로브에서 확인했다.
- 병렬 검증 중 한 종료 실행에서 pose 수신 age가 2.235초로 정체되어 disarm 기대가 실패했다.
  실행기는 신선하지 않은 pose에 disarm을 보내지 않았으며 land4·cancel2는 전송했다.
  [실패 원본](evidence/M8_server_shutdown_stale_pose_failure.log)을 보존했다.
  fixture나 안전 조건을 완화하지 않은 단독 재실행은 7종 통과했다. 발행·DDS·수신 중
  어느 구간이 정체됐는지는 이 로그만으로 확정하지 못했다.
- `--mock` 서버를 최신 Python 코드로 재시작한 뒤 `verify_m5_admin.mjs`와
  `verify_m6_admin.mjs` 재실행. [현재 정적 소스 해시와 제어 흐름](evidence/M8_admin_current_source.log),
  [로스터 HTTP/UI](evidence/M8_fleet_ui.log). 과거 M5 snapshot 라우팅을 제거했다.
  [READY](img/M8_admin_ready.png) · [RUNNING](img/M8_admin_running.png) ·
  [LANDING](img/M8_admin_landing.png) · [로스터 확인](img/M8_roster_confirm.png).
- clean·merged 설치는 별도 pristine 컨테이너의 non-root UID1000, `env -i`에서 검증했다.
  clean은 build/install 없는 상태에서 설치했다. merged는 자체 merged 인터페이스를
  먼저 빌드한 뒤 installer를 실행했으며, 외부 overlay를 사용하지 않았다.
  최신 installer SHA256 `bd11f8fa84d0f76b060d9f722a06ce4b9a3004189fd7e436f07340df2d83a41e`.
  Ubuntu apt OpenCV4.5.4의 실제 ArUco marker1 검출/JPEG/노드 import 통과,
  source YAML3개·setup_env 보존, 일반 사용자 소유권 확인.
  [전체 재현 명령/이미지 ID/로그 해시](evidence/M8_install_manifest.log).
  Xvfb는 가상 모니터이며 실제 듀얼 모니터·GPU·거리 가독성 증거가 아니다.

## 스펙과 다르게 한 것

- B4의 리뷰는 `fleet.robot_type`, v2.6 §5.3 예시는 루트 `robot_type`이다.
  리뷰 키를 기본 YAML에 쓰되 루트 키도 호환 별칭으로 지원했다. 둘이 다르면
  `fleet.robot_type`을 선택하고 경고한다. 어느 표기로도 코드를 바꿀 필요가 없다.
- A1 리뷰 표의 “구 IO close → runner.io 교체”보다 v2.6 §5.5 본문의
  “실행기 참조 교체 → 구 IO close”를 적용했다. 구 IO 정리 예외 동안에도
  실행기가 살아 있는 새 어댑터를 가리키게 하기 위함이다. 제어 시퀀스는 변경하지 않았다.
- `--check-config`는 현장 미입력이나 없는 ROS 그래프를 녹색으로 바꾸지 않는다.
  동봉 설정의 최초 로스터 조건에서 생성은 PASS/스페어6은 WARN이지만,
  역할 LIMO IP2개 미입력과 그래프 부재로 CLI는 exit1이다.
  [기존 빈 run/roster.yaml을 보존한 실행](evidence/M8_existing_incomplete_roster_check_config.log)은
  역할 미배정 FAIL을 유지한다. 부트스트랩은 저장 로스터가 **없을 때만** 적용한다.
  기존 배정을 자동 덮어쓰지 않으며 관리자 로스터 편집으로 복구한다.
- 비-X11 실제 키오스크 실행 거부와 GPU 증거 Stage A 이관은 이번 리드 결정이다.
  이전 M7 설치 증거의 한계는 M7 보고서와 manifest에 명시하고 원본을 보존했다.

## 열린 질문 / 사용자 결정 필요

구현을 막는 열린 질문은 없다. 2026-09-14 결정으로 §5.8 우선, A3 자동 종료 요구 철회,
서버 종료/부분 기동 실패 시 스택 유지, IDLE 명시적 정지 버튼 추가를 확정했다.
[결정 원문](evidence/M8_stack_shutdown_decision.md), [원문 SHA256](evidence/M8_sources.log).
실제 기체 URI/IP·LIMO 상태 타입·현장 GPU 메모리 측정은 Stage A–D 담당자가 채운다.

## 다음 마일스톤에서 알아야 할 것

1. 후속 전체를 담은 dashboard 한 커밋을 리드가 재확인한다. 자동 종료 정책 변경은
   새 결정 원문과 위 검증 로그로 대조한다.
2. 리드 재확인 후 Stage A. 관찰 전용이며 관리자 실행 버튼은 쓰지 않는다.
3. Stage A는 Webots와 3D 비교·카메라4타일·phase8개·GPU 시작/30분 증거를 사람이 남긴다.
   Stage B 전 실제 URI/IP, Stage C LIMO 타입/전압, Stage D 비행/착륙은 아직 미검증이다.
