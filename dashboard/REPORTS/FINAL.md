# 최종 인계 — M4–M7 — 2026-09-13

> v2.6 후속: 아래 수치·보류 판정은 M7 당시 기록이다. 리드는 스택 재입양 정책을
> 결정했고 GPU 바이트 증거를 Stage A로 이관했다. 2026-09-14 스택 종료 결정과 최신
> 수정·검증 상태는 [M8 후속 보고서](M8_followup.md)를 따른다. Stage A는 후속
> 한 커밋의 리드 재확인 뒤 사람이 시작한다.

역할 드론 4대와 LIMO 2대만 보이는 3D 참관자 화면, 임베드 카메라,
관리자 실행 제어, 로스터·스택 관리, 설치·키오스크·Stage A–D 테스트 킷을
구현했다. **자동 검증은 통과했으며 전체 수용 완료는 아래 2건 때문에 보류한다.**
Webots·실기체·실제 행사 모니터는 아직 검증하지 않았다.

## 마일스톤과 변경

| 단계 | 보고서 | 결과 |
|---|---|---|
| M3 후속 | [M4의 후속 절](M4.md) | 필수 4건을 첫 커밋 `efce545`으로 분리·푸시 |
| M4 | [트윈·영상·연출](M4.md) | 역할 6대 입체 무대, 카메라 4타일, 전체 mock/FHD/4K; GPU 메모리 화면 증거 보류 |
| M5 | [관리자·실행 주기](M5.md) | 명령 수용표·0.6초 hold·정확한 1.5초 연결 잠금·§5.5 안전 종료·고아 BT/preflight 복구 |
| M6 | [로스터·스택](M6.md) | 6대 기본 + 접힌 교체 설정, 생성/적용 해시·동시 저장 방어·소유 스택 기동; 스택 고아 정책 미결 |
| M7 | [설치·테스트 킷](M7.md) | 일반 사용자 설치·실제 ROS 이미지 런타임 자기검사·키오스크·runbook·Stage A–D |

M4 `107681f` → M5 `df6c0f6` → M6 `afe20b2`를 `dashboard` 브랜치에 커밋·푸시했다. M7은 이 보고서를 포함하는 인계 커밋이다. 각 보고서의 파일·
명령·출력·이미지가 수용 근거다. 마일스톤 사이에는 승인을 기다리지 않았다.

## 직접 확인한 것

- **Python 3.10.12 / ROS Humble: 424 passed.** 기존 시뮬 검출 영상 테스트에만 OpenCV 4.10 wheel을 추가한 [환경](evidence/M7_test_environment.log)이다. [전체 명령과 출력](evidence/M7_final_python.log).
- **Node v26.3.0: 46 passed.** [프런트엔드 회귀](evidence/M7_final_node.log).
- [설정·의존·오프라인 감사](evidence/M7_final_constraints.log): Python runtime 10개,
  작성 JS 9개(문자열 656/숫자 449), vendor 2개 제외. 외부 이름 상수·CDN·외부 fetch·
  dashboard UDP/socket 의존 없음. 런타임 추가 의존은 aiohttp/PyYAML뿐이다.
- [보호 범위·AST 감사](evidence/M7_final_scope.log): Python 3.10 문법 51개 파일,
  원본 BT/preflight/시뮬 검출의 허용된 추가분 이외 로직 보존. 협업자 YAML 3개 보존.
- 실제 DDS 더미 서비스로 land ×4·CancelGoal ×2·fresh-low arm(false), SIGINT 1회,
  비정상 종료·preflight 사망·종료 요청 뒤 rc=-6·중복 start/estop·reset 경합을 확인했다.
  열린 관리자 WS가 있어도 서버 SIGINT는 안전 종료 후 정상적으로 닫혔다.
  [production 서버 프로브](evidence/M5_server_shutdown_green_final.log).
- [M4 시각 검증](evidence/M4_ui_green.log): mock 7구간, stream false, IDLE, low,
  FHD/4K, 실제 배치 라벨 36개 대비 4.5:1 이상, 외부 요청·페이지 오류 0건.
  [대표 화면](img/M4_capture.png), [4K](img/M4_search_4k.png).
- [32분 브라우저 자원 측정](evidence/M4_memory_summary.log): heap/DOM/renderer 자원
  개수의 실제 측정. **이 값은 GPU 메모리 바이트 측정의 대체 증거가 아니다.**
- clean nonroot Ubuntu에서 실제 apt·colcon 설치, isolated/merged 모두 확인했다.
  [실제 ArUco/JPEG 런타임](evidence/M7_install_runtime_green_isolated.log),
  [merged](evidence/M7_install_runtime_green_merged.log),
  [원본 YAML·권한 보존](evidence/M7_install_runtime_preservation.log).
  기존 aideck ROS 패키지의 NumPy/OpenCV 의존도 검사한다. 대시보드 Python 의존과 구별한다.
- [106초 읽기 전용 mock 프로브](evidence/M7_stage_probe.log): state 1,059개,
  pose 14,826행, 8개 phase, 바이너리 수신 9.41–10.20fps.
  [JSONL 샘플](evidence/M7_stage_sample.jsonl), [분석](evidence/M7_stage_analysis.log).

설치 검증의 X11 모니터는 Xvfb 가상 1대다. 실제 듀얼 모니터·WebGL 하드웨어 가속·
10m 가독성 통과로 해석하지 않는다. 테스트 mock 주소·URI는 실제 장비 정보가 아니다.

## 남은 열린 질문과 제한

1. **스택 고아 PID 복구 정책.** §5.5 `run/*.pid`의 reset 정리와 §5.8의 독립 스택
   수명에 대해, 이전 대시보드가 남긴 스택을 외부 실행으로 유지할지, 역할 착륙 확인
   후 종료할지 사용자에게 질문했다. 답변 전 임의 적용하지 않았다. BT/preflight는
   신원·durable SIGINT 기록으로 복구한다. 스택은 graph에 보이면 external로 인식하지만,
   고아 PID만 남고 graph가 보이지 않는 구간의 통합 복구는 미완료다.
2. **Chrome 작업 관리자 GPU 메모리 수용 증거.** 최종 역할 6대 장면의 측정 중
   Mac 잠금으로 native 창에 접근하지 못했다. 잠금 해제를 요청했다.
   [도구 결과](evidence/M4_gpu_capture_blocked.log). 최종 장면에 대해 시작/종료 GPU
   메모리 스크린샷을 다시 수집해야 한다. 이전 prototype 이미지는 최종 증거에서 제외했다.
3. 실제 추가 기체 URI/IP, LIMO 상태 타입·전압 임계값, 실기체 status/pose 발행률,
   물리 착륙·Nav2 정지, 카메라 실영상·무선 간섭은 Stage A–D에서 확인한다.
4. 보호 대상 BT YAML의 운영 플래그는 자동 변경하지 않았다. 실제 fleet 입력과
   안전 설정이 완비되지 않으면 기본 실모드 체크리스트가 시작을 차단하는 것이 정상이다.

최신 표시 요청은 계약보다 우선해 반영했다. 참관자 무대와 관리자 기본 표는 6대이며,
교체 인벤토리·전체 관측·라디오 검사·스페어 무장 경고는 유지한다.

## 협업자용 Stage A 시작 절차

1. 기존 `~/COSHOW` 클론의 로컬 변경을 보관하고 `hoseon/dashboard`를 병합한다.
   신규 clone으로 firmware/build·ROS install을 대체하지 않는다.
   [7단계 runbook](../README.md)의 설치·환경·복구 절차를 따른다.
2. 인터넷이 있는 준비 장소에서 `bash dashboard/install.sh`를 일반 사용자로 실행하고
   자기검사 FAIL이 없는지 확인한다. 실제 URI/IP와 운영 설정은 운영자가 검토한다.
3. 먼저 `bash dashboard/run.sh --mock` 및 `bash dashboard/kiosk.sh`로 화면을 본다.
   mock 종료 후 실제 Webots 노드와 동일 ROS_DOMAIN_ID/RMW 환경을 사용한다.
4. 기존 Webots·fake_limo·외부 BT를 평소 순서로 실행하고 `bash dashboard/run.sh`와
   키오스크를 연다. **Stage A는 관찰 전용이다. 관리자 점검/시작/estop/reset 버튼은
   사용하지 않는다.** 시뮬은 status/arm/add_logging이 없어 실행 게이트를 통과하지 못한다.
5. 별도 터미널에서 아래 읽기 전용 프로브를 실행한다. 기존 외부 BT를 수동 운용하며
   mission_state, 외부 BT 배지, phase/LED, pose, 타일 영상, 발견 P_N을 대조한다.

   ```bash
   python3 dashboard/tests/stage_probe.py --fleet \
     --record dashboard/logs/stage_a.jsonl \
     --dump-poses dashboard/logs/stage_a_poses.csv
   python3 dashboard/tests/analyze_stage.py dashboard/logs/stage_a.jsonl
   ```

6. [Stage A 기대값·기록란](../docs/stage_a.md)에 실제 관측과 로그/이미지 경로를 적는다.
   통과 후 [Stage B](../docs/stage_b.md) → [Stage C](../docs/stage_c.md) →
   [Stage D](../docs/stage_d.md)로 간다. 벤치 B에서는 자동 preflight의 무장 직전 정지를
   사람 반응 속도에 맡기지 않는다. Stage D에서 정상 완주 2회·capture 비상착륙 1회·
   reset→READY 300초 미만·새 core/apport 없음과 실제 정지를 확인한다.
