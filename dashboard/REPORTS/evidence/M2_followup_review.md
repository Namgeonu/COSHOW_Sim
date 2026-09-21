# M1 후속 수정 독립 검토 — 2026-09-13

검토 기준: `73c14ea` 이후 작업 트리와
`M1_review_and_M2_directive.md` §2 필수 4건 및 §3 권고.
검토자는 구현 코드를 수정하거나 커밋하지 않았다. 신규 M2 기능은 검토 범위 밖이다.

## 판정

필수 4건의 구현은 모두 충족한다. 이번 수정에서 차단할 생산 코드 결함이나
스코프 위반은 발견하지 않았다. 아래 P3 테스트 허점 1건을 보완할 것을 권한다.

| 항목 | 확인 내용 |
|---|---|
| 숫자 command goal | `_DroneService.run`의 `sig[2:]` 변경만 허용한 AST 비교가 통과한다. GoTo·Takeoff·Land 회귀 테스트는 실제 서비스 액션의 bb 갱신과 JSON 배열, 같은 명령의 중복 호출 방지를 함께 확인한다. |
| 전이 필드와 연속값 구분 | `_publish_status` 비교 키가 stage/stages/ready/abort_reason과 기체별 kal_ok/pose_ok/sup_ok/sup_why/armed/can_fly로 한정된다. battery_v/pose_err/kal_range는 전체 페이로드에 유지되며 다음 하트비트 또는 전이 발행에 최신값이 포함된다. |
| ready 발행 순서 | `_publish_ready`와 `_set_ready`가 같은 RLock을 사용한다. 지연된 True 발행과 False 전이의 경합 테스트가 있으며 RED 로그에 기존 `[False, True]` 순서가 실제로 잡혀 있다. |
| BT 표시 예외 격리 | phase 계산·직렬화·ROS 발행을 같은 try/except로 감싸고 경고도 한 번만 시도한다. 경고 자체의 실패도 격리한다. pose 누락·정상 경로와 회복, 경고 실패를 테스트한다. |

## 발견 사항

**[P3] 실제 상태 전이를 넣어 비동기 하트비트 테스트의 전제를 복구해야 한다.**

위치: `dashboard/tests/test_preflight.py:143`
(`test_async_change_does_not_delay_next_heartbeat_for_two_seconds`).

현재 `Status()`는 초기값과 같은 armed=False/can_fly=False를 보내고
battery_v만 None에서 0으로 바꾼다. 이번 비교 키 변경 뒤 battery_v는 즉시 발행
대상이 아니므로 0.8초 시점의 `_on_status`가 전이 발행을 하지 않는다.
따라서 테스트는 이제 평상시 하트비트만 기다리며, 이름과 주석에 적힌
“비동기 변경 발행 직후 다음 하트비트가 약 2초 뒤로 밀리는 회귀”를 검증하지 못한다.

`Status.SUPERVISOR_INFO_IS_ARMED` 같은 이산 비트를 변경하고, `_on_status`
직후 발행 수가 정확히 1 늘었음을 먼저 단언한 뒤 다음 하트비트 시간을 측정하면 된다.
현재 생산 코드의 0.1초 확인 타이머와 1초 제한에는 이 결함이 없으며, 테스트 범위 문제다.

## 증거와 한계

- 저장된 `M2_followup_unit_red.log`에서 새 회귀 13건의 실패 원인을 검토했다.
  `M2_followup_unit_green.log`에는 92개 통과가 기록돼 있다. 독립 검토자가 Docker
  테스트 전체를 다시 실행한 것은 아니다.
- `M2_followup_live_green.log`는 실제 BT의 waiting_poses → observe,
  TRANSIENT_LOCAL 구독, 고정 상태 하트비트와 변동 battery 입력을 확인한다.
  4대가 6.4초 동안 보낸 Status 256건에 대해 preflight JSON은 6건,
  수신 간격은 1.002–1.069초다. RED 로그에서는 같은 종류의 입력에 JSON 162건을
  수신해 프레임 수 단언이 실패했다.
- 실제 ROS의 변동 Status 측정은 **stage 1 실패 이후**에 수행했다. stage 3의
  pose/kalman 연속값과 kal_ok 전이는 실제 생산 메서드를 호출하는 가상 시계
  단위 테스트로 확인한다. 이 증거를 ROS stage 3 완주나 실기체 검증으로 해석하지 않는다.
- launch 테스트가 `launch_ros.utilities.evaluate_parameters`의 실제 bool 결과를
  단언하도록 바뀌었다. `M2_followup_launch_boolean.log`에는 4개 통과가 기록돼 있다.
- 독립 검토자가 `python3 dashboard/tests/verify_m1_scope.py`를 직접 재실행해
  명시된 AST 보존 검사와 Python 3.10 문법 검사 통과를 확인했다. 출력이 실제
  비교 대상과 수정·추가 함수 목록을 명시한다. 검토 대상 파일의
  `git diff --check 73c14ea`도 통과했다.
- `REPORTS/M1.md`의 출처 표기·범위 한정·기존 독립 리뷰 문구 정리는 리드가
  병행 작성 중이므로 이 검토의 완료 대상으로 주장하지 않는다.

## 재검토 부록 — P3 해결

위 발견 사항은 최초 검토 시점의 기록이며, 후속 수정으로 **해결됨**을 확인했다.
`test_async_change_does_not_delay_next_heartbeat_for_two_seconds`가
`Status.SUPERVISOR_INFO_IS_ARMED`를 설정해 실제 이산 전이를 만들고,
`_on_status` 직후 발행 수가 정확히 1 증가하며 JSON의 `armed`가 True임을
단언한다. 그 발행 이후 다음 하트비트를 기다리므로 원래 검증 전제가 복구됐다.

수정된 테스트와 `M2_followup_unit_final.log`를 대조했다. 로그에는 M1 관련
네 테스트 파일의 **92 passed in 2.53s**가 기록돼 있다. 이번 재검토는 코드와
저장된 실행 로그의 확인이며, 검토자가 Docker 테스트를 별도로 재실행한 것은 아니다.

M1 필수 후속 4건과 이 검토에서 제기한 P3는 모두 확인 완료다.
현재 이 검토 범위에 미해결 발견 사항은 없다. 신규 M2 기능의 판정은 포함하지 않는다.
