# M2 코어 독립 검토 — 2026-09-13

검토 대상은 작업 트리의 `config.py`, 설정 YAML, `state.py`, `mock.py`,
`server.py`, `checklist.py`, `pinger.py`, `ros_io.py` 및 관련 core/checklist/pinger/ROS
테스트다. 기준은 인계 패키지 v2.3와 `M1_review_and_M2_directive.md`의 M2 범위다.
검토자는 구현 코드를 변경하거나 커밋하지 않았다.

## 판정

검토 중 설정 적응성 결함 1건을 재현해 전달했고, 구현 담당자의 수정 후 같은
조건에서 해결을 직접 확인했다. 현재 이 검토 범위에서 미해결 생산 코드 발견
사항은 없다. 이 판정은 최종 Docker·WebSocket 통합 수용 증거를 대체하지 않는다.

## 발견 및 해결 확인

**[P2, 해결됨] mock의 네임스페이스 노드 이름 분해 누락.**

최초 위치: `dashboard/mock.py`의 `MockWorld.apply` 마지막 노드 목록 구성
(수정 전 214행 부근). 설정의 노드 이름을 그대로 `(name, '/')`에 넣었다.
`nodes.server='/radio/server'`, `nodes.aideck='/vision/camera'`로만 바꾸면
`[('/radio/server', '/'), ('/vision/camera', '/')]`가 생성돼 체크리스트가 찾는
`('server', '/radio')`, `('camera', '/vision')`와 달랐다. 두 전역 노드 행이
blocking FAIL이 되어 정상적인 이름 변경 뒤 mock 시작을 막았다.

구현 담당자가 basename/namespace 분리를 추가했다. 수정 후 같은 입력을 직접
재실행해 아래 결과를 확인했다. `test_mock_accepts_absolute_namespaced_nodes`
회귀 테스트도 확인했다.

```text
MOCK_NAMES [('server', '/radio'), ('camera', '/vision'), ('preflight', '/')]
RENAMED_READY READY []
```

## 주요 검토 결과

- **소켓 격리와 전송 제한:** 집계 루프는 각 `SocketSlots`에 상태와 프레임을
  제공할 뿐 소켓 송신을 기다리지 않는다. 소켓마다 sender 1개, 교체 가능한 상태
  슬롯 1개와 드론 인덱스별 프레임 슬롯을 갖고, 각 send에 5초 `wait_for`가 적용된다.
  timeout 후 close도 제한한다. hello는 sender 등록 전에 한 번 전송한다.
  slow/fast 소켓 회귀 테스트는 느린 연결 종료와 다른 연결의 최신값 전달을 확인한다.
- **수신 시각과 저장 크기:** Store는 수신 시 `clock()`을 기록하고 snapshot에서
  monotonic age를 계산한다. ROS header나 JSON의 발행 `t`로 신선도를 판단하지 않는다.
  변경된 `ROSIO._dispatch`는 스레드 안전 Store를 직접 호출하므로 asyncio 지연이
  수신 시각에 추가되거나 프레임이 `call_soon_threadsafe` 큐에 누적되는 경로가 없다.
  프레임은 로봇별 최신 1장, event history는 200개/전송 30개로 제한한다.
- **ROS 입력과 타입:** 외부 인터페이스는 설정에서 확장하고 타입을 동적 로드한다.
  JSON의 중첩 led/cmd/P_N/search_progress 및 preflight drone/stage 형식을 검증한다.
  알려진 optional null과 누락 키는 필요한 범위에서 허용하고, 잘못된 입력은 해당
  callback에서 격리한다. JSON은 transient local, camera는 best effort,
  detections는 reliable/volatile로 구독한다. spare에는 land/arm/action client를
  만들지 않는다. check-config는 별도 관측 노드에서 실제 발행자·서비스/액션 서버를
  확인하므로 자기 구독자·클라이언트만으로 PASS하지 않는다.
- **설정과 체크리스트:** 역할/플릿 목록을 순회하며 기본 mock 14개 로봇을 처리한다.
  역할 이름 변경 및 다섯 번째 역할 추가 회귀 테스트가 있다. 누락 BT 설정의
  drones/limos/tolerances/durations/search/observe_point를 각각 직접 제거해
  해당 행의 unavailable 처리가 유지되고 evaluate가 예외 없이 반환함을 확인했다.
  기본 mock을 가상 시계로 점검 완료까지 진행한 결과는 `READY`, blocking 실패
  목록 `[]`였다. 경계 나이·배터리, 감시 생존, 외부 동일 이름 개수와 종료 grace,
  잔류 메시지 1건 제외, spare 무장 경고, 외부 스택의 로스터 확인 불가 등을 검토했다.
- **mock와 명령:** phase/LED의 observe→handover→search→capture→dispatch→rescue→return→done
  순서와 M1 JSON 형식을 확인했다. namespace 수정 후 설정 적응성 재현도 통과했다.
  visitor 명령은 거부되고 원격 admin은 WebSocket prepare 전에 차단된다.
  실모드의 네 명령은 M2 관찰 모드 거부 응답만 내며 실제 로봇 제어를 수행하지 않는다.
- **ping:** 셸 없이 유효 IP만 subprocess로 넘기며 플랫폼별 timeout 옵션을 사용한다.
  통신 timeout은 2초이고 timeout/cancel 시 child를 kill/reap한다. RTT 파싱 실패와
  미설정 IP는 nullable 관측으로 표현한다. 대상별 병렬 갱신과 취소 테스트를 확인했다.

## 검토와 증거의 범위

검토자가 직접 실행한 것은 위 namespace 실패/수정 재현, 기본 mock READY,
주요 BT 키 누락 시 체크리스트 처리, 대상 diff의 공백 검사다. ROS와 aiohttp를
포함한 최종 전체 테스트 및 통합 수용 검증은 리드가 병행 수행하므로 이 문서에서
독립 재실행으로 주장하지 않는다. ROS의 수신 시각/큐 수정과 중첩 JSON 검증은
완료된 소스를 확인했으며, 이를 별도 미해결 지적으로 중복 기록하지 않는다.

실모드 bootstrap은 기존 템플릿에서 확인되는 배정만 사용한다. 기본 파일에서
플릿 14대 외에 미배정 역할 슬롯 3개가 있으면 상태 행은 17개가 될 수 있으나,
그 슬롯은 `fleet_id=null`과 blocking 설정 오류로 표시한다. 이를 실제 물리 기체
17대로 해석해서는 안 된다. 정상적으로 역할 4개를 배정한 mock/fixture는 플릿
14개를 표현한다.

고아 PID의 실제 수집·프로세스 생존 감시와 착륙 시퀀스는 M5, 지속 로스터 편집·
생성 파일/해시·스택 기동은 M6 범위다. 이번에는 체크리스트가 제공된 context를
정확히 판정하는지를 검토했으며, 이 미래 실행 기능의 완료를 주장하지 않는다.
