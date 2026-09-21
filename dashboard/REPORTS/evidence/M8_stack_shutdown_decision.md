# M8 결정 — 서버 종료·부분 기동 실패 시 스택 처리 (2026-09-14)

작성: Fable · 대상: Codex Astra · 참조: `FINAL_review_and_stageA_go.md` §2 A3, 스펙 v2.6 §5.8

## 결정

**스펙 §5.8이 우선한다. 자동 종료 경로는 없다.** 리뷰 A3의 "서버 종료 시 `fleet.close()`가 두 스택을 병렬 종료" 요구는 리드가 §5.8과 모순되게 넘긴 것이므로 **철회**한다. 병렬화(`gather`)는 관리자가 IDLE에서 누르는 명시적 `restart_stack`·`stop_stack`에만 적용한다(이미 구현됨).

근거: 스택의 수명은 run 주기가 아니라 하루 운영과 같다. 대시보드 재시작(설정 변경·크래시 복구)마다 라디오 스택을 내리면 10대 재연결에 시간이 들고, 비행 중 대시보드가 죽는 최악의 경우에는 오히려 스택이 살아 있어야 BT의 착륙 명령이 기체에 닿는다. 재입양(`recover_stacks`)이 이 정책의 반대편 절반이다.

## 파생 규칙 (구현·검증 대상)

1. **서버 종료(`Dashboard.shutdown` → `fleet.close()`)**: 소유 스택 자식에 **신호를 보내지 않는다**. 소유권만 해제한다 — pidfile·`run/stack.applied.json`(applied_hash·argv·신원)을 그대로 남겨 다음 대시보드가 재입양하게 한다. 이벤트 "스택은 계속 실행 중 — 다음 기동에서 재입양". `runner.close()`의 estop 시퀀스(BT·preflight)는 종전대로 먼저 완료한다.
2. **부분 기동 실패**(예: crazyflie_server는 떴는데 aideck spawn 실패): 이미 뜬 자식을 **죽이지 않는다**. `stack.crazyflie_server = up(owned)`, `stack.aideck = down`, `applied_hash = null`(→ 체크리스트 "스택 로스터 일치" 실패, "스택 재기동 필요" 표시), 이벤트에 실패 사유. 관리자가 IDLE에서 재기동을 누르면 그때 정리·재spawn.
3. **명시적 정지**: 관리자 화면에 "스택 정지" 버튼이 없으면 추가한다(IDLE에서만, `restart_stack`과 같은 종료 경로, `gather` 병렬). 스펙 §5.8의 "스택 기동/정지" 문구와 일치.
4. **종료 프로브(스택 자식 2개 시나리오)**: 서버 SIGINT 후 두 자식이 **살아 있고** pidfile·applied.json이 남으며, BT/preflight는 종전 시퀀스대로 정리됨을 단언. `EXIT_BOUND`는 스택 종료 시간을 포함하지 않으므로 현행 6 s 유지.
5. 스펙 §5.8·§5.5에 위 문장을 반영했다(v2.6 유지, 문구 추가).

이 결정으로 M8 전체를 **한 커밋**으로 푸시하고 `REPORTS/M8_followup.md`를 최종본으로 갱신하라. 그 뒤 리드가 재확인(Docker 전체 테스트, DDS 프로브 6종 + 스택 자식 2개 시나리오, `--check-config` 동봉 설정, 관리자 스크린샷)하고 Stage A로 간다.
