# 운영 대시보드 구현 기록

설계 기준: `astra_handoff_2026-09-13_final_review.zip` v2.6와
`reviews/FINAL_review_and_stageA_go.md`, `06_staged_test_plan.md`.
[원문 SHA256](REPORTS/evidence/M4_sources.log).
대화의 사용자 지시를 문서보다 우선한다.

작업 브랜치: `HOSEONGI/COSHOW_Sim:dashboard`, 기준 `2a464d4`.
시각 참고: [`Design.md`](Design.md)의 Apple 분석을 M3–M5에서 기존
남색·글래스·Pretendard·안전 버튼 계약 안의 여백·위계·마감에 적용한다.

## 마일스톤

- [x] M1: ROS 관측 발행기, AI Deck 초기화 수정, 레인 포팅 및 Docker 증거
- [x] M2 첫 커밋: M1 후속 4건, 회귀 13개, 연속 Status 발행률, 보고서 부록 (`6c0de20`)
- [x] M2: 설정 기반 ROS 어댑터·플릿 관측·집계·WebSocket·mock·체크리스트·ping·check-config
- [x] M3 첫 커밋: M2 후속 8건 + 독립 리뷰 타입 누락 수정, Docker 224개 회귀 (`ba63352`)
- [x] M3: 동봉 폰트·three.js, 디자인 토큰, 참관자 레이아웃 FHD·4K·대기 검증, 대수 변경·재접속 검증
- [x] M4 구현: 역할 6대 3D·임베드 카메라·내러티브·상태별 화면 검증
- [x] M4 GPU 증거 결정: 32분 자원 정체 인정, GPU 바이트 시작/30분은 Stage A 사람이 기록(v2.6)
- [x] M4 첫 커밋: M3 후속 4건, 명시적 hello 계약·JS 리터럴 검사·우하 연결 표시·4K 헤더 내부 넘침
- [x] M5: 운영 화면·프로세스 관리·비상착륙·BT/preflight 감시와 Docker 안전 순서 검증
- [x] M6 구현: 기본 역할 6대·접힌 교체 설정·로스터·생성 설정·스택 기동/재기동
- [x] M6 재시작 정책 결정: 스택 신원 확인 후 재입양, applied_hash 복원(v2.6)
- [x] M7: 설치·실행·키오스크 스크립트, runbook, Stage A–D 킷, Docker 통합 424/Node 46 검증, FINAL 인계
- [x] M8 후속 한 커밋: HIGH 3건·MEDIUM·스택 고아 정책, 항목별 회귀·DDS·HEAD 화면·최신 clean 설치 증거
- [x] M8 계약 결정(2026-09-14): §5.8 우선·A3 철회. 서버 종료/부분 기동 실패 시 스택 유지, IDLE 명시적 정지/재기동만 병렬 종료
- [ ] 리드 재확인 → Stage A–D: 사람이 실행한 결과의 수정/재검증

M4→M7은 승인 대기 없이 연속 진행한다. M7 뒤 `REPORTS/FINAL.md`와 단계별 테스트 킷을 제공한다.
각 마일스톤은 직접 실행한 증거, `REPORTS/M<n>.md`,
`dashboard: M<n> ...` 커밋 및 해당 브랜치 푸시로 남긴다.

## v2.3에서 해결된 계약 질문

- 외부 BT/preflight는 노드 개수, 종료 후 20초 DDS 잔류 유예, 종료 후
  최근 3초 실수신 2건을 함께 판정한다. 소유 프로세스는 허용한다.
- preflight 사망 시 살아 있는 BT에는 ①②②'③⑤⑥ 전체 순서를 적용한다.
- mock 시각은 근사치이며 실제 BT phase와 LED 전환 순서를 우선한다.
- `run.external_bt` 관찰 예외, DONE의 mission age 무시를 적용한다.
  DONE에 도달해도 살아 있는 BT pid는 유지한다.
- 자기 SIGINT 뒤 BT rc=-6도 요청한 종료다. 종료 코드를 성공 판정에 쓰지
  않는다. 자식에 RLIMIT_CORE=0; 원본 종료 경로는 수정하지 않는다.

## 적용한 세부 해석

- `search_progress`는 상세 규칙대로 드론별 정수 인덱스다.
- `hello.drones/limos`는 역할 목록, `state.robots`는 역할과 스페어 관측이다.
  스페어에는 카메라·드론 서비스·Nav2 클라이언트를 만들지 않는다.
- §5.3 기존 문자열 키를 유지하고 §5.7의 타입은 `types` 매핑에 추가한다.
- 물리 IP는 fleet 배정을 우선하고 기존 network 매핑은 fallback이다.
- LIMO 상태 타입·배터리 임계값은 확인될 때까지 정보 없음(non-blocking).
- 카메라 시작 게이트는 §5.5의 stream_ok와 fps를 사용한다. 프레임 age는
  전송 상태와 화면 수신 끊김 표시를 위해 별도로 유지한다.

## 검증 범위

로컬 mock과 ROS Humble Docker 하네스만 직접 실행한다.
Webots·실기체·AI Deck UDP 검증은 하지 않는다. 기존 시나리오
설정·XML·서버·fake_limo·월드를 수정하지 않는다. v2.5 §4.6은 기존
시뮬 검출 노드에 이미지/fps/stream_ok 발행을 추가하도록 명시적으로 허용한다.

## 최종 인계

M4 `107681f`, M5 `df6c0f6`, M6 `afe20b2`을 순서대로 커밋·푸시했다.
M7과 전체 수용 보류 2건은 [FINAL](REPORTS/FINAL.md)에 기록한다.
