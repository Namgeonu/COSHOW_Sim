# CO-SHOW 2026 운영 대시보드

Ubuntu 22.04 · ROS 2 Humble · Python 3.10용 운영 화면이다. 참관자 화면은
**역할 드론 4대와 LIMO 2대**를 하나의 3D 무대에 표시하고 카메라 4개를 타일에
임베드한다. 관리자 기본 표도 역할 6대다. 교체용 전체 인벤토리는 접힌
**기체 교체 설정**에서만 펼친다. 역할 수·이름·좌표는 설정에서 읽는다.

글꼴·three.js·샘플 JPEG는 모두 동봉했다. 런타임 Python 의존성은
`aiohttp`, `PyYAML`뿐이며 CDN·웹폰트·외부 요청·AI Deck UDP 수신기를 쓰지 않는다.
카메라는 ROS CompressedImage 토픽을 받는다. 수신 신선도는 로컬 monotonic 시계로
계산하며 헤더 stamp나 `use_sim_time`을 쓰지 않는다.

## 행사 머신에서 시작하는 7단계

1. **기존 협업자 클론을 갱신한다.** firmware 빌드와 ROS install이 들어 있는
   `~/COSHOW`에서 작업한다. 로컬 변경을 확인·보관하고 원격 브랜치를 병합한다.
   `hoseon` 원격이 이미 있으면 추가 명령은 생략한다. 갱신도 fetch → merge →
   설치 재실행 순서다. 대시보드는 협업자 설정 YAML을 덮어쓰지 않는다.

   **클론은 반드시 `~/COSHOW`에서 접근할 수 있어야 한다.** 기존 `setup_env.sh`가
   이 경로를 사용하므로 설치기는 다른 클론 경로를 변경 작업 전에 차단하고 안내한다.
   로컬 변경이 있으면 먼저 커밋하거나 `git stash push -u`로 보관하고, 병합 뒤
   `git stash pop`의 충돌을 검토한다. 설치를 위해 `setup_env.sh`를 고치지 않는다.

   ```bash
   cd ~/COSHOW
   git status --short
   git remote add hoseon https://github.com/HOSEONGI/COSHOW_Sim
   git fetch hoseon
   git merge hoseon/dashboard
   ```

2. **인터넷이 있는 준비 장소에서 설치한다.** 전체 스크립트를 sudo로 실행하지
   않는다. 필요한 시스템 패키지 설치에만 내부 sudo가 쓰인다. 마지막 자기검사
   표에 FAIL이 없어야 한다. 기존 ROS 인터페이스는 재사용하며 aideck 패키지는
   수정한 설치 모듈이 선택되도록 재빌드한다.

   ```bash
   bash dashboard/install.sh
   ```

3. **현장 설정을 확인한다.** `dashboard/config/dashboard.yaml`의 fleet URI/IP,
   역할·표시 이름, 명령 argv, `DISPLAY`, 타입, kiosk offset을 확인한다.
   BT YAML의 `preflight.required: true`, `emergency_land_on_exit: true`,
   `bt_visualiser.enabled: false`는 운영자가 검토해 설정한다. 대시보드가 원본
   BT YAML을 자동 수정하지 않는다. 추가 기체 URI/IP의 null은 실제 값 확인 후
   채운다. 역할 기체의 URI/IP는 필수다. URI 없는 스페어는 생성에서 제외하고
   경고하며, 생성하는 기체는 URI가 중복되지 않고 라디오당 5대 이하여야 한다.

   `aideck_template`은 `../ros2_ws/src/aideck_aruco_ros/config/drones.yaml`로
   명시한다. 역할 타입은 원본 `robots[역할].type`을 보존하며, 없거나 스페어면
   `fleet.robot_type`(동봉값 `cf21`)을 사용한다. 선택 타입은 원본 `robot_types`에
   있어야 한다. 최상위 `robot_type`은 호환용 별칭이며 두 값이 다르면 경고하고
   `fleet.robot_type`을 우선한다.

4. **mock으로 두 화면을 먼저 확인한다.** 이 모드는 ROS·실기체를 제어하지 않으며
   로스터와 생성 파일도 `dashboard/run/mock/`에 따로 저장한다.

   ```bash
   bash dashboard/run.sh --mock
   # 다른 터미널
   VISITOR_SCALE=1.0 bash dashboard/kiosk.sh
   ```

   관리자에서 점검 시작 → READY → 데모 시작을 0.6초 누르면 전체 타임라인이
   재생된다. `--mock-fail`은 준비 실패 예제다. 첫 현장 실행에서 `chrome://gpu`의
   WebGL/Compositing 가속을 확인한다. 필요하면 `FX=low`를 사용한다.
   전광판과 관람 거리에서 `VISITOR_SCALE`(0.7–1.6)을 조정하고 10m 가독성을 기록한다.
   mock 서버를 종료한 뒤 실모드로 진행한다.

5. **ROS와 대시보드를 같은 환경에서 실행한다.** 기존 방식으로 드론 서버·카메라·
   LIMO를 기동하거나, 설정이 완비됐다면 IDLE의 관리자 스택 기동을 사용한다.
   외부에서 실행한 스택은 관측할 수 있지만 대시보드가 종료·재기동하지 않는다.
   외부 스택의 로스터 일치 점검은 건너뛰고 **확인 불가**로 표시한다.
   BT와 preflight는 직접 실행하지 않고 관리자 버튼으로 시작한다.

   대시보드가 종료돼도 드론 서버와 카메라 스택은 계속 실행된다. 정상 서버 종료는
   BT·preflight의 착륙 시퀀스를 먼저 마치고 스택에는 종료 신호를 보내지 않는다.
   다음 대시보드는 남아 있는 PID·명령행·시작 신원을 확인해 자기 스택을 다시
   관리한다. `dashboard/run/`의 스택 PID와 소유권·적용 기록을 지우지 않는다.

   키오스크를 열기 전에 로그아웃하고 로그인 화면의 톱니바퀴 메뉴에서
   **Ubuntu on Xorg**를 선택해 다시 로그인한다. `echo "$XDG_SESSION_TYPE"`가
   `x11`인지 확인한다. 실제 키오스크는 Wayland나 접근 불가능한 X 서버에서
   종료 코드 2로 중단한다. `bash dashboard/kiosk.sh --dry-run`은 X 검사 없이
   생성될 명령만 표시하므로 원격 터미널에서도 설정을 검토할 수 있다.

   ```bash
   bash dashboard/run.sh
   # 같은 ROS_DOMAIN_ID / RMW_IMPLEMENTATION을 가진 다른 터미널
   bash dashboard/kiosk.sh
   ```

   서버 첫 로그에서 domain, RMW, 바인딩 주소를 확인한다. 기본 주소는
   [운영 화면](http://127.0.0.1:8080/admin.html),
   [참관자 화면](http://127.0.0.1:8080/visitor.html)이다.
   관리자 페이지·API·WebSocket은 loopback에서만 허용한다.

6. **점검 시작 → 필수 항목 통과 → 데모 시작.** 무장된 드론이 이륙하므로 시작은
   0.6초 길게 누른다. 비상 착륙은 즉시 동작한다. LANDING에는 모든 제어가 잠긴다.
   착륙 완료 후 ABORTED에서 리셋한다. 정상 DONE도 리셋 전까지 화면을 유지한다.
   관리자는 마지막 상태 수신 1.5초 이후 또는 소켓 종료 즉시 제어를 잠근다.
   모니터 배치가 바뀌었으면 5단계의 Ubuntu on Xorg 로그인과 창 위치를 다시
   확인한 뒤 운영한다.

7. **문제와 복구 증거를 남긴다.** `dashboard/logs/dashboard.log`, `bt.log`,
   `preflight.log`와 [단계별 프로브](tests/stage_probe.py)를 사용한다.
   백엔드가 응답하면 비상 착륙을 사용한다. 백엔드가 종료됐으면 우선 재시작한
   대시보드의 고아 프로세스 reset을 사용한다. 저장된 PID·시작 식별자·SIGINT
   기록을 대조하므로 이미 보낸 SIGINT를 중복 전송하지 않는다. 수동 대응은
   BT에 앞선 종료 요청이 없음을 확인한 경우에만 **Ctrl+C 한 번**을 사용한다.
   이미 LANDING이었거나 `dashboard/run/bt.signal.json`에 해당 프로세스의
   `sigint_sent: true` 기록이 있으면 두 번째 신호를 보내지 않는다. BT 설정의
   착륙 시간 + 2초(현재 rehearsal YAML은 8 + 2 = 10초)를 기다리고 preflight
   터미널에도 앞선 종료 요청이 없을 때만 Ctrl+C를 보낸다. 터미널이 없으면
   `dashboard/run/{bt,preflight}.pid`의 PID를 `ps -p PID -o args=`로 대조한 뒤
   같은 순서로 `kill -INT PID`를 한 번씩만 사용한다. 광범위한 이름 검색 종료는
   피한다. 재시작한 대시보드가 고아 BT/preflight를 보고하면 reset으로 정리한다.
   프로브 로그는 실제 착륙·차량 정지를 대신 증명하지 않는다.

## 역할 기체 교체

1. 실행을 끝내고 IDLE에서 **기체 교체 설정**을 펼친다. 기본 화면에는 여유 기체를
   표시하지 않는다. 스페어 무장 경고는 접힌 상태에서도 요약에 남는다.
2. 드론의 역할 드롭다운을 바꾼다. 추천 로스터는 신선한 링크·배터리 값을 정렬한
   제안이며 자동 저장하지 않는다. **배정 검토**에서 이전 → 이후 물리 기체를 확인한다.
3. **배정 저장** 후 생성 해시와 적용 해시가 달라져 시작이 차단된다. IDLE에서
   **스택 재기동**을 수행하고 배정 일치와 기체별 수신을 확인한다.
4. 점검을 다시 수행한다. 저장은 역할 배정, 카메라 인덱스, ROS 구독을 갱신하며
   교체 전 카메라·pose 수신값을 새 기체에 이어 붙이지 않는다.

실제 저장 위치는 `dashboard/run/roster.yaml`이다. 생성 파일은
`crazyflies.generated.yaml`, `drones.generated.yaml`, `preflight.generated.yaml`이다.
첫 파일은 원본 템플릿의 `robots`만 바꾸며 `robot_types`, `all` 등을 보존한다.
스택 프로세스는 데모 실행과 독립적으로 관리한다. 외부 프로세스는 소유하지 않는다.

기동 중 카메라만 실패해도 먼저 뜬 드론 서버는 계속 실행된다. 실패 사유와
`applied_hash: null`이 표시되고 **스택 재기동 필요** 상태로 실행이 차단된다.
설정을 고친 뒤 IDLE에서 **스택 재기동**을 눌러 남아 있는 소유 스택을 정리하고
두 프로세스를 다시 기동한다. 일부 기동 실패나 대시보드 종료만으로 스택을
자동 정지시키지 않는다.

하루 운영을 마치고 스택도 끄려면 데모를 종료하고 리셋해 IDLE로 돌아온 뒤
**스택 정지**를 누른다. 명시적인 정지·재기동만 소유 스택에 병렬로 SIGINT를
보내며, 10초 동안 종료하지 않는 프로세스에는 SIGKILL을 보낸다. 리셋은 스택을
정지하지 않는다. 외부 스택은 해당 스택을 실행한 터미널에서 종료한다.

대시보드 재시작 후에는 스택 상태와 로스터 일치를 확인한다.
`run/crazyflie_server.pid`, `run/aideck.pid`, `run/*.process.json`,
`run/stack.applied.json`은 `dashboard/` 아래에 보존된다. 명령행·신원이 일치하면
재입양하고, 적용 기록까지 유효하면 해시를 복원한다. 적용 기록을 확인할 수 없으면
IDLE에서 스택을 재기동한다. 신원을 확인할 수 없는 살아 있는 PID는 외부 스택으로
취급하므로 파일을 지우고 중복 기동하지 않는다.

LIMO는 해당 Jetson에서 역할 namespace로 재기동하고 dashboard의 fleet namespace와
IP를 맞춘다. 대시보드가 원격 Jetson이나 LIMO 프로세스를 직접 바꾸지 않는다.

## 이름·환경 변경과 진단

외부 토픽·서비스·노드·IP·타입은 `config/dashboard.yaml`, 영역·마커는
`config/field.yaml`, 관측점·베이스·레인·착륙 설정은 BT YAML에서 읽는다.
경로는 `dashboard/` 기준이고 `~`를 확장한다. BT의 `--config` 경로만
`commands.bt_cwd` 기준이다. 명령은 shell을 통하지 않으므로 `&&`, pipe,
리다이렉션 대신 argv와 `commands.env`를 사용한다.

역할 이름을 바꾸면 dashboard의 역할·display 매핑·로스터 키와 BT의 해당 역할
참조를 함께 바꾼 뒤 서버를 재시작한다. 코드의 문자열을 바꾸지 않는다.
현재 LIMO 상태 타입은 미확인이므로 null이면 정보 없음/SKIP이다. 확정되면
`types.topics`에 실제 타입을 입력한다. LIMO 배터리 임계값은 선택적이며 비차단이다.

```bash
bash dashboard/run.sh --check-config
bash dashboard/kiosk.sh --dry-run
python3 dashboard/tests/stage_probe.py --fleet --record dashboard/logs/session.jsonl --dump-poses dashboard/logs/poses.csv
python3 dashboard/tests/analyze_stage.py dashboard/logs/session.jsonl
```

기록 경로에 이전 파일이 있으면 원본을 보존하고 타임스탬프가 붙은 새 이름을
사용한다. 기록기는 JSONL·CSV를 모두 연 뒤 실제 경로를 `OUTPUT`으로 표시한다.
한 경로를 열지 못하면 이번에 만든 다른 빈 파일도 제거한다. 분석에는
`OUTPUT`에 표시된 이번 JSONL 경로를 사용한다. `--record --dump-poses`처럼
경로를 생략하면 `dashboard/logs/`에 시각을 붙여 자동 저장한다.

`--check-config`는 실제 publisher/service/action server를 검사한다. 자기 구독을
정상 서버로 세지 않는다. 치명 설정 오류·확인 가능한 필수 인터페이스 FAIL이면
종료 코드 1이다. 명시적 null 타입은 SKIP, 미기입 스페어 URI/IP는 WARN이다.
생성 가능한 기체 설정도 검사하므로 역할 URI/IP·타입·템플릿 오류는 FAIL이다.

## 검증과 인계

- [Stage A: Webots 관찰 전용](docs/stage_a.md): 시뮬은 status/arm/add_logging이
  없으므로 관리자 실행 버튼을 사용하지 않는다. 외부 BT 관찰 배지와 phase를 확인한다.
- [Stage B: 비행 없는 벤치](docs/stage_b.md), [Stage C: LIMO](docs/stage_c.md),
  [Stage D: 통합 완주·비상착륙](docs/stage_d.md).
- [최종 보고](REPORTS/FINAL.md), [M4](REPORTS/M4.md), [M5](REPORTS/M5.md),
  [M6](REPORTS/M6.md), [M7](REPORTS/M7.md), [동봉 자산·라이선스](static/VENDOR.md).

Docker는 Python 3.10·ROS 메시지·더미 프로세스와 서비스 순서를 검증한다.
mock은 화면과 프로토콜을 검증한다. Webots·실기체·물리 모니터는 사람이
Stage A–D 순서로 확인한다. 현장 테스트 전 결과를 실기체 승인으로 간주하지 않는다.

### 개발 검증 재현

제공된 Docker 하네스는 가벼운 이미지다. 기존 시뮬 검출 노드의 영상 회귀까지
포함한 424개 결과는 테스트 컨테이너에 OpenCV 4.10.0.84를 추가한 환경에서
측정했다. 기본 이미지에서 OpenCV가 없으면 영상 실행 회귀는 SKIP하고 AST 회귀는 그대로 실행한다.
다음은 인터넷이 있는 개발 환경에서만 실행한다. 운영 대시보드 의존성이나
행사 머신의 설치 패키지를 바꾸는 명령이 아니다.

```bash
bash dashboard/docker/run.sh bash -lc 'python3 -m pip install --no-deps opencv-contrib-python-headless==4.10.0.84 && python3 -m pytest dashboard/tests -q'
node --test dashboard/tests/test_*.mjs
```

[실제 검증 패키지 버전](REPORTS/evidence/M7_test_environment.log)을 함께 보존했다.
운영 AI Deck 노드는 별도 pip wheel 없이 Ubuntu OpenCV 4.5.4로 설치·검출을
검증했다. Webots의 기존 시뮬 환경과 운영 카메라 노드의 호환 경로를 구별한다.
