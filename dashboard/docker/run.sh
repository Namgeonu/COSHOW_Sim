#!/bin/bash
# 사용:  bash run.sh                      -> 컨테이너 셸
#        bash run.sh python3 -m pytest dashboard/tests
#        REBUILD=1 bash run.sh            -> 인터페이스 재빌드
# 전제: FORK 경로에 HOSEONGI/COSHOW_Sim dashboard 브랜치 체크아웃
set -e
HERE="$(cd "$(dirname "$0")" && pwd)"
FORK="${FORK:-$HERE/../..}"
FORK="$(cd "$FORK" && pwd)"
docker build -q -t coshow-humble-dev "$HERE" >/dev/null
TTY=""; [ -t 0 ] && TTY="-it"
docker run --rm $TTY -e REBUILD="${REBUILD:-0}" \
  -v "$FORK":/ws -v coshow_build:/ws_build -p 8080:8080 \
  coshow-humble-dev "$@"
