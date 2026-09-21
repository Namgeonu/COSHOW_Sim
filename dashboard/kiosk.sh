#!/usr/bin/env bash
set -eo pipefail
DASHBOARD_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
dry_run=0
config="$DASHBOARD_DIR/config/dashboard.yaml"
while (($#)); do
  case "$1" in
    --dry-run) dry_run=1; shift ;;
    --config) config="${2:?--config needs a YAML path}"; shift 2 ;;
    *) printf 'Usage: bash dashboard/kiosk.sh [--dry-run] [--config YAML]\n' >&2; exit 2 ;;
  esac
done
VISITOR_SCALE="${VISITOR_SCALE:-1.0}"
FX="${FX:-normal}"
python3 - "$VISITOR_SCALE" "$FX" <<'PY'
import math
import sys
try:
    scale = float(sys.argv[1])
    assert math.isfinite(scale) and .7 <= scale <= 1.6
except (ValueError, AssertionError):
    raise SystemExit('VISITOR_SCALE must be a finite number in 0.7..1.6')
if sys.argv[2] not in ('normal', 'low'):
    raise SystemExit('FX must be normal or low')
PY
if ((!dry_run)); then
  if [[ ${XDG_SESSION_TYPE:-} != x11 || -z ${DISPLAY:-} ]]; then
    printf "X11 required: 로그인 화면에서 'Ubuntu on Xorg'로 재로그인하세요.\n" >&2
    exit 2
  fi
  xrandr --listmonitors >/dev/null
fi
settings="$(python3 - "$config" <<'PY'
import sys
import yaml
with open(sys.argv[1], encoding='utf-8') as stream:
    config = yaml.safe_load(stream)
port = config.get('http', {}).get('port', 8080)
offset = config.get('kiosk', {}).get('visitor_offset_px', 1920)
assert type(port) is int and 1 <= port <= 65535, 'Invalid http.port'
assert type(offset) is int and offset >= 0, 'Invalid kiosk.visitor_offset_px'
print(port)
print(offset)
PY
)"
mapfile -t values <<< "$settings"
port="${values[0]}"
offset="${values[1]}"
browser=''
for candidate in google-chrome chromium chromium-browser; do
  if command -v "$candidate" >/dev/null 2>&1; then browser="$(command -v "$candidate")"; break; fi
done
if [[ -z "$browser" ]]; then printf 'Chrome/Chromium executable not found.\n' >&2; exit 2; fi
base="http://127.0.0.1:$port"
visitor_url="$base/visitor.html"
if [[ $FX == low ]]; then visitor_url="$visitor_url?fx=low"; fi
profile_root="$HOME/.cache/coshow-kiosk"
common=("$browser" --no-first-run --no-default-browser-check --disable-session-crashed-bubble)
admin=("${common[@]}" "--user-data-dir=$profile_root/admin" --new-window --window-position=0,0 --window-size=1280,900 "$base/admin.html")
visitor=("${common[@]}" "--user-data-dir=$profile_root/visitor" --kiosk "--window-position=$offset,0" "--force-device-scale-factor=$VISITOR_SCALE" "$visitor_url")
if ((dry_run)); then
  printf 'WAIT up to 30s for %s/visitor.html (HTTP success required)\n' "$base"
  printf '%q ' xset s off -dpms; printf '\n'
  printf '%q ' xset s noblank; printf '\n'
  printf '%q ' "${admin[@]}"; printf '\n'
  printf '%q ' "${visitor[@]}"; printf '\n'
  exit 0
fi
python3 - "$base/visitor.html" <<'PY'
import sys
import time
import urllib.error
import urllib.request
deadline = time.monotonic() + 30
while True:
    try:
        with urllib.request.urlopen(sys.argv[1], timeout=1) as response:
            if response.status == 200:
                break
    except (OSError, urllib.error.URLError):
        pass
    if time.monotonic() >= deadline:
        raise SystemExit('Dashboard not ready after 30s; start bash dashboard/run.sh first.')
    time.sleep(.25)
PY
mkdir -p "$profile_root/admin" "$profile_root/visitor" "$DASHBOARD_DIR/logs"
xset s off -dpms
xset s noblank
"${admin[@]}" >> "$DASHBOARD_DIR/logs/kiosk-admin.log" 2>&1 &
"${visitor[@]}" >> "$DASHBOARD_DIR/logs/kiosk-visitor.log" 2>&1 &
printf 'Opened admin and visitor with separate profiles. Exit: pkill -f coshow-kiosk\n'
