#!/bin/sh
# Webots 를 GUI 로 띄운다.  사용:  ./run_gui.sh worlds/practice_check.wbt [모드]
#   모드: realtime(기본) | run(그리면서 최대 속도) | fast(안 그리고 최대 속도)
# 콘솔 출력은 화면과 run_gui.log 에 동시에 남는다.
set -e
WORLD="${1:-worlds/practice.wbt}"
MODE="${2:-run}"
WORLD_ABS="$(cd "$(dirname "$WORLD")" && pwd)/$(basename "$WORLD")"

echo "Webots GUI 실행: $WORLD_ABS  (모드: $MODE)"
exec /Applications/Webots.app/Contents/MacOS/webots \
  --mode="$MODE" --stdout --stderr "$WORLD_ABS"
