#!/bin/bash
# 16개 월드를 Webots GUI 로 차례로 보여준다 (사람이 눈으로 보기 위한 것).
#
# 쓰는 법:  bash debug/gui_tour.sh [속도] [월드...]
#   속도: realtime (기본, 1:1) 또는 fast (빠르게)
#   월드를 지정하지 않으면 16개 전부.
#
# 보다가 다음 월드로 넘어가려면 그 Webots 창을 닫으면 된다.
# ⚠️ 월드의 로봇 배치와 config.START_X/Y 는 반드시 일치해야 한다 (config.py:95).
#    그래서 월드마다 고쳐 쓰고, 넘기기 전에 **다시 읽어 확인** 한다.
set -u
cd "$(dirname "$0")/.."

MODE="${1:-realtime}"; shift || true
if [ "$#" -gt 0 ]; then WORLDS="$*"; else
  WORLDS=""
  for L in maze open comb corridor; do for i in 0 1 2 3; do WORLDS="$WORLDS ${L}${i}"; done; done
fi

STARTS_X=(-2.5 0.0 -2.5 2.5)
STARTS_Y=(-2.5 -2.5 0.0 0.0)
BACKUP=$(mktemp -t sar_cfg)
cp config.py "$BACKUP"
restore() { cp "$BACKUP" config.py; rm -f "$BACKUP"; }
trap restore EXIT

n=0
total=$(echo $WORLDS | wc -w | tr -d ' ')
for W in $WORLDS; do
  n=$((n+1))
  L=$(echo "$W" | sed 's/[0-9]*$//'); i=$(echo "$W" | grep -o '[0-9]*$')
  cp "$BACKUP" config.py
  /usr/bin/sed -i '' "s/^START_X = .*/START_X = ${STARTS_X[$i]}              # [m]/" config.py
  /usr/bin/sed -i '' "s/^START_Y = .*/START_Y = ${STARTS_Y[$i]}              # [m]/" config.py
  GOT=$(.venv/bin/python3 -c "import config;print(config.START_X, config.START_Y)")
  if [ "$GOT" != "${STARTS_X[$i]} ${STARTS_Y[$i]}" ]; then
    echo "❌ $W: config 시작점 확인 실패 ('$GOT') — 멈춘다"; exit 1
  fi

  LOG=$(mktemp -t sar_gui)
  echo ""
  echo "=============================================================="
  echo "  [$n/$total]  $W    시작점 (${STARTS_X[$i]}, ${STARTS_Y[$i]})"
  echo "=============================================================="
  SAR_LAYOUT="$L" SAR_START="$i" \
    /Applications/Webots.app/Contents/MacOS/webots --mode="$MODE" \
      --stdout --stderr "worlds/${W}_check.wbt" > "$LOG" 2>&1 &
  PID=$!

  # 임무가 끝나면([[DONE]]) 다음으로. 창을 닫으면 Webots 가 죽고 역시 다음으로.
  while kill -0 "$PID" 2>/dev/null; do
    grep -q '\[\[DONE\]\]' "$LOG" 2>/dev/null && break
    sleep 2
  done
  sleep 2
  kill "$PID" 2>/dev/null || true
  wait "$PID" 2>/dev/null || true

  grep -E "찾은 목표물|걸린 시간|달린 거리" "$LOG" | sed 's/^/     /'
  rm -f "$LOG"
done
echo ""
echo "투어 끝."
