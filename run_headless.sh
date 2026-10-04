#!/bin/sh
# 헤드리스로 월드를 실행한다.  사용:  ./run_headless.sh [월드 — 기본 worlds/apartment_competition_check.wbt] [최대초]
#
# 컨트롤러가 "[[DONE]]" 을 찍으면 곧바로 끝낸다. 최대초는 그게 안 나올 때의 안전장치일 뿐이다.
#
# ⚠️ 상한은 **벽시계** 다. 그런데 채점 기준은 **시뮬 시간** 이고, 둘은 무관하다 —
#    Webots 의 robot.step(timestep) 은 컨트롤러가 얼마나 오래 계산했든 시뮬을
#    정확히 timestep 만큼 전진시키므로, 계산이 느려도 로봇의 행동은 한 틱도
#    달라지지 않는다 (증거: 사람 월드 3회 반복이 머신 부하와 무관하게 소수점까지
#    동일했다). 그러니 이 상한은 **행동을 재는 값이 아니라 멈춤을 막는 안전장치**다.
#    너무 낮게 잡으면 머신이 바쁠 때 멀쩡한 실행이 잘려 "실패" 로 보인다
#    (실측: 1400초 상한에서 comb0/comb2 가 시뮬 460/600초 지점에 잘렸다).
# (예전에는 무조건 최대초만큼 sleep 해서, 30 초에 끝난 일도 몇 분씩 기다렸다.)
set -e
export SAR_VIZ=0          # 헤드리스에서 창을 켜 봐야 느리기만 하다 (한 번 그리는 데 ~32 ms)

WORLD="${1:-worlds/apartment_competition_check.wbt}"
LIMIT="${2:-600}"
WORLD_ABS="$(cd "$(dirname "$WORLD")" && pwd)/$(basename "$WORLD")"
LOG="$(mktemp -t sar_headless)"

/Applications/Webots.app/Contents/MacOS/webots \
  --batch --mode=fast --no-rendering --minimize --stdout --stderr \
  "$WORLD_ABS" > "$LOG" 2>&1 &
PID=$!

# ⚠️ 예전에는 `sleep 1` 횟수를 초로 셌다. 한 반복에 sleep 1 + grep 이 들어가므로
#    CPU 경합이 있으면 한 반복이 1초를 넘어 **상한이 늘어난다** (실측: 상한 1400초
#    인데 33분이 지나도 안 끝났다). 시작 시각을 기준으로 재야 한다.
STARTED=$(date +%s)
elapsed=0
while [ "$elapsed" -lt "$LIMIT" ]; do
  if grep -q '\[\[DONE\]\]' "$LOG" 2>/dev/null; then break; fi
  # ⚠️ 아래 kill -0 이 보는 것은 **Webots** 다. 컨트롤러가 죽어도 Webots 는 살아
  #    있으므로 이 검사만으로는 상한까지 꽉 기다린다 (comb2 에서 1400초를 날렸다).
  #    그래서 컨트롤러 크래시 문구를 직접 본다.
  if grep -qE "controller crashed|^Traceback \(most recent call last\)" "$LOG" 2>/dev/null; then
    echo "[run_headless] ❌ 컨트롤러가 죽었다(크래시 또는 예외) — 기다리지 않는다" >&2
    break
  fi
  kill -0 "$PID" 2>/dev/null || break     # Webots 자체가 죽었으면 더 기다릴 것 없다
  sleep 1
  elapsed=$(( $(date +%s) - STARTED ))
done

kill "$PID" 2>/dev/null || true
wait "$PID" 2>/dev/null || true
cat "$LOG"
rm -f "$LOG"
echo "[run_headless] 실제 대기 ${elapsed}s (상한 ${LIMIT}s)" >&2
