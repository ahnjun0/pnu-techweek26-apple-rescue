#!/bin/bash
# 같은 월드를 설정만 바꿔 **동시에** 돌리고 결과를 나란히 본다.
#
# 쓰는 법:  bash debug/ab.sh worlds/apartment_check.wbt "기준=" "스캔매칭=SCANMATCH_ENABLED=True" ...
#   각 인자는 "이름=SAR_SET 값" 이다 (첫 '=' 앞이 이름). SAR_PROFILE 은 바깥에서 준다.
# 시뮬레이션은 결정적이다 — 부하가 달라도 같은 설정이면 결과가 같다 (run_headless.sh 참고).
set -u
cd "$(dirname "$0")/.."
WORLD="$1"; shift
STAMP=$(date +%H%M)
find . -name __pycache__ -not -path './.venv/*' -not -path './competition/*' -exec rm -rf {} + 2>/dev/null
pids=()
for SPEC in "$@"; do
  NAME="${SPEC%%=*}"; SET="${SPEC#*=}"
  # 오타는 여기서 잡는다 (Webots 를 켜기 전에)
  SAR_SET="$SET" .venv/bin/python3 -c "import config" || { echo "❌ $NAME: SAR_SET 오류"; exit 1; }
  OUT="$PWD/debug/out/ab_${STAMP}_${NAME}"; mkdir -p "$OUT"   # 절대경로 — 컨트롤러는 제 폴더에서 돈다
  SAR_SET="$SET" SAR_OUT="$OUT" ./run_headless.sh "$WORLD" 1500 > "$OUT/run.log" 2>&1 &
  pids+=($!)
done
wait "${pids[@]}"
for SPEC in "$@"; do
  NAME="${SPEC%%=*}"; OUT="$PWD/debug/out/ab_${STAMP}_${NAME}"
  echo "=== $NAME (${SPEC#*=}) ==="
  grep -E "상태  |찾은 목표물|걸린 시간|달린 거리|닿아 있던|오도메트리 최대|방문함|못 감|\[포기\]" "$OUT/run.log" | sed 's/^/  /'
done
