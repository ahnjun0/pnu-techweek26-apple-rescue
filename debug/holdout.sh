#!/bin/bash
# 처음 보는 무작위 지도로 재는 벤치 (일반화 확인).
#
# ⚠️ 왜 필요한가: 지금까지의 성적은 **우리가 설계한 16월드** 에서 나온 것이고,
#    이번 세션에만 같은 16월드로 벤치를 열두 번 넘게 돌려 무엇을 남길지 정했다.
#    그건 검증 세트에 맞춰 고른 것이라 낙관적인 숫자다. 대회 지도는 처음 보는
#    지도이므로, **튜닝에 한 번도 쓰지 않은 시드** 로 재야 진짜 실력을 안다.
#    무작위 지도는 목표물을 아무 데나 놓는다 — 우리 16월드는 전부 모서리다.
#
# 쓰는 법:  bash debug/holdout.sh [결과디렉터리] [시드...]
set -u
cd "$(dirname "$0")/.."
OUT="${1:-/tmp/holdout}"; shift || true
mkdir -p "$OUT"
SEEDS="${*:-101 102 103 104 105 106 107 108}"

printf "%-9s %8s %8s %7s %7s %8s %8s\n" 시드 거리 시간 목표물 탐색률 최선대비 시간배수
cp config.py "$OUT/config_backup.py"
restore() { cp "$OUT/config_backup.py" config.py; find . -name "config.cpython*.pyc" -delete; }
trap restore EXIT

TOTAL_FOUND=0; TOTAL_N=0
for S in $SEEDS; do
  restore
  /usr/bin/sed -i '' "s/^START_X = .*/START_X = -2.5              # [m]/" config.py
  /usr/bin/sed -i '' "s/^START_Y = .*/START_Y = -2.5              # [m]/" config.py
  GOT=$(.venv/bin/python3 -c "import config;print(config.START_X, config.START_Y)")
  [ "$GOT" = "-2.5 -2.5" ] || { echo "❌ 시드 $S: config 시작점 확인 실패 ('$GOT')"; exit 1; }

  # 월드를 만든다 (풀 수 있는지 생성기가 먼저 검사한다)
  if ! SAR_LAYOUT="rand$S" SAR_START=0 .venv/bin/python3 debug/make_maze_world.py \
        > "$OUT/gen_$S.log" 2>&1; then
    echo "   시드 $S: 못 푸는 지도라 건너뛴다"; continue
  fi
  W="rand${S}0"
  SAR_LAYOUT="rand$S" SAR_START=0 ./run_headless.sh "worlds/${W}_check.wbt" 6000 \
      > "$OUT/$S.log" 2>&1
  d=$(grep -o "달린 거리 *: *[0-9.]*" "$OUT/$S.log" | awk '{print $NF}')
  t=$(grep -o "걸린 시간 *: *[0-9.]*" "$OUT/$S.log" | awk '{print $NF}')
  v=$(grep -oE "찾은 목표물 *: *[0-9]+ 개 \(방문 [0-9]+ 개" "$OUT/$S.log" \
      | grep -oE "방문 [0-9]+" | grep -oE "[0-9]+" | head -1)
  c=$(grep -o "아레나 36 m² 중 [0-9]*%" "$OUT/$S.log" | grep -o "[0-9]*%")
  a=$(SAR_LAYOUT="rand$S" SAR_START=0 .venv/bin/python3 -c "
import sys; sys.path.insert(0,'debug'); sys.path.insert(0,'.')
import teacher
from make_maze_world import build_grid
print(f'{teacher.best_target_tour(build_grid()):.2f}')" 2>/dev/null)
  r=$(.venv/bin/python3 -c "
d='$d'; a='$a'
print(f'{float(d)/float(a):.2f}x' if d and a and float(a)>0 else '-')" 2>/dev/null)
  tr=$(grep -o "시간 배수 *: *[0-9.]*x" "$OUT/$S.log" | grep -oE "[0-9.]+x" | head -1)
  printf "%-9s %8s %8s %6s/3 %7s %8s %8s\n" "$S" "${d:--}" "${t:--}" "${v:--}" "${c:--}" "${r:--}" "${tr:--}"
  [ -n "${v:-}" ] && TOTAL_FOUND=$((TOTAL_FOUND+v))
  TOTAL_N=$((TOTAL_N+3))
done
echo "------------------------------------------------------------"
echo "목표물 합계: $TOTAL_FOUND / $TOTAL_N   (처음 보는 지도)"
