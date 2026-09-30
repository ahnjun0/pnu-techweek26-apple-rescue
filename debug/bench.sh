#!/bin/bash
# 1단계 벤치마크 — 지도 4종 × 시작 위치 4곳 = 16 표본.
#
# ⚠️ 지도 하나를 1회 재던 때는 개선과 운을 구별할 수 없었다. 설정이 조금만
#    달라져도 궤적이 통째로 바뀌어 목표물을 우연히 보거나 놓쳤다
#    (회전 비용 0.0→3/3, 0.5→1/3, 1.0→3/3). 그래서 표본을 16개로 늘린다.
#
# 쓰는 법:  bash debug/bench.sh [결과디렉터리]
set -u
cd "$(dirname "$0")/.."
OUT="${1:-/tmp/bench}"
mkdir -p "$OUT"

printf "%-12s %8s %8s %7s %7s %8s %8s\n" 월드 거리 시간 목표물 탐색률 "최선대비" "시간배수"
TOTAL_FOUND=0; TOTAL_N=0; TOTAL_RATIO=0; RATIO_N=0
# ⚠️ 월드의 로봇 배치와 config.START_X/Y 는 반드시 일치해야 한다 (config.py:93).
#    시작 위치 변형을 만들며 이것을 깨뜨려, 로봇이 5 m 어긋난 곳에 있다고 믿고
#    출발한 채 측정한 적이 있다 (maze1: 1.0 m / 40% / 1-3). 그래서 여기서 맞춘다.
STARTS_X=(-2.5 0.0 -2.5 2.5)
STARTS_Y=(-2.5 -2.5 0.0 0.0)
cp config.py "$OUT/config_backup.py"
# ⚠️ 되돌린 뒤 바이트코드 캐시도 지운다. config.py 를 되돌렸는데
#    __pycache__ 가 옛 값을 계속 내놓아 측정 한 바퀴를 날린 적이 있다
#    (파일엔 1.5 인데 import 는 999 를 냈다).
restore() { cp "$OUT/config_backup.py" config.py; find . -name "config.cpython*.pyc" -delete; }
trap restore EXIT

for L in maze open comb corridor; do
  for i in 0 1 2 3; do
    W="${L}${i}"
    restore
    /usr/bin/sed -i '' "s/^START_X = .*/START_X = ${STARTS_X[$i]}              # [m]/" config.py
    /usr/bin/sed -i '' "s/^START_Y = .*/START_Y = ${STARTS_Y[$i]}              # [m]/" config.py
    GOT=$(.venv/bin/python3 -c "import config;print(config.START_X, config.START_Y)")
    if [ "$GOT" != "${STARTS_X[$i]} ${STARTS_Y[$i]}" ]; then
      echo "❌ $W: config 시작점 확인 실패 ('$GOT')"; exit 1
    fi
    SAR_LAYOUT="$L" SAR_START="$i" ./run_headless.sh "worlds/${W}_check.wbt" 6000 > "$OUT/$W.log" 2>&1
    d=$(grep -o "달린 거리 *: *[0-9.]*" "$OUT/$W.log" | awk '{print $NF}')
    t=$(grep -o "걸린 시간 *: *[0-9.]*" "$OUT/$W.log" | awk '{print $NF}')
    # ⚠️ 매 틱 상태줄에도 "방문" 이 나온다 ("목표물 방문 (-0.05, +2.40)").
    #    그걸 집어서 목표물 수가 빈 값이 된 적이 있다. 요약줄만 본다.
        # ⚠️ 점수는 **방문** 이다 (대회 기준: "식별하여 각 객체 위치까지 이동").
    #    찾기만 하고 못 간 것은 점수가 아니다. 요약줄의 "(방문 N 개)" 를 본다.
    v=$(grep -oE "찾은 목표물 *: *[0-9]+ 개 \(방문 [0-9]+ 개" "$OUT/$W.log" | grep -oE "방문 [0-9]+" | grep -oE "[0-9]+" | head -1)
    c=$(grep -o "아레나 36 m² 중 [0-9]*%" "$OUT/$W.log" | grep -o "[0-9]*%")
    # teacher A (이 지도·이 시작점의 이론 최선)
    a=$(SAR_LAYOUT=$L SAR_START=$i .venv/bin/python3 -c "
import sys, os; sys.path.insert(0,'debug'); sys.path.insert(0,'.')
import teacher
from make_maze_world import build_grid
print(f'{teacher.best_target_tour(build_grid()):.2f}')" 2>/dev/null)
    r=$(.venv/bin/python3 -c "
d='$d'; a='$a'
print(f'{float(d)/float(a):.2f}x' if d and a and float(a)>0 else '-')" 2>/dev/null)
    # ⚠️ 시간 배수는 **달린 거리와 무관한** 기준선이다. "거리만으로 필요한 시간"
    #    은 헛걸음을 기준선에 같이 넣어 버려 이상을 숨긴다 (diag: 1.27배로 보였다).
    tr=$(grep -o "시간 배수 *: *[0-9.]*x" "$OUT/$W.log" | grep -oE "[0-9.]+x" | head -1)
    printf "%-12s %8s %8s %6s/3 %7s %8s %8s\n" "$W" "${d:--}" "${t:--}" "${v:--}" "${c:--}" "${r:--}" "${tr:--}"
    [ -n "${v:-}" ] && TOTAL_FOUND=$((TOTAL_FOUND+v))
    TOTAL_N=$((TOTAL_N+3))
  done
done
echo "------------------------------------------------------------"
echo "목표물 합계: $TOTAL_FOUND / $TOTAL_N"
