#!/bin/bash
# 사람 시나리오 7종을 한 번씩 돌려 **회피가 튼튼한지** 본다.
#
# ⚠️ 2단계를 보행자 궤적 하나·속도 하나로만 쟀었다. 그 월드는 결정적이라 3회
#    반복해도 같은 값이 나오므로 "접촉 0초" 가 튼튼함의 증거가 아니었다.
#    (사용자 지적: "부딪히지 않은 것은 우연일 확률이 더 높아 보인다.
#     '작은 사람' 은 왜 테스트해보지 않았나?")
set -u
cd "$(dirname "$0")/.."
OUT="${1:-/tmp/people_scen}"
mkdir -p "$OUT"
.venv/bin/python3 debug/make_people_world.py > /dev/null

printf "%-9s %6s %7s %8s %7s %9s %8s %8s\n" 시나리오 반경 거리 시간 목표물 닿은시간 최근접 시간배수
for spec in "base 0.191" "fast 0.191" "slow 0.191" "cross 0.191" "diag 0.191" "thin 0.09" "child 0.05"; do
  set -- $spec; NAME=$1; R=$2
  SAR_PERSON_RADIUS="$R" ./run_headless.sh "worlds/people_${NAME}_check.wbt" 6000 \
      > "$OUT/$NAME.log" 2>&1
  d=$(grep -o "달린 거리 *: *[0-9.]*" "$OUT/$NAME.log" | awk '{print $NF}')
  t=$(grep -o "걸린 시간 *: *[0-9.]*" "$OUT/$NAME.log" | awk '{print $NF}')
  v=$(grep -oE "찾은 목표물 *: *[0-9]+ 개 \(방문 [0-9]+ 개" "$OUT/$NAME.log" \
      | grep -oE "방문 [0-9]+" | grep -oE "[0-9]+" | head -1)
  touch=$(grep -o "닿아 있던 시간: *[0-9.]*" "$OUT/$NAME.log" \
          | awk '{s+=$NF}END{printf "%.1f", s+0}')
  near=$(grep -oE "pedestrian1|PEDESTRIAN1 과 가장 가까웠던 거리: [0-9.]+" "$OUT/$NAME.log" \
         | grep -oE "[0-9.]+$" | head -1)
  tr=$(grep -o "시간 배수 *: *[0-9.]*x" "$OUT/$NAME.log" | grep -oE "[0-9.]+x" | head -1)
  printf "%-9s %6s %7s %8s %6s/3 %8ss %8s %8s\n" "$NAME" "$R" "${d:--}" "${t:--}" "${v:--}" "${touch:-0.0}" "${near:--}" "${tr:--}"
done
