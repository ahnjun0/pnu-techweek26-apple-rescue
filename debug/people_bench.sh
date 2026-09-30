#!/bin/bash
# 2단계 벤치마크 — 사람이 있는 월드는 **반복해서 중앙값** 으로 본다.
#
# ⚠️ 원래 이 머리말은 "Pedestrian 이 기구학적이라 실행마다 크게 흔들리니 반복이
#    필요하다" 였다. **다시 재 보니 지금은 아니다** — 3회가 소수점까지 동일했다
#    (49.6 m / 279.6초 / 3-3 / 접촉 1.2초, 세 번 모두). 월드가 결정적이므로
#    A/B 는 1회로 판정할 수 있고, 그래서 2단계 작업이 3배 빨라졌다.
#    그래도 기본을 3회로 둔다: 설정을 바꾸면 결정성이 깨질 수 있고, 깨졌는지
#    알 수 있는 유일한 방법이 반복이다.
#
# 쓰는 법:  bash debug/people_bench.sh [반복수] [결과디렉터리]
set -u
cd "$(dirname "$0")/.."
N="${1:-3}"
OUT="${2:-/tmp/people_bench}"
mkdir -p "$OUT"

echo "사람 있는 월드 ${N}회 반복 — 중앙값으로 판단한다"
printf "%-6s %8s %8s %7s %9s %8s %7s\n" 회차 거리 시간 목표물 닿은시간 최근접 헛것비율
for i in $(seq 1 "$N"); do
  ./run_headless.sh worlds/stress_check.wbt 1100 > "$OUT/run$i.log" 2>&1
  d=$(grep -o "달린 거리 *: *[0-9.]*" "$OUT/run$i.log" | awk '{print $NF}')
  t=$(grep -o "걸린 시간 *: *[0-9.]*" "$OUT/run$i.log" | awk '{print $NF}')
  # ⚠️ 매 틱 상태줄에도 "방문" 이 나온다 ("목표물 방문 (-0.05, +2.40)").
  #    그걸 집으면 목표물 수가 빈 값이 된다. 요약줄만 본다.
    # ⚠️ 점수는 **방문** 이다 (대회 기준: "식별하여 각 객체 위치까지 이동").
  #    찾기만 하고 못 간 것은 점수가 아니다. 요약줄의 "(방문 N 개)" 를 본다.
  v=$(grep -oE "찾은 목표물 *: *[0-9]+ 개 \(방문 [0-9]+ 개" "$OUT/run$i.log" | grep -oE "방문 [0-9]+" | grep -oE "[0-9]+" | head -1)
  # 세 물체와 닿아 있던 시간의 합 (없으면 0)
  touch=$(grep -o "닿아 있던 시간: *[0-9.]*" "$OUT/run$i.log" \
          | awk '{s+=$NF}END{printf "%.1f", s+0}')
  near=$(grep -o "과 가장 가까웠던 거리: *[0-9.]*" "$OUT/run$i.log" \
         | awk '{print $NF}' | sort -n | head -1)
  fake=$(grep -o "헛것 비율 *[0-9]*%" "$OUT/run$i.log" | grep -o "[0-9]*%")
  printf "%-6s %8s %8s %6s/3 %8ss %8s %7s\n" \
         "$i" "${d:--}" "${t:--}" "${v:--}" "${touch:-0}" "${near:--}" "${fake:--}"
done

echo "------------------------------------------------------------"
echo "중앙값:"
for key in "달린 거리" "걸린 시간"; do
  med=$(grep -h -o "$key *: *[0-9.]*" "$OUT"/run*.log | awk '{print $NF}' \
        | sort -n | awk '{a[NR]=$1}END{print (NR%2)?a[(NR+1)/2]:(a[NR/2]+a[NR/2+1])/2}')
  echo "  $key: $med"
done
# ⚠️ 매 틱 상태줄에도 "방문" 이 나온다. 요약줄만 본다 (위 v= 와 같은 이유).
tv=$(grep -h -oE "찾은 목표물 *: *[0-9]+ 개 \(방문 [0-9]+ 개" "$OUT"/run*.log \
     | grep -oE "방문 [0-9]+" | grep -oE "[0-9]+" | sort -n \
     | awk '{a[NR]=$1}END{print (NR%2)?a[(NR+1)/2]:(a[NR/2]+a[NR/2+1])/2}')
echo "  목표물(중앙값): $tv / 3"
tt=$(for f in "$OUT"/run*.log; do grep -o "닿아 있던 시간: *[0-9.]*" "$f" \
     | awk '{s+=$NF}END{printf "%.1f\n", s+0}'; done | sort -n \
     | awk '{a[NR]=$1}END{print (NR%2)?a[(NR+1)/2]:(a[NR/2]+a[NR/2+1])/2}')
echo "  닿아 있던 시간 합(중앙값): ${tt}s   ← 채점 기준 2번"
