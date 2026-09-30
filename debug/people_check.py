"""사람 검출기가 실제로 맞히는지 '정답' 과 대조한다.

받는 것: debug/out/tape.npz (mission_check 가 남긴 원시 기록).
내놓는 것: 검출률 / 유령 / 위치 오차, 그리고 문턱값을 바꿔 본 표.
핵심 아이디어: Supervisor 로 읽은 사람의 진짜 위치가 정답이다. 주행 로직은
그 값을 절대 안 보지만, 채점은 그걸로 할 수 있다 (CLAUDE.md 규칙 1).

⚠️ 지도를 "그 순간까지 만들어진 상태" 로 재현해서 채점해야 한다. 완성된 지도로
   채점하면 수치가 부풀려진다 — 실제로 80% 라고 믿었던 것이 41% 였다.

쓰는 법:
    ./run_headless.sh worlds/practice_check.wbt 400     # 기록을 남기고
    .venv/bin/python3 debug/people_check.py      # 채점한다
"""

import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import config          # noqa: E402
import mapping         # noqa: E402
import people          # noqa: E402

TAPE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "out", "tape.npz")
CORRECT_WITHIN = 0.5   # [m] 이 안이면 "맞혔다" 로 센다


def score(tape, **kw):
    """한 설정으로 기록 전체를 채점한다."""
    pose, ranges = tape["pose"], tape["ranges"]
    person, truth = tape["person"], tape["truth"]

    grid = mapping.new_map()
    hit = total = ghost = 0
    errs = []
    for i in range(len(pose)):
        if i == 0 or i % config.MAP_UPDATE_EVERY == 0:
            mapping.update(grid, tuple(pose[i]), ranges[i])
        found = people.people(tuple(pose[i]), ranges[i], grid, **kw)

        visible = math.hypot(truth[i][0] - person[i][0],
                             truth[i][1] - person[i][1]) <= config.MAP_MAX_RAY_RANGE
        if not visible:
            ghost += len(found)          # 사람이 안 보이는데 찾았다면 전부 유령
            continue
        total += 1
        if not found:
            continue
        best = min(found, key=lambda p: math.hypot(p[0] - person[i][0],
                                                   p[1] - person[i][1]))
        err = math.hypot(best[0] - person[i][0], best[1] - person[i][1])
        if err < CORRECT_WITHIN:
            hit += 1
            errs.append(err)
            ghost += len(found) - 1
        else:
            ghost += len(found)
    errs = np.array(errs) if errs else np.array([9.9])
    return dict(rate=hit / max(total, 1), ghost=ghost / max(total, 1),
                median=float(np.median(errs)), total=total)


def main():
    if not os.path.exists(TAPE):
        print(f"기록이 없다: {TAPE}")
        print("먼저 ./run_headless.sh worlds/practice_check.wbt 400 을 돌릴 것.")
        return 1
    tape = np.load(TAPE)
    print(f"기록 {len(tape['pose'])} 틱, {tape['t'][-1]:.0f}초\n")

    now = score(tape)
    print(f"지금 설정 (config.py): 검출 {100*now['rate']:.0f}%"
          f"  유령 {now['ghost']:.2f}/틱  위치 오차 중앙값 {now['median']*100:.1f}cm"
          f"  (사람이 보이던 틱 {now['total']})\n")

    print("문턱값을 바꿔 보면:")
    print(f"{'폭 범위':>14s} {'끊김':>6s} {'짝':>5s} {'검출률':>7s} {'유령/틱':>8s} {'오차':>8s}")
    for width in ((0.04, 0.35), (0.06, 0.50), (0.10, 0.60)):
        for pair in (0.5, 0.8):
            r = score(tape, width_range=width, pair_within=pair)
            here = " ← 지금" if (width == tuple(config.PEOPLE_WIDTH_RANGE)
                                and pair == config.PEOPLE_PAIR_WITHIN) else ""
            print(f"  {width[0]:5.2f}~{width[1]:4.2f} {config.PEOPLE_SEGMENT_JUMP:6.2f}"
                  f" {pair:5.2f} {100*r['rate']:6.0f}% {r['ghost']:8.2f}"
                  f" {r['median']*100:7.1f}cm{here}")

    print("\n⚠️ 검출률이 낮다고 바로 나쁜 것은 아니다. 계획기는 '사람 둘레를 비싸게'")
    print("   할 뿐이라 몇 틱 놓쳐도 다음 계획에서 반영된다. 반대로 유령이 많으면")
    print("   엉뚱한 곳을 비싸게 보고 돌아가므로 그쪽이 더 위험하다.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
