"""미로를 미리 다 아는 'teacher' 의 경로와 우리 로봇의 경로를 비교한다.

왜 필요한가: "비효율적으로 보인다" 를 고치려면 **얼마나** 비효율적인지 알아야 한다.
SEG(방 단위 탐색) 같은 기능을 붙이기 전에, 그 기능이 가져올 수 있는 이득의
상한을 먼저 재는 것이 순서다. 상한이 작으면 만들 이유가 없다.

teacher 둘:
  A. 전지(全知)  — 벽도 목표물 위치도 다 안다. 세 곳을 돌고 복귀만 하면 된다.
                   탐색이 필요 없으므로 이것이 **절대 하한** 이다.
  B. 벽만 앎     — 목표물이 어디 있는지는 모른다. 그러니 목표물을 놓치지 않으려면
                   **빈 공간을 모두 봐야** 한다. 지도를 아는 상태에서의 최적 탐색이며
                   이것이 우리 탐색과 견줄 **공정한 상대** 다.
                   (고전적으로 watchman route problem. 최적해는 NP-난해라
                    '관측점 고르기(greedy set cover) + 순회(2-opt)' 로 근사한다.)

B 를 근사로 푸는 것이 왜 괜찮은가: 근사해는 최적해보다 **길다**. 따라서
"우리 / B" 비율은 진짜 비율보다 **작게** 나온다. 즉 우리에게 유리하게 나오므로,
이 값이 1 에 가까우면 "개선 여지가 없다" 는 결론은 안전하다.
"""
import itertools
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import common
import config
import mapping
import planner
from make_maze_world import START, TARGETS, build_grid

VIEW_STRIDE = 8          # [칸] 관측점 후보를 이 간격으로만 둔다 (0.4 m)
RAY_COUNT = 180          # 관측점 하나에서 쏘는 광선 수


def visible_from(occupied, free, row, col, reach_cells):
    """(row, col) 에 서서 볼 수 있는 빈 칸들의 bool 마스크.

    LiDAR 처럼 광선을 쏘아 벽에 막히면 거기서 끊는다.
    """
    seen = np.zeros_like(free)
    height, width = free.shape
    for k in range(RAY_COUNT):
        angle = 2.0 * math.pi * k / RAY_COUNT
        dr, dc = math.sin(angle), math.cos(angle)
        for step in range(1, reach_cells + 1):
            r = int(round(row + dr * step))
            c = int(round(col + dc * step))
            if not (0 <= r < height and 0 <= c < width):
                break
            if occupied[r, c]:
                break
            if free[r, c]:
                seen[r, c] = True
    return seen


def choose_viewpoints(grid):
    """빈 칸을 모두 보기 위한 관측점들을 욕심쟁이 집합덮개로 고른다."""
    occupied = mapping.is_occupied(grid)
    free = mapping.is_free(grid)
    standable = free & ~planner.inflate(grid, config.PLANNER_INFLATION_MARGIN)
    reach = common.to_cells(config.LIDAR_MAX_RANGE)

    rows, cols = np.nonzero(standable)
    spots = [(int(r), int(c)) for r, c in zip(rows, cols)
             if r % VIEW_STRIDE == 0 and c % VIEW_STRIDE == 0]
    print(f"  관측점 후보 {len(spots)} 곳에서 가시영역을 계산한다...")
    coverage = {s: visible_from(occupied, free, s[0], s[1], reach) for s in spots}

    # 어차피 아무 관측점에서도 안 보이는 칸(벽 틈새 등)은 덮개 대상에서 뺀다.
    reachable_view = np.zeros_like(free)
    for m in coverage.values():
        reachable_view |= m
    todo = free & reachable_view
    total = int(todo.sum())

    chosen = []
    while todo.any():
        best, best_gain = None, 0
        for s, m in coverage.items():
            gain = int((m & todo).sum())
            if gain > best_gain:
                best, best_gain = s, gain
        if best is None:
            break
        chosen.append(best)
        todo &= ~coverage[best]
    print(f"  빈 칸 {total} 개를 덮는 데 관측점 {len(chosen)} 곳이 필요하다")
    return [common.to_world(r, c) for r, c in chosen]


def tour_length(grid, points, start):
    """start 에서 출발해 points 를 모두 들르고 start 로 돌아오는 길이 [m]."""
    spots = [start] + list(points)
    n = len(spots)

    def hop(a, b):
        path = planner.plan(grid, a, b)
        if not path:
            return math.inf
        return sum(common.distance(*p, *q) for p, q in zip(path, path[1:]))

    d = [[0.0] * n for _ in range(n)]
    for i in range(n):
        for j in range(i + 1, n):
            d[i][j] = d[j][i] = hop(spots[i], spots[j])

    if n <= 9:      # 작으면 완전탐색
        best = math.inf
        for order in itertools.permutations(range(1, n)):
            route = (0,) + order + (0,)
            best = min(best, sum(d[route[k]][route[k + 1]]
                                 for k in range(len(route) - 1)))
        return best

    # 가까운 곳부터 잇고(nearest neighbour) 2-opt 로 다듬는다
    unvisited = set(range(1, n))
    route = [0]
    while unvisited:
        last = route[-1]
        nxt = min(unvisited, key=lambda j: d[last][j])
        route.append(nxt)
        unvisited.remove(nxt)
    route.append(0)

    improved = True
    while improved:
        improved = False
        for i in range(1, len(route) - 2):
            for j in range(i + 1, len(route) - 1):
                a, b, c, e = route[i - 1], route[i], route[j], route[j + 1]
                if d[a][b] + d[c][e] > d[a][c] + d[b][e] + 1e-9:
                    route[i:j + 1] = reversed(route[i:j + 1])
                    improved = True
    return sum(d[route[k]][route[k + 1]] for k in range(len(route) - 1))


def approach_points(grid, target, count=8):
    """목표물 주위에서 로봇이 실제로 서는 자리 후보들.

    ⚠️ 처음엔 teacher A 가 목표물 **중심** 까지 가는 경로를 쟀다. 그런데 로봇은
       APPROACH_DISTANCE(0.57 m) 앞에서 멈춘다. 그래서 teacher 가 과대평가돼,
       우리 로봇이 "하한보다 짧게" 가는 말이 안 되는 결과가 나왔다
       (open 지도에서 우리 16.9 m < teacher 20.0 m).
       목표물을 '점' 이 아니라 '고리' 로 보고, 고리 위 자리들 중 최선을 고른다.
    """
    blocked = planner.inflate(grid, config.PLANNER_INFLATION_MARGIN)
    spots = []
    for k in range(count):
        angle = 2.0 * math.pi * k / count
        x = target[0] + config.APPROACH_DISTANCE * math.cos(angle)
        y = target[1] + config.APPROACH_DISTANCE * math.sin(angle)
        cell = common.to_cell(x, y)
        if common.in_bounds(*cell) and not blocked[cell]:
            spots.append((x, y))
    return spots or [target]


def best_target_tour(grid):
    """목표물 셋을 '고리 위 어느 자리로 갈지' 까지 골라 가며 최단 순회를 찾는다."""
    choices = [approach_points(grid, t) for t in TARGETS]

    def hop(a, b):
        path = planner.plan(grid, a, b)
        if not path:
            return math.inf
        return sum(common.distance(*p, *q) for p, q in zip(path, path[1:]))

    best = math.inf
    for order in itertools.permutations(range(len(TARGETS))):
        for picks in itertools.product(*[choices[i] for i in order]):
            stops = [START] + list(picks) + [START]
            total = sum(hop(stops[k], stops[k + 1]) for k in range(len(stops) - 1))
            best = min(best, total)
    return best


def main():
    grid = build_grid()
    print("=" * 66)
    print("[teacher] 미로를 다 아는 로봇이라면 얼마나 갔을까")
    print("=" * 66)

    print("\nA. 전지 teacher — 목표물 위치까지 안다 (절대 하한)")
    print(f"   (목표물 중심이 아니라 {config.APPROACH_DISTANCE:.2f} m 앞까지만 간다 —"
          " 우리 로봇과 같은 조건)")
    a = best_target_tour(grid)
    print(f"   경로 길이: {a:.2f} m")

    print("\nB. 벽만 아는 teacher — 목표물을 찾으려면 빈 공간을 다 봐야 한다")
    print("   ⚠️ B 는 빈 칸을 100% 보는 경로다. 우리 로봇은 목표물 셋을 찾으면"
          " 거기서 멈추므로(94%), B 보다 짧을 수 있다 — B 는 하한이 아니다.")
    views = choose_viewpoints(grid)
    b = tour_length(grid, views, START)
    print(f"   경로 길이: {b:.2f} m   (관측점 {len(views)} 곳 순회)")

    mine = os.environ.get("SAR_ACTUAL")
    print("\n" + "-" * 66)
    print(f"[{os.environ.get('SAR_LAYOUT', 'maze')}]  A(전지) {a:.2f} m   B(벽만 앎) {b:.2f} m")
    if mine:
        mine = float(mine)
        print(f"  우리 로봇 {mine:.2f} m")
        print(f"    A 대비 {mine / a:.2f} 배 / B 대비 {mine / b:.2f} 배"
              "   ← B 대비가 '탐색 전략이 얼마나 나쁜가' 다")
    print("-" * 66)


if __name__ == "__main__":
    main()
