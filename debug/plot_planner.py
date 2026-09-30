"""가짜 지도 위에서 A* 와 프론티어가 어떻게 도는지 그림으로 저장한다. (Phase 3 검증)

받는 것: 없음 — 지도를 코드로 지어낸다. Webots 가 필요 없다.
내놓는 것: debug/out/*.png
핵심 아이디어: 로봇을 돌리기 전에, 알고리즘만 따로 눈으로 확인한다.
실행: .venv/bin/python3 debug/plot_planner.py
"""

import os
import sys

import matplotlib
matplotlib.use("Agg")           # 창 없이 파일로만 저장
import matplotlib.pyplot as plt
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import common          # noqa: E402
import config          # noqa: E402
import exploration     # noqa: E402
import mapping         # noqa: E402
import planner         # noqa: E402
import viz             # noqa: E402

OUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "out")


# --------------------------------------------------------------------------
# 가짜 지도 만들기 — 월드 좌표 [m] 로 적는다 (해상도가 바뀌어도 그대로 쓸 수 있게)
# --------------------------------------------------------------------------

def blank_free(x0, y0, x1, y1):
    """(x0,y0)~(x1,y1) 사각형만 "빈 칸" 이고 나머지는 "모름" 인 지도."""
    grid = mapping.new_map()
    r0, c0 = common.to_cell(x0, y0)
    r1, c1 = common.to_cell(x1, y1)
    grid[r0:r1, c0:c1] = config.LOG_ODDS_MIN
    return grid


def draw_wall(grid, x0, y0, x1, y1):
    """두 점을 잇는 벽을 긋는다 (칸 단위, Bresenham)."""
    rows, cols = mapping.bresenham(*common.to_cell(x0, y0), *common.to_cell(x1, y1))
    keep = common.in_bounds(rows, cols)
    grid[rows[keep], cols[keep]] = config.LOG_ODDS_MAX
    return grid


def rooms_map():
    """방 두 개와 문 하나 — practice.wbt 를 닮았지만 월드와 무관한 가짜 지도."""
    grid = blank_free(-3.0, -3.0, 3.0, 3.0)
    draw_wall(grid, -3.0, 3.0, 3.0, 3.0)
    draw_wall(grid, -3.0, -3.0, 3.0, -3.0)
    draw_wall(grid, -3.0, -3.0, -3.0, 3.0)
    draw_wall(grid, 3.0, -3.0, 3.0, 3.0)
    draw_wall(grid, 0.0, -3.0, 0.0, -0.5)     # 세로벽 (아래쪽)
    draw_wall(grid, 0.0, 0.5, 0.0, 3.0)       # 세로벽 (위쪽) → 가운데 1 m 문
    draw_wall(grid, 1.5, 0.0, 3.0, 0.0)
    return grid


def partially_explored_map():
    """아직 절반만 둘러본 지도 — 프론티어가 생기는 상황."""
    grid = mapping.new_map()
    r0, c0 = common.to_cell(-2.5, -2.5)
    r1, c1 = common.to_cell(2.5, 0.3)
    grid[r0:r1, c0:c1] = config.LOG_ODDS_MIN
    draw_wall(grid, -2.5, -2.5, 2.5, -2.5)
    draw_wall(grid, -2.5, -2.5, -2.5, 0.3)
    draw_wall(grid, 2.5, -2.5, 2.5, 0.3)
    draw_wall(grid, -0.5, -1.2, -0.5, 0.3)    # 방 안의 칸막이
    return grid


def dead_end_map():
    """막다른 골목 — A* 가 실패해야 하는 상황 (블랙리스트가 필요한 이유).

    ⚠️ 벽을 아레나 크기(-3~3)로만 그으면 A* 가 지도 바깥의 "모르는 공간" 으로
    빙 돌아서 성공해 버린다 (PLANNER_ALLOW_UNKNOWN 이 True 라서 그렇다).
    정말로 길이 없는 상황을 만들려면 벽이 지도 전체를 가로질러야 한다.
    """
    grid = blank_free(-3.0, -3.0, 3.0, 3.0)
    bottom = common.to_world(0, 0)[1]
    top = common.to_world(config.MAP_HEIGHT_CELLS - 1, 0)[1]
    draw_wall(grid, 0.5, bottom, 0.5, top)
    return grid


# --------------------------------------------------------------------------
# 그리기
# --------------------------------------------------------------------------

def base_axes(ax, grid, title):
    ax.imshow(viz.grid_to_image(grid), cmap="gray", vmin=0.0, vmax=1.0,
              origin="lower", extent=viz.map_extent(), interpolation="nearest")
    ax.set_title(title, fontsize=11)
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_aspect("equal")
    ax.grid(alpha=0.15)
    ax.set_xlim(-3.4, 3.4)
    ax.set_ylim(-3.4, 3.4)


def draw_inflation(ax, grid):
    """팽창 영역을 반투명 빨강으로 — "여기는 못 간다" 를 눈으로 보게."""
    inflated = planner.inflate(grid)
    only_inflated = inflated & ~mapping.is_occupied(grid)
    overlay = np.zeros(grid.shape + (4,), dtype=float)
    overlay[only_inflated] = (1.0, 0.2, 0.2, 0.28)
    ax.imshow(overlay, origin="lower", extent=viz.map_extent(),
              interpolation="nearest")


def draw_path(ax, path, colour="tab:orange", label=None):
    if not path:
        return
    ax.plot([p[0] for p in path], [p[1] for p in path], "-o",
            color=colour, linewidth=2.0, markersize=4, label=label)


def figure_astar():
    """세 가지 출발/도착에 대해 A* 경로와 팽창 영역을 그린다."""
    grid = rooms_map()
    # 직선으로 가면 벽을 뚫게 되는 조합만 고른다 (우회가 눈에 보이도록).
    cases = [
        ((-2.5, -2.5), (2.5, -2.5), "아래 방 → 아래 방 (문으로 돌아가야 한다)"),
        ((-2.5, 2.5), (2.5, 1.0), "위 방 → 오른쪽 위 (문 통과)"),
        ((2.5, -2.5), (2.5, 2.5), "오른쪽 아래 → 오른쪽 위 (가로벽 우회)"),
    ]

    fig, axes = plt.subplots(1, 3, figsize=(16, 5.4))
    for ax, (start, goal, title) in zip(axes, cases):
        path = planner.plan(grid, start, goal)
        base_axes(ax, grid, title)
        draw_inflation(ax, grid)
        draw_path(ax, path)
        ax.plot(*start, "o", color="tab:blue", markersize=10)
        ax.plot(*goal, "*", color="lime", markersize=18,
                markeredgecolor="black")
        if path:
            length = sum(common.distance(*a, *b) for a, b in zip(path, path[1:]))
            ax.set_title(f"{title}\n웨이포인트 {len(path)}개, 길이 {length:.2f} m",
                         fontsize=10)
        else:
            ax.set_title(f"{title}\n경로 없음", fontsize=10)

    fig.suptitle("A* — 빨간 영역 = 로봇 반경만큼 부풀린 '못 가는 곳'", fontsize=13)
    fig.tight_layout(rect=(0, 0, 1, 0.94))
    return save(fig, "astar.png")


def figure_corner():
    """대각선이 모서리를 뚫지 않는지 확대해서 본다."""
    grid = blank_free(-1.0, -1.0, 1.0, 1.0)
    # 계단 모양 벽. 팽창을 끄고 순수 A* 규칙만 본다.
    blocked = np.zeros(grid.shape, dtype=bool)
    unknown = np.zeros(grid.shape, dtype=bool)
    r, c = common.to_cell(0.0, 0.0)
    for k in range(6):
        blocked[r + k, c + k] = True
        blocked[r + k + 1, c + k] = True

    start = (r - 2, c - 2)
    goal = (r + 8, c + 8)
    path = planner.astar(blocked, unknown, start, goal)

    fig, ax = plt.subplots(figsize=(6.5, 6.5))
    ax.imshow(np.where(blocked, 0.0, 1.0), cmap="gray", vmin=0, vmax=1,
              origin="lower", interpolation="nearest")
    if path:
        ax.plot([p[1] for p in path], [p[0] for p in path], "-o",
                color="tab:orange", markersize=4)
    ax.plot(start[1], start[0], "o", color="tab:blue", markersize=10)
    ax.plot(goal[1], goal[0], "*", color="lime", markersize=18,
            markeredgecolor="black")
    ax.set_xlim(c - 4, c + 12)
    ax.set_ylim(r - 4, r + 12)
    ax.set_title("대각선이 벽 모서리를 뚫지 않는가\n(경로가 계단 벽을 통과하면 안 된다)")
    ax.set_xlabel("col")
    ax.set_ylabel("row")
    ax.grid(alpha=0.3)
    fig.tight_layout()
    return save(fig, "corner.png")


def figure_frontier():
    """프론티어가 상식적인 곳에 찍히는지, 어디를 고르는지 본다."""
    grid = partially_explored_map()
    robot = (0.0, -2.0)

    points = exploration.frontier_points(grid)
    clusters = exploration.cluster(exploration.frontier_mask(grid))
    candidates = exploration.candidate_list(grid, robot)
    goal = candidates[0][0] if candidates else None
    path = planner.plan(grid, robot, goal) if goal else None

    fig, axes = plt.subplots(1, 2, figsize=(13, 6))

    base_axes(axes[0], grid, f"프론티어 칸 {len(points)}개, 덩어리 {len(clusters)}개")
    axes[0].plot([p[0] for p in points], [p[1] for p in points], ".",
                 color="tab:green", markersize=3, label="프론티어 칸")
    for row, col, size in clusters:
        x, y = common.to_world(row, col)
        axes[0].plot(x, y, "o", color="tab:purple", markersize=7)
        axes[0].annotate(f"{size}", (x, y), textcoords="offset points",
                         xytext=(6, 5), fontsize=9, color="tab:purple")
    axes[0].plot(*robot, "o", color="tab:blue", markersize=10)
    axes[0].legend(loc="upper right", fontsize=8)

    base_axes(axes[1], grid, "고른 목표와 거기까지 가는 길")
    draw_inflation(axes[1], grid)
    draw_path(axes[1], path)
    axes[1].plot(*robot, "o", color="tab:blue", markersize=10)
    if goal:
        axes[1].plot(*goal, "*", color="lime", markersize=18,
                     markeredgecolor="black")
        for (x, y), score, size in candidates[1:6]:
            axes[1].plot(x, y, "x", color="tab:gray", markersize=8)
            axes[1].annotate(f"{score:.2f}", (x, y), textcoords="offset points",
                             xytext=(6, 4), fontsize=8, color="tab:gray")
        axes[1].set_title(f"고른 목표 ({goal[0]:+.2f}, {goal[1]:+.2f})\n"
                          f"회색 x = 다음 후보들 (점수 낮을수록 좋다)", fontsize=10)

    fig.suptitle("프론티어 — 초록 점 = 빈 칸인데 옆이 아직 모르는 칸", fontsize=13)
    fig.tight_layout(rect=(0, 0, 1, 0.94))
    return save(fig, "frontier.png")


def figure_blacklist():
    """A* 가 실패하는 목표를 몇 번 겪으면 포기하고 다른 데로 가는지 본다."""
    grid = dead_end_map()
    # 벽 너머에도 "빈 칸" 이 있는 것처럼 꾸며 놓는다 (지도에는 있지만 갈 수 없는 곳).
    robot = (-2.0, 0.0)
    unreachable = (2.0, 0.0)
    reachable = (-2.0, 2.0)

    blacklist = exploration.Blacklist(max_failures=3)
    lines = []
    for attempt in range(1, 5):
        path = planner.plan(grid, robot, unreachable)
        if path is None:
            banned = blacklist.record_failure(*unreachable)
            lines.append(f"{attempt}회차: A* 실패 → 실패 {blacklist.failures(*unreachable)}회"
                         + ("  ➜ 블랙리스트 등록" if banned else ""))
        if blacklist.contains(*unreachable):
            break

    fallback = planner.plan(grid, robot, reachable)

    fig, ax = plt.subplots(figsize=(7, 7))
    base_axes(ax, grid, "갈 수 없는 목표 → 블랙리스트 → 다른 목표")
    draw_inflation(ax, grid)
    draw_path(ax, fallback, colour="tab:green", label="대신 고른 목표로 가는 길")
    ax.plot(*robot, "o", color="tab:blue", markersize=10)
    ax.plot(*unreachable, "X", color="red", markersize=16, label="벽 너머 — 못 감")
    ax.plot(*reachable, "*", color="lime", markersize=18,
            markeredgecolor="black", label="대신 고른 목표")
    ax.legend(loc="lower right", fontsize=8)
    ax.text(0.02, 0.02, "\n".join(lines), transform=ax.transAxes,
            fontsize=8, va="bottom", ha="left",
            bbox=dict(boxstyle="round", facecolor="white", alpha=0.9))
    fig.tight_layout()
    return save(fig, "blacklist.png")


def save(fig, name):
    os.makedirs(OUT_DIR, exist_ok=True)
    path = os.path.join(OUT_DIR, name)
    fig.savefig(path, dpi=100, bbox_inches="tight")
    plt.close(fig)
    print(f"  저장: {path}")
    return path


def main():
    print("가짜 지도에서 A* / 프론티어를 그린다 (Webots 불필요)")
    figure_astar()
    figure_corner()
    figure_frontier()
    figure_blacklist()
    print("\n끝. debug/out/ 을 열어서 볼 것.")


if __name__ == "__main__":
    main()
