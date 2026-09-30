"""D* Lite 가 A* 와 같은 답을 주는지, 그리고 재계획이 실제로 싼지 확인한다."""

import math

import numpy as np
import pytest

import common
import config
import dstar
import mapping
import planner


def empty_blocked():
    return np.zeros((config.MAP_HEIGHT_CELLS, config.MAP_WIDTH_CELLS), dtype=bool)


def flat_cost():
    return np.ones((config.MAP_HEIGHT_CELLS, config.MAP_WIDTH_CELLS),
                   dtype=np.float32)


def path_cost(path, cost):
    """경로를 따라가며 실제로 드는 비용. 두 계획기를 공정하게 비교하려는 것."""
    total = 0.0
    for a, b in zip(path, path[1:]):
        step = math.hypot(b[0] - a[0], b[1] - a[1])
        total += step * float(cost[b[0], b[1]])
    return total


# --- 기본 동작 ---------------------------------------------------------------

def test_finds_a_straight_path_in_open_space():
    engine = dstar.DStarLite(empty_blocked(), flat_cost(), (80, 60))
    path = engine.plan((80, 40))
    assert path is not None
    assert path[0] == (80, 40) and path[-1] == (80, 60)


def test_steps_are_adjacent():
    engine = dstar.DStarLite(empty_blocked(), flat_cost(), (110, 120))
    path = engine.plan((40, 40))
    for a, b in zip(path, path[1:]):
        assert abs(a[0] - b[0]) <= 1 and abs(a[1] - b[1]) <= 1


def test_never_walks_through_a_blocked_cell():
    blocked = empty_blocked()
    blocked[70:90, 80] = True
    engine = dstar.DStarLite(blocked, flat_cost(), (80, 100))
    path = engine.plan((80, 60))
    assert path is not None
    assert not any(blocked[r, c] for r, c in path)


def test_returns_none_when_walled_off():
    blocked = empty_blocked()
    blocked[:, 80] = True
    engine = dstar.DStarLite(blocked, flat_cost(), (80, 100))
    assert engine.plan((80, 60)) is None


def test_diagonal_cannot_cut_through_a_corner():
    """A* 와 같은 규칙이어야 한다. 안 그러면 두 계획기가 다르게 군다."""
    blocked = empty_blocked()
    blocked[80, 81] = True
    blocked[81, 80] = True
    engine = dstar.DStarLite(blocked, flat_cost(), (81, 81))
    path = engine.plan((80, 80))
    assert path is None or len(path) > 2


# --- A* 와 같은 답을 주는가 ---------------------------------------------------

@pytest.mark.parametrize("start,goal", [
    ((80, 40), (80, 60)),
    ((40, 40), (100, 120)),
    ((120, 30), (40, 130)),
])
def test_matches_astar_cost_in_open_space(start, goal):
    """길이가 똑같을 필요는 없지만(같은 비용의 다른 길이 있다), 비용은 같아야 한다."""
    blocked, cost = empty_blocked(), flat_cost()
    mine = dstar.DStarLite(blocked, cost, goal).plan(start)
    theirs = planner.astar(blocked, cost, start, goal)
    assert mine is not None and theirs is not None
    assert path_cost(mine, cost) == pytest.approx(path_cost(theirs, cost), rel=0.02)


def test_matches_astar_around_a_wall():
    blocked, cost = empty_blocked(), flat_cost()
    blocked[60:100, 80] = True
    start, goal = (80, 60), (80, 100)

    mine = dstar.DStarLite(blocked, cost, goal).plan(start)
    theirs = planner.astar(blocked, cost, start, goal)
    assert path_cost(mine, cost) == pytest.approx(path_cost(theirs, cost), rel=0.05)


def test_respects_cell_costs():
    blocked, cost = empty_blocked(), flat_cost()
    cost[80, 41:60] = 9.0
    path = dstar.DStarLite(blocked, cost, (80, 60)).plan((80, 40))
    assert path is not None
    expensive = sum(1 for r, c in path if cost[r, c] > 1.0)
    assert expensive < 19, "비싼 칸을 그대로 직진하면 안 된다"


# --- 재계획이 실제로 싼가 (이걸 쓰는 이유) ------------------------------------

def test_replanning_after_a_small_change_is_cheaper_than_the_first_solve():
    blocked, cost = empty_blocked(), flat_cost()
    goal = (80, 130)
    engine = dstar.DStarLite(blocked, cost, goal)

    engine.plan((80, 20))
    first = engine.expansions

    # 경로에서 멀리 떨어진 곳에 장애물이 하나 생겼다
    changed = [(20, 20)]
    blocked[20, 20] = True
    engine.update(blocked, cost, changed)
    engine.plan((80, 21))
    second = engine.expansions

    assert second < first, (
        f"재계획({second} 칸)이 처음({first} 칸)보다 싸야 D* Lite 를 쓸 이유가 있다")


def test_replanning_finds_a_way_around_a_new_wall():
    blocked, cost = empty_blocked(), flat_cost()
    goal = (80, 100)
    engine = dstar.DStarLite(blocked, cost, goal)
    assert engine.plan((80, 60)) is not None

    blocked[70:90, 80] = True                      # 길 한복판에 벽이 생겼다
    changed = [(r, 80) for r in range(70, 90)]
    engine.update(blocked, cost, changed)

    path = engine.plan((80, 60))
    assert path is not None
    assert not any(blocked[r, c] for r, c in path)


def test_replanning_reports_no_path_when_the_way_closes():
    blocked, cost = empty_blocked(), flat_cost()
    goal = (80, 100)
    engine = dstar.DStarLite(blocked, cost, goal)
    assert engine.plan((80, 60)) is not None

    blocked[:, 80] = True
    engine.update(blocked, cost, [(r, 80) for r in range(blocked.shape[0])])
    assert engine.plan((80, 60)) is None
