"""경로 계획이 맞는지, 그리고 로봇·지도가 바뀌어도 되는지 확인한다. Webots 없이 돈다."""

import math

import numpy as np
import pytest

from sar import common
from sar import config
from sar import mapping
from sar import planner


def empty_blocked():
    return np.zeros((config.MAP_HEIGHT_CELLS, config.MAP_WIDTH_CELLS), dtype=bool)


def flat_cost():
    """모든 칸의 비용이 같은 배열 — A* 자체만 볼 때 쓴다."""
    return np.ones((config.MAP_HEIGHT_CELLS, config.MAP_WIDTH_CELLS),
                   dtype=np.float32)


def wall_grid(col, row_from, row_to, gap=None):
    """세로벽 하나짜리 지도를 만든다. gap=(시작row, 끝row) 면 거기에 문을 낸다."""
    grid = mapping.new_map()
    grid[:] = config.LOG_ODDS_MIN          # 전부 "빈 칸" 으로 시작
    for row in range(row_from, row_to):
        if gap and gap[0] <= row <= gap[1]:
            continue
        grid[row, col] = config.LOG_ODDS_MAX
    return grid


# --- 팽창 -------------------------------------------------------------------

def test_inflate_grows_obstacles():
    grid = mapping.new_map()
    grid[80, 80] = config.LOG_ODDS_MAX
    inflated = planner.inflate(grid)

    radius = int(math.ceil((config.ROBOT_RADIUS + config.PLANNER_INFLATION_MARGIN)
                           / config.MAP_RESOLUTION))
    assert inflated[80, 80]
    assert inflated[80, 80 + radius - 1], "로봇 반경 안은 막혀 있어야 한다"
    assert not inflated[80, 80 + radius + 2], "너무 멀리까지 막으면 못 지나간다"


def test_inflate_seals_the_map_border():
    """경로가 지도 밖으로 새지 않도록 테두리는 막혀 있어야 한다."""
    inflated = planner.inflate(mapping.new_map())
    assert inflated[0, :].all() and inflated[-1, :].all()
    assert inflated[:, 0].all() and inflated[:, -1].all()


def test_inflate_of_empty_map_has_no_interior_obstacle():
    inflated = planner.inflate(mapping.new_map())
    assert not inflated[50:110, 50:110].any()


# --- A* ---------------------------------------------------------------------

def test_astar_straight_line_in_open_space():
    path = planner.astar(empty_blocked(), flat_cost(), (80, 40), (80, 60))
    assert path is not None
    assert path[0] == (80, 40) and path[-1] == (80, 60)
    assert len(path) == 21, "직선이면 딱 21칸이어야 한다"


def test_astar_steps_are_adjacent():
    path = planner.astar(empty_blocked(), flat_cost(), (40, 40), (90, 110))
    for a, b in zip(path, path[1:]):
        assert abs(a[0] - b[0]) <= 1 and abs(a[1] - b[1]) <= 1


def test_astar_never_walks_through_a_blocked_cell():
    blocked = empty_blocked()
    blocked[70:90, 80] = True
    path = planner.astar(blocked, flat_cost(), (80, 60), (80, 100))
    assert path is not None
    assert not any(blocked[r, c] for r, c in path)


def test_astar_returns_none_when_goal_is_walled_off():
    blocked = empty_blocked()
    blocked[60:100, 90] = True      # 목표를 둘러싸는 상자
    blocked[60:100, 110] = True
    blocked[60, 90:111] = True
    blocked[99, 90:111] = True
    path = planner.astar(blocked, flat_cost(), (80, 40), (80, 100))
    assert path is None


def test_astar_returns_none_when_goal_is_inside_a_wall():
    blocked = empty_blocked()
    blocked[80, 100] = True
    assert planner.astar(blocked, flat_cost(), (80, 40), (80, 100)) is None


def test_astar_goal_equals_start():
    path = planner.astar(empty_blocked(), flat_cost(), (80, 80), (80, 80))
    assert path == [(80, 80)]


def test_astar_avoids_expensive_cells():
    """비싼 칸은 피해 간다. 모르는 칸·벽 근처를 비싸게 매기는 것이 이 위에 얹힌다."""
    blocked = empty_blocked()
    cost = flat_cost()
    cost[80, 41:60] = 5.0          # 직선 경로가 비싸다

    path = planner.astar(blocked, cost, (80, 40), (80, 60))
    assert path is not None
    expensive_steps = sum(1 for r, c in path if cost[r, c] > 1.0)
    assert expensive_steps < 19, "비싼 칸을 그대로 직진하면 안 된다"


def test_cost_layer_is_expensive_near_walls(monkeypatch):
    """벽에 가까울수록 비싸야 경로가 빈 공간 가운데로 흐른다.

    기본값은 꺼져 있으므로(아래 주석 참고) 여기서는 켜고 기능만 확인한다.
    """
    monkeypatch.setattr(config, "PLANNER_SOFT_WEIGHT", 2.0)
    grid = mapping.new_map()
    grid[:] = config.LOG_ODDS_MIN
    grid[80, 80] = config.LOG_ODDS_MAX

    cost = planner.cost_layer(grid)
    hard = common.to_cells(config.ROBOT_RADIUS + config.PLANNER_INFLATION_MARGIN)
    near = cost[80, 80 + hard + 1]
    far = cost[80, 80 + hard + common.to_cells(config.PLANNER_SOFT_CLEARANCE) + 3]
    assert near > far, "팽창 경계 바로 밖이 더 비싸야 한다"
    assert far == pytest.approx(1.0, abs=0.01), "충분히 멀면 보통 비용이어야 한다"


def test_cost_layer_marks_unknown_as_expensive():
    grid = mapping.new_map()          # 전부 모름
    cost = planner.cost_layer(grid)
    assert cost[80, 80] >= config.PLANNER_UNKNOWN_COST


def test_obstacle_distance_counts_cells_from_walls():
    grid = mapping.new_map()
    grid[:] = config.LOG_ODDS_MIN
    grid[80, 80] = config.LOG_ODDS_MAX

    distance = planner.obstacle_distance(grid, 10)
    assert distance[80, 80] == 0
    assert distance[80, 81] == 1
    assert distance[80, 85] == 5
    assert distance[80, 120] == 10, "상한에서 잘려야 한다"


# --- 대각선이 모서리를 뚫지 못하는가 (제일 중요) -------------------------------

def test_diagonal_cannot_cut_through_a_corner():
    """
    . X        (0,0) 에서 (1,1) 로 대각선 한 번에 가면 벽 사이를 뚫는 것이다.
    X .        양 옆이 둘 다 막혔으므로 그 대각선은 금지여야 한다.
    """
    blocked = empty_blocked()
    blocked[80, 81] = True
    blocked[81, 80] = True

    path = planner.astar(blocked, flat_cost(), (80, 80), (81, 81))
    # 한 칸 대각선으로 바로 가면 길이 2. 돌아가야 하므로 더 길어야 한다.
    assert path is None or len(path) > 2


def test_diagonal_is_refused_even_when_only_one_side_is_blocked():
    """한쪽만 막혀도 대각선은 금지하는 엄격한 규칙을 쓴다.

    팽창까지 해 놓고 굳이 모서리를 스치는 경로를 만들 이유가 없다 (안전거리 우선).
    돌아가는 길은 있으므로 경로 자체는 나와야 한다.
    """
    blocked = empty_blocked()
    blocked[80, 81] = True

    path = planner.astar(blocked, flat_cost(), (80, 80), (81, 81))
    assert path is not None
    assert path != [(80, 80), (81, 81)], "모서리를 스쳐 지나갔다"
    assert not any(blocked[r, c] for r, c in path)


def test_no_corner_cutting_along_a_full_wall():
    """벽 끝 모서리를 대각선으로 스쳐 지나가지 않는지, 경로 전체를 검사한다."""
    blocked = empty_blocked()
    blocked[70:90, 80] = True

    path = planner.astar(blocked, flat_cost(), (80, 70), (80, 90))
    assert path is not None
    for (r0, c0), (r1, c1) in zip(path, path[1:]):
        if r0 != r1 and c0 != c1:
            assert not blocked[r1, c0] and not blocked[r0, c1], \
                f"({r0},{c0})→({r1},{c1}) 가 모서리를 뚫었다"


# --- 웨이포인트 줄이기 --------------------------------------------------------

def test_simplify_collapses_a_straight_line():
    path = [(80, c) for c in range(40, 61)]
    assert planner.simplify(path, empty_blocked()) == [(80, 40), (80, 60)]


def test_simplify_keeps_the_ends():
    path = planner.astar(empty_blocked(), flat_cost(), (40, 40), (100, 120))
    simple = planner.simplify(path, empty_blocked())
    assert simple[0] == path[0] and simple[-1] == path[-1]


def test_simplify_never_cuts_through_a_wall():
    blocked = empty_blocked()
    blocked[70:90, 80] = True
    path = planner.astar(blocked, flat_cost(), (80, 70), (80, 90))
    simple = planner.simplify(path, blocked)
    for a, b in zip(simple, simple[1:]):
        assert planner.line_is_clear(blocked, a, b)


def test_simplify_handles_short_paths():
    assert planner.simplify([], empty_blocked()) == []
    assert planner.simplify([(1, 1)], empty_blocked()) == [(1, 1)]


# --- plan (월드 좌표 입구) ----------------------------------------------------

def test_plan_returns_world_coordinates_from_start_to_goal():
    grid = mapping.new_map()
    grid[:] = config.LOG_ODDS_MIN
    path = planner.plan(grid, (-2.0, 0.0), (2.0, 0.0))

    assert path is not None
    assert common.distance(*path[0], -2.0, 0.0) < 0.1
    assert common.distance(*path[-1], 2.0, 0.0) < 0.1


def test_plan_goes_through_a_wide_door():
    grid = wall_grid(col=80, row_from=40, row_to=120, gap=(70, 90))
    path = planner.plan(grid, (-2.0, 0.0), (2.0, 0.0))
    assert path is not None
    # 문이 가운데(y=0 근처)에 있으니 크게 우회하지 않아야 한다
    assert max(abs(y) for _, y in path) < 1.5


def test_plan_detours_around_a_door_too_narrow_for_the_robot():
    """문이 로봇보다 좁으면 통과한다고 믿으면 안 된다. 돌아가거나 실패해야 한다."""
    grid = wall_grid(col=80, row_from=40, row_to=120, gap=(79, 81))  # 3칸=0.15m
    path = planner.plan(grid, (-2.0, 0.0), (2.0, 0.0))
    if path is not None:
        assert max(abs(y) for _, y in path) > 1.5, "좁은 문으로 지나가려 했다"


def test_planned_path_keeps_away_from_walls():
    """계획된 경로 자체가 이미 안전거리를 지켜야 한다 (팽창의 존재 이유)."""
    grid = wall_grid(col=80, row_from=40, row_to=120, gap=(65, 95))
    path = planner.plan(grid, (-2.0, 0.0), (2.0, 0.0))
    assert path is not None

    occupied_rows, occupied_cols = np.nonzero(mapping.is_occupied(grid))
    walls = [common.to_world(int(r), int(c))
             for r, c in zip(occupied_rows, occupied_cols)]

    for x, y in path:
        closest = min(common.distance(x, y, wx, wy) for wx, wy in walls)
        assert closest >= config.ROBOT_RADIUS, \
            f"경로점 ({x:.2f},{y:.2f}) 이 벽에서 {closest:.3f} m 밖에 안 떨어졌다"


def test_plan_fails_when_there_is_no_way_through():
    grid = wall_grid(col=80, row_from=0, row_to=config.MAP_HEIGHT_CELLS)
    assert planner.plan(grid, (-2.0, 0.0), (2.0, 0.0)) is None


def test_plan_when_goal_sits_inside_the_inflated_zone():
    """벽에 붙어 있는 목표물에 다가갈 때. 근처의 갈 수 있는 칸으로 대신 간다."""
    grid = mapping.new_map()
    grid[:] = config.LOG_ODDS_MIN
    grid[80, 100] = config.LOG_ODDS_MAX          # 목표물이 곧 장애물

    goal_x, goal_y = common.to_world(80, 100)
    path = planner.plan(grid, (-2.0, 0.0), (goal_x, goal_y))

    assert path is not None, "벽에 붙은 목표에 다가갈 수 없으면 목표물을 못 찾는다"
    assert common.distance(*path[-1], goal_x, goal_y) < 0.6


def test_carved_escape_does_not_become_a_shortcut():
    """로봇이 벽에 붙어 팽창을 뚫어 줬을 때, 그 틈을 지름길로 쓰면 안 된다.

    ⚠️ 이걸 안 막으면 계획 경로가 벽에서 0.20 m 밖에 안 떨어지게 나오고,
       그건 정지거리 안쪽이라 주행기가 곧바로 정지를 건다.
       실제로 그 고리 때문에 로봇이 벽 모서리에서 전체 시간의 23% 를 허비했다.
    """
    grid = mapping.new_map()
    grid[:] = config.LOG_ODDS_MIN
    grid[60:100, 80] = config.LOG_ODDS_MAX          # 세로벽 (양끝이 뚫려 있다)

    # 벽 바로 옆에서 출발한다 → 팽창 안이라 탈출로를 뚫게 된다
    start = common.to_world(80, 83)
    path = planner.plan(grid, start, (2.0, 0.0))
    assert path is not None

    occupied_rows, occupied_cols = np.nonzero(mapping.is_occupied(grid))
    walls = [common.to_world(int(r), int(c))
             for r, c in zip(occupied_rows, occupied_cols)]

    # 출발점 근처(뚫어 준 구간)를 벗어난 뒤에는 정상 여유를 지켜야 한다
    guaranteed = config.ROBOT_RADIUS + config.PLANNER_INFLATION_MARGIN
    for x, y in path[1:]:
        if common.distance(x, y, *start) < 0.5:
            continue
        closest = min(common.distance(x, y, wx, wy) for wx, wy in walls)
        assert closest >= guaranteed - config.MAP_RESOLUTION, (
            f"({x:.2f},{y:.2f}) 이 벽에서 {closest:.3f} m 밖에 안 떨어졌다 "
            f"(약속은 {guaranteed:.3f} m)")


def test_plan_when_robot_is_already_too_close_to_a_wall():
    """로봇이 이미 팽창 영역 안에 있어도 움직일 수는 있어야 한다."""
    grid = mapping.new_map()
    grid[:] = config.LOG_ODDS_MIN
    grid[80, 60] = config.LOG_ODDS_MAX

    start_x, start_y = common.to_world(80, 61)   # 벽 바로 옆
    path = planner.plan(grid, (start_x, start_y), (0.0, 2.0))
    assert path is not None
    # 탈출로를 뚫어 줬더라도 진짜 벽 칸을 지나가면 안 된다
    occupied = mapping.is_occupied(grid)
    for x, y in path:
        row, col = common.to_cell(x, y)
        assert not occupied[row, col]


def test_carving_an_escape_does_not_open_real_walls(monkeypatch):
    """벽에 붙어 있다고 해서 벽을 통과해도 되는 것은 아니다."""
    grid = mapping.new_map()
    grid[:] = config.LOG_ODDS_MIN
    grid[:, 60] = config.LOG_ODDS_MAX            # 지도를 완전히 가르는 벽

    start_x, start_y = common.to_world(80, 61)   # 벽 바로 오른쪽에 붙어 있음
    goal_x, goal_y = common.to_world(80, 40)     # 벽 왼쪽 — 갈 방법이 없다
    assert planner.plan(grid, (start_x, start_y), (goal_x, goal_y)) is None


def test_plan_outside_the_map_fails_gracefully():
    grid = mapping.new_map()
    grid[:] = config.LOG_ODDS_MIN
    assert planner.plan(grid, (-2.0, 0.0), (99.0, 99.0)) is None


# --- 재계획 판단 --------------------------------------------------------------

def test_path_is_blocked_is_false_for_a_clear_path():
    grid = mapping.new_map()
    grid[:] = config.LOG_ODDS_MIN
    path = planner.plan(grid, (-2.0, 0.0), (2.0, 0.0))
    assert not planner.path_is_blocked(path, grid)


def test_path_is_blocked_notices_a_new_obstacle():
    grid = mapping.new_map()
    grid[:] = config.LOG_ODDS_MIN
    path = planner.plan(grid, (-2.0, 0.0), (2.0, 0.0))

    row, col = common.to_cell(0.0, 0.0)          # 경로 한복판에 사람이 섰다
    grid[row, col] = config.LOG_ODDS_MAX
    assert planner.path_is_blocked(path, grid)


def test_path_is_blocked_ignores_the_robots_own_position():
    """경로의 첫 점은 로봇 자신이다. 벽에 붙어 있다고 "막혔다" 가 되면 안 된다.

    그러면 매 틱 A* 를 다시 돌려(8 ms) 경로가 계속 갈아엎힌다.
    """
    grid = mapping.new_map()
    grid[:] = config.LOG_ODDS_MIN
    grid[80, 60] = config.LOG_ODDS_MAX

    # 벽 바로 옆에서 출발해 멀리 트인 곳으로 가는 경로
    start = common.to_world(80, 62)
    path = [start, common.to_world(80, 120)]
    assert not planner.path_is_blocked(path, grid), \
        "출발점이 팽창 안이라고 막혔다고 하면 안 된다"


def test_path_is_blocked_still_sees_an_obstacle_further_along():
    grid = mapping.new_map()
    grid[:] = config.LOG_ODDS_MIN
    grid[80, 60] = config.LOG_ODDS_MAX          # 출발점 옆 벽
    grid[80, 100] = config.LOG_ODDS_MAX         # 경로 한복판의 새 장애물

    start = common.to_world(80, 62)
    path = [start, common.to_world(80, 130)]
    assert planner.path_is_blocked(path, grid)


def test_path_is_blocked_handles_empty_input():
    grid = mapping.new_map()
    assert not planner.path_is_blocked(None, grid)
    assert not planner.path_is_blocked([(0.0, 0.0)], grid)


# --- 범용성: 로봇·지도가 바뀌어도 되는가 ---------------------------------------

@pytest.mark.parametrize("robot_radius,margin", [
    (0.05, 0.02),    # 아주 작은 로봇
    (0.13, 0.08),    # 지금 TurtleBot3 Burger
    (0.30, 0.10),    # 큰 로봇
])
def test_inflation_follows_the_robot_size(monkeypatch, robot_radius, margin):
    """당일 로봇이 바뀌면 config 만 고치면 되어야 한다."""
    monkeypatch.setattr(config, "ROBOT_RADIUS", robot_radius)
    monkeypatch.setattr(config, "PLANNER_INFLATION_MARGIN", margin)

    grid = mapping.new_map()
    grid[80, 80] = config.LOG_ODDS_MAX
    inflated = planner.inflate(grid)

    expected = int(math.ceil((robot_radius + margin) / config.MAP_RESOLUTION))
    assert inflated[80, 80 + expected - 1]
    assert not inflated[80, 80 + expected + 2]


def test_bigger_robot_refuses_a_door_a_smaller_one_takes(monkeypatch):
    """같은 문이라도 로봇이 크면 못 지나가야 한다 — 팽창이 실제로 작동하는지."""
    grid = wall_grid(col=80, row_from=40, row_to=120, gap=(76, 84))  # 9칸 = 0.45 m

    monkeypatch.setattr(config, "ROBOT_RADIUS", 0.08)
    monkeypatch.setattr(config, "PLANNER_INFLATION_MARGIN", 0.02)
    small = planner.plan(grid, (-2.0, 0.0), (2.0, 0.0))

    monkeypatch.setattr(config, "ROBOT_RADIUS", 0.30)
    monkeypatch.setattr(config, "PLANNER_INFLATION_MARGIN", 0.10)
    big = planner.plan(grid, (-2.0, 0.0), (2.0, 0.0))

    assert small is not None and max(abs(y) for _, y in small) < 1.5, \
        "작은 로봇은 이 문을 지나갈 수 있어야 한다"
    assert big is None or max(abs(y) for _, y in big) > 1.5, \
        "큰 로봇이 이 문을 지나간다고 하면 끼어 버린다"


@pytest.mark.parametrize("resolution", [0.025, 0.05, 0.10])
def test_planner_works_at_other_map_resolutions(monkeypatch, resolution):
    """해상도를 바꿔도 같은 월드 좌표로 같은 결론이 나와야 한다."""
    cells = int(round(8.0 / resolution))
    monkeypatch.setattr(config, "MAP_RESOLUTION", resolution)
    monkeypatch.setattr(config, "MAP_WIDTH_CELLS", cells)
    monkeypatch.setattr(config, "MAP_HEIGHT_CELLS", cells)

    grid = mapping.new_map()
    grid[:] = config.LOG_ODDS_MIN
    assert grid.shape == (cells, cells)

    path = planner.plan(grid, (-2.0, 0.0), (2.0, 0.0))
    assert path is not None
    assert common.distance(*path[-1], 2.0, 0.0) < 0.2


def test_planner_works_on_a_non_square_map(monkeypatch):
    """대회 맵이 정사각형이 아닐 수도 있다."""
    monkeypatch.setattr(config, "MAP_WIDTH_CELLS", 200)
    monkeypatch.setattr(config, "MAP_HEIGHT_CELLS", 100)

    grid = mapping.new_map()
    grid[:] = config.LOG_ODDS_MIN
    assert grid.shape == (100, 200)

    path = planner.plan(grid, (-2.0, 0.0), (2.0, 0.0))
    assert path is not None


def test_exact_refuses_to_quietly_move_the_goal():
    """exact=True 면 목표를 바꾸지 않는다 — 못 가면 None 이다.

    ⚠️ 이 한 가지 성질을 몰라서 같은 버그를 세 번 만들었다:
       ① 프론티어 후보를 plan() 의 참/거짓으로 걸렀다 (통과 못 할 것이 없었다)
       ② 주행기의 ARRIVED 를 "목표 도착" 으로 읽었다
       ③ _replan 의 "안 되면 좁게" 가 영영 실행되지 않았다
       셋 다 로봇이 속도 0 으로 수백 초를 서 있는 것으로 나타났다.
    """
    grid = mapping.new_map()
    r0, c0 = common.to_cell(-2.0, -2.0)
    r1, c1 = common.to_cell(2.0, 2.0)
    grid[r0:r1, c0:c1] = config.LOG_ODDS_MIN          # 빈 방
    wr0, wc0 = common.to_cell(-2.0, 1.0)
    wr1, wc1 = common.to_cell(2.0, 1.2)
    grid[wr0:wr1, wc0:wc1] = config.LOG_ODDS_MAX      # 가로 벽

    inside_wall = (0.0, 1.1)
    blocked = planner.inflate(grid, config.PLANNER_INFLATION_MARGIN)
    assert blocked[common.to_cell(*inside_wall)], "이 좌표는 막혀 있어야 한다"

    loose = planner.plan(grid, (0.0, 0.0), inside_wall)
    assert loose is not None, "기본값은 목표를 바꿔서라도 길을 준다"
    assert common.distance(*loose[-1], *inside_wall) > common.cells_to_metres(1), \
        "그 길의 끝은 요청한 목표가 아니다"

    strict = planner.plan(grid, (0.0, 0.0), inside_wall, exact=True)
    assert strict is None, "exact=True 면 못 간다고 말해야 한다"
