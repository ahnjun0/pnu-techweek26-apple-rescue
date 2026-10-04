"""지도 만들기가 맞는지 Webots 없이 확인한다.

제일 중요한 것 두 가지:
  1. 지도가 뒤집히지 않는가 (정면에 있는 벽이 지도에서도 로봇 앞에 찍히는가)
  2. 누적이 되는가 (지나간 사람의 흔적이 나중에 지워지는가)
"""

import math

import numpy as np
import pytest

from sar import common
from sar import config
from sar import mapping


def ranges_all(value):
    """모든 광선이 같은 거리를 봤다고 치는 LiDAR 배열."""
    return np.full(config.LIDAR_RESOLUTION, value, dtype=np.float64)


def ray_index_for(angle):
    """로봇 기준 angle [rad] 방향을 보는 광선의 인덱스."""
    angles = common.lidar_angles()
    return int(np.argmin(np.abs(common.wrap_angle(angles - angle))))


# --- Bresenham ---------------------------------------------------------------

def test_bresenham_includes_both_ends():
    rows, cols = mapping.bresenham(3, 4, 3, 9)
    assert (rows[0], cols[0]) == (3, 4)
    assert (rows[-1], cols[-1]) == (3, 9)


def test_bresenham_horizontal_is_contiguous():
    rows, cols = mapping.bresenham(2, 0, 2, 5)
    assert list(cols) == [0, 1, 2, 3, 4, 5]
    assert set(rows) == {2}


def test_bresenham_diagonal():
    rows, cols = mapping.bresenham(0, 0, 4, 4)
    assert list(zip(rows, cols)) == [(0, 0), (1, 1), (2, 2), (3, 3), (4, 4)]


def test_bresenham_works_backwards():
    rows, cols = mapping.bresenham(9, 9, 5, 2)
    assert (rows[0], cols[0]) == (9, 9)
    assert (rows[-1], cols[-1]) == (5, 2)


def test_bresenham_single_cell():
    rows, cols = mapping.bresenham(7, 7, 7, 7)
    assert list(zip(rows, cols)) == [(7, 7)]


def test_bresenham_steps_are_adjacent():
    """칸을 건너뛰면 벽 사이로 광선이 새어 나간다."""
    rows, cols = mapping.bresenham(0, 0, 13, 37)
    for i in range(1, len(rows)):
        assert abs(int(rows[i] - rows[i - 1])) <= 1
        assert abs(int(cols[i] - cols[i - 1])) <= 1


# --- 지도가 뒤집히지 않는가 ---------------------------------------------------

def test_wall_in_front_lands_in_front():
    """로봇이 +x 를 보고 있고 정면 1 m 에 벽이 있으면, 지도의 +x 쪽 1 m 에 찍혀야 한다."""
    grid = mapping.new_map()
    ranges = np.full(config.LIDAR_RESOLUTION, np.inf)
    ranges[ray_index_for(0.0)] = 1.0

    mapping.update(grid, (0.0, 0.0, 0.0), ranges, stride=1)

    expected_row, expected_col = common.to_cell(1.0, 0.0)
    rows, cols = np.nonzero(mapping.is_occupied(grid))
    assert len(rows) >= 1
    assert min(abs(int(r) - expected_row) + abs(int(c) - expected_col)
               for r, c in zip(rows, cols)) <= 1


@pytest.mark.parametrize("ray_angle,robot_theta,wx,wy", [
    (0.0, 0.0, 1.0, 0.0),                    # 정면, 동쪽을 봄  → 동쪽 1 m
    (math.pi / 2, 0.0, 0.0, 1.0),            # 왼쪽, 동쪽을 봄  → 북쪽 1 m
    (-math.pi / 2, 0.0, 0.0, -1.0),          # 오른쪽           → 남쪽 1 m
    (0.0, math.pi / 2, 0.0, 1.0),            # 정면, 북쪽을 봄  → 북쪽 1 m
    (math.pi / 2, math.pi / 2, -1.0, 0.0),   # 왼쪽, 북쪽을 봄  → 서쪽 1 m
    (0.0, math.pi, -1.0, 0.0),               # 정면, 서쪽을 봄  → 서쪽 1 m
])
def test_hit_lands_at_the_right_world_spot(ray_angle, robot_theta, wx, wy):
    """로봇 자세와 광선 방향을 바꿔 가며 벽이 월드의 맞는 자리에 찍히는지 본다."""
    grid = mapping.new_map()
    ranges = np.full(config.LIDAR_RESOLUTION, np.inf)
    ranges[ray_index_for(ray_angle)] = 1.0

    mapping.update(grid, (0.0, 0.0, robot_theta), ranges, stride=1)

    expected_row, expected_col = common.to_cell(wx, wy)
    rows, cols = np.nonzero(mapping.is_occupied(grid))
    closest = min(abs(int(r) - expected_row) + abs(int(c) - expected_col)
                  for r, c in zip(rows, cols))
    assert closest <= 1, f"벽이 엉뚱한 곳에 찍혔다 (가장 가까운 칸이 {closest} 칸 떨어짐)"


def test_robot_offset_from_origin():
    """로봇이 원점이 아니어도 맞는 자리에 찍히는가."""
    grid = mapping.new_map()
    ranges = np.full(config.LIDAR_RESOLUTION, np.inf)
    ranges[ray_index_for(0.0)] = 1.0

    mapping.update(grid, (2.0, -1.0, 0.0), ranges, stride=1)

    expected_row, expected_col = common.to_cell(3.0, -1.0)
    rows, cols = np.nonzero(mapping.is_occupied(grid))
    assert min(abs(int(r) - expected_row) + abs(int(c) - expected_col)
               for r, c in zip(rows, cols)) <= 1


# --- 빈 공간 ----------------------------------------------------------------

def test_cells_along_the_ray_become_free():
    grid = mapping.new_map()
    ranges = np.full(config.LIDAR_RESOLUTION, np.inf)
    ranges[ray_index_for(0.0)] = 1.0

    for _ in range(3):   # 빈 공간은 한 방에 안 정해진다. 몇 번 봐야 한다
        mapping.update(grid, (0.0, 0.0, 0.0), ranges, stride=1)

    mid_row, mid_col = common.to_cell(0.5, 0.0)
    assert mapping.is_free(grid)[mid_row, mid_col]


def test_no_hit_ray_still_clears_space():
    """아무것도 못 맞힌 광선도 "거기까지는 비었다" 는 정보다."""
    grid = mapping.new_map()
    ranges = np.full(config.LIDAR_RESOLUTION, np.inf)

    for _ in range(3):
        mapping.update(grid, (0.0, 0.0, 0.0), ranges, stride=1)

    row, col = common.to_cell(1.0, 0.0)
    assert mapping.is_free(grid)[row, col]
    assert not mapping.is_occupied(grid).any(), "못 맞힌 광선이 벽을 만들면 안 된다"


def test_unknown_at_the_start():
    grid = mapping.new_map()
    assert mapping.is_unknown(grid).all()
    assert not mapping.is_occupied(grid).any()
    assert not mapping.is_free(grid).any()


# --- 누적(log-odds) 이 실제로 되는가 ------------------------------------------

def test_repeated_hits_build_confidence():
    grid = mapping.new_map()
    ranges = np.full(config.LIDAR_RESOLUTION, np.inf)
    ranges[ray_index_for(0.0)] = 1.0
    row, col = common.to_cell(1.0, 0.0)

    before = grid[row, col]
    mapping.update(grid, (0.0, 0.0, 0.0), ranges, stride=1)
    after_one = grid[row, col]
    mapping.update(grid, (0.0, 0.0, 0.0), ranges, stride=1)
    after_two = grid[row, col]

    assert before < after_one < after_two


def test_walking_person_trace_gets_erased():
    """제일 중요한 성질. 사람이 서 있던 칸이 사람이 떠난 뒤 빈 칸으로 바뀌는가."""
    grid = mapping.new_map()
    row, col = common.to_cell(1.0, 0.0)

    # 1) 사람이 정면 1 m 에 잠깐 서 있다 (몇 틱).
    with_person = np.full(config.LIDAR_RESOLUTION, np.inf)
    with_person[ray_index_for(0.0)] = 1.0
    for _ in range(3):
        mapping.update(grid, (0.0, 0.0, 0.0), with_person, stride=1)
    assert mapping.is_occupied(grid)[row, col], "사람이 있을 때는 막힘으로 보여야 한다"

    # 2) 사람이 지나갔다. 이제 그 방향은 뻥 뚫려 있다.
    without_person = np.full(config.LIDAR_RESOLUTION, np.inf)
    for _ in range(15):
        mapping.update(grid, (0.0, 0.0, 0.0), without_person, stride=1)

    assert not mapping.is_occupied(grid)[row, col], "사람 흔적이 지워져야 한다"
    assert mapping.is_free(grid)[row, col]


def test_log_odds_is_clamped():
    """한번 굳어 버리면 영영 안 바뀌는 것을 막는다."""
    grid = mapping.new_map()
    ranges = np.full(config.LIDAR_RESOLUTION, np.inf)
    ranges[ray_index_for(0.0)] = 1.0

    for _ in range(500):
        mapping.update(grid, (0.0, 0.0, 0.0), ranges, stride=1)

    assert grid.max() <= config.LOG_ODDS_MAX
    assert grid.min() >= config.LOG_ODDS_MIN


# --- 방어 ------------------------------------------------------------------

def test_out_of_range_reading_is_ignored_as_a_wall():
    """LiDAR 최대거리보다 먼 값은 벽으로 찍지 않는다."""
    grid = mapping.new_map()
    ranges = ranges_all(config.MAP_MAX_RAY_RANGE + 1.0)
    mapping.update(grid, (0.0, 0.0, 0.0), ranges, stride=1)
    assert not mapping.is_occupied(grid).any()


def test_too_close_reading_is_ignored():
    """minRange 보다 가까운 값은 못 믿는다."""
    grid = mapping.new_map()
    ranges = ranges_all(config.LIDAR_MIN_RANGE / 2.0)
    mapping.update(grid, (0.0, 0.0, 0.0), ranges, stride=1)
    assert not mapping.is_occupied(grid).any()
    assert not mapping.is_free(grid).any()


def test_robot_outside_the_map_does_nothing():
    grid = mapping.new_map()
    far = (config.MAP_ORIGIN_X - 10.0, 0.0, 0.0)
    mapping.update(grid, far, ranges_all(1.0), stride=1)
    assert mapping.is_unknown(grid).all()


def test_rays_pointing_off_map_do_not_crash():
    """지도 가장자리에 서 있어도 터지지 않아야 한다."""
    grid = mapping.new_map()
    corner_x = config.MAP_ORIGIN_X + 0.1
    corner_y = config.MAP_ORIGIN_Y + 0.1
    mapping.update(grid, (corner_x, corner_y, 0.0), ranges_all(2.0), stride=1)
    assert np.isfinite(grid).all()


def test_nan_readings_do_not_crash():
    grid = mapping.new_map()
    ranges = ranges_all(1.0)
    ranges[::3] = np.nan
    mapping.update(grid, (0.0, 0.0, 0.0), ranges, stride=1)
    assert np.isfinite(grid).all()


def test_stride_skips_rays():
    """stride 를 키우면 처리하는 광선이 줄어 지도에 찍히는 칸도 줄어야 한다."""
    dense = mapping.new_map()
    sparse = mapping.new_map()
    ranges = ranges_all(1.0)

    mapping.update(dense, (0.0, 0.0, 0.0), ranges, stride=1)
    mapping.update(sparse, (0.0, 0.0, 0.0), ranges, stride=8)

    assert mapping.is_occupied(sparse).sum() < mapping.is_occupied(dense).sum()


def test_ray_does_not_paint_free_beyond_a_known_wall():
    """⚠️ 회귀 방지: 거리값이 하나 잘못 들어와도 벽 너머를 비우면 안 된다.

    자세 추정이 잠깐 튀거나 반사가 생기면 실제보다 먼 거리가 들어온다.
    그때 광선을 끝까지 그으면 벽 바깥이 "빈 칸" 이 되고, 거기에 프론티어가
    잡혀 로봇이 갈 수 없는 곳을 목표로 삼는다.
    실측: 아레나 밖에 빈 칸 595 개, 프론티어 140 개 중 136 개가 아레나 밖이었다.
    """
    grid = mapping.new_map()
    pose = (0.0, 0.0, 0.0)
    angles = common.lidar_angles()
    front = int(np.argmin(np.abs(angles)))

    def flat_wall_at(x):
        """x = 상수 인 평평한 벽을 봤을 때의 거리 배열."""
        out = np.full(config.LIDAR_RESOLUTION, np.inf, dtype=np.float32)
        facing = np.abs(angles) < math.radians(60)
        out[facing] = x / np.cos(angles[facing])
        return out

    for _ in range(5):
        mapping.update(grid, pose, flat_wall_at(1.0), stride=1)
    wall_cell = common.to_cell(1.0, 0.0)
    assert mapping.is_occupied(grid)[wall_cell], "벽이 먼저 굳어야 한다"

    # 이제 정면 광선 하나만 "2.5 m" 라는 틀린 값을 준다.
    bad = flat_wall_at(1.0)
    bad[front] = 2.5
    mapping.update(grid, pose, bad, stride=1)

    beyond = [common.to_cell(x, 0.0) for x in (1.3, 1.7, 2.1, 2.4)]
    free = mapping.is_free(grid)
    assert not any(free[c] for c in beyond), "벽 너머가 빈 칸이 됐다"


def test_wall_ray_still_erases_a_stale_person_trace():
    """벽에서 멈추게 했다고 해서 사람 흔적까지 남으면 안 된다.

    멈추되 "그 칸까지는" 빈 칸 표를 주기 때문에, 진짜 벽이 아닌 것은 지워진다.
    """
    grid = mapping.new_map()
    pose = (0.0, 0.0, 0.0)
    front = int(np.argmin(np.abs(common.lidar_angles())))

    person = np.full(config.LIDAR_RESOLUTION, np.inf, dtype=np.float32)
    person[front] = 1.0
    for _ in range(3):
        mapping.update(grid, pose, person, stride=1)
    spot = common.to_cell(1.0, 0.0)
    assert mapping.is_occupied(grid)[spot], "사람이 먼저 찍혀야 한다"

    # 사람이 지나갔다 — 이제 그 방향이 뻥 뚫려 있다.
    empty = np.full(config.LIDAR_RESOLUTION, np.inf, dtype=np.float32)
    for _ in range(20):
        mapping.update(grid, pose, empty, stride=1)
    assert not mapping.is_occupied(grid)[spot], "사람 흔적이 안 지워졌다"
