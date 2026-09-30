"""좌표계 "계약서" 가 깨지지 않았는지 확인한다. Webots 없이 돈다."""

import math

import numpy as np
import pytest

import common
import config


# --- to_cell / to_world 왕복 -------------------------------------------------

def test_origin_cell_is_zero_zero():
    """지도 원점 자체는 (row=0, col=0) 셀에 들어간다."""
    assert common.to_cell(config.MAP_ORIGIN_X, config.MAP_ORIGIN_Y) == (0, 0)


def test_roundtrip_cell_to_world_to_cell():
    """셀 중심으로 갔다가 돌아오면 같은 셀이어야 한다."""
    for row, col in [(0, 0), (1, 0), (0, 1), (37, 99), (159, 159)]:
        x, y = common.to_world(row, col)
        assert common.to_cell(x, y) == (row, col)


def test_row_follows_y_and_col_follows_x():
    """row 는 y, col 은 x 를 따라간다 — 뒤집히면 지도가 통째로 뒤집힌다."""
    r0, c0 = common.to_cell(0.0, 0.0)
    r_east, c_east = common.to_cell(1.0, 0.0)   # 동쪽으로 1 m
    r_north, c_north = common.to_cell(0.0, 1.0)  # 북쪽으로 1 m

    assert c_east > c0 and r_east == r0, "x 가 커지면 col 만 커져야 한다"
    assert r_north > r0 and c_north == c0, "y 가 커지면 row 만 커져야 한다"


def test_one_meter_is_exactly_that_many_cells():
    step = int(round(1.0 / config.MAP_RESOLUTION))
    r0, c0 = common.to_cell(0.0, 0.0)
    r1, c1 = common.to_cell(1.0, 1.0)
    assert (r1 - r0, c1 - c0) == (step, step)


def test_to_cell_accepts_arrays():
    xs = np.array([-4.0, 0.0, 3.9])
    ys = np.array([-4.0, 0.0, 3.9])
    rows, cols = common.to_cell(xs, ys)
    assert rows.shape == (3,) and cols.shape == (3,)
    assert (rows[0], cols[0]) == (0, 0)


# --- in_bounds ---------------------------------------------------------------

@pytest.mark.parametrize("row,col,expected", [
    (0, 0, True),
    (config.MAP_HEIGHT_CELLS - 1, config.MAP_WIDTH_CELLS - 1, True),
    (-1, 0, False),
    (0, -1, False),
    (config.MAP_HEIGHT_CELLS, 0, False),
    (0, config.MAP_WIDTH_CELLS, False),
])
def test_in_bounds(row, col, expected):
    assert common.in_bounds(row, col) is expected


def test_start_pose_is_inside_the_map():
    row, col = common.to_cell(config.START_X, config.START_Y)
    assert common.in_bounds(row, col), "START_X/Y 가 지도 밖이면 아무것도 안 된다"


def test_to_cells_converts_a_distance():
    """거리를 칸 수로 바꾸는 유일한 함수. 팽창 반경 계산이 여기에 의존한다."""
    assert common.to_cells(config.MAP_RESOLUTION) == 1
    assert common.to_cells(1.0) == int(math.ceil(1.0 / config.MAP_RESOLUTION))
    assert common.to_cells(0.0) == 0
    # 딱 떨어지지 않으면 올림한다 (안전거리는 모자라느니 넉넉한 게 낫다)
    assert common.to_cells(config.MAP_RESOLUTION * 1.5) == 2
    assert common.to_cells(config.MAP_RESOLUTION * 1.5, round_up=False) == 1


def test_to_cells_follows_the_resolution(monkeypatch):
    monkeypatch.setattr(config, "MAP_RESOLUTION", 0.10)
    assert common.to_cells(1.0) == 10
    monkeypatch.setattr(config, "MAP_RESOLUTION", 0.025)
    assert common.to_cells(1.0) == 40


def test_map_bounds_matches_to_world():
    """map_bounds 가 to_cell/to_world 와 어긋나면 화면만 딴 데를 보게 된다."""
    x_min, x_max, y_min, y_max = common.map_bounds()

    assert common.to_cell(x_min, y_min) == (0, 0)
    # 오른쪽/위쪽 끝은 "바로 바깥" 이어야 한다 (반열린 구간)
    assert not common.in_bounds(*common.to_cell(x_max, y_max))
    last = (config.MAP_HEIGHT_CELLS - 1, config.MAP_WIDTH_CELLS - 1)
    assert common.to_cell(x_max - 1e-6, y_max - 1e-6) == last


def test_lidar_angles_are_verified_and_consistent():
    """측정으로 확정한 LiDAR 방향이 유지되는지 지킨다."""
    assert config.LIDAR_ORIENTATION_VERIFIED, \
        "debug/lidar_orientation.py 로 측정하고 config 를 고칠 것"

    angles = common.lidar_angles()
    assert len(angles) == config.LIDAR_RESOLUTION
    assert np.all(angles >= -math.pi) and np.all(angles < math.pi)

    # 측정 결과: 정면은 인덱스 180, 왼쪽(+90도)은 90, 오른쪽은 270
    assert angles[180] == pytest.approx(0.0, abs=1e-9)
    assert angles[90] == pytest.approx(math.pi / 2, abs=1e-9)
    assert angles[270] == pytest.approx(-math.pi / 2, abs=1e-9)


def test_lidar_index_increases_clockwise():
    """인덱스가 늘면 각도가 줄어야 한다 (시계 방향). 부호가 뒤집히면 지도가 거울이 된다."""
    angles = common.lidar_angles()
    assert common.angle_diff(angles[181], angles[180]) < 0


# --- 각도 --------------------------------------------------------------------

@pytest.mark.parametrize("raw,expected", [
    (0.0, 0.0),
    (math.pi / 2, math.pi / 2),
    (-math.pi / 2, -math.pi / 2),
    (2 * math.pi, 0.0),
    (3 * math.pi, -math.pi),      # +pi 는 접혀서 -pi 가 된다 ([-pi, +pi) 규약)
    (-3 * math.pi, -math.pi),
])
def test_wrap_angle(raw, expected):
    assert common.wrap_angle(raw) == pytest.approx(expected)


def test_wrap_angle_always_in_range():
    for a in np.linspace(-20.0, 20.0, 401):
        w = common.wrap_angle(float(a))
        assert -math.pi <= w < math.pi


def test_angle_diff_takes_the_short_way():
    """179° 에서 -179° 로 갈 때는 358° 가 아니라 2° 만 돌아야 한다."""
    d = common.angle_diff(math.radians(-179), math.radians(179))
    assert d == pytest.approx(math.radians(2), abs=1e-9)


def test_angle_diff_sign_is_counterclockwise_positive():
    assert common.angle_diff(0.5, 0.0) > 0


def test_distance():
    assert common.distance(0.0, 0.0, 3.0, 4.0) == pytest.approx(5.0)
