"""팀 전체가 공유하는 "계약서" — 좌표 변환과 각도 규약.

받는 것: 월드 좌표 (x, y)[m] 또는 격자 좌표 (row, col).
내놓는 것: 반대쪽 좌표계의 값.
핵심 아이디어: 변환 함수는 to_cell/to_world 딱 하나씩만 존재한다.
다른 파일에서 int(x / res) 같은 변환을 직접 쓰면 반드시 어딘가 뒤집힌다.
"""

import math

import numpy as np

import config

# ==========================================================================
# 좌표계 규약  (모든 모듈이 이것만 따른다)
# ==========================================================================
#
# [월드 좌표]  Webots R2025a 기본값인 ENU · Z-up 을 그대로 쓴다.
#   x : 동쪽(오른쪽), 단위 m
#   y : 북쪽(위쪽),   단위 m
#   z : 위,           우리는 2D 만 다루므로 무시
#   theta : 로봇이 바라보는 방향. +x 축이 0, 반시계 방향이 + . 단위 rad.
#           항상 wrap_angle 로 [-pi, +pi) 범위에 넣어 둔다.
#           나침반 값에서 theta 를 얻는 식은 localization.py 참고.
#
# [pose 표현]  어디서든 pose 는 길이 3 짜리 tuple  (x, y, theta)  이다.
#              dict 나 클래스로 감싸지 않는다.
#
# [격자 좌표]  numpy 배열 grid[row, col] 로 접근한다. (numpy 관례와 동일)
#   row : y 방향 인덱스.  row 가 커지면 y 가 커진다 (북쪽으로 간다)
#   col : x 방향 인덱스.  col 이 커지면 x 가 커진다 (동쪽으로 간다)
#
#   ⚠️ 그래서 matplotlib 으로 그릴 때는 origin="lower" 를 써야 월드와 같은
#      모양으로 보인다. imshow 기본값(origin="upper")은 위아래가 뒤집힌다.
#
# [격자 원점]  (row=0, col=0) 셀의 "왼쪽 아래 모서리" 가 월드
#              (MAP_ORIGIN_X, MAP_ORIGIN_Y) 이다.
#              to_world 는 셀의 "중심" 좌표를 돌려준다.
#
#   world_x = MAP_ORIGIN_X + (col + 0.5) * MAP_RESOLUTION
#   world_y = MAP_ORIGIN_Y + (row + 0.5) * MAP_RESOLUTION
#
# ==========================================================================


def to_cell(x, y):
    """월드 좌표 (x, y)[m] → 격자 (row, col). 스칼라·numpy 배열 모두 받는다.

    경계 밖 좌표도 그대로 계산해서 돌려준다 (음수나 큰 값이 나올 수 있다).
    범위 확인이 필요하면 in_bounds 를 따로 부를 것.
    """
    col = np.floor((np.asarray(x) - config.MAP_ORIGIN_X) / config.MAP_RESOLUTION)
    row = np.floor((np.asarray(y) - config.MAP_ORIGIN_Y) / config.MAP_RESOLUTION)
    if np.isscalar(x) and np.isscalar(y):
        return int(row), int(col)
    return row.astype(np.int32), col.astype(np.int32)


def to_world(row, col):
    """격자 (row, col) → 그 셀 중심의 월드 좌표 (x, y)[m]."""
    x = config.MAP_ORIGIN_X + (np.asarray(col) + 0.5) * config.MAP_RESOLUTION
    y = config.MAP_ORIGIN_Y + (np.asarray(row) + 0.5) * config.MAP_RESOLUTION
    if np.isscalar(row) and np.isscalar(col):
        return float(x), float(y)
    return x, y


def in_bounds(row, col):
    """격자 좌표가 지도 안인가? 스칼라·배열 모두 받는다."""
    row = np.asarray(row)
    col = np.asarray(col)
    ok = (row >= 0) & (row < config.MAP_HEIGHT_CELLS) & \
         (col >= 0) & (col < config.MAP_WIDTH_CELLS)
    return bool(ok) if ok.ndim == 0 else ok


def wrap_angle(a):
    """각도를 [-pi, +pi) 범위로 접는다. 스칼라·배열 모두 받는다."""
    if np.isscalar(a):
        return float((a + math.pi) % (2.0 * math.pi) - math.pi)
    return (np.asarray(a) + math.pi) % (2.0 * math.pi) - math.pi


def angle_diff(target, current):
    """target - current 를 [-pi, +pi) 로 접어서 돌려준다.

    "얼마나, 어느 쪽으로 돌아야 하는가" 를 구할 때 쓴다. + 면 반시계.
    """
    return wrap_angle(np.asarray(target) - np.asarray(current)) \
        if not (np.isscalar(target) and np.isscalar(current)) \
        else wrap_angle(target - current)


def cells_to_metres(cells):
    """칸 수 → 길이 [m]. to_cells 의 반대.

    스캔 정합처럼 "몇 칸 옮겼나" 를 다시 미터로 돌릴 때 쓴다.
    해상도 곱셈도 나눗셈처럼 여기 한 곳에만 둔다 (CLAUDE.md 규칙 4).
    """
    return cells * config.MAP_RESOLUTION


def to_cells(metres, round_up=True):
    """길이 [m] → 칸 수. 팽창 반경 같은 "거리를 칸으로" 바꿀 때 쓴다.

    to_cell 이 위치를 바꾸는 함수라면 이건 거리를 바꾸는 함수다.
    해상도 나눗셈이 여기저기 흩어지지 않게 이것만 쓴다.
    """
    value = metres / config.MAP_RESOLUTION
    return int(math.ceil(value)) if round_up else int(value)


def cell_area():
    """격자 한 칸의 넓이 [m^2]. 탐색 면적을 보고할 때 쓴다."""
    return config.MAP_RESOLUTION * config.MAP_RESOLUTION


def map_bounds():
    """지도가 덮는 월드 범위 (x_min, x_max, y_min, y_max) [m].

    화면 표시(extent)나 범위 확인에 쓴다. 여기서 한 번만 계산해야
    to_cell/to_world 와 어긋날 일이 없다.
    """
    return (
        config.MAP_ORIGIN_X,
        config.MAP_ORIGIN_X + config.MAP_WIDTH_CELLS * config.MAP_RESOLUTION,
        config.MAP_ORIGIN_Y,
        config.MAP_ORIGIN_Y + config.MAP_HEIGHT_CELLS * config.MAP_RESOLUTION,
    )


def lidar_angles():
    """LiDAR 광선 하나하나가 로봇 기준으로 어느 방향인지 [rad] 배열로 돌려준다.

    길이는 config.LIDAR_RESOLUTION, 0 은 로봇 정면, + 는 왼쪽(반시계).
    OFFSET / SIGN 의 의미와 측정 근거는 config.py 의 해당 줄 주석 참고.
    이 식은 여기에만 있어야 한다 — 여러 곳에 흩어지면 반드시 한쪽이 뒤집힌다.
    """
    n = config.LIDAR_RESOLUTION
    index = np.arange(n)
    step = 2.0 * math.pi / n
    return wrap_angle(config.LIDAR_ANGLE_OFFSET + config.LIDAR_ANGLE_SIGN * index * step)


def distance(ax, ay, bx, by):
    """두 월드 좌표 사이의 유클리드 거리 [m]."""
    return math.hypot(bx - ax, by - ay)
