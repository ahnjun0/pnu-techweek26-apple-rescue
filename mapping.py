"""LiDAR 측정을 모아 "어디가 막혔고 어디가 비었는가" 격자 지도를 만든다.

받는 것: pose (x, y, theta) 와 LiDAR 거리 배열 [m].
내놓는 것: log-odds 격자 (row, col). 값이 크면 막힘, 작으면 빈 공간, 0 근처면 모름.
핵심 아이디어: 덮어쓰지 않고 더한다(log-odds). 그래서 걸어간 사람의 흔적이 지워진다.
광선이 지나간 칸은 빈 공간 쪽으로, 광선이 멈춘 칸은 막힘 쪽으로 조금씩 민다.
순수 numpy — Webots 없이 pytest 로 돈다.
"""

import math

import cv2
import numpy as np

import common
import config


def new_map():
    """전부 "모름"(log-odds 0) 인 빈 지도를 만든다."""
    return np.zeros((config.MAP_HEIGHT_CELLS, config.MAP_WIDTH_CELLS),
                    dtype=np.float32)


def bresenham(r0, c0, r1, c1):
    """격자 위 두 점을 잇는 칸들을 돌려준다 (양 끝 포함). 정수 연산만 쓴다.

    광선이 "지나간" 칸을 알아내려고 쓴다. 반환은 (rows, cols) 배열 두 개.
    """
    dr = abs(r1 - r0)
    dc = abs(c1 - c0)
    step_r = 1 if r1 >= r0 else -1
    step_c = 1 if c1 >= c0 else -1

    rows = []
    cols = []
    r, c = r0, c0
    if dc >= dr:
        error = dc // 2
        for _ in range(dc + 1):
            rows.append(r)
            cols.append(c)
            error -= dr
            if error < 0:
                r += step_r
                error += dc
            c += step_c
    else:
        error = dr // 2
        for _ in range(dr + 1):
            rows.append(r)
            cols.append(c)
            error -= dc
            if error < 0:
                c += step_c
                error += dr
            r += step_r
    return np.array(rows, dtype=np.int32), np.array(cols, dtype=np.int32)


def update(grid, pose, ranges, stride=None, max_range=None):
    """LiDAR 한 번 측정치를 지도에 더한다. grid 를 제자리에서 고친다.

    pose      : (x, y, theta) — 로봇이 있다고 믿는 곳
    ranges    : LiDAR 거리 배열. inf/nan 은 "아무것도 못 맞혔다" 는 뜻이다
    stride    : 광선을 몇 개마다 하나씩 쓸지 (성능). 기본 config.LIDAR_RAY_STRIDE
    max_range : 이보다 먼 측정은 버린다. 기본 config.MAP_MAX_RAY_RANGE
    """
    stride = config.LIDAR_RAY_STRIDE if stride is None else stride
    max_range = config.MAP_MAX_RAY_RANGE if max_range is None else max_range

    x, y, theta = pose
    robot_row, robot_col = common.to_cell(x, y)
    if not common.in_bounds(robot_row, robot_col):
        return grid   # 로봇이 지도 밖이면 아무것도 못 한다

    angles = common.lidar_angles()
    ranges = np.asarray(ranges, dtype=np.float64)

    # 광선을 다 돌면서 "통과한 칸" 과 "맞은 칸" 을 따로 모은다.
    # 바로바로 더하지 않는 이유: 1 m 거리에서 이웃 광선은 1.75 cm 밖에 안 떨어져 있어
    # 같은 칸을 지난다. 먼저 맞은 칸을 옆 광선이 곧바로 "빈 칸" 으로 지워 버린다.
    # 그래서 한 스캔 안에서는 "맞은 칸이 이긴다" 로 정리한 뒤 한꺼번에 더한다.
    free_rows, free_cols = [], []
    hit_rows, hit_cols = [], []

    # 지금까지 "막혔다" 고 믿는 칸. 광선은 여기서 멈춘다 (아래 설명).
    solid = is_occupied(grid)

    for i in range(0, len(ranges), stride):
        distance = ranges[i]

        # 못 맞힌 광선: "적어도 max_range 까지는 비어 있다" 고만 쓴다.
        hit = np.isfinite(distance) and config.LIDAR_MIN_RANGE <= distance <= max_range
        if not hit:
            if np.isfinite(distance) and distance < config.LIDAR_MIN_RANGE:
                continue          # 너무 가까운 값은 믿지 않는다
            distance = max_range

        world_angle = theta + angles[i]
        end_x = x + distance * math.cos(world_angle)
        end_y = y + distance * math.sin(world_angle)
        end_row, end_col = common.to_cell(end_x, end_y)

        rows, cols = bresenham(robot_row, robot_col, end_row, end_col)

        # 지도 밖으로 나간 부분은 잘라 낸다.
        keep = common.in_bounds(rows, cols)
        rows, cols = rows[keep], cols[keep]
        if len(rows) == 0:
            continue

        # ⚠️ 이미 막혔다고 믿는 칸 "너머" 까지 비우면 안 된다.
        #    거리값 하나가 잘못 들어오면 (자세 추정이 잠깐 튀거나, 반사,
        #    움직이는 물체) 광선이 벽을 지나쳐 그어진다. 그러면 벽 바깥에
        #    빈 칸이 생기고, 거기에 프론티어가 잡혀 로봇이 갈 수 없는 곳을
        #    목표로 삼는다. 실측: 아레나 밖에 빈 칸 595 개가 생겼고,
        #    프론티어 140 개 중 136 개가 아레나 밖이었다.
        #
        #    막힌 칸 "까지" 는 남긴다 — 그래야 그 칸도 빈 칸 표를 받아서,
        #    지나간 사람의 흔적처럼 진짜 벽이 아닌 것은 계속 지워진다.
        #    로봇 자신이 선 칸(0번)은 제외한다. 벽에 눌려 있을 때 그 칸이
        #    막힘으로 찍혀 있으면 모든 광선이 곧바로 잘려 지도가 멈춘다.
        blocked_at = np.nonzero(solid[rows[1:], cols[1:]])[0]
        truncated = False
        if len(blocked_at):
            stop = int(blocked_at[0]) + 2        # +1: 1부터 셌다, +1: 그 칸 포함
            if stop < len(rows):
                rows, cols = rows[:stop], cols[:stop]
                truncated = True

        # 광선이 실제로 뭔가를 맞혔고, 그 끝점이 지도 안에 남아 있을 때만 벽으로 친다.
        ray_hit = hit and keep[-1] and not truncated
        if ray_hit:
            hit_rows.append(rows[-1])
            hit_cols.append(cols[-1])
            rows, cols = rows[:-1], cols[:-1]   # 나머지는 통과한 칸

        if len(rows):
            free_rows.append(rows)
            free_cols.append(cols)

    # --- 모은 것을 한 번에 반영한다 ---------------------------------------
    delta = np.zeros_like(grid)

    if free_rows:
        fr = np.concatenate(free_rows)
        fc = np.concatenate(free_cols)
        # 여러 광선이 같은 칸을 지나면 그만큼 더 강하게 비운다 → np.add.at
        np.add.at(delta, (fr, fc), config.LOG_ODDS_FREE)

    if hit_rows:
        hr = np.asarray(hit_rows, dtype=np.int32)
        hc = np.asarray(hit_cols, dtype=np.int32)
        delta[hr, hc] = 0.0          # 맞은 칸에 묻은 "빈 칸" 표를 지운다
        np.add.at(delta, (hr, hc), config.LOG_ODDS_OCCUPIED)

    grid += delta
    np.clip(grid, config.LOG_ODDS_MIN, config.LOG_ODDS_MAX, out=grid)
    return grid


# --------------------------------------------------------------------------
# 지도 읽기 — 다른 모듈은 log-odds 숫자를 직접 비교하지 말고 이 함수들을 쓴다.
# --------------------------------------------------------------------------

def mark_camera_seen(seen, grid, pose, fov, reach, rays=None):
    """카메라 시야 안의 빈 칸을 '봤다' 고 표시한다 (제자리에서 seen 을 고친다).

    ⚠️ 왜 필요한가: 탐색 종료는 **LiDAR** 커버리지로 판단하는데, 목표물을 찾는
       것은 **카메라** 다. LiDAR 는 360°, 카메라는 57° 라 둘이 크게 어긋난다.
       지도를 다 그려도 카메라가 그쪽을 한 번도 안 봤으면 목표물은 못 찾는다.
       실측(무작위 월드 rand2): 탐색 90%, 정상 종료, 복귀 완료 — 그런데 카메라는
       80% 만 훑었고 목표물 셋 중 **하나만** 찾았다.
    """
    rays = config.CAMERA_COVER_RAYS if rays is None else rays
    occupied = is_occupied(grid)
    height, width = grid.shape
    here = common.to_cell(pose[0], pose[1])
    half = fov / 2.0
    for k in range(rays):
        angle = pose[2] - half + fov * k / max(rays - 1, 1)
        dr, dc = math.sin(angle), math.cos(angle)
        for step in range(1, reach + 1):
            r = int(round(here[0] + dr * step))
            c = int(round(here[1] + dc * step))
            if not (0 <= r < height and 0 <= c < width) or occupied[r, c]:
                break
            seen[r, c] = True


def is_occupied(grid):
    """막힌 칸 마스크 (bool 배열)."""
    return grid >= config.LOG_ODDS_OCCUPIED_THRESHOLD


def is_free(grid):
    """빈 칸 마스크."""
    return grid <= config.LOG_ODDS_FREE_THRESHOLD


def despeckle(occupied, min_neighbours=None):
    """외딴 점유칸(잡음)을 지운 마스크. **벽은 매끈하다** 는 사전지식을 쓴다.

    ⚠️ 왜 필요한가: 아레나 벽은 두께 0.1 m = 2칸이고 길게 이어지므로 진짜 벽칸은
       이웃이 여럿이다. 반면 LiDAR 잡음은 **혼자 떠 있는 한 칸** 으로 찍힌다.
       그 한 칸이 문 가운데 찍히면 planner.inflate 로 부풀려져 **문이 닫힌다** —
       실제로는 지나갈 수 있는 곳을 "갈 수 없다" 고 판정한다.
       실측: 저장된 지도에서 점유칸 1063개 중 **252개(24%)가 이웃 0개** 였다.
       그리고 comb0 은 LiDAR 탐색률 93% / 프론티어 후보 0개 로 끝났다 —
       남은 7% 안에 세 번째 목표물이 있었는데 "갈 수 없는 곳" 이 된 것이다.
       (월드를 만들 때 진짜 지도에서는 경로가 있음을 검증했으므로, 막힌 것은
        로봇의 지도뿐이다.)

    기본값은 "이웃이 0개인 칸만" 지운다 — 벽에 붙은 잡음은 남기므로 보수적이다.
    """
    min_neighbours = (config.MAP_DESPECKLE_MIN_NEIGHBOURS
                      if min_neighbours is None else min_neighbours)
    if min_neighbours <= 0 or not occupied.any():
        return occupied
    # ⚠️ 커널은 **실수** 여야 한다. np.ones((3,3), np.uint8) 을 줬더니 이웃이
    #    3개인 칸이 1로 셔져서, 한 칸 두께 벽이 통째로 지워졌다 (테스트가 잡았다).
    counts = cv2.filter2D(occupied.astype(np.uint8), cv2.CV_8U,
                          np.ones((3, 3), np.float32),
                          borderType=cv2.BORDER_CONSTANT)
    neighbours = counts.astype(np.int16) - occupied.astype(np.int16)
    return occupied & (neighbours >= min_neighbours)


def is_unknown(grid):
    """아직 모르는 칸 마스크. 프론티어는 여기와 빈 칸의 경계에서 나온다."""
    return ~is_occupied(grid) & ~is_free(grid)
