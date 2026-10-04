"""LiDAR 스캔을 지도에 맞춰 위치 추정을 고친다 (거리장 위의 가우스-뉴턴 정합).

받는 것: 오도메트리가 믿는 pose, LiDAR 거리 배열, 지금까지의 지도.
내놓는 것: 고친 pose (x, y, theta) — 방향은 나침반을 믿고 x, y 만 고친다.
핵심 아이디어: 지도의 벽까지 거리장(likelihood field)을 만들고, 스캔 점들이 벽에
가장 잘 얹히도록 x, y 를 연속값으로 옮긴다. 바퀴가 헛돌아도 벽은 제자리에 있으므로,
이것이 추측항법의 누적 오차를 끊어 준다.
순수 numpy + OpenCV — Webots 없이 pytest 로 돈다.

⚠️ 지도가 틀어져 있으면 스캔도 그쪽으로 끌려간다. 그래서 "많이" 고치지 않는다.
   한 번에 고칠 수 있는 양을 좁게 제한해, 오도메트리를 보정만 하고 대체하지 않는다.
"""

import math

import cv2
import numpy as np

from . import common
from . import config
from . import mapping


def likelihood_field(grid):
    """각 칸이 가장 가까운 벽에서 몇 칸 떨어져 있는지 (float32).

    스캔 점이 여기 얹혔을 때 값이 작을수록 벽에 잘 맞은 것이다.
    벽이 하나도 없으면 맞출 대상이 없으므로 None.
    """
    occupied = mapping.is_occupied(grid)
    if not occupied.any():
        return None
    return cv2.distanceTransform((~occupied).astype(np.uint8), cv2.DIST_L2, 5)


def known_cells(grid):
    """지도에서 **아는** 칸 (빈 칸이든 벽이든). 모르는 칸은 맞춤에 쓰면 안 된다.

    ⚠️ likelihood_field 는 distanceTransform(~occupied) 이라 **미탐색 칸을
       "벽에서 먼 곳"** 으로 계산한다. 로봇 앞쪽 벽은 아직 지도에 없으니 그쪽
       스캔점은 비용이 높고, pose 를 **뒤로** 밀면 이미 지도에 있는 벽 위로
       옮겨져 비용이 내려간다 — 진짜 이득이 있는 체계적 후진 편향이다.
       (그래서 성분별 이득 검사로도 걸리지 않는다. 앨리어싱이 아니다.)
       실측(corridor2): 20~40초에 로봇은 앞으로 0.70 m 갔는데 추정은 뒤로
       0.75 m 가서 오차 145.6 cm, 지도가 망가져 61초에 0/3.
       기어갈 때는 지도가 따라잡아 안 보였고, 0.2 m/s 로 달리자
       **로봇이 자기 지도를 앞질러** 터졌다.
    """
    return ~mapping.is_unknown(grid)


def _scan_points(ranges, max_range):
    """쓸 만한 광선만 골라 로봇 기준 좌표로. (x, y) 배열 두 개."""
    angles = common.lidar_angles()
    values = np.asarray(ranges, dtype=np.float32)
    good = (np.isfinite(values) & (values >= config.LIDAR_MIN_RANGE)
            & (values <= max_range))
    if good.sum() < config.SCANMATCH_MIN_POINTS:
        return None, None
    r, a = values[good], angles[good]
    return r * np.cos(a), r * np.sin(a)


def _bilinear(image, rows, cols):
    """image 를 (rows, cols) 연속 좌표에서 보간한다. 범위 밖은 None."""
    height, width = image.shape
    r0 = np.floor(rows).astype(np.int64)
    c0 = np.floor(cols).astype(np.int64)
    if r0.min() < 0 or c0.min() < 0 or r0.max() + 1 >= height or c0.max() + 1 >= width:
        return None
    fr, fc = rows - r0, cols - c0
    return ((1 - fr) * (1 - fc) * image[r0, c0] + (1 - fr) * fc * image[r0, c0 + 1]
            + fr * (1 - fc) * image[r0 + 1, c0] + fr * fc * image[r0 + 1, c0 + 1])


def match_fine(pose, ranges, field, known=None):
    """거리장을 보간해 x, y 를 **연속값**으로 고친다 (가우스-뉴턴). 방향은 그대로 둔다.

    비용: 스캔 점 i 가 가장 가까운 벽에서 떨어진 거리 d_i(칸) 의 제곱합.
        min_Δ  Σ d_i(p_i + Δ)²   →   (JᵀJ) Δ = −Jᵀ d,   J_i = ∇d(p_i)
    JᵀJ 의 고윳값이 작은 방향은 근거가 없는 방향이라 버린다 (복도 축퇴).
    돌려주는 값은 match 와 같다: (고친 pose, 옮긴 거리 [m]).
    """
    if field is None:
        return pose, 0.0
    px, py = _scan_points(ranges, config.MAP_MAX_RAY_RANGE)
    if px is None:
        return pose, 0.0
    x, y, theta = pose
    c, s = math.cos(theta), math.sin(theta)
    wx, wy = x + px * c - py * s, y + px * s + py * c
    if known is not None:
        rows0, cols0 = common.to_cell(wx, wy)
        height, width = known.shape
        inside = (rows0 >= 0) & (rows0 < height) & (cols0 >= 0) & (cols0 < width)
        keep = np.zeros_like(inside)
        keep[inside] = known[rows0[inside], cols0[inside]]
        wx, wy = wx[keep], wy[keep]
    rows, cols = common.to_cell_float(wx, wy)
    cap = common.to_cells_float(config.SCANMATCH_FINE_CAP)
    # ⚠️ 비교에 쓰는 점 집합은 **처음에 한 번** 고른다. 옮길 때마다 다시 고르면 분모가
    #    달라져 비용끼리 비교가 안 된다 (match 의 known 필터에서 배운 것).
    d0 = _bilinear(field, rows, cols)
    if d0 is None:
        return pose, 0.0
    near = d0 < cap
    if int(near.sum()) < config.SCANMATCH_MIN_POINTS:
        return pose, 0.0
    rows, cols = rows[near], cols[near]
    grad_r, grad_c = np.gradient(field)

    def cost(sr, sc):
        d = _bilinear(field, rows + sr, cols + sc)
        return math.inf if d is None else float(np.mean(np.minimum(d, cap) ** 2))

    shift_r = shift_c = 0.0
    current = cost(0.0, 0.0)
    for _ in range(config.SCANMATCH_FINE_ITERS):
        d = _bilinear(field, rows + shift_r, cols + shift_c)
        gr = _bilinear(grad_r, rows + shift_r, cols + shift_c)
        gc = _bilinear(grad_c, rows + shift_r, cols + shift_c)
        if d is None or gr is None or gc is None:
            break
        jac = np.stack([gr, gc], axis=1)
        info = jac.T @ jac / len(d)
        rhs = -(jac.T @ d) / len(d)
        values, vectors = np.linalg.eigh(info)
        step = np.zeros(2)
        for k in range(2):
            if values[k] >= config.SCANMATCH_FINE_MIN_EIG:
                step += vectors[:, k] * (vectors[:, k] @ rhs) / values[k]
        # 거리장은 벽에서 V 자라 한 걸음이 넘어가기 쉽다 — 비용이 줄 때까지 반씩 줄인다
        scale = 1.0
        while scale > 0.05:
            trial = cost(shift_r + scale * step[0], shift_c + scale * step[1])
            if trial < current:
                break
            scale *= 0.5
        else:
            break
        shift_r += scale * step[0]
        shift_c += scale * step[1]
        current = trial
        if scale * math.hypot(*step) < 0.01:
            break
    limit = common.to_cells_float(config.SCANMATCH_RANGE)
    moved = math.hypot(shift_r, shift_c)
    if moved > limit:                       # 한 번에 크게 옮기지 않는다
        shift_r *= limit / moved
        shift_c *= limit / moved
    blend = config.SCANMATCH_FINE_BLEND
    dx = common.cells_to_metres(shift_c) * blend
    dy = common.cells_to_metres(shift_r) * blend
    return (x + dx, y + dy, theta), math.hypot(dx, dy)
