"""LiDAR 스캔을 지도에 맞춰 위치 추정을 고친다 (Correlative Scan Matching).

받는 것: 오도메트리가 믿는 pose, LiDAR 거리 배열, 지금까지의 지도.
내놓는 것: 고친 pose (x, y, theta).
핵심 아이디어: 지금 스캔을 조금씩 옮겨·돌려 보고, 스캔 점들이 지도의 벽에
가장 잘 얹히는 자리를 고른다. 바퀴가 헛돌아도 벽은 제자리에 있으므로,
이것이 추측항법의 누적 오차를 끊어 준다.
순수 numpy + OpenCV — Webots 없이 pytest 로 돈다.

⚠️ 지도가 틀어져 있으면 스캔도 그쪽으로 끌려간다. 그래서 "많이" 고치지 않는다.
   한 번에 고칠 수 있는 양을 좁게 제한해, 오도메트리를 보정만 하고 대체하지 않는다.
"""

import math

import cv2
import numpy as np

import common
import config
import mapping


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


def match(pose, ranges, field, max_range=None, known=None):
    """pose 를 조금씩 옮겨 보고 스캔이 지도에 가장 잘 맞는 자리를 돌려준다.

    field 는 likelihood_field(grid) 의 결과. None 이면 pose 를 그대로 돌려준다.
    돌려주는 값: (고친 pose, 얼마나 좋아졌나) — 좋아진 정도는 0 이면 그대로다.
    """
    if field is None:
        return pose, 0.0
    px, py = _scan_points(ranges, max_range or config.MAP_MAX_RAY_RANGE)
    if px is None:
        return pose, 0.0

    x, y, theta = pose
    # ⚠️ 훑는 폭은 **격자 해상도** 다. 그보다 잘게 움직일 수 없기 때문이다.
    #    예전에는 config.SCANMATCH_STEP(0.025 m)로 칸 수를 따로 계산했는데,
    #    그 값이 해상도(0.05 m)보다 작아서 to_cells 가 1칸으로 올림하는 한편
    #    span 은 0.025 기준으로 계산됐다. 그 결과 **실제 탐색 범위가 설정값의
    #    두 배**가 됐다 (±0.10 m 라고 적어 놓고 ±0.20 m 를 훑었다).
    #    실측(comb1): 한 틱에 20.1 cm 씩 튀는 보정이 나왔고 — 상한 0.14 cm 를
    #    넘는다 — 보정 15번 중 11번이 드리프트를 키워 순 +60 cm 를 더했다.
    #    최종 오차 61 cm 가 사실상 전부 여기서 나왔고, 그 탓에 통로가 닫힌 것으로
    #    그려져 목표물 둘을 놓쳤다.
    span = common.to_cells(config.SCANMATCH_RANGE) or 1     # 칸 단위
    shifts = np.arange(-span, span + 1)                     # 칸 단위
    turns = np.linspace(-config.SCANMATCH_TURN, config.SCANMATCH_TURN,
                        2 * config.SCANMATCH_TURN_STEPS + 1)

    height, width = field.shape

    def cost_at(rows, cols):
        """이 자리의 비용. 지도 밖으로 나가면 inf."""
        if (rows.min() < 0 or rows.max() >= height
                or cols.min() < 0 or cols.max() >= width):
            return math.inf
        return float(field[rows, cols].mean())

    # ⚠️ 제자리(이동 0)의 비용을 먼저 재 둔다. 이게 없으면 우도장이 평평할 때
    #    잡음이 고른 아무 자리로나 옮겨간다 — **긴 균일 복도** 가 정확히 그렇다.
    #    진행 방향으로 특징이 없어 비용이 거의 같으므로, 8틱마다 조금씩 밀리다
    #    누적된다. 실측(corridor2): 곧게 직진하는 중 40초에 오도메트리 오차가
    #    105 cm 로 튀었고, 위치를 잃자 61초 만에 0/3 으로 끝났다.
    #    (고전적으로 corridor aliasing 이라 부르는 문제다.)
    c0, s0 = math.cos(theta), math.sin(theta)
    rows0, cols0 = common.to_cell(x + px * c0 - py * s0, y + px * s0 + py * c0)

    # ⚠️ **아는 칸에 떨어진 점만** 쓴다 (known_cells 의 설명 참고). 그런데 그
    #    골라내기를 이동량마다 새로 하면 안 된다 — 기여하는 점의 **집합이 달라져**
    #    평균을 서로 비교할 수 없다. 뒤로 밀면 아는 칸에 떨어지는 점이 늘어
    #    평균이 내려가므로, 후진 편향이 형태만 바꿔 되살아난다.
    #    실측: 그렇게 했더니 오차가 첫 20초에 95 cm 생기고 벽에 닿았다(-0.7 cm).
    #    → 제자리에서 아는 칸에 떨어진 점을 **한 번** 골라, 그 집합으로만 전부
    #      비교한다. 분모가 같아야 비교가 성립한다.
    if known is not None:
        if (0 <= rows0.min() and rows0.max() < height
                and 0 <= cols0.min() and cols0.max() < width):
            keep = known[rows0, cols0]
            if int(keep.sum()) < config.SCANMATCH_MIN_POINTS:
                return pose, 0.0
            px, py = px[keep], py[keep]
            rows0, cols0 = rows0[keep], cols0[keep]
        else:
            return pose, 0.0
    base_cost = cost_at(rows0, cols0)

    best = (math.inf, 0, 0, 0.0)
    for dtheta in turns:
        c, s = math.cos(theta + dtheta), math.sin(theta + dtheta)
        rows, cols = common.to_cell(x + px * c - py * s, y + px * s + py * c)
        for dr in shifts:
            for dc in shifts:
                cost = cost_at(rows + dr, cols + dc)
                if cost < best[0]:
                    best = (cost, dr, dc, dtheta)

    cost, dr, dc, dtheta = best
    if not math.isfinite(cost):
        return pose, 0.0
    # 제자리보다 **뚜렷하게** 나을 때만 옮긴다. 평평한 우도장에서 잡음을 따라가지
    # 않게 하는 유일한 장치다 (위 corridor aliasing 설명 참고).
    if math.isfinite(base_cost) and (base_cost - cost) < config.SCANMATCH_MIN_GAIN:
        return pose, 0.0

    # ⚠️ 그런데 이 검사가 **벡터 전체** 를 통과시키면 안 된다. 복도에서는 옆으로
    #    가는 성분만 근거가 있고 진행 방향 성분은 근거가 없는데, 합쳐서 문턱을
    #    넘으면 근거 없는 성분까지 같이 적용된다. 0.128초마다 조금씩, 156번이면
    #    1 m 다. 실측(corridor2): 20초에 오차 0.2 cm → 40초에 100.3 cm, 추정
    #    -0.79 / 진짜 +0.21 (= 정확히 20칸). 위치를 잃자 61초에 0/3 으로 끝났다.
    #    로봇이 느리게 갈 때는 갱신 사이 이동이 0.3 cm 라 드러나지 않았다.
    #    → 성분마다 따로 묻는다. **혼자서도** 문턱을 넘는 성분만 남긴다.
    #    (교과서적으로는 비용면의 축퇴(degeneracy) 를 걸러내는 것이다. 지도축으로
    #     나누므로 축에 평행한 복도에서 정확히 듣고, 비스듬한 복도에서는 덜 듣는다.)
    #    ⚠️ corridor2 의 후진 편향은 이것이 아니라 아래 known 필터가 잡았다.
    #       그래서 이 장치를 빼 보고 재 봤는데, 빼는 쪽이 나빴다:
    #         있음  comb0  56.2 m / 554초 / 2-3
    #         없음  comb0 117.5 m / 883초 / 2-3
    #       목표물은 같고 거리·시간만 두 배로 늘어 남긴다.
    if math.isfinite(base_cost):
        c1, s1 = math.cos(theta + dtheta), math.sin(theta + dtheta)
        rows1, cols1 = common.to_cell(x + px * c1 - py * s1,
                                      y + px * s1 + py * c1)

        def alone(ar, ac):
            """(ar, ac) 만 적용했을 때의 비용."""
            return cost_at(rows1 + ar, cols1 + ac)

        if base_cost - alone(dr, 0) < config.SCANMATCH_COMPONENT_GAIN:
            dr = 0
        if base_cost - alone(0, dc) < config.SCANMATCH_COMPONENT_GAIN:
            dc = 0
        if base_cost - alone(0, 0) < config.SCANMATCH_COMPONENT_GAIN:
            dtheta = 0.0
        if dr == 0 and dc == 0 and dtheta == 0.0:
            return pose, 0.0
    # 칸 단위 이동을 다시 미터로
    moved = (common.cells_to_metres(dc), common.cells_to_metres(dr))
    fixed = (x + moved[0], y + moved[1], common.wrap_angle(theta + dtheta))
    return fixed, math.hypot(*moved)


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
