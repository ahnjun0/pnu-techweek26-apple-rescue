"""정밀 스캔 매칭(scanmatch.match_fine)을 가상의 방에서 확인한다."""

import math

import numpy as np

import common
import config
import mapping
import scanmatch


def _room_ranges(pose, half=1.61, notch=(0.92, 0.41)):
    """한 변 2*half 인 방 (⚠️ 벽을 칸 경계에 두지 않는다 — 지도는 벽을 칸 중심으로 기록해
    경계 위의 벽은 반 칸 안에서 비용이 평평하다) + 한쪽 벽의 턱(모든 방향으로 특징이 있게)을 pose 에서 본 LiDAR."""
    x, y, theta = pose
    out = []
    for a in common.lidar_angles():
        dx, dy = math.cos(theta + a), math.sin(theta + a)
        best = config.LIDAR_MAX_RANGE
        for wall, d in ((half - x, dx), (-half - x, dx)):
            if abs(d) > 1e-9 and wall / d > 0:
                best = min(best, wall / d)
        for wall, d in ((half - y, dy), (-half - y, dy)):
            if abs(d) > 1e-9 and wall / d > 0:
                best = min(best, wall / d)
        # 턱: x = notch[0] 인 짧은 벽 (y 가 notch[1] 보다 클 때만)
        if abs(dx) > 1e-9:
            k = (notch[0] - x) / dx
            if 0 < k < best and y + k * dy > notch[1]:
                best = k
        out.append(best)
    return np.array(out, dtype=np.float32)


def _mapped(pose):
    grid = mapping.new_map()
    ranges = _room_ranges(pose)
    for _ in range(6):
        mapping.update(grid, pose, ranges)
    return grid, ranges


def test_match_fine_recovers_a_sub_cell_offset_without_touching_heading():
    """5 cm 칸보다 작은 어긋남도 연속값으로 되돌리고, 방향은 그대로 둔다."""
    true_pose = (0.0, 0.0, 0.3)
    grid, ranges = _mapped(true_pose)
    field = scanmatch.likelihood_field(grid)
    guess = (0.03, -0.02, 0.3)
    fixed, _ = scanmatch.match_fine(guess, ranges, field, known=scanmatch.known_cells(grid))
    assert fixed[2] == 0.3, "방향은 나침반이 맞다 — 건드리지 않는다"
    before = math.hypot(guess[0] - true_pose[0], guess[1] - true_pose[1])
    after = math.hypot(fixed[0] - true_pose[0], fixed[1] - true_pose[1])
    assert after < before / 2, (before, after)


def test_match_fine_never_moves_more_than_the_limit_in_one_call():
    true_pose = (0.0, 0.0, 0.0)
    grid, ranges = _mapped(true_pose)
    field = scanmatch.likelihood_field(grid)
    _, moved = scanmatch.match_fine((0.3, 0.0, 0.0), ranges, field,
                                    known=scanmatch.known_cells(grid))
    assert moved <= config.SCANMATCH_RANGE + 1e-9
