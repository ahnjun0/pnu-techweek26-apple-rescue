"""practice.wbt 의 기하 구조가 말이 되는지 확인한다. Webots 없이 돈다.

여기서 걸러내려는 것: "목표물이 있는 방이 벽으로 완전히 막혀 있어 갈 수가 없다".
실제로 처음에 W2 와 W3 가 만나 오른쪽 위 방이 밀폐됐고, 목표물 2 번에 갈 수 없었다.
⚠️ worlds/practice.wbt 의 벽·목표물 좌표를 고치면 아래 표도 같이 고칠 것.
"""

import re
from collections import deque

import numpy as np
import pytest

import config

WORLD = "worlds/practice.wbt"

ARENA_HALF = 3.0        # RectangleArena floorSize 6 6
CHECK_RESOLUTION = 0.05

# (이름, 중심x, 중심y, 크기x, 크기y) — practice.wbt 의 DEF W* 와 같아야 한다
WALLS = [
    ("W1", -1.0, -1.5, 0.1, 3.0),
    ("W2", 1.0, 1.5, 0.1, 3.0),
    ("W3", 2.5, 0.0, 1.0, 0.1),
    ("W4", 0.25, -1.5, 2.5, 0.1),
]

TARGETS = [
    ("목표 1", -2.2, 1.8),
    ("목표 2", 2.3, 2.0),
    ("목표 3", 0.3, -2.4),
]

START = (config.START_X, config.START_Y)


# --------------------------------------------------------------------------
# 위 표가 실제 월드 파일과 같은지부터 확인한다 (표만 고치고 월드를 안 고치면 소용없다)
# --------------------------------------------------------------------------

def read_world():
    with open(WORLD, encoding="utf-8") as handle:
        return handle.read()


def solid_block(text, def_name):
    """DEF <이름> Solid { ... } 블록에서 translation 과 마지막 size 를 뽑는다."""
    start = text.index(f"DEF {def_name} Solid")
    end = text.index("\nDEF ", start + 1) if "\nDEF " in text[start + 1:] else len(text)
    block = text[start:end]
    translation = re.search(r"translation\s+(\S+)\s+(\S+)\s+(\S+)", block)
    size = re.search(r"size\s+(\S+)\s+(\S+)\s+(\S+)", block)
    return ([float(v) for v in translation.groups()],
            [float(v) for v in size.groups()] if size else None)


@pytest.mark.parametrize("name,cx,cy,sx,sy", WALLS)
def test_wall_table_matches_the_world_file(name, cx, cy, sx, sy):
    translation, size = solid_block(read_world(), name)
    assert (translation[0], translation[1]) == pytest.approx((cx, cy)), \
        f"{name} 의 위치가 월드 파일과 다르다. 이 파일의 WALLS 표를 고칠 것"
    assert (size[0], size[1]) == pytest.approx((sx, sy)), \
        f"{name} 의 크기가 월드 파일과 다르다. 이 파일의 WALLS 표를 고칠 것"


def test_robot_start_matches_config():
    """월드의 로봇 위치와 config.START_X/Y 가 어긋나면 처음부터 지도가 틀어진다."""
    text = read_world()
    match = re.search(r"TurtleBot3Burger \{\s*\n\s*translation\s+(\S+)\s+(\S+)", text)
    assert match, "월드에서 TurtleBot3Burger 의 translation 을 못 찾았다"
    assert (float(match.group(1)), float(match.group(2))) == \
        pytest.approx((config.START_X, config.START_Y))


def test_target_table_matches_the_world_file():
    text = read_world()
    for index, (_, tx, ty) in enumerate(TARGETS, start=1):
        translation, _ = solid_block(text, f"TARGET_{index}")
        assert (translation[0], translation[1]) == pytest.approx((tx, ty))


# --------------------------------------------------------------------------
# 진짜 검사: 시작점에서 목표 3개에 다 갈 수 있는가
# --------------------------------------------------------------------------

def build_occupancy():
    """벽을 격자에 칠하고, 로봇 반경만큼 부풀린 "못 가는 칸" 배열을 만든다."""
    n = int(round(2 * ARENA_HALF / CHECK_RESOLUTION))

    def index_of(value):
        return int(round((value + ARENA_HALF) / CHECK_RESOLUTION))

    blocked = np.zeros((n, n), dtype=bool)   # [row = y, col = x]
    for _, cx, cy, sx, sy in WALLS:
        r0 = max(0, index_of(cy - sy / 2))
        r1 = min(n, index_of(cy + sy / 2) + 1)
        c0 = max(0, index_of(cx - sx / 2))
        c1 = min(n, index_of(cx + sx / 2) + 1)
        blocked[r0:r1, c0:c1] = True

    # 로봇은 점이 아니다. 반경만큼 부풀려서 "실제로 지나갈 수 있는가" 를 본다.
    radius = int(round(config.ROBOT_RADIUS / CHECK_RESOLUTION))
    rows, cols = np.nonzero(blocked)
    inflated = blocked.copy()
    for dr in range(-radius, radius + 1):
        for dc in range(-radius, radius + 1):
            if dr * dr + dc * dc <= radius * radius:
                inflated[np.clip(rows + dr, 0, n - 1),
                         np.clip(cols + dc, 0, n - 1)] = True
    # 아레나 테두리도 막는다
    inflated[:radius, :] = inflated[-radius:, :] = True
    inflated[:, :radius] = inflated[:, -radius:] = True
    return inflated, index_of


def reachable_from_start():
    inflated, index_of = build_occupancy()
    n = inflated.shape[0]
    start = (index_of(START[1]), index_of(START[0]))
    assert not inflated[start], "로봇 시작 위치가 벽 안에 박혀 있다"

    seen = np.zeros_like(inflated)
    seen[start] = True
    queue = deque([start])
    while queue:
        r, c = queue.popleft()
        for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            nr, nc = r + dr, c + dc
            if 0 <= nr < n and 0 <= nc < n and not seen[nr, nc] and not inflated[nr, nc]:
                seen[nr, nc] = True
                queue.append((nr, nc))
    return seen, index_of


@pytest.mark.parametrize("name,tx,ty", TARGETS)
def test_every_target_is_reachable(name, tx, ty):
    """목표물이 밀폐된 방에 갇혀 있으면 로봇이 아무리 잘해도 완주할 수 없다."""
    seen, index_of = reachable_from_start()

    # 목표물 자체는 장애물이므로, 그 둘레 어딘가에 설 수 있으면 된다.
    # ⚠️ 여기서는 로봇 반경만 쓴다. 실제 주행은 planner 가 여유(margin)까지 부풀리므로
    #    이 검사를 통과해도 문이 너무 좁으면 못 지나간다. 아래 test 가 그걸 본다.
    radius = int(round(0.35 / CHECK_RESOLUTION))
    row, col = index_of(ty), index_of(tx)
    patch = seen[max(0, row - radius):row + radius + 1,
                 max(0, col - radius):col + radius + 1]
    assert patch.any(), f"{name} ({tx}, {ty}) 근처에 갈 수가 없다 — 벽으로 막힌 방이다"


def test_doors_are_wide_enough_for_the_planner():
    """연결돼 있기만 해선 부족하다. 계획할 때 부풀리는 여유까지 빼고도 길이 남아야 한다.

    실제로 문을 0.55 m 로 냈다가 남는 폭이 0.13 m 뿐이라 로봇이 문틀을 긁었다.
    """
    saved = config.ROBOT_RADIUS
    try:
        config.ROBOT_RADIUS = saved + config.PLANNER_INFLATION_MARGIN
        seen, index_of = reachable_from_start()
        for name, tx, ty in TARGETS:
            radius = int(round(0.45 / CHECK_RESOLUTION))
            row, col = index_of(ty), index_of(tx)
            patch = seen[max(0, row - radius):row + radius + 1,
                         max(0, col - radius):col + radius + 1]
            assert patch.any(), \
                (f"{name} 까지 가는 문이 너무 좁다 "
                 f"(로봇 반경 {saved} + 여유 {config.PLANNER_INFLATION_MARGIN} 기준)")
    finally:
        config.ROBOT_RADIUS = saved


def test_map_covers_the_whole_arena():
    """지도 격자가 아레나보다 작으면 가장자리를 영영 못 그린다."""
    x_min, x_max, y_min, y_max = (
        config.MAP_ORIGIN_X,
        config.MAP_ORIGIN_X + config.MAP_WIDTH_CELLS * config.MAP_RESOLUTION,
        config.MAP_ORIGIN_Y,
        config.MAP_ORIGIN_Y + config.MAP_HEIGHT_CELLS * config.MAP_RESOLUTION,
    )
    assert x_min <= -ARENA_HALF and x_max >= ARENA_HALF
    assert y_min <= -ARENA_HALF and y_max >= ARENA_HALF
