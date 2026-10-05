"""지도 위에서 "여기서 저기까지 어떻게 갈까" 를 계산한다.

받는 것: log-odds 지도, 출발 월드좌표, 목표 월드좌표.
내놓는 것: 지나갈 지점들 [(x, y), ...] 또는 길이 없으면 None.
핵심 아이디어: (1) 벽을 로봇 반경 + 여유만큼 부풀리고 (2) 8방향 A* 로 찾고
(3) 직선으로 보이는 구간을 합쳐 웨이포인트 수를 줄인다.
대각선이 벽 모서리를 뚫고 지나가지 못하게 막는 것이 핵심이다.
순수 numpy — Webots 없이 pytest 로 돈다. 지도 크기·해상도는 전부 config 에서 읽는다.
"""

import heapq
import math

import functools

import cv2
import numpy as np

from . import common
from . import config
from . import mapping

# 8방향 이동. (dr, dc, 한 걸음 비용[칸])
_MOVES = [
    (-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
    (-1, -1, math.sqrt(2)), (-1, 1, math.sqrt(2)),
    (1, -1, math.sqrt(2)), (1, 1, math.sqrt(2)),
]


def _disk_offsets(radius_cells):
    """반지름 안에 들어오는 (dr, dc) 목록. 팽창에 쓴다."""
    offsets = []
    for dr in range(-radius_cells, radius_cells + 1):
        for dc in range(-radius_cells, radius_cells + 1):
            if dr * dr + dc * dc <= radius_cells * radius_cells:
                offsets.append((dr, dc))
    return offsets


@functools.lru_cache(maxsize=8)
def _disk_kernel(radius_cells):
    """_disk_offsets 와 똑같은 원반을 OpenCV 커널로."""
    size = 2 * radius_cells + 1
    kernel = np.zeros((size, size), np.uint8)
    for dr, dc in _disk_offsets(radius_cells):
        kernel[dr + radius_cells, dc + radius_cells] = 1
    return kernel


def margin_ladder():
    """갈 길을 물을 때 쓰는 여유의 순서: 평소(None) → 좁게 → (켜져 있으면) 더 좁게.

    후보 고르기(exploration.candidate_list)·길 짜기(mission._replan)·집 도달 판정이
    **같은 순서** 를 써야 한다 — 하나만 다르면 고른 목표로 길을 못 짜거나 그 반대가 된다.
    """
    ladder = [None, config.PLANNER_SQUEEZE_MARGIN]
    if config.PLANNER_TIGHT_MARGIN is not None:
        ladder.append(config.PLANNER_TIGHT_MARGIN)
    return ladder


def inflate(grid, margin=None):
    """막힌 칸을 로봇 반경 + 여유만큼 부풀린 "가면 안 되는 칸" 마스크를 만든다.

    로봇을 점으로 보고 길을 찾기 위한 표준 수법이다.
    이렇게 해야 계획된 경로 자체가 이미 안전거리를 지킨다.
    """
    margin = config.PLANNER_INFLATION_MARGIN if margin is None else margin
    radius_cells = common.to_cells(config.ROBOT_RADIUS + margin)

    blocked = mapping.is_occupied(grid)

    # ⚠️ 이건 "모폴로지 팽창" 이라는 이름이 붙은 표준 연산이다. 직접 짰다가
    #    이미 쓰고 있는 OpenCV 로 바꿨다 — 결과가 한 칸도 다르지 않으면서
    #    27배 빠르다 (2.43 ms -> 0.09 ms). 커널은 우리 원반 모양 그대로 쓴다.
    if radius_cells > 0 and blocked.any():
        inflated = cv2.dilate(blocked.astype(np.uint8),
                              _disk_kernel(radius_cells)).astype(bool)
    else:
        inflated = blocked.copy()

    # 지도 테두리도 못 가는 곳으로 둔다 (경로가 지도 밖으로 새지 않게).
    # 장애물이 하나도 없어도 이건 해야 한다.
    if radius_cells > 0:
        inflated[:radius_cells, :] = True
        inflated[-radius_cells:, :] = True
        inflated[:, :radius_cells] = True
        inflated[:, -radius_cells:] = True
    return inflated


def obstacle_distance(grid, max_cells):
    """각 칸이 가장 가까운 벽에서 몇 칸 떨어져 있는지 (max_cells 에서 자른다).

    벽에서 시작해 한 칸씩 부풀려 나가며 센다. numpy 로만 해서 빠르다.
    """
    reached = mapping.is_occupied(grid)
    distance = np.full(grid.shape, max_cells, dtype=np.int16)
    distance[reached] = 0
    for step in range(1, max_cells + 1):
        grown = reached.copy()
        grown[:-1, :] |= reached[1:, :]
        grown[1:, :] |= reached[:-1, :]
        grown[:, :-1] |= reached[:, 1:]
        grown[:, 1:] |= reached[:, :-1]
        fresh = grown & ~reached
        if not fresh.any():
            break
        distance[fresh] = step
        reached = grown
    return distance


def cost_layer(grid, carved=None, people=()):
    """칸마다의 이동 비용 배수 (1.0 = 보통). 벽에 가까울수록 비싸다.

    갈 수는 있지만 굳이 가고 싶지 않은 곳을 비싸게 만들어,
    경로가 빈 공간 가운데로 흐르게 한다 (주행이 부드러워진다).
    carved 는 "로봇이 벽에 붙어 있어 어쩔 수 없이 뚫어 준 칸" 마스크다.
    people 을 주면 그 둘레를 비싸게 만든다 (ROS 의 social costmap layer 와 같은 방식).
    ⚠️ 딱딱하게 막지 않는다. 막으면 사람이 통로를 메웠을 때 갈 길이 없어진다.
       비싸게만 만들면 "돌아갈 길이 있으면 돌아가고, 없으면 그냥 간다" 가 된다.
    """
    cost = np.ones(grid.shape, dtype=np.float32)

    soft_cells = common.to_cells(config.PLANNER_SOFT_CLEARANCE)
    hard_cells = common.to_cells(config.ROBOT_RADIUS
                                 + config.PLANNER_INFLATION_MARGIN)
    distance = obstacle_distance(grid, hard_cells + soft_cells)

    # 팽창 경계(hard_cells) 바로 바깥이 제일 비싸고, soft_cells 만큼 멀어지면 1 이 된다.
    beyond = (distance - hard_cells).astype(np.float32)
    nearness = np.clip(1.0 - beyond / max(soft_cells, 1), 0.0, 1.0)
    cost += (config.PLANNER_SOFT_WEIGHT - 1.0) * nearness

    # 모르는 칸은 아는 길보다 비싸게 (탐색 중에는 지나갈 수 있어야 하니 막지는 않는다)
    if len(people) and config.PEOPLE_COST_RADIUS > 0.0:
        span = common.to_cells(config.PEOPLE_COST_RADIUS)
        if span > 0:
            rows = np.arange(grid.shape[0])[:, None]
            cols = np.arange(grid.shape[1])[None, :]
            for wx, wy in people:
                row, col = common.to_cell(wx, wy)
                if not common.in_bounds(row, col):
                    continue
                r0, r1 = max(0, row - span), min(grid.shape[0], row + span + 1)
                c0, c1 = max(0, col - span), min(grid.shape[1], col + span + 1)
                near = np.hypot(rows[r0:r1] - row, cols[:, c0:c1] - col)
                close = np.clip(1.0 - near / span, 0.0, 1.0)
                cost[r0:r1, c0:c1] += (config.PEOPLE_COST_WEIGHT - 1.0) * close

    cost[mapping.is_unknown(grid)] *= config.PLANNER_UNKNOWN_COST

    if carved is not None and carved.any():
        cost[carved] *= config.PLANNER_CARVED_COST

    return cost


def astar(blocked, cost, start, goal, max_nodes=None):
    """8방향 A*. start/goal 은 (row, col). 경로(칸 목록) 또는 None.

    cost 는 칸마다의 이동 비용 배수 배열 (1.0 = 보통, 클수록 피한다).
    대각선 이동은 양 옆 칸이 둘 다 비어 있을 때만 허용한다.
    안 그러면 벽 모서리를 대각선으로 "뚫고" 지나가는 경로가 나온다.
    """
    max_nodes = config.PLANNER_MAX_NODES if max_nodes is None else max_nodes
    height, width = blocked.shape

    if not (common.in_bounds(*start) and common.in_bounds(*goal)):
        return None
    if blocked[goal]:
        return None

    def heuristic(row, col):
        """옥타일 거리 — 8방향에서 남은 거리를 절대 과대평가하지 않는다."""
        dr = abs(row - goal[0])
        dc = abs(col - goal[1])
        return (dr + dc) + (math.sqrt(2) - 2.0) * min(dr, dc)

    came_from = {}
    best_cost = {start: 0.0}
    queue = [(heuristic(*start), 0.0, start)]
    visited = set()
    expanded = 0

    while queue:
        _, travelled, current = heapq.heappop(queue)
        if current in visited:
            continue
        visited.add(current)
        expanded += 1
        if expanded > max_nodes:
            return None

        if current == goal:
            path = [current]
            while current in came_from:
                current = came_from[current]
                path.append(current)
            path.reverse()
            return path

        row, col = current
        for dr, dc, step in _MOVES:
            nr, nc = row + dr, col + dc
            if not (0 <= nr < height and 0 <= nc < width) or blocked[nr, nc]:
                continue
            # 대각선은 모서리를 뚫지 못한다: 옆 두 칸이 "모두" 열려 있어야 한다.
            # 한쪽만 막혀도 금지하는 엄격한 규칙을 쓴다. 팽창까지 했는데도 굳이
            # 모서리를 스치며 지나가는 경로를 만들 이유가 없다 (안전거리 우선).
            if dr != 0 and dc != 0:
                if blocked[row + dr, col] or blocked[row, col + dc]:
                    continue

            new_cost = travelled + step * float(cost[nr, nc])
            if new_cost < best_cost.get((nr, nc), math.inf):
                best_cost[(nr, nc)] = new_cost
                came_from[(nr, nc)] = current
                heapq.heappush(queue, (new_cost + heuristic(nr, nc),
                                       new_cost, (nr, nc)))
    return None


def line_is_clear(blocked, a, b):
    """두 칸을 잇는 직선이 막힌 칸을 지나지 않는가. 웨이포인트를 줄일 때 쓴다."""
    rows, cols = mapping.bresenham(a[0], a[1], b[0], b[1])
    return not blocked[rows, cols].any()


def _line_max_cost(cost, a, b):
    rows, cols = mapping.bresenham(a[0], a[1], b[0], b[1])
    return float(cost[rows, cols].max())


def simplify(path, blocked, cost=None):
    """칸 단위 경로를 "꺾이는 곳" 만 남긴 웨이포인트로 줄인다.

    지금 지점에서 직선으로 보이는 가장 먼 지점까지 건너뛴다.
    경로가 짧아지고 부드러워진다.

    ⚠️ cost 를 주면 "지름길이 원래 경로보다 비싸지 않을 때만" 건너뛴다.
       이걸 안 하면, A* 가 비싼 구역(사람이 지나다니는 자리 등)을 피해 돌아간
       경로를 여기서 다시 직선으로 펴 버려 애써 피한 곳을 통과하게 된다.
    """
    if not path or len(path) <= 2:
        return list(path)

    result = [path[0]]
    index = 0
    while index < len(path) - 1:
        furthest = index + 1
        for candidate in range(len(path) - 1, index, -1):
            if not line_is_clear(blocked, path[index], path[candidate]):
                continue
            if cost is not None:
                shortcut = _line_max_cost(cost, path[index], path[candidate])
                original = max(float(cost[r, c])
                               for r, c in path[index:candidate + 1])
                if shortcut > original + 1e-6:
                    continue        # 지름길이 더 비싸다 — 원래 길을 지킨다
            furthest = candidate
            break
        result.append(path[furthest])
        index = furthest
    return result


def plan(grid, start_xy, goal_xy, margin=None, allow_unknown=None, people=(),
         exact=False):
    """월드 좌표로 주고받는 바깥쪽 입구. 웨이포인트 [(x, y), ...] 또는 None.

    goal 칸이 팽창 때문에 막혀 있으면, 그 근처에서 갈 수 있는 가장 가까운 칸을
    대신 목표로 삼는다 (벽에 붙은 목표물에 다가갈 때 필요하다).

    exact=True 면 그 **대체를 하지 않는다** — 목표 칸에 못 서면 None 을 준다.

    ⚠️ 이 대체가 조용히 일어나는 바람에 같은 버그를 세 번 만들었다:
       ① 프론티어 후보를 "plan() 이 참이면 갈 수 있다" 로 걸렀다 → 통과 못 할
          것이 없는 필터였고, 못 가는 구석을 영원히 골랐다 (한자리에 800초).
       ② 주행기가 대체된 경로의 끝에서 ARRIVED 를 내는데 임무가 그걸 "도착" 으로
          쳤다 → 목표를 못 가 놓고 성공 처리.
       ③ _replan 이 "평소 여유로 먼저, 안 되면 좁게" 를 하는데, 평소 여유에서
          plan() 이 대체로 항상 성공해 **좁은 여유를 영영 안 써 봤다**
          → 좁은 문 뒤가 목표일 때 속도 0 으로 600초를 돌았다.
       그러므로 "정말 거기 갈 수 있는가" 를 물을 때는 반드시 exact=True 다.
    """
    allow_unknown = (config.PLANNER_ALLOW_UNKNOWN if allow_unknown is None
                     else allow_unknown)

    blocked = inflate(grid, margin)
    if not allow_unknown:
        blocked = blocked | mapping.is_unknown(grid)

    start = common.to_cell(*start_xy)
    goal = common.to_cell(*goal_xy)
    carved = None

    # 로봇이 이미 팽창 영역 안에 있을 수 있다 (벽에 바짝 붙은 경우).
    # 출발점 한 칸만 뚫어 봐야 사방이 막혀 있어 여전히 못 나간다.
    # 그래서 로봇 주변 팽창 반경만큼을 "진짜 벽이 아닌 한" 풀어 준다.
    # 근거: 로봇이 지금 거기 있다는 것은 거기가 벽이 아니라는 뜻이다.
    if common.in_bounds(*start) and blocked[start]:
        opened = _carve_escape(blocked, mapping.is_occupied(grid), start)
        carved = blocked & ~opened      # 뚫어 준 칸 — 지나갈 수는 있지만 아주 비싸다
        blocked = opened

    if common.in_bounds(*goal) and blocked[goal]:
        if exact:
            return None
        replacement = nearest_free(blocked, goal)
        if replacement is None:
            return None
        goal = replacement

    cost = cost_layer(grid, carved, people)
    path = astar(blocked, cost, start, goal)
    if path is None:
        return None

    waypoints = simplify(path, blocked, cost)
    return [common.to_world(row, col) for row, col in waypoints]


def _carve_escape(blocked, really_occupied, start):
    """start 주변에서 "팽창 때문에 막힌" 칸만 풀어 준다. 진짜 벽은 그대로 둔다."""
    radius_cells = common.to_cells(
        config.ROBOT_RADIUS + config.PLANNER_INFLATION_MARGIN) + 1
    height, width = blocked.shape
    opened = blocked.copy()
    for dr, dc in _disk_offsets(radius_cells):
        r, c = start[0] + dr, start[1] + dc
        if 0 <= r < height and 0 <= c < width and not really_occupied[r, c]:
            opened[r, c] = False
    return opened


def nearest_free(blocked, cell, max_radius_cells=None):
    """cell 주변에서 막히지 않은 가장 가까운 칸을 찾는다. 없으면 None."""
    if max_radius_cells is None:
        max_radius_cells = common.to_cells(1.0)
    height, width = blocked.shape
    row, col = cell
    for radius in range(1, max_radius_cells + 1):
        best = None
        best_distance = math.inf
        for dr in range(-radius, radius + 1):
            for dc in range(-radius, radius + 1):
                # 껍데기만 본다 (안쪽은 이전 radius 에서 이미 봤다)
                if max(abs(dr), abs(dc)) != radius:
                    continue
                nr, nc = row + dr, col + dc
                if 0 <= nr < height and 0 <= nc < width and not blocked[nr, nc]:
                    d = dr * dr + dc * dc
                    if d < best_distance:
                        best_distance = d
                        best = (nr, nc)
        if best is not None:
            return best
    return None


def path_is_blocked(waypoints, grid, margin=None):
    """이미 세운 경로 위에 새 장애물이 생겼는가.

    매 틱 A* 를 다시 돌리지 않기 위해 이것만 확인한다 (성능 규칙 5).

    ⚠️ 경로의 첫 점은 로봇 자신의 위치다. 로봇이 벽에 바짝 붙어 있으면 그 점이
       팽창 영역 안이라, 그대로 검사하면 "막혔다" 가 매 틱 나온다.
       그러면 매 틱 A* 를 다시 돌려(8 ms) 경로가 계속 갈아엎힌다.
       그래서 앞쪽의 "이미 팽창 안인" 구간은 건너뛰고 그 뒤부터 본다.
    """
    if not waypoints or len(waypoints) < 2:
        return False

    blocked = inflate(grid, margin)
    corners = [common.to_cell(x, y) for x, y in waypoints]

    # 경로를 칸 단위로 펼친다 (웨이포인트 단위로 건너뛰면 2점 경로에서 검사할 게 없다)
    cells = []
    for a, b in zip(corners, corners[1:]):
        if not common.in_bounds(*a) or not common.in_bounds(*b):
            return True
        rows, cols = mapping.bresenham(a[0], a[1], b[0], b[1])
        cells.extend(zip(rows.tolist(), cols.tolist()))
    if not cells:
        return False

    # 로봇이 서 있는 자리의 "이미 막힌" 칸들은 건너뛴다 (거기서 빠져나오는 중이다).
    # ⚠️ 건너뛰는 범위를 로봇 자기 발자국(팽창 반경)으로 제한해야 한다.
    #    제한 없이 건너뛰었더니 경로 앞의 장애물까지 건너뛰어 버려서
    #    "막혔다" 가 영영 안 나왔다.
    skip_limit = common.to_cells(config.ROBOT_RADIUS
                                 + config.PLANNER_INFLATION_MARGIN)
    index = 0
    while index < min(len(cells), skip_limit) and blocked[cells[index]]:
        index += 1

    return any(blocked[cell] for cell in cells[index:])
