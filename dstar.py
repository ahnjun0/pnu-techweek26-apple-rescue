"""D* Lite — 지도가 조금 바뀌었을 때 처음부터 다시 풀지 않는 경로 계획.

받는 것: 막힌 칸 마스크, 비용 배열, 출발/목표 칸.
내놓는 것: A* 와 같은 칸 목록 경로.
핵심 아이디어: 목표에서 거꾸로 푼 결과를 들고 있다가, 바뀐 칸 주변만 고쳐 쓴다.
언제 이득인가: 지도가 크고 재계획이 잦을 때. 우리 경우에는 A* 가 이미 8 ms 라
              이득이 거의 없다 — 그래도 대회 맵이 훨씬 클 수 있어 대안으로 둔다.
순수 numpy/heapq — Webots 없이 pytest 로 돈다.
"""

import heapq
import math

import config

# 8방향 이동 (planner 와 같은 규칙을 쓴다)
_MOVES = [
    (-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
    (-1, -1, math.sqrt(2)), (-1, 1, math.sqrt(2)),
    (1, -1, math.sqrt(2)), (1, 1, math.sqrt(2)),
]


class DStarLite:
    """한 목표에 대한 계획을 들고 있다가, 지도가 바뀌면 그 부분만 고친다.

    쓰는 법:
        planner = DStarLite(blocked, cost, goal)
        path = planner.plan(start)          # 처음
        planner.update(new_blocked, new_cost, changed_cells)
        path = planner.plan(new_start)      # 바뀐 부분만 다시 푼다
    """

    def __init__(self, blocked, cost, goal):
        self.blocked = blocked
        self.cost = cost
        self.goal = goal
        self.height, self.width = blocked.shape

        # g: 지금까지 알아낸 "목표까지의 비용", rhs: 이웃을 보고 다시 센 값.
        # 둘이 다르면 그 칸은 아직 정리가 안 된 것이다 (inconsistent).
        self.g = {}
        self.rhs = {}
        self.queue = []
        # 칸마다 "지금 유효한 항목의 일련번호". heapq 는 삭제가 없어서,
        # 낡은 항목이 튀어나오면 번호가 달라 건너뛴다.
        # ⚠️ 번호 없이 키만으로 비교했더니, 같은 키로 두 번 들어간 항목이
        #    둘 다 유효하게 처리되면서 g 를 inf 로 되돌리고 무한 반복했다
        #    (16개 칸이 6만 번 처리됐다).
        self.entries = {}
        self._serial = 0
        self.km = 0.0                 # 로봇이 움직인 만큼 쌓이는 보정값
        self.last_start = None
        self.expansions = 0           # 몇 칸이나 다시 봤는지 (A* 와 비교용)

        self.rhs[goal] = 0.0
        self._push(goal)

    # ------------------------------------------------------------------

    def _get_g(self, cell):
        return self.g.get(cell, math.inf)

    def _get_rhs(self, cell):
        return self.rhs.get(cell, math.inf)

    def _heuristic(self, a, b):
        """옥타일 거리. A* 와 같은 것을 쓴다."""
        dr = abs(a[0] - b[0])
        dc = abs(a[1] - b[1])
        return (dr + dc) + (math.sqrt(2) - 2.0) * min(dr, dc)

    def _key(self, cell):
        best = min(self._get_g(cell), self._get_rhs(cell))
        start = self.last_start if self.last_start else cell
        return (best + self._heuristic(start, cell) + self.km, best)

    def _push(self, cell):
        self._serial += 1
        self.entries[cell] = self._serial
        heapq.heappush(self.queue, (self._key(cell), self._serial, cell))

    def _neighbours(self, cell):
        row, col = cell
        for dr, dc, step in _MOVES:
            nr, nc = row + dr, col + dc
            if not (0 <= nr < self.height and 0 <= nc < self.width):
                continue
            if self.blocked[nr, nc]:
                continue
            # 대각선은 양옆이 둘 다 열려 있어야 한다 (planner 와 같은 규칙)
            if dr != 0 and dc != 0:
                if self.blocked[row + dr, col] or self.blocked[row, col + dc]:
                    continue
            yield (nr, nc), step

    def _edge_cost(self, a, b, step):
        """a 에서 b 로 가는 비용. 밟는 칸의 비용 배수를 곱한다."""
        return step * float(self.cost[b[0], b[1]])

    def _update(self, cell):
        if cell != self.goal:
            best = math.inf
            for neighbour, step in self._neighbours(cell):
                best = min(best, self._get_g(neighbour)
                           + self._edge_cost(cell, neighbour, step))
            self.rhs[cell] = best
        if self._get_g(cell) != self._get_rhs(cell):
            self._push(cell)
        else:
            # ⚠️ 정리가 끝난 칸은 큐에서 "빼야" 한다.
            # heapq 는 삭제가 없으니, 유효한 항목 표(entries)에서 지워서
            # 나중에 튀어나오면 무시되게 한다.
            # 이걸 빼먹었더니 정리된 칸의 낡은 항목이 다시 처리되어
            # g 를 inf 로 되돌리고, 그게 다시 큐에 들어가며 무한 반복이 됐다.
            self.entries.pop(cell, None)

    def _compute(self, start, max_expansions):
        """큐가 정리될 때까지 (또는 상한까지) 푼다."""
        while self.queue:
            key, serial, cell = self.queue[0]
            if not (key < self._key(start)
                    or self._get_rhs(start) != self._get_g(start)):
                break
            heapq.heappop(self.queue)
            if self.entries.get(cell) != serial:
                continue            # 오래된 항목이다
            if self._get_g(cell) == self._get_rhs(cell):
                # 이미 정리된 칸은 건드리지 않는다 (건드리면 g 가 inf 로 되돌아간다)
                self.entries.pop(cell, None)
                continue
            self.expansions += 1
            if self.expansions > max_expansions:
                return False

            new_key = self._key(cell)
            if key < new_key:
                self._push(cell)
            elif self._get_g(cell) > self._get_rhs(cell):
                self.g[cell] = self._get_rhs(cell)
                for neighbour, _ in self._neighbours(cell):
                    self._update(neighbour)
            else:
                self.g[cell] = math.inf
                self._update(cell)
                for neighbour, _ in self._neighbours(cell):
                    self._update(neighbour)
        return True

    # ------------------------------------------------------------------

    def plan(self, start, max_expansions=None):
        """start 에서 목표까지의 경로(칸 목록). 없으면 None."""
        max_expansions = (config.PLANNER_MAX_NODES if max_expansions is None
                          else max_expansions)
        if self.blocked[start] or self.blocked[self.goal]:
            return None

        if self.last_start is None:
            self.last_start = start
        else:
            # 로봇이 움직인 만큼 키가 어긋나므로 km 로 보정한다 (D* Lite 의 핵심)
            self.km += self._heuristic(self.last_start, start)
            self.last_start = start

        self.expansions = 0
        if not self._compute(start, max_expansions):
            return None
        if self._get_g(start) == math.inf and self._get_rhs(start) == math.inf:
            return None

        # 목표 쪽으로 제일 싼 이웃을 따라 내려간다
        path = [start]
        cell = start
        limit = self.height * self.width
        while cell != self.goal and len(path) < limit:
            best = None
            best_cost = math.inf
            for neighbour, step in self._neighbours(cell):
                value = self._edge_cost(cell, neighbour, step) \
                    + self._get_g(neighbour)
                if value < best_cost:
                    best_cost = value
                    best = neighbour
            if best is None or not math.isfinite(best_cost):
                return None
            cell = best
            path.append(cell)
        return path if cell == self.goal else None

    def update(self, blocked, cost, changed):
        """지도가 바뀌었다고 알려 준다. changed 는 바뀐 칸 목록.

        바뀐 칸과 그 이웃만 다시 정리한다 — 이게 D* Lite 를 쓰는 이유다.
        """
        self.blocked = blocked
        self.cost = cost
        for cell in changed:
            self._update(cell)
            for neighbour, _ in self._neighbours(cell):
                self._update(neighbour)
