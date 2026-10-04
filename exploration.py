""""아직 안 가 본 곳" 을 찾아 다음에 갈 목표를 고른다 (프론티어 탐색).

받는 것: log-odds 지도와 로봇 위치.
내놓는 것: 다음에 갈 월드 좌표 (x, y), 또는 더 볼 곳이 없으면 None.
핵심 아이디어: "빈 칸인데 바로 옆이 모르는 칸" 인 곳이 경계(프론티어)다.
그 경계를 덩어리로 묶고, 가깝고 큰 덩어리를 고른다. 실패한 곳은 블랙리스트에 넣는다.
순수 numpy — Webots 없이 pytest 로 돈다.
"""

from collections import deque

import numpy as np

import common
import config
import mapping
import planner


def frontier_mask(grid, reachable_only=True):
    """"빈 칸인데 상하좌우 중 하나가 모르는 칸" 인 칸들의 bool 마스크.

    막힌 칸 옆의 모르는 칸은 프론티어가 아니다 (벽 뒤라 갈 수가 없다).

    reachable_only=True 면 팽창 영역(로봇이 설 수 없는 곳) 안의 칸도 뺀다.
    왜 필요한가: 벽이 한두 칸 덜 찍혀서 벽 속에 "모르는 칸" 이 남으면, 그 옆
    빈 칸이 프론티어가 된다. 로봇은 절대 거기 못 가는데 계속 가려고 해서
    탐색이 안 끝난다. 실제로 이것 때문에 지도를 다 그리고도 DONE 이 안 됐다.
    """
    free = mapping.is_free(grid)
    unknown = mapping.is_unknown(grid)

    # 네 방향으로 한 칸씩 밀어서 "옆이 모르는 칸인가" 를 한 번에 본다.
    neighbour_unknown = np.zeros_like(unknown)
    neighbour_unknown[:-1, :] |= unknown[1:, :]     # 위쪽 이웃
    neighbour_unknown[1:, :] |= unknown[:-1, :]     # 아래쪽 이웃
    neighbour_unknown[:, :-1] |= unknown[:, 1:]     # 오른쪽 이웃
    neighbour_unknown[:, 1:] |= unknown[:, :-1]     # 왼쪽 이웃

    mask = free & neighbour_unknown
    if reachable_only:
        # 로봇 몸이 들어갈 수 없는 칸은 목표로 삼아 봐야 소용없다.
        # ⚠️ 판정에는 "좁은 여유" 를 쓴다. 평소 여유로 재면, 정작 폴백으로는
        #    갈 수 있는 곳까지 미리 버려서 탐색이 일찍 끝나 버린다.
        #    (경로 계획도 "평소 여유로 안 되면 좁게" 순서로 시도한다.)
        # ⚠️ 여기서 걸러낼 것은 "장애물 **안** 의 프론티어" 뿐이다. 계획용 여유로
        #    지우면 벽 근처 프론티어가 전멸한다 (config 의 실측 근거 참고).
        mask = mask & ~planner.inflate(grid,
                                       margin=config.FRONTIER_REACHABLE_MARGIN)
    return mask


def cluster(mask, min_size=None):
    """붙어 있는 프론티어 칸들을 덩어리로 묶는다 (8방향 연결).

    돌려주는 것: [(중심 row, 중심 col, 칸 수), ...] 를 큰 것부터 정렬한 목록.
    작은 덩어리는 잡음(지도 가장자리의 한두 칸)이라 버린다.
    """
    min_size = config.FRONTIER_MIN_CLUSTER if min_size is None else min_size

    seen = np.zeros_like(mask)
    clusters = []
    rows, cols = np.nonzero(mask)
    height, width = mask.shape

    for start_row, start_col in zip(rows, cols):
        if seen[start_row, start_col]:
            continue
        queue = deque([(int(start_row), int(start_col))])
        seen[start_row, start_col] = True
        members = []
        while queue:
            r, c = queue.popleft()
            members.append((r, c))
            for dr in (-1, 0, 1):
                for dc in (-1, 0, 1):
                    nr, nc = r + dr, c + dc
                    if (0 <= nr < height and 0 <= nc < width
                            and mask[nr, nc] and not seen[nr, nc]):
                        seen[nr, nc] = True
                        queue.append((nr, nc))

        if len(members) >= min_size:
            member_rows = [m[0] for m in members]
            member_cols = [m[1] for m in members]
            centre_row = int(round(sum(member_rows) / len(members)))
            centre_col = int(round(sum(member_cols) / len(members)))
            # 중심이 덩어리 밖으로 튈 수 있다 (ㄱ 자 모양). 가장 가까운 멤버로 당긴다.
            if (centre_row, centre_col) not in set(members):
                centre_row, centre_col = min(
                    members,
                    key=lambda m: (m[0] - centre_row) ** 2 + (m[1] - centre_col) ** 2)
            clusters.append((centre_row, centre_col, len(members)))

    clusters.sort(key=lambda item: item[2], reverse=True)
    return clusters


class Blacklist:
    """가 보려다 실패한 목표들을 기억한다. 같은 곳을 무한히 다시 고르지 않게.

    "몇 번 실패했는가" 를 세어, 정해진 횟수를 넘으면 그 근처를 아예 제외한다.
    """

    def __init__(self, radius=None, max_failures=None):
        self.radius = (config.FRONTIER_BLACKLIST_RADIUS if radius is None
                       else radius)
        self.max_failures = (config.FRONTIER_MAX_FAILURES if max_failures is None
                             else max_failures)
        self.banned = []      # [(x, y), ...] 완전히 제외된 곳
        self._failures = {}   # 반올림한 좌표 -> 실패 횟수

    @staticmethod
    def _key(x, y):
        return (round(x, 1), round(y, 1))

    def record_failure(self, x, y, immediate=False):
        """한 번 실패했다고 기록한다. 한도를 넘으면 블랙리스트에 넣는다.

        immediate=True 면 한 번으로 바로 제외한다.
        ⚠️ "몇 번 봐 준다" 는 사람이 잠깐 막고 선 경우를 위한 것이다. 사람은
           2~3 초면 지나간다. 그러니 "오래 매달렸는데 못 갔다" 처럼 이미
           결정적인 증거에는 봐 줄 이유가 없다 — 봐 줬더니 닿을 수 없는
           프론티어 하나에 3 x 45 = 135 초를 썼다.

        돌려주는 값: 이번에 블랙리스트에 들어갔으면 True.
        """
        key = self._key(x, y)
        self._failures[key] = self._failures.get(key, 0) + 1
        if immediate or self._failures[key] >= self.max_failures:
            self.banned.append((x, y))
            return True
        return False

    def failures(self, x, y):
        return self._failures.get(self._key(x, y), 0)

    def contains(self, x, y):
        """이 좌표가 블랙리스트 반경 안에 있는가."""
        return any(common.distance(x, y, bx, by) <= self.radius
                   for bx, by in self.banned)

    def clear(self):
        self.banned.clear()
        self._failures.clear()


def choose(grid, robot_xy, blacklist=None, min_distance=None):
    """다음에 갈 곳을 고른다. 월드 좌표 (x, y) 또는 더 볼 곳이 없으면 None.

    **갈 수 있는 프론티어 중 경로가 가장 짧은 곳.** 규칙은 이 한 줄이다.

    ⚠️ 한때 여기에 점수 층이 넷 쌓여 있었다 — 순회 풀기(후보 3개), 정보 이득
       비율, 회전량을 거리로 환산한 항, 카메라 미관측 폴백. 그리고 "엄격한 통과 →
       안 되면 느슨한 통과" 두 갈래였다. 그 층들을 얹을 때마다 크기를 맞추는 일이
       새 버그를 만들었다 (이득 가중치가 거리를 삼켰고, 회전 벌점이 비율을
       삼켰다 — docs/무엇을-빼기로-했나.md).
       거리를 **A* 경로 길이** 로 재면 "갈 수 있나" 와 "얼마나 가까운가" 가 한
       가지가 된다. 두 갈래도 필요 없다: 프론티어를 설 수 있는 자리로 옮긴 뒤
       A* 로 물으면, 느슨한 통과가 하던 일이 주 경로에서 그냥 일어난다.
    """
    candidates = candidate_list(grid, robot_xy, blacklist, min_distance)
    return candidates[0][0] if candidates else None


# ⚠️ 여기에 "아무것도 안 남으면 필터를 풀고 갈 수 있는 것 중 가장 가까운 것을
#    고른다" 는 마지막 수단을 넣어 봤다가 **기각했다.**
#    만든 목적(시드 222: 갈 수 있는 프론티어가 있는데 필터에 걸려 한 발도 못
#    움직인다)을 **달성하지 못했고**, 처음 보는 지도 18개 성적도 50/54 로 같았다.
#    대신 16월드에서 시간이 18% 늘고 comb0 이 284 -> 809초로 터졌다
#    (발밑 프론티어를 반복해서 고르는, 옛 시험이 경고하던 모양이다).
#    16월드 목표물이 47 -> 48 로 늘었지만, 이 세션 내내 46~48 을 오가며 실패
#    월드만 바뀌던 범위 안이라 근거로 삼기 약하다.
#    시드 222 의 원인은 확정돼 있다 — 로봇 지도에서 복도가 x=-1.3~-1.0 구간에서
#    막혀 보이고(한 자리에서만 스캔한 탓), 갈 수 있는 유일한 프론티어가
#    [너무가까움] + [신선도초과] 에 걸린다. 고치는 방법은 아직 못 찾았다.


def _worth_driving_to():
    """목표로 삼을 만한 최소 거리 [m].

    ⚠️ FRONTIER_MIN_DISTANCE(0.25 m)는 도착 허용치 FOLLOW_GOAL_TOLERANCE(0.20 m)
       보다 겨우 5 cm 크다. 그런 목표를 내주면 로봇은 5 cm 만 가고 곧바로 "도착"
       판정을 받아 목표가 지워지고, 그 틱은 출력이 (0,0) 이 된다. 이것이 매 틱
       반복되면 로봇이 제자리에서 멈췄다 골랐다만 한다.
       실측: maze 지도에서 완전 정지 182초(20%) + 제자리 회전 229초(25%).
       그래서 "가 볼 가치가 있는" 거리는 도착 허용치의 몇 배여야 한다.

    참고: github.com/Leety09/autonomous-frontier-explorer 도 0.8 m 미만 후보를
    버린다 ("Filter extremely close noise").
    """
    return max(config.FRONTIER_MIN_DISTANCE,
               config.FRONTIER_WORTH_DRIVING * config.FOLLOW_GOAL_TOLERANCE)


def candidate_list(grid, robot_xy, blacklist=None, min_distance=None,
                   stale_radius=None):
    """갈 수 있는 프론티어들을 **경로가 짧은 순서** 로. [((x, y), 경로길이, 칸수), ...]

    choose 가 쓰는 속이다. 화면에 후보를 다 그려 보고 싶을 때도 쓴다.
    점수는 A* 경로 길이 하나이므로 단위가 m 이고, 가중치가 없다.
    """
    # 기본값은 "가 볼 가치가 있는 거리" 지만, 명시로 넘긴 값은 그대로 쓴다
    # (시험에서 0 을 넘겨 순위만 보고 싶을 때가 있다).
    min_distance = _worth_driving_to() if min_distance is None else min_distance

    # ⚠️ 프론티어 칸을 그대로 목표로 내주면 안 된다. 프론티어는 벽 근처에 생기므로
    #    그 자리는 로봇이 **설 수 없는** 곳일 수 있다. 그러면 계획기가 목표를 몰래
    #    바꿔 "도착" 을 내고, 임무는 그것을 실패로 처리해 같은 목표를 다시 고른다
    #    (실측 comb0: 최근접 0.21 m 에서 후진과 제자리 회전만 반복).
    #    → 프론티어는 "갈 이유" 로만 쓰고, **목표는 설 수 있는 자리로 옮긴다.**
    # ⚠️ 막힘의 정의는 planner.plan 과 **같아야** 한다 (planner.py:280-282):
    #    팽창 + **미지**. 미지를 빼먹으면 nearest_free 가 설 자리를 미지 칸으로
    #    잡는데, 프론티어는 정의상 미지 칸에 붙어 있으므로 이 일이 실제로 일어난다.
    #    그러면 plan(exact=True) 이 그 자리로 길을 못 찾아 "길이 없다" 로 버려진다.
    #    실측(comb0): 덩어리 4개가 전부 "길이 없다" 로 거부됐는데, 미지를 넣자
    #    둘은 진짜 설 자리(0.55·0.74 m 밖)가 드러나며 원인이 신선도 반경으로
    #    바뀌었다 — 버그 두 겹이 서로를 가리고 있었다.
    blocked = (planner.inflate(grid, config.PLANNER_INFLATION_MARGIN)
               | mapping.is_unknown(grid))

    scored = []
    for row, col, size in cluster(frontier_mask(grid)):
        cell = (row, col)
        if not common.in_bounds(*cell):
            continue
        spot = common.to_world(row, col)
        if blocked[cell]:
            cell = planner.nearest_free(blocked, cell)
            if cell is None:
                continue
        x, y = common.to_world(*cell)
        # 옮긴 자리가 프론티어에서 너무 멀면 "거기 가려던 이유" 가 사라진다
        # (frontier_near 가 거짓이 되어 다음 틱에 목표가 곧바로 버려진다).
        stale = (config.FRONTIER_STALE_RADIUS if stale_radius is None
                 else stale_radius)
        if common.distance(x, y, *spot) > stale:
            continue
        if common.distance(x, y, *robot_xy) < min_distance:
            continue
        if blacklist is not None and blacklist.contains(x, y):
            continue
        # ⚠️ exact=True 로 물어야 한다. plan() 은 목표가 막혀 있으면 근처 칸으로
        #    **목표를 몰래 바꿔** 길을 준다 — 그러면 통과 못 할 것이 없는 필터가
        #    되어 못 가는 구석을 영원히 고른다 (실측: 한자리에 800초).
        path = planner.plan(grid, robot_xy, (x, y), exact=True)
        if not path:
            # ⚠️ 평소 여유로 안 되면 **좁은 여유로 한 번 더** 묻는다.
            #    연습 16월드로 두 번 재고 기각했었다 (거리·시간 +20%). 그런데 그 지도들에는
            #    **좁은 문이 없었다.** 대회형 apartment 첫 완주에서 231.9초에 탐색을
            #    포기했는데, 남은 프론티어 덩어리 17개 중 **13개가 "평소=막힘, 좁게=통과"**
            #    였다 (아파트 문 폭). 처음 보는 무작위 지도 시드 211 도 이걸로 2/3 -> 3/3.
            #    ⚠️ 목표 지점은 평소 여유로 고른 자리 그대로다 — 좁게 묻는 것은 "길이 있나" 뿐.
            #    그리고 _replan 도 같은 규칙을 써야 한다 (한쪽만 넣었다가 maze0 이 3/3 -> 2/3).
            path = planner.plan(grid, robot_xy, (x, y), exact=True,
                                margin=config.PLANNER_SQUEEZE_MARGIN)
        if not path:
            continue
        # ⚠️ 첫 웨이포인트는 로봇이 **있는 칸의 중심** 이라 로봇 위치와 다르다.
        #    그 토막을 빼먹으면 점수가 직선거리보다 짧아질 수 있다 (최대 한 칸).
        #    순위는 안 바뀌지만 "점수 = 로봇에서 후보까지 거리" 라는 정의가 깨진다.
        length = (common.distance(*robot_xy, *path[0])
                  + sum(common.distance(*a, *b) for a, b in zip(path, path[1:])))
        scored.append(((x, y), length, size))

    # 큰 경계(새 방 입구)를 가까운 작은 경계들 뒤로 밀지 않게 — 크기만큼 거리를 깎는다 (끔이 기본).
    # ⚠️ 대회 조건 실행: 서쪽 방 입구 앞 131칸 경계를 끝까지 안 가고 작은 경계만 오가다 시간이 끝났다.
    bonus = config.FRONTIER_SIZE_BONUS if config.FRONTIER_USE_SIZE else 0.0
    scored.sort(key=lambda item: item[1] - bonus * item[2])
    return scored


def frontier_near(grid, x, y, radius=None):
    """(x, y) 로부터 radius 안에 아직 프론티어 칸이 남아 있는가.

    "거기 가려던 이유가 아직 유효한가" 를 확인할 때 쓴다.
    이미 다 봐 버린 곳으로 계속 기어가지 않도록.
    """
    radius = config.FRONTIER_STALE_RADIUS if radius is None else radius
    mask = frontier_mask(grid)
    row, col = common.to_cell(x, y)
    span = common.to_cells(radius)
    r0 = max(0, row - span)
    c0 = max(0, col - span)
    patch = mask[r0:row + span + 1, c0:col + span + 1]
    return bool(patch.any())


def frontier_points(grid):
    """화면 표시용 — 모든 프론티어 칸의 월드 좌표 목록."""
    rows, cols = np.nonzero(frontier_mask(grid))
    return [common.to_world(int(r), int(c)) for r, c in zip(rows, cols)]
