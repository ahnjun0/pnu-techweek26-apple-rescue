"""프론티어 찾기·고르기·블랙리스트가 맞는지 확인한다. Webots 없이 돈다."""

import numpy as np
import pytest

from sar import common
from sar import config
from sar import exploration
from sar import mapping
from sar import planner


def map_with_free_box(row0, row1, col0, col1):
    """가운데에 "다 둘러본 빈 방" 하나만 있는 지도. 나머지는 전부 모름."""
    grid = mapping.new_map()
    grid[row0:row1, col0:col1] = config.LOG_ODDS_MIN
    return grid


# --- 프론티어 찾기 ------------------------------------------------------------

def test_free_cell_next_to_unknown_is_a_frontier():
    grid = mapping.new_map()
    grid[80, 80] = config.LOG_ODDS_MIN      # 빈 칸 하나, 주위는 전부 모름
    assert exploration.frontier_mask(grid)[80, 80]


def test_fully_surrounded_free_cell_is_not_a_frontier():
    grid = mapping.new_map()
    grid[79:82, 79:82] = config.LOG_ODDS_MIN
    assert not exploration.frontier_mask(grid)[80, 80], \
        "사방이 다 빈 칸이면 더 볼 게 없다"


def test_unknown_cell_is_never_a_frontier():
    grid = mapping.new_map()
    grid[80, 80] = config.LOG_ODDS_MIN
    assert not exploration.frontier_mask(grid)[80, 81]


def test_occupied_cell_is_never_a_frontier():
    grid = mapping.new_map()
    grid[80, 80] = config.LOG_ODDS_MAX
    assert not exploration.frontier_mask(grid)[80, 80]


def test_free_cell_walled_off_from_unknown_is_not_a_frontier():
    """벽 뒤의 모르는 공간은 프론티어가 아니다. 거기 가 봐야 소용없다."""
    grid = mapping.new_map()
    grid[79:82, 79:82] = config.LOG_ODDS_MIN
    grid[78, 79:82] = config.LOG_ODDS_MAX
    grid[82, 79:82] = config.LOG_ODDS_MAX
    grid[79:82, 78] = config.LOG_ODDS_MAX
    grid[79:82, 82] = config.LOG_ODDS_MAX
    assert not exploration.frontier_mask(grid)[80, 80]


def test_empty_map_has_no_frontier():
    assert not exploration.frontier_mask(mapping.new_map()).any(), \
        "아무것도 모르면 프론티어도 없다 (출발 전 상태)"


def test_frontier_appears_on_the_edge_of_an_explored_area():
    grid = map_with_free_box(70, 90, 70, 90)
    mask = exploration.frontier_mask(grid)
    assert mask[70, 75] and mask[89, 75], "빈 방의 테두리가 프론티어다"
    assert not mask[80, 80], "방 한가운데는 프론티어가 아니다"


# --- 덩어리 묶기 --------------------------------------------------------------

def test_cluster_finds_one_ring_for_one_room():
    clusters = exploration.cluster(exploration.frontier_mask(
        map_with_free_box(70, 90, 70, 90)))
    assert len(clusters) == 1
    assert clusters[0][2] > 20


def test_cluster_separates_two_distant_rooms():
    grid = mapping.new_map()
    grid[30:40, 30:40] = config.LOG_ODDS_MIN
    grid[110:120, 110:120] = config.LOG_ODDS_MIN
    assert len(exploration.cluster(exploration.frontier_mask(grid))) == 2


def test_cluster_drops_noise_that_is_too_small():
    grid = mapping.new_map()
    grid[80, 80] = config.LOG_ODDS_MIN       # 빈 칸 하나 = 프론티어 1칸
    assert exploration.cluster(exploration.frontier_mask(grid),
                               min_size=4) == []
    assert len(exploration.cluster(exploration.frontier_mask(grid),
                                   min_size=1)) == 1


def test_cluster_is_sorted_biggest_first():
    grid = mapping.new_map()
    grid[30:34, 30:40] = config.LOG_ODDS_MIN     # 작은 방
    grid[100:120, 100:130] = config.LOG_ODDS_MIN  # 큰 방
    clusters = exploration.cluster(exploration.frontier_mask(grid))
    assert len(clusters) == 2
    assert clusters[0][2] > clusters[1][2]


def test_cluster_centre_is_a_real_frontier_cell():
    """ㄱ 자 모양이면 무게중심이 덩어리 밖으로 튄다. 실제 칸으로 당겨야 한다."""
    grid = map_with_free_box(70, 90, 70, 90)
    mask = exploration.frontier_mask(grid)
    for row, col, _ in exploration.cluster(mask):
        assert mask[row, col], "고른 지점이 프론티어가 아니면 거기 가 봐야 소용없다"


# --- 고르기 -------------------------------------------------------------------

def test_choose_returns_none_when_nothing_is_left():
    assert exploration.choose(mapping.new_map(), (0.0, 0.0)) is None


def test_choose_returns_a_point_inside_the_map():
    grid = map_with_free_box(70, 90, 70, 90)
    goal = exploration.choose(grid, (0.0, 0.0))
    assert goal is not None
    assert common.in_bounds(*common.to_cell(*goal))


def test_choose_prefers_the_closer_of_two_equal_clusters():
    grid = mapping.new_map()
    grid[78:83, 78:83] = config.LOG_ODDS_MIN        # 원점 근처
    grid[20:25, 20:25] = config.LOG_ODDS_MIN        # 멀리
    goal = exploration.choose(grid, (0.0, 0.0), min_distance=0.0)
    assert common.distance(0.0, 0.0, *goal) < 1.0

def test_choose_ignores_frontiers_right_under_the_robot():
    """발밑 프론티어는 **다른 후보가 있으면** 고르지 않는다.

    ⚠️ 바로 발밑을 목표로 삼으면 5 cm 만 가고 "도착" 판정을 받아 목표가 지워지고,
       그 틱 출력이 (0,0) 이 된다. 매 틱 반복되면 제자리에서 멈췄다 골랐다만 한다
       (실측: maze 지도에서 완전 정지 182초(20%) + 제자리 회전 229초(25%)).
    """
    grid = mapping.new_map()
    here = common.to_cell(0.0, 0.0)
    # 발밑에 작은 방, 그리고 멀리 큰 방 — 둘을 넓은 복도로 잇는다
    grid[here[0] - 3:here[0] + 3, here[1] - 3:here[1] + 3] = config.LOG_ODDS_MIN
    grid[here[0] - 40:here[0] - 20, here[1] - 10:here[1] + 10] = config.LOG_ODDS_MIN
    grid[here[0] - 20:here[0] - 3, here[1] - 9:here[1] + 9] = config.LOG_ODDS_MIN

    robot = common.to_world(*here)
    goal = exploration.choose(grid, robot)
    assert goal is not None, "갈 수 있는 프론티어가 있는데 아무것도 안 골랐다"
    assert common.distance(*goal, *robot) >= exploration._worth_driving_to(), (
        f"발밑({common.distance(*goal, *robot):.2f} m)을 골랐다 — 멀리 갈 수 있는데도")


# ⚠️ 여기 있던 test_choose_takes_a_close_frontier_when_it_is_the_only_one 을
#    지웠다. 그 규칙(마지막 수단)을 기각했기 때문이다 — 목적이던 시드 222 를
#    못 고쳤고 16월드 시간이 18% 늘었다 (exploration.choose 의 주석 참고).

def test_choose_skips_a_blacklisted_frontier():
    # ⚠️ 두 번째 후보는 **갈 수 있는 곳** 이어야 한다. 예전에는 후보를 낼 때
    #    도달성을 아예 안 봤으므로 로봇과 이어지지 않은 외딴 빈 칸도 후보였다.
    #    지금은 A* 경로 길이가 점수라 길이 없으면 후보가 아니다.
    #    그래서 방 둘을 **벽으로 감싼 복도** 로 잇는다. 그냥 빈 복도로 이으면
    #    프론티어 테두리가 한 덩어리로 합쳐져 후보가 하나만 남는다 —
    #    이 시험이 보려는 것은 블랙리스트이지 도달성이나 덩어리 나누기가 아니다.
    grid = mapping.new_map()
    #    ⚠️ 복도는 넓어야 한다. 좁게 두면 팽창이 벽에서 안쪽으로 몇 칸씩 먹어
    #       복도가 통째로 막히고, 그러면 두 번째 방에 길이 없다.
    grid[70:90, 70:90] = config.LOG_ODDS_MIN        # 로봇이 있는 방
    grid[20:35, 70:90] = config.LOG_ODDS_MIN        # 멀리 있는 방
    grid[35:70, 71:89] = config.LOG_ODDS_MIN        # 잇는 복도 (넓게)
    grid[35:70, 70] = config.LOG_ODDS_MAX           # 복도 벽 (왼쪽)
    grid[35:70, 89] = config.LOG_ODDS_MAX           # 복도 벽 (오른쪽)

    first = exploration.choose(grid, (0.0, 0.0), min_distance=0.0)

    blacklist = exploration.Blacklist()
    blacklist.banned.append(first)
    second = exploration.choose(grid, (0.0, 0.0), blacklist, min_distance=0.0)

    assert second is not None
    assert common.distance(*first, *second) > blacklist.radius


def test_choose_returns_none_when_everything_is_blacklisted():
    grid = map_with_free_box(70, 90, 70, 90)
    blacklist = exploration.Blacklist(radius=99.0)
    blacklist.banned.append((0.0, 0.0))
    assert exploration.choose(grid, (3.0, 3.0), blacklist) is None


# --- 블랙리스트 ---------------------------------------------------------------

def test_blacklist_needs_repeated_failures():
    blacklist = exploration.Blacklist(max_failures=3)
    assert blacklist.record_failure(1.0, 1.0) is False
    assert blacklist.record_failure(1.0, 1.0) is False
    assert blacklist.record_failure(1.0, 1.0) is True
    assert blacklist.contains(1.0, 1.0)


def test_blacklist_does_not_ban_after_one_failure():
    """한 번 실패했다고 바로 포기하면 잠깐 사람이 지나간 곳도 영영 안 간다."""
    blacklist = exploration.Blacklist(max_failures=3)
    blacklist.record_failure(1.0, 1.0)
    assert not blacklist.contains(1.0, 1.0)


def test_blacklist_counts_nearby_failures_together():
    """정확히 같은 좌표가 아니어도 사실상 같은 목표다."""
    blacklist = exploration.Blacklist(max_failures=3)
    blacklist.record_failure(1.00, 1.00)
    blacklist.record_failure(1.01, 1.01)
    blacklist.record_failure(1.02, 0.99)
    assert blacklist.contains(1.0, 1.0)


def test_blacklist_counts_failures_within_its_radius_even_across_rounding_cells():
    """반경 안의 실패는 같은 목표다 — 0.1 m 반올림 칸이 달라도.

    ⚠️ 회귀 방지. 실패를 좌표 반올림(0.1 m) 칸별로 셌다. 대회 월드(2026-10-06) 녹화: 같은 자리의
       세 실패 (-4.47,-6.22) (-4.53,-6.28) (-4.57,-6.32) 가 서로 다른 칸으로 세어져 금지되지 않았다.
    """
    blacklist = exploration.Blacklist(max_failures=3)
    for x, y in [(-4.47, -6.22), (-4.53, -6.28), (-4.57, -6.32)]:
        blacklist.record_failure(x, y)
    assert blacklist.contains(-4.53, -6.28)


def test_blacklist_covers_a_radius_not_a_point():
    blacklist = exploration.Blacklist(radius=0.5, max_failures=1)
    blacklist.record_failure(2.0, 2.0)
    assert blacklist.contains(2.3, 2.0)
    assert not blacklist.contains(3.0, 2.0)


def test_blacklist_keeps_separate_goals_apart():
    blacklist = exploration.Blacklist(max_failures=2)
    blacklist.record_failure(1.0, 1.0)
    blacklist.record_failure(5.0, 5.0)
    assert not blacklist.contains(1.0, 1.0)
    assert not blacklist.contains(5.0, 5.0)


def test_blacklist_clear():
    blacklist = exploration.Blacklist(max_failures=1)
    blacklist.record_failure(1.0, 1.0)
    blacklist.clear()
    assert not blacklist.contains(1.0, 1.0)
    assert blacklist.failures(1.0, 1.0) == 0


# --- 탐색이 실제로 끝나는가 ----------------------------------------------------

def test_exploration_finishes_when_the_room_is_closed():
    """벽으로 완전히 둘러싸인 방을 다 보면 프론티어가 없어져야 한다.

    이게 안 되면 로봇이 영원히 멈추지 않는다.
    """
    grid = mapping.new_map()
    grid[70:90, 70:90] = config.LOG_ODDS_MIN          # 빈 방
    grid[69, 69:91] = config.LOG_ODDS_MAX             # 사방 벽
    grid[90, 69:91] = config.LOG_ODDS_MAX
    grid[69:91, 69] = config.LOG_ODDS_MAX
    grid[69:91, 90] = config.LOG_ODDS_MAX

    assert exploration.choose(grid, common.to_world(80, 80)) is None


# --- 범용성 -------------------------------------------------------------------

@pytest.mark.parametrize("resolution", [0.025, 0.05, 0.10])
def test_frontiers_work_at_other_resolutions(monkeypatch, resolution):
    cells = int(round(8.0 / resolution))
    monkeypatch.setattr(config, "MAP_RESOLUTION", resolution)
    monkeypatch.setattr(config, "MAP_WIDTH_CELLS", cells)
    monkeypatch.setattr(config, "MAP_HEIGHT_CELLS", cells)

    grid = mapping.new_map()
    quarter = cells // 4
    grid[quarter:quarter * 2, quarter:quarter * 2] = config.LOG_ODDS_MIN

    goal = exploration.choose(grid, (0.0, 0.0), min_distance=0.0)
    assert goal is not None
    assert common.in_bounds(*common.to_cell(*goal))


def test_frontiers_work_on_a_non_square_map(monkeypatch):
    monkeypatch.setattr(config, "MAP_WIDTH_CELLS", 200)
    monkeypatch.setattr(config, "MAP_HEIGHT_CELLS", 100)

    # ⚠️ 빈 칸을 **로봇이 있는 자리** 에 칠해야 한다. 예전에는 (40:60, 90:130) 에
    #    칠했는데 원점의 칸은 (80, 80) 이라 로봇이 그 방 밖이었다 — 도달성을 안
    #    보던 때라 통과했을 뿐이다.
    grid = mapping.new_map()
    here = common.to_cell(0.0, 0.0)
    grid[here[0] - 10:here[0] + 10, here[1] - 20:here[1] + 20] = config.LOG_ODDS_MIN
    assert exploration.choose(grid, (0.0, 0.0), min_distance=0.0) is not None


def test_frontier_edges_of_the_map_do_not_crash():
    """지도 가장자리에 빈 칸이 닿아도 인덱스 오류가 나면 안 된다."""
    grid = mapping.new_map()
    grid[0, :] = config.LOG_ODDS_MIN
    grid[-1, :] = config.LOG_ODDS_MIN
    grid[:, 0] = config.LOG_ODDS_MIN
    grid[:, -1] = config.LOG_ODDS_MIN
    exploration.choose(grid, (0.0, 0.0))     # 안 터지면 통과


def test_only_offers_places_we_can_reach():
    """⚠️ 회귀 방지: 판정을 풀 때도 "갈 수 있는 곳" 만 내줘야 한다.

    좁은 문 뒤의 방은 경계가 전부 벽 가까이라 평소 판정으로는 다 걸러진다.
    그렇다고 판정을 그냥 풀면 벽 바깥의 못 가는 프론티어까지 살아나서
    탐색이 영영 안 끝난다 (실측: 95% 를 다 보고도 900초 동안 100 m 를 헤맸다).
    """
    grid = mapping.new_map()
    # 사방이 막힌 작은 방 하나만 본 상태를 만든다 — 방 밖은 전부 모르는 칸이다.
    for x in np.arange(-1.0, 1.01, 0.05):
        for y in (-1.0, 1.0):
            grid[common.to_cell(x, y)] = config.LOG_ODDS_MAX
    for y in np.arange(-1.0, 1.01, 0.05):
        for x in (-1.0, 1.0):
            grid[common.to_cell(x, y)] = config.LOG_ODDS_MAX
    for x in np.arange(-0.9, 0.91, 0.05):
        for y in np.arange(-0.9, 0.91, 0.05):
            grid[common.to_cell(x, y)] = config.LOG_ODDS_MIN

    spot = exploration.choose(grid, (0.0, 0.0))
    if spot is not None:
        path = planner.plan(grid, (0.0, 0.0), spot)
        if not path:
            path = planner.plan(grid, (0.0, 0.0), spot,
                                margin=config.PLANNER_SQUEEZE_MARGIN)
        assert path, f"갈 수 없는 곳 {spot} 을 목표로 내줬다"


def test_plan_alone_is_not_a_reachability_test():
    """planner.plan() 이 길을 준다고 거기 갈 수 있는 것이 아니다.

    ⚠️ 이 성질을 몰라서 loose 폴백이 못 가는 목표를 통과시켰다.
       plan() 은 목표 칸이 막혀 있으면 1 m 안의 갈 수 있는 칸으로 **목표를
       바꿔서** 길을 준다. 이 테스트는 그 사실 자체를 못박아 둔다 — 누가
       plan() 의 참/거짓만 보고 도달 가능성을 판정하면 여기서 걸린다.
    """
    grid = mapping.new_map()
    # 빈 방 하나를 만들고 한쪽을 벽으로 막는다
    r0, c0 = common.to_cell(-2.0, -2.0)
    r1, c1 = common.to_cell(2.0, 2.0)
    grid[r0:r1, c0:c1] = -10.0
    wr0, wc0 = common.to_cell(-2.0, 1.0)
    wr1, wc1 = common.to_cell(2.0, 1.2)
    grid[wr0:wr1, wc0:wc1] = 10.0          # 가로 벽

    inside_wall = (0.0, 1.1)               # 벽 한복판 — 절대 못 선다
    blocked = planner.inflate(grid, config.PLANNER_INFLATION_MARGIN)
    assert blocked[common.to_cell(*inside_wall)], "이 좌표는 막혀 있어야 한다"

    path = planner.plan(grid, (0.0, 0.0), inside_wall)
    assert path is not None, "plan() 은 목표를 바꿔서라도 길을 준다"
    gap = common.distance(*path[-1], *inside_wall)
    assert gap > common.cells_to_metres(1), \
        "경로의 끝이 목표가 아니다 — plan() 의 참 하나로 판정하면 안 된다"
def test_frontier_near_a_wall_is_not_thrown_away():
    """벽 근처 프론티어를 버리면 안 된다 — 그 위에 설 필요가 없으니까.

    ⚠️ 회귀 방지. 도달가능 필터가 **계획용 여유**(팽창 반경 0.28 m = 6칸)로
       프론티어를 지웠다. 프론티어는 "알려진 세계의 경계" 라 벽 근처에 생기므로
       벽에서 30 cm 안쪽을 다 지우면 빗살(comb) 구조에서는 거의 전부가 사라진다.
       로봇은 프론티어를 **볼 수 있는** 곳에 서면 되고, 설 자리는 뒤에서
       nearest_free 가 정한다.

    자료는 **실제로 실패한 실행의 종료 지도** 다 (comb3, 목표물 2/3, 탐색률 93%).
    합성 격자로는 이 상황을 그대로 만들기 어려웠다 — 통로 폭과 벽 배치가 같이
    맞아야 재현되기 때문에, 진짜 지도를 고정 자료로 둔다.
    """
    import os

    import numpy as np

    from sar import exploration

    here = os.path.dirname(os.path.abspath(__file__))
    grid = np.load(os.path.join(here, "data", "comb3_end_grid.npz"))["grid"]

    loose = exploration.frontier_mask(grid, reachable_only=False)
    kept = exploration.frontier_mask(grid, reachable_only=True)
    clusters = exploration.cluster(kept)

    assert int(loose.sum()) > 100, "자료가 바뀌었다 (프론티어가 거의 없다)"
    assert clusters, (
        f"프론티어를 전부 버렸다 — 로봇이 '갈 곳 없다' 고 끝낸다 "
        f"(필터 없음 {int(loose.sum())}칸 → 도달가능 {int(kept.sum())}칸)")


def test_candidates_are_places_the_robot_can_stand():
    """목표로 내주는 자리는 로봇이 **설 수 있는** 곳이어야 한다.

    ⚠️ 회귀 방지. 프론티어 칸을 그대로 목표로 내주면, 프론티어가 벽 근처에 생기므로
       설 수 없는 자리가 목표가 된다. 그러면 _replan 이 목표를 몰래 바꾸고 로봇이
       벽에 붙어 좁은 틈에 갇힌다 (실측 comb0: 최근접 0.21 m 에서 후진·제자리회전만
       반복, 최선대비 1.49x -> 2.77x).
       프론티어는 "갈 이유" 로만 쓰고 목표는 설 수 있는 자리로 옮겨야 한다.
    """
    import os

    import numpy as np

    from sar import common
    from sar import config
    from sar import exploration
    from sar import planner

    here = os.path.dirname(os.path.abspath(__file__))
    grid = np.load(os.path.join(here, "data", "comb3_end_grid.npz"))["grid"].copy()

    # ⚠️ 이 자료는 탐색이 **끝난** 지도다. 거기서는 후보 0개가 정답이다 (실제로
    #    남은 덩어리 4개 중 2개는 길이 없고, 2개는 설 자리가 프론티어에서
    #    0.70/0.85 m 떨어져 신선도 반경 0.45 m 를 넘는다).
    #    그래서 오른쪽 절반을 "아직 못 본" 상태로 되돌려 **탐색 중간** 을 만든다.
    grid[:, grid.shape[1] // 2:] = 0.0

    blocked = planner.inflate(grid, config.PLANNER_INFLATION_MARGIN)
    candidates = exploration.candidate_list(grid, (-2.5, -2.5))
    assert candidates, "자료가 바뀌었다 (후보가 없다)"

    for spot, _score, _size in candidates:
        cell = common.to_cell(*spot)
        assert common.in_bounds(*cell), f"지도 밖을 목표로 냈다: {spot}"
        assert not blocked[cell], (
            f"설 수 없는 자리를 목표로 냈다: {spot}"
            f" (팽창 여유 {config.PLANNER_INFLATION_MARGIN} m)")

# ⚠️ 여기 있던 시험 여섯 개를 지웠다. 검사하던 **기능 자체를 없앴기 때문** 이다:
#      덩어리 크기 보너스 / 회전 벌점(2개) / 카메라 미관측 폴백 / 순회(2개)
#    목표 선택의 점수를 "A* 경로 길이" 하나로 합쳤으므로 크기를 맞출 항이 없다.
#    (제거 근거와 그때의 성적: docs/무엇을-빼기로-했나.md)


# ⚠️ "느슨한 폴백" 시험 둘도 지웠다. 폴백 자체가 없어졌고(주 경로에 합쳐졌다),
#    monkeypatch 로 candidate_list 를 가짜로 바꿔 두었기 때문에 **공허하게
#    통과** 하고 있었다 — 가짜 통과가 없는 것보다 나쁘다.
#    그 교훈이 지금 지켜지는 곳:
#      "경로의 끝이 진짜 목표인지" → candidate_list 의 planner.plan(exact=True)
#                                  + test_only_offers_places_we_can_reach
#      "로봇 코앞을 목표로 내주지 않기" → _worth_driving_to()
#                                  + test_choose_ignores_frontiers_right_under_the_robot


def test_score_is_the_astar_path_length():
    """점수는 **A* 경로 길이** 그 자체다 — 직선거리도, 가중치 합도 아니다.

    ⚠️ 이 한 줄이 지금 목표 선택의 전부다. 예전에는 점수에 층이 넷 있었고
       (덩어리 크기, 정보 이득 비율, 회전량 환산, 카메라 미관측), 층을 얹을
       때마다 항의 크기를 맞추는 일이 새 버그를 만들었다 — 이득 가중치가 거리를
       삼켰고, 회전 벌점이 비율을 삼켰다.
       경로 길이 하나면 단위가 m 이고 맞출 것이 없다. 그래서 정의를 못 박는다.
    """
    # 방 둘을 벽으로 감싼 복도로 잇는다 (프론티어 덩어리가 둘이 되도록).
    grid = mapping.new_map()
    grid[70:90, 70:90] = config.LOG_ODDS_MIN        # 로봇이 있는 방
    grid[20:35, 70:90] = config.LOG_ODDS_MIN        # 멀리 있는 방
    grid[35:70, 71:89] = config.LOG_ODDS_MIN        # 잇는 복도
    grid[35:70, 70] = config.LOG_ODDS_MAX
    grid[35:70, 89] = config.LOG_ODDS_MAX

    robot = (0.0, 0.0)
    candidates = exploration.candidate_list(grid, robot, min_distance=0.0)
    assert candidates, "자료가 성립하지 않는다 (후보가 없다)"

    for spot, score, _size in candidates:
        path = planner.plan(grid, robot, spot, exact=True)
        assert path, f"점수가 붙었는데 길이 없다: {spot}"
        length = (common.distance(*robot, *path[0])
                  + sum(common.distance(*a, *b) for a, b in zip(path, path[1:])))
        assert abs(score - length) < 1e-9, (
            f"점수가 경로 길이가 아니다: {score:.3f} vs {length:.3f} ({spot})")
        # 경로 길이는 직선거리보다 짧을 수 없다 — 단위가 m 임을 같이 확인한다.
        assert score >= common.distance(*robot, *spot) - 1e-9

    # 그리고 그 점수의 **오름차순** 으로 나와야 한다 (가장 가까운 것이 앞).
    assert len(candidates) >= 2, "정렬을 볼 수 없다 (후보가 하나뿐)"
    scores = [item[1] for item in candidates]
    assert scores == sorted(scores), f"경로가 짧은 순서가 아니다: {scores}"


def test_standing_spot_is_never_in_unknown_space():
    """설 자리를 **미지 칸** 으로 잡으면 안 된다.

    ⚠️ 회귀 방지. 막힘을 `inflate(grid)` 로만 정의하면 미지 칸이 "비어 있다" 로
       보인다. 그런데 프론티어는 **정의상 미지 칸에 붙어 있으므로** nearest_free 가
       바로 그 미지 칸을 설 자리로 고른다. planner.plan 은 미지를 막힘으로 치므로
       그 자리로는 길이 없고, 후보가 통째로 "길이 없다" 로 버려진다.
       실측(comb0): 덩어리 4개가 전부 그렇게 버려졌고, 갈 수 있는 목표물
       (+2.40,-2.50) 을 못 찾은 채 91% 에서 탐색이 끝났다.
    """
    grid = mapping.new_map()
    here = common.to_cell(0.0, 0.0)
    grid[here[0] - 20:here[0] + 20, here[1] - 20:here[1] + 20] = config.LOG_ODDS_MIN
    # 한쪽에 벽을 세워 그 앞 프론티어가 팽창 영역에 들어가게 한다
    grid[here[0] + 20:here[0] + 22, here[1] - 20:here[1] + 20] = config.LOG_ODDS_MAX

    unknown = mapping.is_unknown(grid)
    for spot, _score, _size in exploration.candidate_list(
            grid, (0.0, 0.0), min_distance=0.0):
        cell = common.to_cell(*spot)
        assert common.in_bounds(*cell), f"지도 밖: {spot}"
        assert not unknown[cell], (
            f"미지 칸을 설 자리로 냈다: {spot}"
            f" — planner.plan 은 미지를 막힘으로 치므로 길이 없다")


def test_corner_frontiers_are_usable():
    """**모서리** 프론티어도 쓸 수 있어야 한다.

    ⚠️ 회귀 방지. 프론티어는 벽에 붙어 생기고, 설 수 있는 가장 가까운 자리는
       벽에서 (로봇반경 + 팽창여유) 만큼 떨어져 있다. 모서리는 벽이 둘이라 그
       거리가 두 축으로 겹치므로, 신선도 반경이 그보다 작으면 모서리 프론티어를
       **구조적으로** 못 쓴다.
       실측(comb0·corridor3): 못 찾은 목표물 둘이 모두 아레나 모서리에 있었고,
       그 옆 프론티어의 설 자리가 0.55 m 밖이라 0.45 m 상한에 걸렸다.
    """
    reach = config.ROBOT_RADIUS + config.PLANNER_INFLATION_MARGIN
    assert config.FRONTIER_STALE_RADIUS >= reach * 1.5, (
        f"신선도 반경 {config.FRONTIER_STALE_RADIUS:.2f} m 는 모서리를 못 덮는다 "
        f"(벽 하나에서만도 {reach:.2f} m, 모서리는 두 축으로 겹친다)")


def test_a_frontier_whose_stand_spot_falls_across_an_unseen_strip_is_still_offered():
    """설 자리가 안 본 띠 **건너편** 에 잡혀도, 로봇 쪽에 설 자리가 있으면 후보로 낸다.

    ⚠️ 회귀 방지. 대회 월드(2026-10-05, 370초 녹화 재생): 거실 의자와 소파 사이 좁은 목
       (평소 여유로는 막히고 좁은 여유로만 통과) 끝에 56칸 경계가 있었다. 설 자리는 평소
       여유로 안 막힌 가장 가까운 칸이라, 얇은 미탐색 띠 건너 넓게 트인 복도에 잡혔고
       거기는 갈 수 없어 그 경계가 통째로 버려졌다. 로봇 쪽으로 0.22 m 옮긴 자리에는
       길이 있었다 — 거기 서면 띠 너머가 보이고 사과로 가는 복도가 열린다.
    """
    # 칸 번호로 그린다 (월드 좌표 경계는 부동소수점 반올림으로 한 줄씩 밀린다).
    r, c = common.to_cell(0.0, 0.0)
    half = common.to_cells(config.ROBOT_RADIUS + config.PLANNER_INFLATION_MARGIN)
    grid = mapping.new_map()
    grid[r - 40:r + 40, c - 40:c + 40] = config.LOG_ODDS_MAX           # 바탕은 벽
    # 로봇이 올라온 복도 — 가운데 칸이 양쪽 벽에서 정확히 평소 팽창 반경만큼 떨어진다.
    # 평소 여유로는 전부 막히고, 좁은 여유로는 가운데 한 줄만 열린다.
    grid[r - 30:r, c - half + 1:c + half] = config.LOG_ODDS_MIN
    grid[r:r + 2, c - half + 1:c + half] = 0.0                       # 안 본 얇은 띠 (2칸)
    grid[r + 2:r + 30, c - 30:c + 30] = config.LOG_ODDS_MIN             # 건너편 넓은 방
    robot = common.to_world(r - 20, c)
    found = exploration.candidate_list(grid, robot, min_distance=0.0)
    near_side = [spot for spot, _, _ in found if common.to_cell(*spot)[0] < r]
    assert near_side, f"로봇 쪽 설 자리로 경계를 내야 한다: {found}"
    assert planner.plan(grid, robot, near_side[0], exact=True,
                        margin=config.PLANNER_SQUEEZE_MARGIN), "그 자리는 실제로 갈 수 있어야 한다"


def test_size_bonus_is_capped_so_a_far_big_frontier_cannot_jump_a_much_closer_door(monkeypatch):
    """큰 경계 가산점(칸당 FRONTIER_SIZE_BONUS)에는 상한이 있다.

    ⚠️ 대회 월드(2026-10-06 녹화 재생, 140.7초): 화장실 문 경계(경로 2.15 m, 14칸)와 서쪽
       경계(3.02 m, 99칸)의 점수가 2.01 대 2.03 이었다 — 99칸이 0.99 m 를 깎아 받아, 1 m 가까운
       좁은 문 방을 뒤로 밀었다. 화장실 사과는 그 뒤 650초가 지나서야 찾았다.
    """
    monkeypatch.setattr(config, "FRONTIER_SIZE_BONUS", 0.01)
    monkeypatch.setattr(config, "FRONTIER_SIZE_BONUS_CAP", 0.5)
    assert exploration.frontier_score(2.15, 14) < exploration.frontier_score(3.02, 99)
    assert exploration.frontier_score(3.0, 20) > exploration.frontier_score(3.0, 99), \
        "길이가 같으면 큰 경계가 여전히 먼저다"
