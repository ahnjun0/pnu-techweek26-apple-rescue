"""상태 머신을 확인한다. 일부는 가짜 월드에서 전체 루프를 통째로 돌려 본다.

Webots 를 켜기 전에 "혼자 탐색하고, 부딪히지 않고, 결국 멈추는가" 를 여기서 본다.
"""

import math

import numpy as np
import pytest

from sar import common
from sar import config
import fake_world
from sar import exploration
from sar import follower
from sar import mapping
from sar import mission
from sar import planner


def clear_lidar(distance=3.0):
    return np.full(config.LIDAR_RESOLUTION, distance, dtype=np.float64)


def blocking_distance():
    """"멈춰야 하지만 눌린 건 아닌" 거리.

    ⚠️ 그냥 "정지거리 - 5 cm" 로 쓰면 안 된다. 눌림 판정(SAFETY_PINNED_DISTANCE)
       안으로 들어가면 임무가 정지·대기가 아니라 탈출로 넘어가 버린다.
       실제로 그것 때문에 테스트 3개가 한꺼번에 깨졌다.
    """
    return (config.SAFETY_PINNED_DISTANCE + config.SAFETY_STOP_DISTANCE) / 2.0


def run(world, start, seconds=200.0, dt=0.064, speed_limit=None):
    """가짜 월드에서 임무를 돌린다. 기록을 담은 dict 를 돌려준다.

    dt 를 Webots 틱(0.032)의 2배로 잡아 테스트를 빠르게 한다.
    """
    pose = start
    brain = mission.Mission(pose)
    trail = [pose[:2]]
    collisions = 0
    max_speed = 0.0
    ticks = int(seconds / dt)

    ranges = world.lidar(pose)
    for tick in range(ticks):
        if tick % 2 == 0:                 # LiDAR 는 두 틱에 한 번만 (속도)
            ranges = world.lidar(pose)
        speed, turn = brain.step(pose, ranges, dt)
        max_speed = max(max_speed, abs(speed))
        pose, hit = world.move(pose, speed, turn, dt)
        if hit:
            collisions += 1
        trail.append(pose[:2])
        if brain.state == mission.DONE:
            break

    return {
        "mission": brain,
        "pose": pose,
        "trail": trail,
        "collisions": collisions,
        "max_speed": max_speed,
        "seconds": (tick + 1) * dt,
        "done": brain.state == mission.DONE,
    }


# --- 상태 머신의 기본 동작 (월드 없이) ------------------------------------------

def test_path_is_always_a_list_never_none():
    """planner.plan 은 실패하면 None 을 준다. mission.path 는 언제나 리스트여야 한다.

    화면 표시나 기록 코드가 len(path) 를 쓰기 때문이다.
    실제로 여기서 TypeError 가 났었다.
    """
    brain = mission.Mission((0.0, 0.0, 0.0))
    brain.state = mission.EXPLORE

    # 지도를 벽으로 완전히 갈라 놓아 A* 가 반드시 실패하게 만든다
    brain.grid[:] = config.LOG_ODDS_MIN
    _, col = common.to_cell(1.0, 0.0)
    brain.grid[:, col] = config.LOG_ODDS_MAX
    brain.goal = (2.5, 0.0)

    brain._replan((0.0, 0.0, 0.0))
    assert brain.path == [], "실패했으면 None 이 아니라 빈 리스트여야 한다"
    assert len(brain.path) == 0        # len() 이 터지지 않아야 한다


def test_path_stays_a_list_through_a_whole_run():
    """한 판 돌리는 동안 한 번도 None 이 되면 안 된다."""
    world = fake_world.two_rooms()
    pose = (-2.0, -2.0, 0.0)
    brain = mission.Mission(pose)
    dt = 0.064
    ranges = world.lidar(pose)
    for tick in range(600):
        if tick % 2 == 0:
            ranges = world.lidar(pose)
        speed, turn = brain.step(pose, ranges, dt)
        assert isinstance(brain.path, list), f"{tick} 틱에서 path 가 리스트가 아니다"
        pose, _ = world.move(pose, speed, turn, dt)
        if brain.state == mission.DONE:
            break


def test_starts_with_a_scan():
    """아무것도 모르는 채로 목표를 고르면 곧바로 DONE 이 되어 버린다."""
    assert mission.Mission((0.0, 0.0, 0.0)).state == mission.SCAN


def test_scan_spins_in_place_then_switches_to_explore():
    brain = mission.Mission((0.0, 0.0, 0.0))
    theta = 0.0
    dt = 0.064
    for _ in range(400):
        speed, turn = brain.step((0.0, 0.0, theta), clear_lidar(2.0), dt)
        assert speed == 0.0, "시작 스캔에서는 움직이지 않는다"
        theta = common.wrap_angle(theta + turn * dt)
        if brain.state != mission.SCAN:
            break
    assert brain.state == mission.EXPLORE
    assert mapping.is_occupied(brain.grid).any(), "스캔하며 지도를 만들어야 한다"


def test_mission_does_not_finish_before_it_has_looked_around():
    """첫 틱에 "갈 곳 없음" 으로 끝나 버리면 로봇이 아예 안 움직인다."""
    brain = mission.Mission((0.0, 0.0, 0.0))
    brain.step((0.0, 0.0, 0.0), clear_lidar(2.0), 0.032)
    assert brain.state != mission.DONE


def test_done_state_keeps_the_robot_still():
    brain = mission.Mission((0.0, 0.0, 0.0))
    brain.state = mission.DONE
    assert brain.step((0.0, 0.0, 0.0), clear_lidar(), 0.032) == (0.0, 0.0)


def test_mission_builds_a_map_as_it_goes():
    brain = mission.Mission((0.0, 0.0, 0.0))
    brain.state = mission.EXPLORE
    for _ in range(3 * config.MAP_UPDATE_EVERY):
        brain.step((0.0, 0.0, 0.0), clear_lidar(1.5), 0.032)
    assert mapping.is_occupied(brain.grid).any()


def test_being_pinned_beats_everything_else():
    """몸이 눌리면 무엇을 하던 중이든 빠져나오는 것이 최우선이다."""
    brain = mission.Mission((0.0, 0.0, 0.0))
    brain.state = mission.EXPLORE

    pinned = clear_lidar()
    angles = common.lidar_angles()
    pinned[np.abs(angles) < 0.3] = config.SAFETY_PINNED_DISTANCE - 0.02

    for _ in range(config.SAFETY_PINNED_TICKS):
        speed, _ = brain.step((0.0, 0.0, 0.0), pinned, 0.032)
    assert speed < 0.0, "눌렸으면 기다리지 말고 빠져나와야 한다"


def test_a_blocked_path_is_eventually_replanned_around():
    """앞을 막은 것이 지도에 쌓이고 나면, 기다리기보다 돌아가는 길을 찾는다.

    한 번 본 것만으로는 안 바뀐다 — log-odds 는 증거를 쌓아야 뒤집힌다.
    그 덕에 LiDAR 가 한 번 튀었다고 멀쩡한 길이 막히지 않는다.
    """
    brain = mission.Mission((0.0, 0.0, 0.0))
    brain.state = mission.EXPLORE

    row, _ = common.to_cell(0.0, 0.0)
    _, col_end = common.to_cell(2.0, 0.0)
    _, col_start = common.to_cell(-0.5, 0.0)
    brain.grid[row - 1:row + 2, col_start:col_end + 1] = config.LOG_ODDS_MIN

    brain.goal = (2.0, 0.0)
    original = [(0.0, 0.0), (2.0, 0.0)]
    brain.path = list(original)

    blocked = clear_lidar()
    angles = common.lidar_angles()
    blocked[np.abs(angles) < 0.1] = blocking_distance()

    for _ in range(80):
        brain.step((0.0, 0.0, 0.0), blocked, 0.1)

    obstacle_cell = common.to_cell(blocking_distance(), 0.0)
    assert mapping.is_occupied(brain.grid)[obstacle_cell], \
        "계속 보이는 장애물은 결국 지도에 찍혀야 한다"
    assert brain.path != original, "막힌 것을 알았으면 경로를 다시 세워야 한다"

    # ⚠️ 단언 두 개를 넣었다가 뺐다. 둘 다 이 테스트가 볼 것이 아니었다:
    #    - "정면이 막혔으면 전진하면 안 된다": DWA 는 멈추는 대신 비껴 간다.
    #      좁은 장애물이면 그게 옳다. 충돌 회피는 tests/test_dwa.py 가 본다.
    #    - "다시 세운 경로가 막혀 있으면 안 된다": 경로의 첫 점은 로봇 자신이라
    #      벽에 붙어 있으면 당연히 팽창 안이다. tests/test_planner.py 가 본다.


def test_clears_the_blacklist_once_rather_than_finishing_early():
    """프론티어가 남았는데 전부 블랙리스트면, 한 번은 비우고 다시 해 봐야 한다."""
    brain = mission.Mission((0.0, 0.0, 0.0))
    brain.state = mission.EXPLORE
    brain.grid[70:90, 70:90] = config.LOG_ODDS_MIN      # 볼 곳이 남아 있다

    # 모든 프론티어를 덮는 커다란 블랙리스트
    brain.blacklist = mission.exploration.Blacklist(radius=99.0)
    brain.blacklist.banned.append((0.0, 0.0))

    assert brain._frontiers_remain((0.0, 0.0, 0.0))
    brain._pick_goal((0.0, 0.0, 0.0))
    assert brain._blacklist_resets >= 1, "비우고 다시 시도했어야 한다"


def test_a_goal_that_never_works_gets_blacklisted():
    brain = mission.Mission((0.0, 0.0, 0.0))
    for _ in range(config.FRONTIER_MAX_FAILURES):
        brain.goal = (2.0, 2.0)
        brain._fail_goal("테스트")
    assert brain.blacklist.contains(2.0, 2.0)
    assert brain.goal is None

def test_does_not_stop_for_something_coming_closer():
    """⚠️ 회귀 방지: 다가오는 것 앞에서 멈춰 서면 안 된다.

    한때 "다가오면 멈춰서 지나가길 기다린다" 를 넣었다가 Webots 로 재 보고 뺐다.
    보행자는 physics 없는 운동학 물체라 무한한 힘으로 민다 — 멈춰 선 로봇은
    길에 놓인 물건이라 계속 밟힌다.
      비키기 끔 169초 / 사람과 닿은 시간 2.7초
      비키기 켬 446~900초 / 40~141초 (바퀴가 헛돌아 추정 위치 108 m 폭주)
    몸이 진짜로 눌렸을 때(is_pinned) 빠져나오는 것은 그대로 남아 있다.
    """
    brain = mission.Mission((0.0, 0.0, 0.0))
    brain.state = mission.EXPLORE
    brain.goal = (2.0, 0.0)
    brain.path = [(0.0, 0.0), (2.0, 0.0)]

    angles = common.lidar_angles()
    gap = 1.4
    for _ in range(30):
        ranges = clear_lidar()
        gap = max(config.SAFETY_PINNED_DISTANCE + 0.10, gap - 0.03)
        ranges[np.abs(angles) < 0.3] = gap
        speed, turn = brain.step((0.0, 0.0, 0.0), ranges, 0.032)
        assert "기다림" not in brain.status, f"멈춰서 기다렸다: {brain.status}"

def test_being_pinned_overrides_everything_else():
    """보행자가 로봇을 벽으로 밀어붙이면, 경로고 목표고 제쳐 두고 빠져나와야 한다."""
    brain = mission.Mission((0.0, 0.0, 0.0))
    brain.state = mission.EXPLORE
    brain.goal = (2.0, 0.0)
    brain.path = [(0.0, 0.0), (2.0, 0.0)]

    pinned = clear_lidar()
    angles = common.lidar_angles()
    pinned[np.abs(angles) < 0.3] = config.SAFETY_PINNED_DISTANCE - 0.02

    # 두 틱 연속이어야 믿는다 (한 틱 잡음에 속지 않기 위해).
    for _ in range(config.SAFETY_PINNED_TICKS):
        speed, _ = brain.step((0.0, 0.0, 0.0), pinned, 0.032)
    assert speed < 0.0, "앞에 눌렸으면 뒤로 빠져야 한다"
    assert "눌렸" in brain.status


def test_a_single_tick_lidar_spike_is_ignored():
    """⚠️ 회귀 방지: LiDAR 가 한 틱만 튀는 일이 실제로 있다.

    앞뒤로는 70 cm 인데 그 틱만 13 cm 를 찍는다. 그걸 믿고 1초를 후진하면
    경로를 버리고 제자리에서 맴돈다 — 한 자리에서 40초 동안 15번 연속으로
    이 일이 났다. 진짜 눌림은 여러 틱에 걸쳐 천천히 가까워진다.
    """
    brain = mission.Mission((0.0, 0.0, 0.0))
    brain.state = mission.EXPLORE
    brain.goal = (2.0, 0.0)
    brain.path = [(0.0, 0.0), (2.0, 0.0)]

    angles = common.lidar_angles()
    far = clear_lidar()
    far[np.abs(angles) < 0.3] = 0.70
    spike = far.copy()
    spike[np.abs(angles) < 0.3] = 0.13

    brain.step((0.0, 0.0, 0.0), far, 0.032)
    brain.step((0.0, 0.0, 0.0), spike, 0.032)          # 딱 한 틱만 튄다
    speed, _ = brain.step((0.0, 0.0, 0.0), far, 0.032)

    assert "눌렸" not in brain.status, f"한 틱 잡음에 속았다: {brain.status}"
    assert speed >= 0.0, "한 틱 잡음 때문에 후진하면 안 된다"


def test_stuck_robot_backs_up_and_gives_up_on_the_goal():
    """Phase 2 에서 겪은 "바퀴가 헛돌아 오도메트리 폭주" 를 막는 장치."""
    brain = mission.Mission((0.0, 0.0, 0.0))
    brain.state = mission.EXPLORE
    brain.goal = (2.0, 0.0)
    brain.path = [(0.0, 0.0), (2.0, 0.0)]

    pose = (0.0, 0.0, 0.0)
    for _ in range(int(config.MISSION_STUCK_SECONDS / 0.1) + 5):
        brain.step(pose, clear_lidar(), 0.1)     # 가라고 하는데 pose 가 안 변한다

    assert brain._backup_left > 0.0, "끼임을 알아채고 후진해야 한다"
    speed, _ = brain.step(pose, clear_lidar(), 0.1)
    assert speed < 0.0, "후진이어야 한다"


def test_moving_robot_is_not_treated_as_stuck():
    brain = mission.Mission((0.0, 0.0, 0.0))
    brain.state = mission.EXPLORE
    brain.goal = (5.0, 0.0)
    brain.path = [(0.0, 0.0), (5.0, 0.0)]

    x = 0.0
    for _ in range(int(config.MISSION_STUCK_SECONDS / 0.1) + 5):
        brain.step((x, 0.0, 0.0), clear_lidar(), 0.1)
        x += 0.02                                  # 계속 앞으로 가고 있다
    assert brain._backup_left == 0.0


def test_goal_timeout_is_recorded_as_a_failure():
    brain = mission.Mission((0.0, 0.0, 0.0))
    brain.goal = (2.0, 2.0)
    brain._goal_age = config.MISSION_GOAL_TIMEOUT + 1.0
    brain._needs_new_goal((0.0, 0.0, 0.0))
    assert brain.blacklist.failures(2.0, 2.0) == 1


# --- 가짜 월드에서 전체 루프 ----------------------------------------------------

@pytest.mark.parametrize("name,world,start", [
    ("빈 방", fake_world.open_room(), (-2.0, -2.0, 0.0)),
    ("방 두 개", fake_world.two_rooms(), (-2.0, -2.0, 0.0)),
])
@pytest.mark.slow
def test_explores_and_then_stops(name, world, start):
    """혼자 돌아다니다가 더 볼 곳이 없으면 멈춰야 한다 (무한루프 금지).

    제한 시간은 넉넉히 둔다 — 여기서 재는 것은 "끝나는가" 지 "빠른가" 가 아니다.
    """
    # 목표물이 없으니 둘러보기까지 마쳐야 끝난다. 시간을 넉넉히 준다.
    result = run(world, start, seconds=700.0)

    assert result["done"], \
        f"{name}: 제한 시간 안에 탐색을 못 끝냈다 (상태: {result['mission'].status})"
    explored = mapping.is_free(result["mission"].grid).sum()
    assert explored > 2000, f"{name}: 탐색한 면적이 너무 작다 ({explored} 칸)"


@pytest.mark.slow
def test_never_touches_a_wall():
    """충돌은 실격이다."""
    result = run(fake_world.two_rooms(), (-2.0, -2.0, 0.0), seconds=320.0)
    assert result["collisions"] == 0, \
        f"벽에 {result['collisions']} 틱 동안 닿았다"


@pytest.mark.slow
def test_keeps_a_safe_distance_from_walls():
    """스치듯 지나가면 감점이다. 지나온 자취 전체가 벽에서 떨어져 있어야 한다."""
    world = fake_world.two_rooms()
    result = run(world, (-2.0, -2.0, 0.0), seconds=240.0)

    too_close = sum(1 for x, y in result["trail"]
                    if world.collides(x, y, radius=config.ROBOT_RADIUS + 0.03))
    assert too_close == 0, f"{too_close} 틱 동안 벽에 너무 가까웠다"


@pytest.mark.slow
def test_respects_the_speed_limit_in_a_real_run():
    result = run(fake_world.open_room(), (0.0, 0.0, 0.0), seconds=60.0)
    assert result["max_speed"] <= config.FOLLOW_MAX_SPEED + 1e-9


@pytest.mark.slow
def test_explores_a_harder_map():
    result = run(fake_world.four_rooms(), (-2.0, -2.0, 0.0), seconds=700.0)
    assert result["done"], f"방 네 개를 못 끝냈다 (상태: {result['mission'].status})"
    assert result["collisions"] == 0


@pytest.mark.slow
def test_stays_put_once_done():
    """DONE 이 된 뒤에도 움직이면 마지막에 자리를 이탈한다.

    목표물이 없는 월드에서는 지도를 다 그린 뒤 몇 곳을 둘러보고서야 끝난다
    (카메라를 안 주므로 목표물을 영영 못 찾는다). 그만큼 시간을 준다.
    """
    result = run(fake_world.open_room(), (0.0, 0.0, 0.0), seconds=600.0)
    assert result["done"]

    brain = result["mission"]
    where = result["pose"]
    for _ in range(50):
        assert brain.step(where, clear_lidar(), 0.064) == (0.0, 0.0)


def test_gives_up_escaping_and_stops_the_wheels():
    """⚠️ 회귀 방지: 빠져나와지지 않으면 바퀴를 멈춰야 한다.

    사람에게 눌려 아무리 해도 못 빠져나오는데 계속 바퀴를 돌리면, 엔코더만
    쌓여서 추정 위치가 폭주한다. 실측: 840초 동안 매달리다 추정 위치가
    108 m 까지 갔고, 그 판은 지도·경로·복귀가 전부 무너졌다.
    바퀴를 멈추면 엔코더가 안 움직이니 추정 위치가 보존된다.
    """
    brain = mission.Mission((0.0, 0.0, 0.0))
    brain.state = mission.EXPLORE
    brain.goal = (2.0, 0.0)
    brain.path = [(0.0, 0.0), (2.0, 0.0)]

    # 앞쪽만 눌려 있다. 사방을 다 막으면 지도가 다 차서 임무가 먼저 끝나 버려
    # 정작 재려는 상황이 안 나온다.
    angles = common.lidar_angles()
    pinned = clear_lidar()
    pinned[np.abs(angles) < 0.5] = config.SAFETY_PINNED_DISTANCE - 0.05

    dt = 0.032
    stopped_at = None
    spun = 0.0
    for i in range(int((config.MISSION_ESCAPE_GIVEUP + 2.0) / dt)):
        speed, turn = brain.step((0.0, 0.0, 0.0), pinned, dt)
        if "바퀴를 멈추고" in brain.status:
            stopped_at = i * dt
            assert abs(speed) < 1e-6 and abs(turn) < 1e-6, \
                f"멈춘다고 해 놓고 명령은 v={speed} w={turn}"
            break
        spun += abs(speed) * dt        # 헛돈 거리 = 오도메트리가 먹는 거짓 거리

    assert stopped_at is not None, "끝없이 바퀴를 돌렸다 (오도메트리가 폭주한다)"
    assert stopped_at <= config.MISSION_ESCAPE_GIVEUP + 0.5, \
        f"멈추기까지 {stopped_at:.1f}초나 걸렸다"
    assert spun < 2.0, f"멈추기 전에 {spun:.1f} m 어치나 헛돌았다"


def test_frozen_wheels_stay_frozen_until_it_actually_clears():
    """⚠️ 회귀 방지: 정해진 시간마다 무조건 다시 시도하면 계속 헛돈다.

    실측: 3초 멈췄다 6초 헛돌기를 반복해 추정 위치가 -12 m 까지 밀렸다.
    상황이 실제로 트일 때까지 (LiDAR 로 확인) 멈춰 있어야 한다.
    """
    brain = mission.Mission((0.0, 0.0, 0.0))
    brain.state = mission.EXPLORE
    brain.goal = (2.0, 0.0)
    brain.path = [(0.0, 0.0), (2.0, 0.0)]

    angles = common.lidar_angles()
    pinned = clear_lidar()
    pinned[np.abs(angles) < 0.5] = config.SAFETY_PINNED_DISTANCE - 0.05
    dt = 0.032

    # 계속 눌려 있으면 언젠가 바퀴를 멈춘다
    for _ in range(int((config.MISSION_ESCAPE_GIVEUP + 2.0) / dt)):
        brain.step((0.0, 0.0, 0.0), pinned, dt)
        if "바퀴를 멈추고" in brain.status:
            break
    assert "바퀴를 멈추고" in brain.status, "끝없이 바퀴를 돌렸다"

    # 상황이 그대로면 계속 멈춰 있어야 한다 (바퀴를 다시 돌리면 안 된다)
    spun = 0.0
    for _ in range(int(8.0 / dt)):
        speed, turn = brain.step((0.0, 0.0, 0.0), pinned, dt)
        spun += abs(speed) * dt
    assert spun < 0.05, f"상황이 그대로인데 {spun:.2f} m 어치를 헛돌았다"

    # 트이면 곧바로 다시 움직인다
    moved = False
    for _ in range(int(1.0 / dt)):
        speed, turn = brain.step((0.0, 0.0, 0.0), clear_lidar(), dt)
        if abs(speed) > 0.01 or abs(turn) > 0.01:
            moved = True
            break
    assert moved, "트였는데도 계속 멈춰 있었다"


def test_unreachable_goal_is_failed_not_counted_as_arrival(monkeypatch):
    """경로가 끝났어도 목표에 못 닿았으면 실패다 — 안 그러면 영원히 제자리다.

    ⚠️ 회귀 방지. planner 는 목표 칸이 팽창으로 막혀 있으면 근처의 갈 수 있는
       칸으로 바꿔 경로를 낸다 (planner.nearest_free). 주행기는 그 끝에서
       ARRIVED 를 내므로, 목표가 1 m 떨어져 있어도 "도착" 이 나온다.
       그걸 성공으로 쳐서 블랙리스트에 넣지 않으면 다음 틱에 같은 목표를 다시
       골라 **속도 0 으로 영원히 돈다.** 사람 없는 월드에서 실제로 한자리에
       800초를 서 있었다 (목표까지 0.98 m 남은 채 "도착" 이라고 했다).
    """
    brain = mission.Mission((0.0, 0.0, 0.0))
    brain.state = mission.EXPLORE
    brain.goal = (1.0, 0.0)          # 로봇에서 1 m — 허용치(0.2 m)보다 훨씬 멀다
    assert common.distance(0.0, 0.0, *brain.goal) > config.FOLLOW_GOAL_TOLERANCE

    # 목표를 새로 고르지도, 다시 계획하지도 않게 두고 "도착" 만 흉내 낸다.
    brain.path = [(0.0, 0.0), (0.2, 0.0)]
    brain.path_index = 0
    brain._goal_age = 0.0
    brain._since_replan = 0.0
    monkeypatch.setattr(follower, "step",
                        lambda *a, **k: (0.0, 0.0, follower.ARRIVED, 1))
    monkeypatch.setattr(planner, "path_is_blocked", lambda *a, **k: False)
    # 목표 근처에 프론티어가 "있다" 고 두어야 _needs_new_goal 이 목표를 먼저
    # 버리지 않는다. 우리가 보려는 것은 그 다음의 도착 판정이다.
    monkeypatch.setattr(exploration, "frontier_near", lambda *a, **k: True)

    brain._explore((0.0, 0.0, 0.0), clear_lidar(), dt=0.064)

    assert brain.goal is None, "못 닿은 목표를 그대로 쥐고 있으면 안 된다"
    assert "못 닿았다" in brain.status, \
        f"실패로 기록돼야 한다. 실제 상태: {brain.status}"
    assert brain.blacklist.failures, "실패가 한 번도 기록되지 않았다"


def test_keeps_looking_while_targets_are_missing():
    """목표물을 아직 못 찾았으면, 갈 데가 남은 한 복귀하지 않는다.

    ⚠️ 회귀 방지. 예전 _frontiers_remain() 은 frontier_mask() 를 기본값
       (reachable_only=True)으로 불러, 좁은 틈 뒤 방의 프론티어를 전부 걸렀다.
       그래서 "갈 데가 없다" 고 판단하고 2/3 만 찾은 채 집에 갔다
       (corridor 지도, LiDAR 90%, 못 찾은 것은 (-2.5,+2.5)).
       채점 기준 1번이 '모든 구조 대상 식별' 이라 이것이 가장 비싼 실패다.
    """
    brain = mission.Mission((0.0, 0.0, 0.0))
    brain.state = mission.EXPLORE
    brain.grid[70:90, 70:90] = config.LOG_ODDS_MIN      # 볼 곳이 남아 있다

    # 목표물을 하나도 못 찾은 상태여야 한다
    assert brain._targets_missing()

    # 모든 후보를 덮는 블랙리스트 — 그래도 포기하면 안 된다
    brain.blacklist = mission.exploration.Blacklist(radius=99.0)
    brain.blacklist.banned.append((0.0, 0.0))

    brain._pick_goal((0.0, 0.0, 0.0))
    assert brain._blacklist_resets >= 1, "목표물이 모자라면 비우고 다시 봐야 한다"
    # 그리고 한 번으로 끝나지 않고 더 매달릴 수 있어야 한다
    assert (config.MISSION_MAX_BLACKLIST_RESETS_SEARCHING
            > config.MISSION_MAX_BLACKLIST_RESETS), \
        "찾는 중일 때의 재시도 한도가 더 커야 한다"


def test_goes_to_look_at_an_unconfirmed_candidate_before_giving_up():
    """확정 못 한 후보가 있으면, 복귀하기 전에 가서 확인한다.

    ⚠️ 회귀 방지. 확정 문턱은 DETECT_MIN_SIGHTINGS(25)인데, 그에 못 미친 후보는
       nearest_unvisited() 에 안 잡혀 임무 판단에서 **보이지 않았다.** 그래서
       "저기 뭔가 있다" 는 정보를 쥐고도 다 했다고 판단하고 집에 갔다
       (corridor 지도: 전체 후보 3, 확정 2, 상태 DONE, 목표물 2/3).
    """
    brain = mission.Mission((0.0, 0.0, 0.0))
    brain.state = mission.EXPLORE

    # 몇 번만 본 후보 하나 — 확정엔 못 미친다
    brain.targets.add(2.0, 0.0)
    for _ in range(config.DETECT_VERIFY_SIGHTINGS):
        brain.targets.add(2.0, 0.0)
    maybe = brain.targets.best_unconfirmed()
    assert maybe is not None, "확인해 볼 후보로 잡혀야 한다"
    assert not brain.targets.confirmed, "아직 확정은 아니어야 한다"

    # 더 탐색할 곳도 없고 목표물도 모자란 상황
    brain._decide_next((0.0, 0.0, 0.0), exploring_possible=False)

    assert brain.state == mission.APPROACH, \
        f"후보를 확인하러 가야 한다. 실제 상태: {brain.state} / {brain.status}"
    assert "확인하러" in brain.status


def test_gives_up_and_returns_before_time_runs_out():
    """시간이 얼마 안 남으면 찾은 것만 들고 복귀한다.

    ⚠️ 제한 시간을 다 쓰고 못 끝내면 **복귀 점수까지 0** 이다.
       실측: corridor3 이 900초 초과 / 131 m / 목표물 2-3 으로 끝나며 복귀도 못 했다.
       공고 기준이 "각 객체 위치까지 이동한 뒤 시작 지점으로 복귀" 이므로 복귀도 점수다.
    """
    brain = mission.Mission((0.0, 0.0, 0.0))
    brain.state = mission.EXPLORE
    brain.targets.add(1.0, 0.0)
    for _ in range(config.DETECT_MIN_SIGHTINGS):
        brain.targets.add(1.0, 0.0)
    assert brain.targets.confirmed, "확정된 목표물이 하나는 있어야 하는 상황이다"
    brain.targets.mark_visited(1.0, 0.0)

    # ⚠️ 검사는 _step() 이 **매 틱** 한다. _decide_next() 안에 뒀다가 한 번도
    #    발동하지 않았다 — EXPLORE 로 한 목표를 붙들고 있으면 거기를 안 거친다.
    #    그러니 시험도 실제 경로(step)로 해야 한다.
    ranges = clear_lidar()

    brain.elapsed = config.MISSION_TIME_LIMIT * 0.5
    brain.step((0.0, 0.0, 0.0), ranges, 0.032)
    assert brain.state != mission.RETURN, "시간이 남았는데 벌써 복귀하면 안 된다"

    # 시작점에서 멀리 떨어진 곳에서 마감을 넘겨 본다 (시작점에 있으면 복귀가
    # 곧바로 끝나 DONE 이 되므로, 복귀가 걸렸는지 구별되지 않는다).
    brain.elapsed = config.MISSION_TIME_LIMIT - 1.0
    brain.step((3.0, 3.0, 0.0), ranges, 0.032)
    assert brain.state in (mission.RETURN, mission.DONE), \
        f"시간 예산을 넘기면 복귀해야 한다. 실제: {brain.state} / {brain.status}"
    assert "시간 예산" in brain.status or "복귀" in brain.status, \
        f"복귀 사유가 남아야 한다: {brain.status}"


def test_keeps_searching_when_nothing_found_yet():
    """하나도 못 찾았으면 시간이 지나도 계속 찾는다 (복귀해 봐야 0점이다)."""
    brain = mission.Mission((0.0, 0.0, 0.0))
    brain.state = mission.EXPLORE
    assert not brain.targets.confirmed

    brain.elapsed = config.MISSION_TIME_LIMIT - 1.0
    brain.step((0.0, 0.0, 0.0), clear_lidar(), 0.032)
    assert brain.state != mission.RETURN, \
        "찾은 것이 없으면 시간이 지나도 계속 찾아야 한다"


def test_a_distant_person_does_not_let_the_robot_stand_still():
    """멀리 있는 사람(이나 유령) 때문에 멈춰 서면 안 된다.

    ⚠️ 회귀 방지. `allow_idle` 이 `bool(self._people)` 이었다. **유령 하나만
       있어도 정지 벌점이 통째로 꺼진다.** 검출은 완벽할 수 없는데 그 위에 중요한
       안전장치를 얹은 것이 잘못이었다.
       실측(comb0, 사람이 **없는** 월드): 유령 5,670회(0.10/틱) 때문에 로봇이
       640초부터 끝까지 한자리에 붙박여 복귀를 6.16 m 남기고 실패했다.
    """
    brain = mission.Mission((0.0, 0.0, 0.0))
    pose = (0.0, 0.0, 0.0)

    brain._people = []
    assert not brain._person_is_close(pose), "아무도 없으면 허용하지 않는다"

    far = config.PEOPLE_IDLE_RADIUS + 1.0
    brain._people = [(far, 0.0, 0.0, 0.0)]
    assert not brain._person_is_close(pose), \
        f"{far} m 밖의 사람 때문에 멈춰 서면 안 된다"

    near = config.PEOPLE_IDLE_RADIUS * 0.5
    brain._people = [(near, 0.0, 0.0, 0.0)]
    assert brain._person_is_close(pose), \
        f"{near} m 앞에 사람이 있으면 가만히 있는 것도 선택지여야 한다"


def test_return_deadline_follows_how_far_home_is():
    """복귀 마감은 **시작점까지의 거리**에서 나온다 — 고정 비율이 아니다.

    ⚠️ 처음엔 "제한 시간의 75%" 였다. 근거가 없었고, maze3 이 752초 걸리는
       궤적에서 675초 선에 걸려 목표물 3/3 -> 1/3 이 됐다 (후보 3개를 보고도
       확정·방문은 1개). 가까이 있으면 더 탐색해도 되는데 똑같이 잘랐다.
    """
    brain = mission.Mission((0.0, 0.0, 0.0))
    brain.start_pose = (0.0, 0.0, 0.0)

    # ⚠️ 복귀 거리는 A* 비용 때문에 몇 틱마다만 다시 잰다 (캐시). 시험할 때는
    #    그 캐시를 비워 줘야 두 위치가 각각 계산된다.
    near = brain._return_deadline((0.3, 0.0, 0.0))
    brain._home_gap = None
    far = brain._return_deadline((5.0, 0.0, 0.0))
    assert near > far, \
        f"시작점에 가까우면 더 오래 탐색할 수 있어야 한다: 가까이 {near:.0f}s, 멀리 {far:.0f}s"
    assert far > config.MISSION_TIME_LIMIT * 0.5, \
        "너무 일찍 돌아서면 탐색 시간을 헛되이 버린다"
    assert near <= config.MISSION_TIME_LIMIT - config.MISSION_RETURN_MIN, \
        "아무리 가까워도 최소 여유는 남겨야 한다"


def test_sighting_on_the_way_is_checked_first():
    """가는 길에 "뭔가 봤다" 면, 들르는 게 짧을 때 먼저 확인한다.

    ⚠️ 회귀 방지. 예전에는 지금 들고 있는 프론티어 목표를 **끝낸 뒤에야**
       APPROACH 로 갔다. 그래서 카메라로 확정해 놓고도 지나치고 나중에 되돌아왔다.
       실측(maze0): 40초에 남동 목표물 옆을 지나 북동으로 먼저 가고 100~120초에
       남동으로 되돌아왔다 (GUI 로 사용자가 짚어 줬다).
    """
    brain = mission.Mission((0.0, 0.0, 0.0))
    brain.state = mission.EXPLORE
    brain.grid[:] = config.LOG_ODDS_MIN          # 전부 빈 곳으로 둔다

    # 멀리 있는 탐색 목표를 들고 있다
    brain.goal = (3.0, 0.0)
    brain.path = [(0.0, 0.0), (3.0, 0.0)]
    brain.path_index = 0

    # 옆(1.2 m)에 **미확정** 후보가 있다 (몇 번만 봤다).
    # ⚠️ 확정된 목표물로는 이 검사가 성립하지 않는다 — _step 이 그보다 먼저
    #    무조건 APPROACH 로 보내므로 _explore 에 도달하지 못한다.
    for _ in range(config.DETECT_VERIFY_SIGHTINGS):
        brain.targets.add(1.2, 0.0)
    assert brain.targets.best_unconfirmed() is not None
    assert brain.targets.nearest_unvisited(0.0, 0.0) is None

    ranges = np.full(config.LIDAR_RESOLUTION, 3.0)
    brain.step((0.0, 0.0, 0.0), ranges, 0.064)

    # ⚠️ 상태가 아니라 **상태줄** 로 본다. 합성 격자는 전부 빈 곳이라 프론티어가
    #    없고, 그러면 _decide_next 가 원래 경로로도 APPROACH 로 보낸다 — 그것과
    #    구별해야 "끼어들기가 작동했다" 를 확인할 수 있다.
    assert "본 것부터 확인한다" in brain.status, (
        f"옆의 목표물을 지나쳐 멀리 갔다 (상태 {brain.state}, 상태줄 {brain.status})")
    assert brain.state == mission.APPROACH


def test_after_interrupting_the_robot_really_heads_for_the_unconfirmed_candidate():
    """끼어든 다음 틱에 APPROACH 가 그 후보로 **실제로** 간다 (EXPLORE 로 되튀지 않는다).

    ⚠️ 회귀 방지. 확정 전 후보는 nearest_unvisited 에 안 잡히고 _verify_tried 도 비어
       APPROACH 가 곧바로 EXPLORE 로 나갔다. 추정 위치가 매번 조금씩 바뀌어
       _interrupted_for 에도 안 걸려 다음 틱에 또 끼어들었다 — 상태줄만 "본 것부터
       확인한다" 를 찍고 실제로는 안 갔다. 대회 월드(2026-10-05): 화장실 사과를 22회
       보고(확정 25회) 떠났다. 배준호(bae-junho 브랜치)가 짚은 문제와 같은 고침이다.
    """
    brain = mission.Mission((0.0, 0.0, 0.0))
    brain.state = mission.EXPLORE
    brain.grid[:] = config.LOG_ODDS_MIN
    brain.goal = (4.0, 0.0)
    brain.path = [(0.0, 0.0), (4.0, 0.0)]
    brain.path_index = 0
    far = config.VERIFY_LOOK_RANGE + 0.5      # 바라보기 거리 밖 — 다가가야 한다
    for _ in range(config.DETECT_VERIFY_SIGHTINGS):
        brain.targets.add(far, 0.0)
    ranges = np.full(config.LIDAR_RESOLUTION, 5.0)
    brain.step((0.0, 0.0, 0.0), ranges, 0.064)
    assert brain.state == mission.APPROACH
    brain.step((0.0, 0.0, 0.0), ranges, 0.064)
    assert brain.state == mission.APPROACH, brain.status
    assert brain.goal is not None and common.distance(*brain.goal, far, 0.0) < 0.1, \
        f"후보 쪽으로 경로를 짜야 한다 (목표 {brain.goal}, 상태줄 {brain.status})"


def _unconfirmed_candidate(brain, x, y):
    for _ in range(config.DETECT_VERIFY_SIGHTINGS):
        brain.targets.add(x, y)
    candidate = brain.targets.best_unconfirmed()
    assert candidate is not None and not candidate.confirmed
    return candidate


def test_robot_turns_to_look_at_a_close_unconfirmed_candidate_out_of_view():
    """확정 전 후보 가까이 왔는데 카메라 밖이면 그쪽으로 돈다 — 보지 않으면 확정될 수 없다.

    ⚠️ 회귀 방지. 대회 월드(2026-10-05, 녹화 재생): 화장실 사과를 1.4 m 앞에서 10회 보고,
       돌아가는 경로를 따라 머리가 돌아 화면 밖으로 놓친 채 0.61 m 앞에 "도착" 해 방문으로
       쳤다. 확정 전이라 목표물로 세지 않았고, 방문 표시 때문에 다시는 후보로도 안 골랐다.
    """
    brain = mission.Mission((0.0, 0.0, 0.0))
    brain.grid[:] = config.LOG_ODDS_MIN
    brain.state = mission.APPROACH
    candidate = _unconfirmed_candidate(brain, 0.0, 1.0)     # 왼쪽 1 m — 카메라 밖
    speed, turn = brain._approach((0.0, 0.0, 0.0), clear_lidar(), 0.064)
    assert speed == 0.0 and turn > 0.0, (speed, turn, brain.status)
    assert not candidate.visited


def test_robot_looks_at_an_unconfirmed_candidate_then_gives_up_if_it_never_confirms():
    brain = mission.Mission((0.0, 0.0, 0.0))
    brain.grid[:] = config.LOG_ODDS_MIN
    brain.state = mission.APPROACH
    candidate = _unconfirmed_candidate(brain, 1.0, 0.0)     # 정면 1 m
    speed, turn = brain._approach((0.0, 0.0, 0.0), clear_lidar(), 0.064)
    assert speed == 0.0 and turn == 0.0, "서서 바라본다"
    assert not candidate.visited
    for _ in range(int(config.VERIFY_LOOK_TIME / 0.064) + 2):
        if brain.state != mission.APPROACH:
            break
        brain._approach((0.0, 0.0, 0.0), clear_lidar(), 0.064)
    assert candidate.visited, "끝내 확정이 안 되면 건너뛴다"
    assert brain.state != mission.APPROACH


def test_a_candidate_that_confirms_while_looking_is_approached_and_visited():
    brain = mission.Mission((0.0, 0.0, 0.0))
    brain.grid[:] = config.LOG_ODDS_MIN
    brain.state = mission.APPROACH
    candidate = _unconfirmed_candidate(brain, 1.0, 0.0)
    brain._approach((0.0, 0.0, 0.0), clear_lidar(), 0.064)
    while not candidate.confirmed:                          # 바라보는 사이 확정됐다
        brain.targets.add(1.0, 0.0)
    speed, _ = brain._approach((0.0, 0.0, 0.0), clear_lidar(), 0.064)
    assert brain.goal is not None, "확정되면 평소처럼 다가간다"


# ⚠️ 여기 있던 test_far_target_does_not_interrupt_a_closer_goal 을 지웠다.
#    "멀리 있는 목표물 때문에 가까운 탐색 목표를 버리면 안 된다" 는 규칙 자체를
#    없앴기 때문이다. 목표물과 프론티어를 **같은 자격으로 거리 비교** 한 것이
#    잘못이었다 — 목표물은 확실한 점수이고 프론티어는 추측이며, 언젠가 그
#    목표물에 가야 하므로 지금 지나치면 왕복이 추가된다.
#    (사용자가 GUI 에서 여러 번 짚었다: "분명히 카메라에서 목표물을 보았음에도
#     패스함", "검출했으면 그 쪽으로 가는 게 맞지 않아?",
#     "왜 카메라 데이터가 우선되지 않는 거야?")
#    옛 규칙의 근거는 "왕복 방지" 라는 추론뿐이었고 측정이 없었다. 반대 방향에는
#    측정이 있다 (maze0: 40초에 지나치고 100~120초에 되돌아왔다).
#    ⚠️ 그리고 이 시험은 상태줄 문구로 판정해서, 문구를 바꾸자 **공허하게
#       통과** 하고 있었다 — 없는 것보다 나쁘다.

def test_far_goal_gets_more_patience_than_a_near_one():
    """먼 목표를 가까운 목표와 같은 시간에 자르면 안 된다.

    ⚠️ 회귀 방지. MISSION_GOAL_TIMEOUT 이 **거리와 무관한 고정값**(45초)이라,
       1 m 목표에는 후하고 뱀길 6 m 목표에는 가혹했다. 시간 초과는 즉시
       블랙리스트에 올리므로 대가가 크다.
       실측(comb1): 빗살이 위아래로 엇갈려 서쪽으로 가려면 y>1 로 돌아야 하는데,
       그 경로가 약 6 m 라 45초에 아슬아슬하게 걸렸다. 서쪽 목표를 잡을 때마다
       절반쯤에서 버려지고 블랙리스트 리셋마다 반복됐다 (80·260·440초에 같은
       목표). 46초에 탐색률 50% 를 찍고 754초 동안 한 칸도 못 늘렸다 → 1/3, 51%.
    """
    brain = mission.Mission((0.0, 0.0, 0.0))

    brain.path = []
    assert brain._goal_patience() == config.MISSION_GOAL_TIMEOUT, (
        "경로가 없으면 하한을 써야 한다")

    # 코앞 목표 — 하한 그대로
    brain.path = [(0.0, 0.0), (1.0, 0.0)]
    near = brain._goal_patience()
    assert near == config.MISSION_GOAL_TIMEOUT

    # 뱀길 6 m — 더 기다려 줘야 한다
    brain.path = [(0.0, 0.0), (0.0, -1.5), (-2.3, -1.5), (-2.3, 1.2)]
    far = brain._goal_patience()
    assert far > near, f"먼 목표에 더 기다려 주지 않는다 (가까움 {near}, 멂 {far})"
    assert far >= 6.0 / config.FOLLOW_MAX_SPEED, (
        f"최고속으로 가는 시간보다도 짧게 준다 ({far:.0f}초)")


def test_camera_lead_may_cross_unknown_ground():
    """카메라가 본 방향으로는 미탐색 칸을 지나서라도 가 본다.

    ⚠️ 카메라가 그 방향으로 뭔가를 봤다는 것은 **시선이 뚫려 있다** 는 뜻이다 —
       적어도 그만큼은 빈 공간이다. 그런데 그 구역이 지도에 아직 미탐색이면
       계획기가 거부한다 (PLANNER_ALLOW_UNKNOWN = False). 그 기본값 자체는 옳다
       (켜 두면 벽 바깥 경로가 생겼다). 단서는 **근거가 다르므로** 이 경우에만
       미탐색 통과를 허용한다.
    """
    from sar import planner

    grid = mapping.new_map()          # 전부 미탐색
    # 로봇 주변만 빈 칸으로 (여기서 출발할 수 있어야 한다)
    r0, c0 = common.to_cell(-0.6, -0.6)
    r1, c1 = common.to_cell(0.6, 0.6)
    grid[min(r0, r1):max(r0, r1), min(c0, c1):max(c0, c1)] = config.LOG_ODDS_MIN

    far = (2.0, 0.0)                  # 미탐색 한가운데
    assert planner.plan(grid, (0.0, 0.0), far) is None, (
        "실험이 성립하지 않는다 — 평소에도 미탐색으로 길이 난다")
    assert planner.plan(grid, (0.0, 0.0), far, allow_unknown=True), (
        "미탐색 통과를 허용해도 길을 못 찾는다")


def test_sweep_goes_where_the_camera_has_not_looked():
    """둘러볼 자리는 **카메라가 안 본 곳** 중 가장 가까운 곳이어야 한다.

    ⚠️ 회귀 방지. 예전 규칙은 "이미 둘러본 자리에서 **가장 먼** 빈 칸" 이었다.
       SWEEP 의 존재 이유가 "LiDAR 는 360°인데 카메라는 57°뿐이라 목표물 쪽을
       한 번도 안 봤을 수 있다" 인데, 정작 카메라 커버리지를 안 봤다.
       그래서 구조적으로 맵을 가로지르며 왕복했다 — 실측(comb0): 280~420초 동안
       두 구석을 오가며 탐색 면적이 180초에 96칸밖에 안 늘었다.
       (사용자가 GUI 에서 "왔던 곳을 재탐색한다" 고 짚었다.)
    """
    brain = mission.Mission((0.0, 0.0, 0.0))
    here = common.to_cell(0.0, 0.0)

    # 넓은 빈 방을 만들고, 카메라는 **전부 봤다** 고 둔다.
    brain.grid[here[0] - 30:here[0] + 30, here[1] - 30:here[1] + 30] = \
        config.LOG_ODDS_MIN
    brain.camera_seen[:, :] = True

    # 딱 두 군데만 안 봤다고 둔다 — 가까운 쪽과 먼 쪽.
    near = (here[0] + 12, here[1])
    far = (here[0] - 25, here[1] - 25)
    for cell in (near, far):
        brain.camera_seen[cell[0] - 2:cell[0] + 3, cell[1] - 2:cell[1] + 3] = False

    spot = brain._pick_sweep_point((0.0, 0.0, 0.0))
    assert spot is not None, "안 본 곳이 있는데 자리를 못 냈다"

    near_xy = common.to_world(*near)
    far_xy = common.to_world(*far)
    assert (common.distance(*spot, *near_xy)
            < common.distance(*spot, *far_xy)), (
        f"가까운 미관측 구역이 아니라 먼 쪽을 골랐다: {spot}"
        f" (가까운 곳 {near_xy}, 먼 곳 {far_xy})")


def test_sweep_does_not_offer_the_same_spot_twice():
    """이미 시도한 자리는 다시 내주지 않아야 한다 — 못 갔더라도.

    ⚠️ 회귀 방지. 옛 코드는 **360°를 다 돈 뒤에만** 자리를 기록했다. 그래서 못 간
       자리는 기록되지 않고, 고르는 기준이 "가장 먼 곳" 이라 영원히 다시 뽑혔다
       (실측 comb0: 같은 지점을 280초와 360초에 두 번 목표로 잡았다).
    """
    brain = mission.Mission((0.0, 0.0, 0.0))
    here = common.to_cell(0.0, 0.0)
    brain.grid[here[0] - 30:here[0] + 30, here[1] - 30:here[1] + 30] = \
        config.LOG_ODDS_MIN
    brain.camera_seen[:, :] = True
    spot_cell = (here[0] + 12, here[1])
    brain.camera_seen[spot_cell[0] - 2:spot_cell[0] + 3,
                      spot_cell[1] - 2:spot_cell[1] + 3] = False

    first = brain._pick_sweep_point((0.0, 0.0, 0.0))
    assert first is not None
    brain._sweep_points.append(first)
    second = brain._pick_sweep_point((0.0, 0.0, 0.0))
    if second is not None:
        assert common.distance(*first, *second) > config.MISSION_SWEEP_SPACING, (
            f"같은 자리를 다시 냈다: {first} -> {second}")


def test_sweep_only_offers_spots_we_can_reach():
    """둘러볼 자리는 **갈 수 있는** 곳이어야 한다.

    ⚠️ 회귀 방지. 탐색 쪽은 A*(exact=True)로 도달성을 확인하는데 둘러보기는 안
       했다. 그래서 못 가는 자리를 고른 뒤 "못 가겠으면 그 자리에서 둘러본다" 로
       빠져 **엉뚱한 자리에서 360°를 돌았고**, 남은 미관측 구역이 갈 수 없는
       주머니면 그 짓을 무한히 반복했다 (사용자: "무제한 동글동글 회전했어").
    """
    brain = mission.Mission((0.0, 0.0, 0.0))
    here = common.to_cell(0.0, 0.0)

    # 로봇이 있는 방
    brain.grid[here[0] - 20:here[0] + 20, here[1] - 20:here[1] + 20] = \
        config.LOG_ODDS_MIN
    # 벽으로 완전히 막힌 별채 (갈 수 없다)
    far = (here[0] + 45, here[1])
    brain.grid[far[0] - 6:far[0] + 6, far[1] - 6:far[1] + 6] = config.LOG_ODDS_MIN
    brain.grid[far[0] - 8:far[0] - 6, far[1] - 8:far[1] + 8] = config.LOG_ODDS_MAX
    brain.grid[far[0] + 6:far[0] + 8, far[1] - 8:far[1] + 8] = config.LOG_ODDS_MAX
    brain.grid[far[0] - 8:far[0] + 8, far[1] - 8:far[1] - 6] = config.LOG_ODDS_MAX
    brain.grid[far[0] - 8:far[0] + 8, far[1] + 6:far[1] + 8] = config.LOG_ODDS_MAX

    # 카메라는 **별채만** 못 봤다고 둔다
    brain.camera_seen[:, :] = True
    brain.camera_seen[far[0] - 6:far[0] + 6, far[1] - 6:far[1] + 6] = False

    spot = brain._pick_sweep_point((0.0, 0.0, 0.0))
    assert spot is None, (
        f"갈 수 없는 별채를 둘러볼 자리로 냈다: {spot}"
        f" — 거기 가려다 실패하면 엉뚱한 자리에서 돌게 된다")


# ⚠️ 여기 있던 test_target_goal_is_a_place_we_can_stand 를 지웠다.
#    "목표를 설 수 있는 자리로 옮긴다" 를 목표물에도 적용해 봤는데 **주저가
#    전혀 안 줄었다** (목표물 근처 18.4초/85% -> 18.9초/84%). 느린 띠는 겨냥이
#    아니라 장애물과의 거리로 정해지고, 방문 판정을 받으려면 로봇이 물리적으로
#    그 띠 안에 들어가야 하기 때문이다. 성적도 나빠졌다
#    (484.5 -> 541.2 m, 3570 -> 4118초, 47/48 -> 46/48).


def test_retrace_goes_back_along_the_trail_to_the_start():
    trail = [(0.1 * i, 0.0) for i in range(11)]          # (0,0) → (1,0)
    back = mission.retrace(trail, join=0.05)
    assert back[0] == pytest.approx((1.0, 0.0))
    assert back[-1] == (0.0, 0.0), "시작점에서 끝나야 한다"


def test_retrace_skips_a_loop_but_never_jumps_further_than_join():
    # 오른쪽으로 갔다가 한 바퀴 돌아 출발점 옆을 지나 위로 올라간다.
    trail = [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.1, 0.1), (0.0, 1.0)]
    back = mission.retrace(trail, join=0.3)
    assert back == [(0.0, 1.0), (0.1, 0.1), (0.0, 0.0)], "고리 (1,0)-(1,1) 은 건너뛴다"
    for a, b in zip(back[1:], back[2:]):
        assert common.distance(*a, *b) <= 0.3 + 1e-9


def test_return_retraces_the_trail_when_the_map_has_no_path():
    brain = mission.Mission(start_pose=(0.0, 0.0, 0.0))
    brain._crumbs = [(0.0, 0.0), (0.5, 0.0), (1.0, 0.0)]
    brain.state = mission.RETURN
    brain._replan = lambda pose: setattr(brain, "path", [])   # 지도에 길이 없다
    pose = (1.0, 0.0, math.pi)
    for _ in range(int(config.RETURN_DIRECT_AFTER / 0.064) + 5):
        speed, _ = brain._return(pose, clear_lidar(), 0.064)
    assert "되짚" in brain.status
    assert brain._retrace[-1] == (0.0, 0.0)
    assert speed > 0.0, "되짚는 길로 실제로 움직여야 한다"


def _stuck_return_brain():
    brain = mission.Mission(start_pose=(0.0, 0.0, 0.0))
    brain.grid[:] = config.LOG_ODDS_MIN                      # 지도에는 집까지 길이 있다
    brain._refresh_plan_grid()
    brain._crumbs = [(0.0, 0.0), (0.0, 0.5), (0.0, 1.0), (0.5, 1.0), (1.0, 1.0)]  # 들어온 길
    brain.state = mission.RETURN
    return brain


def test_return_backs_out_along_the_trail_when_stuck_although_the_map_has_a_path():
    """지도에 집까지 길은 있는데 한 자리에서 맴돌기만 하면, 들어온 길을 조금 되짚어 나온다.

    ⚠️ 회귀 방지. 대회 월드(2026-10-05): 화장실 사과를 방문한 자리에서 복귀를 시작했는데,
       팽창 영역 안이라 주행기가 "주행 → 장애물 정지 → 제자리 회전" 을 되풀이하며 190초
       동안 0.3 m 안에서 맴돌다 복귀 시간 초과로 멈췄다 (사과 2/2 를 찾고도 복귀 실패).
       지도에 길이 **없을 때** 만 되짚게 되어 있었다.
    """
    brain = _stuck_return_brain()
    pose = (1.0, 1.0, 0.0)                                   # 자리가 그대로다
    for _ in range(int(config.RETURN_STALL_TIME / 0.064) + 2):
        brain._return(pose, clear_lidar(), 0.064)
    assert "막혀" in brain.status, brain.status
    assert brain._unstick, "들어온 길을 되짚어야 한다"
    end = brain._unstick[-1]
    assert common.distance(*end, 0.0, 0.5) < 1e-6, f"1.5 m 만 되짚는다 (끝 {end})"


def test_a_return_that_keeps_moving_never_backs_out():
    brain = _stuck_return_brain()
    for k in range(int(config.RETURN_STALL_TIME * 1.5 / 0.064)):
        x = 1.0 - 0.01 * k                                   # 조금씩이라도 나아간다
        brain._return((x, 1.0, math.pi), clear_lidar(), 0.064)
    assert not brain._unstick and "막혀" not in brain.status


def test_after_backing_out_the_robot_plans_home_again():
    brain = _stuck_return_brain()
    brain._unstick = [(1.0, 1.0), (0.5, 1.0)]
    brain._return((0.5, 1.0, math.pi), clear_lidar(), 0.064)  # 되짚을 길 끝에 왔다
    assert not brain._unstick
    assert brain.path, "새 자리에서 집까지 다시 짠다"


def _no_more_sweeps(brain):
    brain._sweep_points = [(9.0, 9.0)] * config.MISSION_SWEEP_POINTS


def test_trapped_robot_retraces_out_instead_of_ending_exploration():
    """갈 경계가 안 보이는데 집까지도 길이 없으면 "다 봤다" 가 아니라 "갇혔다" 다.

    ⚠️ 회귀 방지. 실측(대회 월드, 2026-10-05 녹화 재생): 좁은 문으로 북동쪽 방에 들어간 뒤
       그 문이 지도에서 좁은 여유로도 못 지나가게 칠해졌다. 계획기에게 방 밖이 전부
       "갈 수 없음" 이 되어 422초에 탐색을 끝냈다 (478초 남음, 사과 1/2). 복귀는 지나온 길
       되짚기로 그 문을 빠져나갔고, 빠져나온 자리에서는 거실 쪽 경계 12개에 길이 있었다.
    """
    brain = mission.Mission(start_pose=(0.0, 0.0, 0.0))
    brain.state = mission.EXPLORE
    _no_more_sweeps(brain)
    # 지도가 전부 미탐색 — 집까지도 길이 없다
    brain._decide_next((1.0, 0.0, 0.0), exploring_possible=False)
    assert brain.state == mission.RETURN, "빠져나오는 일은 복귀의 되짚기가 한다"
    assert brain._resume_after_escape
    assert "갇혔다" in brain.status


def test_after_escaping_the_robot_goes_back_to_exploring():
    brain = mission.Mission(start_pose=(0.0, 0.0, 0.0))
    brain.state = mission.RETURN
    brain._resume_after_escape = True
    brain.grid[:] = config.LOG_ODDS_MIN          # 빠져나왔다 — 이제 집까지 길이 있다
    brain._refresh_plan_grid()
    brain._return((1.0, 0.0, 0.0), clear_lidar(), 0.064)
    assert brain.state == mission.EXPLORE, brain.status
    assert not brain._resume_after_escape
    # ⚠️ 되짚기 없이 길이 바로 생겼다 — 잠깐 막혔던 것이라 횟수에 세지 않는다.
    #    대회 월드(2026-10-05): 67·70초에 0.2초짜리 막힘 두 번에 한도를 다 써서
    #    575초에 정말 갇혔을 때는 빠져나와 탐색을 잇지 못했다.
    assert brain._escape_resumes == 0


def test_an_escape_that_needed_retracing_counts_toward_the_limit():
    brain = mission.Mission(start_pose=(0.0, 0.0, 0.0))
    brain.state = mission.RETURN
    brain._resume_after_escape = True
    brain._retrace = [(1.0, 0.0), (0.0, 0.0)]    # 지나온 길을 되짚던 중이다
    brain.grid[:] = config.LOG_ODDS_MIN
    brain._refresh_plan_grid()
    brain._return((1.0, 0.0, 0.0), clear_lidar(), 0.064)
    assert brain.state == mission.EXPLORE, brain.status
    assert brain._escape_resumes == 1


def test_after_escaping_past_the_deadline_the_robot_keeps_going_home():
    brain = mission.Mission(start_pose=(0.0, 0.0, 0.0))
    brain.state = mission.RETURN
    brain._resume_after_escape = True
    brain.grid[:] = config.LOG_ODDS_MIN
    brain._refresh_plan_grid()
    brain.elapsed = config.MISSION_TIME_LIMIT    # 복귀 마감을 넘겼다
    brain._return((1.0, 0.0, 0.0), clear_lidar(), 0.064)
    assert brain.state == mission.RETURN
    assert not brain._resume_after_escape, "마감을 넘겼으면 다시 탐색하지 않는다"


def test_escape_resumes_are_limited():
    brain = mission.Mission(start_pose=(0.0, 0.0, 0.0))
    brain.state = mission.EXPLORE
    _no_more_sweeps(brain)
    brain._escape_resumes = config.EXPLORE_ESCAPE_RESUMES
    brain._decide_next((1.0, 0.0, 0.0), exploring_possible=False)
    assert brain.state == mission.RETURN
    assert not brain._resume_after_escape
    assert "갇혔다" not in brain.status


def test_robot_that_is_not_trapped_ends_exploration_as_before():
    brain = mission.Mission(start_pose=(0.0, 0.0, 0.0))
    brain.state = mission.EXPLORE
    _no_more_sweeps(brain)
    brain.grid[:] = config.LOG_ODDS_MIN          # 집까지 길이 있다
    brain._refresh_plan_grid()
    brain._decide_next((1.0, 0.0, 0.0), exploring_possible=False)
    assert brain.state == mission.RETURN
    assert not brain._resume_after_escape
    assert "갇혔다" not in brain.status


def _walled_room(brain, half_cells, open_side=False):
    """로봇 둘레에 벽으로 닫힌 빈 방을 그린다 (open_side 면 동쪽 벽을 터 경계를 만든다)."""
    r, c = common.to_cell(0.0, 0.0)
    brain.grid[r - half_cells - 1:r + half_cells + 2, c - half_cells - 1:c + half_cells + 2] = \
        config.LOG_ODDS_MAX
    brain.grid[r - half_cells:r + half_cells + 1, c - half_cells:c + half_cells + 1] = \
        config.LOG_ODDS_MIN
    if open_side:
        brain.grid[r - half_cells:r + half_cells + 1, c + half_cells + 1] = 0.0
    brain._refresh_plan_grid()


def test_a_nearby_room_the_camera_never_saw_is_looked_at_before_moving_on():
    """LiDAR 로는 다 그렸는데 카메라가 못 본 가까운 방은, 떠나기 전에 둘러본다.

    ⚠️ 대회 월드(2026-10-05): 110초에 화장실 입구까지 와서 LiDAR 로 안을 다 그렸다 —
       경계가 없어 들어갈 이유가 없었고, 사과는 안쪽 설비에 가려 카메라가 못 봤다.
       558초 뒤(668초)에야 다시 와서 찾았다. 둘러보기는 탐색이 **끝난 뒤** 에만 썼다.
    """
    brain = mission.Mission((0.0, 0.0, 0.0))
    brain.state = mission.EXPLORE
    _walled_room(brain, common.to_cells(0.8))
    assert brain._start_nearby_look((0.0, 0.0, 0.0))
    assert brain.state == mission.SWEEP and brain._look_sweep
    assert "둘러보러" in brain.status


def test_after_a_nearby_look_the_robot_goes_back_to_exploring():
    brain = mission.Mission((0.0, 0.0, 0.0))
    _walled_room(brain, common.to_cells(0.8))
    brain.state = mission.SWEEP
    brain._look_sweep = True
    brain._sweep_turned = 2.0 * math.pi + 0.1        # 한 바퀴 다 돌았다
    brain._sweep_theta = 0.0
    brain._sweep_step((0.0, 0.0, 0.0), clear_lidar(), 0.064)
    assert brain.state == mission.EXPLORE, brain.status
    assert not brain._look_sweep


def test_a_far_unseen_room_is_left_to_exploration():
    brain = mission.Mission((0.0, 0.0, 0.0))
    _walled_room(brain, common.to_cells(0.8))
    brain.camera_seen[:] = True                       # 이 방은 카메라가 봤다
    far = config.EXPLORE_LOOK_DISTANCE + 1.0          # 저 멀리 못 본 방
    r, c = common.to_cell(far, 0.0)
    half = common.to_cells(0.8)
    brain.grid[r - half:r + half + 1, c - half:c + half + 1] = config.LOG_ODDS_MIN
    brain.camera_seen[r - half:r + half + 1, c - half:c + half + 1] = False
    brain._refresh_plan_grid()
    assert not brain._start_nearby_look((0.0, 0.0, 0.0))


def test_an_unseen_strip_next_to_a_frontier_is_left_to_exploration():
    brain = mission.Mission((0.0, 0.0, 0.0))
    _walled_room(brain, common.to_cells(0.4), open_side=True)   # 방 전체가 경계 1 m 안
    assert not brain._start_nearby_look((0.0, 0.0, 0.0))


def test_replan_goes_to_a_goal_only_the_narrow_margin_reaches_instead_of_stopping_short():
    """좁은 여유로만 닿는 목표면, 평소 여유가 근처 칸으로 바꿔 준 경로 대신 좁은 길로 끝까지 간다.

    ⚠️ 회귀 방지. plan() 은 목표 칸이 막혀 있으면 근처 칸으로 목표를 바꿔 "성공" 한다.
       _replan 은 평소 여유로 먼저 묻기 때문에 좁은 복도 안 목표는 입구 앞에서 끝나는 길을
       받았고, 도착하면 "경로는 끝났는데 목표에 못 닿았다" 로 실패했다. 대회 월드(2026-10-05)
       2/2 실행: 탐색 643초 중 실패한 목표에 272초, 그중 이 실패가 14~15번이었다.
    """
    brain = mission.Mission((0.0, 0.0, 0.0))
    r, c = common.to_cell(0.0, 0.0)
    half = common.to_cells(config.ROBOT_RADIUS + config.PLANNER_INFLATION_MARGIN)
    brain.grid[:] = config.LOG_ODDS_MAX
    brain.grid[r - 30:r - 10, c - 20:c + 20] = config.LOG_ODDS_MIN    # 넓은 방
    brain.grid[r - 10:r + 10, c - half + 1:c + half] = config.LOG_ODDS_MIN   # 좁은 복도
    brain._refresh_plan_grid()
    start = common.to_world(r - 20, c)
    brain.goal = common.to_world(r, c)                                  # 복도 안
    brain._replan((start[0], start[1], 0.0))
    assert brain.path, "길이 있어야 한다"
    assert common.distance(*brain.path[-1], *brain.goal) <= config.FOLLOW_GOAL_TOLERANCE, \
        f"목표까지 가야 한다 (끝 {brain.path[-1]}, 목표 {brain.goal})"
    assert brain.squeezing


def test_replan_keeps_the_roomy_path_when_it_reaches_the_goal():
    brain = mission.Mission((0.0, 0.0, 0.0))
    brain.grid[:] = config.LOG_ODDS_MIN
    brain._refresh_plan_grid()
    brain.goal = (1.0, 0.0)
    brain._replan((0.0, 0.0, 0.0))
    assert brain.path and not brain.squeezing


def _arrived_brain(end_gap):
    """경로 끝이 목표에서 end_gap 떨어져 있고, 로봇은 경로 끝 0.19 m 앞에 선 상태."""
    brain = mission.Mission((0.0, 0.0, 0.0))
    brain.state = mission.EXPLORE
    brain.grid[:] = 0.0                                     # 모르는 칸
    r, c = common.to_cell(0.0, 0.0)
    brain.grid[r - 20:r + 20, c - 20:c + common.to_cells(1.2)] = config.LOG_ODDS_MIN
    brain._refresh_plan_grid()
    brain.goal = (1.0, 0.0)                                  # 바로 너머가 모르는 칸 — 경계가 남아 있다
    end = (1.0 - end_gap, 0.0)
    brain.path = [(0.0, 0.0), end]
    brain.path_index = 1
    pose = (end[0] - 0.19, 0.0, 0.0)
    return brain, pose


def test_reaching_a_path_end_right_next_to_the_goal_is_arriving_at_the_goal():
    """경로 끝이 목표에 붙어 있으면(0.2 m 안) 경로 끝 도착이 곧 목표 도착이다.

    ⚠️ 회귀 방지. 주행기는 경로 끝 0.2 m 안이면 "도착", 임무는 목표 0.2 m 안이어야 도착으로
       쳤다. 둘이 겹쳐 로봇이 목표 0.21~0.35 m 앞에 서면 "경로는 끝났는데 목표에 못 닿았다"
       로 실패하고 블랙리스트 횟수가 쌓였다. 대회 월드(2026-10-05) 녹화 재생: 이 실패 11번 중
       8번이 경로 끝-목표 0.05~0.18 m 였다.
    """
    brain, pose = _arrived_brain(end_gap=0.15)
    brain._since_replan = 0.0
    brain._explore(pose, np.full(config.LIDAR_RESOLUTION, 5.0), 0.064)
    assert "경로는 끝났는데" not in brain.status, brain.status


def test_a_path_that_ends_well_short_of_the_goal_is_still_a_failure():
    """경로 끝이 목표에서 멀면(근처 칸으로 바뀐 경우) 예전처럼 실패다 — 같은 목표를 영원히 다시 고르지 않게."""
    brain, pose = _arrived_brain(end_gap=0.45)
    brain._since_replan = 0.0
    brain._explore(pose, np.full(config.LIDAR_RESOLUTION, 5.0), 0.064)
    assert "경로는 끝났는데" in brain.status, brain.status


def test_low_obstacles_are_walls_in_the_plan_grid_but_not_in_the_lidar_map():
    brain = mission.Mission(start_pose=(0.0, 0.0, 0.0))
    for _ in range(config.LOW_MIN_SIGHTINGS):
        brain.low.add(0.5, 0.0)
    brain._refresh_plan_grid()
    cell = common.to_cell(0.5, 0.0)
    assert mapping.is_occupied(brain.plan_grid)[cell]
    assert not mapping.is_occupied(brain.grid)[cell], "LiDAR 지도는 그대로 둔다"
    assert (0.5, 0.0, 0.0, 0.0, 99) in brain._low_ghosts()


def test_slip_is_detected_when_wheels_turn_but_the_compass_does_not():
    """카펫 턱: 바퀴는 1.2 rad/s 로 도는데 나침반은 그대로 → 후진하고 그 자리를 찍는다."""
    brain = mission.Mission(start_pose=(0.0, 0.0, 0.0))
    brain.state = mission.EXPLORE
    dt = 0.064
    for _ in range(int(config.SLIP_SECONDS / dt) + 2):
        brain._check_slip((0.0, 0.0, 0.0), 1.2, dt)
    assert brain.slip_spots == [(0.0, 0.0)] and brain.slip_count == 1
    assert brain._backup_left > 0.0
    assert mapping.is_occupied(brain.plan_grid)[common.to_cell(0.0, 0.0)]


def test_no_slip_when_the_compass_follows_the_wheels():
    brain = mission.Mission(start_pose=(0.0, 0.0, 0.0))
    brain.state = mission.EXPLORE
    dt, theta = 0.064, 0.0
    for _ in range(40):
        theta += 1.1 * dt                   # 나침반도 거의 같이 돈다 (비율 0.92)
        brain._check_slip((0.0, 0.0, theta), 1.2, dt)
    assert brain.slip_spots == []


