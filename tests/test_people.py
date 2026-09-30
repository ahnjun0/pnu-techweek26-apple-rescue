"""사람 추적기(칼만 필터)를 확인한다."""

import numpy as np

import config
import people


def test_kalman_tracker_recovers_walking_speed_from_noisy_positions():
    """한 스캔 위치 오차 23 cm 로는 두 스캔 차이 속도가 쓸모없다 — 필터는 걸러 낸다."""
    rng = np.random.default_rng(0)
    tracker = people.Tracker()
    dt, speed = 0.064, 0.5
    out = []
    for k in range(120):                       # 7.7 초 동안 x 방향으로 0.5 m/s
        x = speed * k * dt
        z = (x + rng.normal(0, 0.1), rng.normal(0, 0.1))
        out = tracker.update([z], dt)
    assert len(out) == 1
    x, y, vx, vy, hits = out[0]
    assert abs(vx - speed) < 0.2 and abs(vy) < 0.2, (vx, vy)
    assert hits == 120


def test_kalman_tracker_keeps_two_people_apart_and_forgets_the_gone():
    tracker = people.Tracker()
    for _ in range(5):
        out = tracker.update([(0.0, 0.0), (3.0, 0.0)], 0.064)
    assert len(out) == 2
    for _ in range(int(config.PEOPLE_KF_FORGET / 0.064) + 2):
        out = tracker.update([(0.0, 0.0)], 0.064)
    assert len(tracker.tracks) == 1, "안 보이는 사람은 잊는다"


# --- 다가오는 사람 (EVADE) ------------------------------------------------------

import math

import mission


def test_person_walking_straight_at_me_is_a_threat():
    me = (0.0, 0.0, 0.0)
    walker = (1.2, 0.0, -0.2, 0.0, 10)          # 1.2 m 앞에서 0.2 m/s 로 다가온다
    assert people.person_threat(me, [walker]) == walker


def test_person_walking_away_is_not_a_threat():
    """뒤따라가는 상황 — 멀어지는 사람 때문에 비키면 영영 앞으로 못 간다."""
    me = (0.0, 0.0, 0.0)
    assert people.person_threat(me, [(0.6, 0.0, 0.2, 0.0, 10)]) is None


def test_standing_person_is_left_to_the_planner():
    me = (0.0, 0.0, 0.0)
    assert people.person_threat(me, [(0.6, 0.0, 0.03, 0.0, 10)]) is None


def test_person_passing_well_to_the_side_is_not_a_threat():
    """나를 향한 성분이 있어도 옆으로 넉넉히 지나가면 위협이 아니다."""
    me = (0.0, 0.0, 0.0)
    passer = (2.0, 2.5, -0.2, 0.0, 10)          # y 2.5 m 떨어진 줄을 따라 지나간다
    assert people.person_threat(me, [passer]) is None


def test_far_person_is_not_a_threat_yet():
    me = (0.0, 0.0, 0.0)
    assert people.person_threat(me, [(5.0, 0.0, -0.2, 0.0, 10)]) is None


def _brain_exploring():
    brain = mission.Mission((0.0, 0.0, 0.0))
    brain.state = mission.EXPLORE
    return brain


def test_mission_enters_evade_and_steps_aside(monkeypatch):
    """사람이 정면에서 오면 EVADE 로 들어가 옆(사람 진행 방향의 직각)으로 돈다."""
    monkeypatch.setattr(config, "PEOPLE_ENABLED", False)
    brain = _brain_exploring()
    walker = (1.0, 0.0, -0.2, 0.0, 10)
    brain._people = [walker]
    open_space = np.full(config.LIDAR_RESOLUTION, 3.0)
    # _step 은 매 틱 사람 목록을 다시 채우므로, 판단 부분만 직접 부른다
    assert people.person_threat((0.0, 0.0, 0.0), brain._people) is not None
    brain._enter_evade()
    v, w = brain._evade((0.0, 0.0, 0.0), open_space, 0.064)
    assert brain.state == mission.EVADE
    heading = brain._evade_heading((0.0, 0.0, 0.0), open_space, walker)
    assert abs(abs(common_wrap(heading)) - math.pi / 2) < 1e-6, heading
    assert v == 0.0 and abs(w) > 0.0, "옆으로 비키려면 먼저 제자리에서 돈다"


def common_wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def test_evade_picks_the_open_side(monkeypatch):
    """한쪽 옆이 벽이면 트인 쪽으로 비킨다."""
    brain = _brain_exploring()
    walker = (1.0, 0.0, -0.2, 0.0, 10)
    ranges = np.full(config.LIDAR_RESOLUTION, 3.0)
    import common
    angles = common.lidar_angles()
    ranges[np.abs(common.wrap_angle(angles - math.pi / 2)) < math.radians(30)] = 0.2   # 왼쪽 벽
    heading = brain._evade_heading((0.0, 0.0, 0.0), ranges, walker)
    assert heading is not None and common_wrap(heading) < 0.0, heading   # 오른쪽


def test_evade_waits_when_boxed_in():
    brain = _brain_exploring()
    walker = (1.0, 0.0, -0.2, 0.0, 10)
    brain._people = [walker]
    brain._enter_evade()
    boxed = np.full(config.LIDAR_RESOLUTION, 0.30)   # 사방이 막혔다 (눌림 거리보다는 멀다)
    assert brain._evade_heading((0.0, 0.0, 0.0), boxed, walker) is None
    assert brain._evade((0.0, 0.0, 0.0), boxed, 0.064) == (0.0, 0.0)


def test_evade_returns_to_what_it_was_doing():
    brain = mission.Mission((0.0, 0.0, 0.0))
    brain.state = mission.APPROACH
    brain._people = [(1.0, 0.0, -0.2, 0.0, 10)]
    brain._enter_evade()
    brain._people = []                               # 사람이 지나갔다
    open_space = np.full(config.LIDAR_RESOLUTION, 3.0)
    for _ in range(int(config.EVADE_CALM_SECONDS / 0.064) + 2):
        if brain.state != mission.EVADE:
            break
        brain._evade((0.0, 0.0, 0.0), open_space, 0.064)
    assert brain.state == mission.APPROACH


def test_evade_gives_up_after_the_time_limit():
    brain = _brain_exploring()
    walker = (1.0, 0.0, -0.2, 0.0, 10)
    brain._people = [walker]
    brain._enter_evade()
    boxed = np.full(config.LIDAR_RESOLUTION, 0.30)
    for _ in range(int(config.EVADE_MAX_SECONDS / 0.064) + 2):
        if brain.state != mission.EVADE:
            break
        brain._evade((0.0, 0.0, 0.0), boxed, 0.064)
    assert brain.state == mission.EXPLORE
