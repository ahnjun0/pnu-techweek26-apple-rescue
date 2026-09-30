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
