"""오도메트리 수식이 맞는지 Webots 없이 확인한다.

엔코더 값을 손으로 만들어 넣고, 로봇이 가야 할 곳에 갔는지 본다.
"""

import math

import pytest

import common
import config
import localization


def rad_for(distance):
    """바퀴가 distance [m] 만큼 굴러가는 데 필요한 회전각 [rad]."""
    return distance / config.WHEEL_RADIUS


# --- 나침반 각도 공식 --------------------------------------------------------

@pytest.mark.parametrize("theta", [0.0, 0.7, math.pi / 2, -math.pi / 2, 2.5, -3.0])
def test_heading_from_compass_roundtrip(theta):
    """theta 인 로봇이 볼 북쪽 벡터 (sin t, cos t) 를 넣으면 theta 가 나와야 한다."""
    values = [math.sin(theta), math.cos(theta), 0.0]
    assert localization.heading_from_compass(values) == pytest.approx(theta, abs=1e-9)


def test_compass_north_means_heading_plus_y():
    """북(+y)을 바라보는 로봇에게 북쪽은 자기 정면(+x) 이다 → 벡터 (1, 0)."""
    assert localization.heading_from_compass([1.0, 0.0, 0.0]) == \
        pytest.approx(math.pi / 2)


# --- 직진 -------------------------------------------------------------------

def test_first_update_only_sets_reference():
    """첫 호출은 기준점만 잡고 pose 를 움직이지 않는다."""
    odo = localization.Odometry(0.0, 0.0, 0.0)
    pose = odo.update(rad_for(5.0), rad_for(5.0), dt=0.032)
    assert pose == (0.0, 0.0, 0.0)


def test_straight_one_meter_along_x():
    odo = localization.Odometry(0.0, 0.0, 0.0)
    odo.update(0.0, 0.0, dt=0.032)
    x, y, theta = odo.update(rad_for(1.0), rad_for(1.0), dt=0.032)
    assert x == pytest.approx(1.0)
    assert y == pytest.approx(0.0)
    assert theta == pytest.approx(0.0)


def test_straight_while_facing_north_moves_along_y():
    """+y 를 보고 1 m 가면 y 만 늘어야 한다. 여기가 틀리면 지도가 90도 돌아간다."""
    odo = localization.Odometry(0.0, 0.0, math.pi / 2)
    odo.update(0.0, 0.0, dt=0.032)
    x, y, _ = odo.update(rad_for(1.0), rad_for(1.0), dt=0.032)
    assert x == pytest.approx(0.0, abs=1e-9)
    assert y == pytest.approx(1.0)


def test_backwards():
    odo = localization.Odometry(0.0, 0.0, 0.0)
    odo.update(0.0, 0.0, dt=0.032)
    x, _, _ = odo.update(rad_for(-0.5), rad_for(-0.5), dt=0.032)
    assert x == pytest.approx(-0.5)


# --- 제자리 회전 -------------------------------------------------------------

def test_spin_in_place_does_not_move_position():
    """좌우 바퀴를 반대로 굴리면 제자리에서 돌기만 해야 한다."""
    odo = localization.Odometry(1.0, 2.0, 0.0)
    odo.update(0.0, 0.0, dt=0.032)
    arc = rad_for(0.05)
    x, y, theta = odo.update(-arc, arc, dt=0.032)
    assert (x, y) == pytest.approx((1.0, 2.0))
    assert theta > 0, "오른쪽 바퀴가 더 빨리 굴면 반시계(+)로 돌아야 한다"


def test_encoder_only_quarter_turn():
    """엔코더만으로 90도 회전. 호 길이 = theta * B / 2 를 좌우에 나눠 준다.

    엔코더 전용 경로는 측정 보정값 WHEEL_BASE_ODOM 을 쓴다 (config.py 주석 참고).
    """
    odo = localization.Odometry(0.0, 0.0, 0.0)
    odo.update(0.0, 0.0, dt=0.032)
    arc = rad_for(math.pi / 2 * config.WHEEL_BASE_ODOM / 2.0)
    _, _, theta = odo.update(-arc, arc, dt=0.032)
    assert theta == pytest.approx(math.pi / 2, abs=1e-9)


# --- theta 소스 우선순위 -----------------------------------------------------

def test_compass_wins_over_gyro_and_encoder():
    """나침반이 있으면 엔코더가 뭐라 하든 나침반 각도를 쓴다."""
    odo = localization.Odometry(0.0, 0.0, 0.0)
    odo.update(0.0, 0.0, dt=0.032, compass_values=[0.0, 1.0, 0.0])
    truth = 1.0
    compass = [math.sin(truth), math.cos(truth), 0.0]
    _, _, theta = odo.update(rad_for(0.01), rad_for(0.01), dt=0.032,
                             compass_values=compass, gyro_z=99.0)
    assert theta == pytest.approx(truth)


def test_gyro_is_used_when_compass_is_absent():
    odo = localization.Odometry(0.0, 0.0, 0.0)
    odo.update(0.0, 0.0, dt=0.1)
    _, _, theta = odo.update(0.0, 0.0, dt=0.1, gyro_z=2.0)
    assert theta == pytest.approx(0.2)


def test_gyro_integration_accumulates():
    odo = localization.Odometry(0.0, 0.0, 0.0)
    odo.update(0.0, 0.0, dt=0.1)
    for _ in range(10):
        odo.update(0.0, 0.0, dt=0.1, gyro_z=1.0)
    assert odo.theta == pytest.approx(1.0)


# --- 닫힌 경로: 정사각형 -----------------------------------------------------

def test_square_loop_returns_to_start():
    """1 m 정사각형을 돌면 제자리로 돌아와야 한다 (수식이 맞다면 오차 0)."""
    odo = localization.Odometry(0.0, 0.0, 0.0)
    left = right = 0.0
    odo.update(left, right, dt=0.032)

    straight = rad_for(1.0)
    turn = rad_for(math.pi / 2 * config.WHEEL_BASE_ODOM / 2.0)
    for _ in range(4):
        left += straight
        right += straight
        odo.update(left, right, dt=0.032)
        left -= turn
        right += turn
        odo.update(left, right, dt=0.032)

    assert odo.x == pytest.approx(0.0, abs=1e-9)
    assert odo.y == pytest.approx(0.0, abs=1e-9)
    assert common.wrap_angle(odo.theta) == pytest.approx(0.0, abs=1e-9)


def test_curved_motion_midpoint_beats_naive():
    """곡선 주행에서 중점법이 "시작 각도 그대로" 보다 실제 원호에 가까운가.

    좌우 속도가 다르면 로봇은 원호를 그린다. 그 원호의 정확한 끝점과 비교한다.
    """
    odo = localization.Odometry(0.0, 0.0, 0.0)
    odo.update(0.0, 0.0, dt=0.032)

    d_left, d_right = 0.10, 0.14
    odo.update(rad_for(d_left), rad_for(d_right), dt=0.032)

    # 원호의 정확한 해
    d = 0.5 * (d_left + d_right)
    dtheta = (d_right - d_left) / config.WHEEL_BASE
    radius = d / dtheta
    exact_x = radius * math.sin(dtheta)
    exact_y = radius * (1.0 - math.cos(dtheta))

    naive_x, naive_y = d, 0.0  # 시작 각도로 그냥 직진시킨 경우

    err_mid = math.hypot(odo.x - exact_x, odo.y - exact_y)
    err_naive = math.hypot(naive_x - exact_x, naive_y - exact_y)
    assert err_mid < err_naive


