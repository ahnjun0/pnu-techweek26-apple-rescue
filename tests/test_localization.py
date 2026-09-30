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


def test_scan_matching_does_not_slide_along_a_featureless_corridor():
    """평평한 우도장에서는 pose 를 옮기지 않는다 (corridor aliasing).

    ⚠️ 회귀 방지. match() 가 늘 최저 비용 자리를 택하고 **제자리 대비 개선폭을
       안 봤다.** 긴 균일 복도는 진행 방향으로 특징이 없어 비용이 거의 같으므로,
       잡음이 고른 자리로 8틱마다 조금씩 밀려 누적된다.
       실측(corridor2): 곧게 직진하던 중 40초에 오도메트리 오차가 105 cm 로 튀었고,
       위치를 잃자 61초 만에 목표물 0/3 으로 끝났다.
    """
    import numpy as np

    import mapping
    import scanmatch

    # 진행 방향(x)으로 특징이 없는 복도: y = ±0.6 에 긴 벽 두 줄
    grid = mapping.new_map()
    r0, c0 = common.to_cell(-3.0, -0.6)
    r1, c1 = common.to_cell(3.0, 0.6)
    grid[r0:r1, c0:c1] = config.LOG_ODDS_MIN
    for y in (-0.6, 0.6):
        wr, wc0 = common.to_cell(-3.0, y)
        _, wc1 = common.to_cell(3.0, y)
        grid[wr, min(wc0, wc1):max(wc0, wc1)] = config.LOG_ODDS_MAX

    field = scanmatch.likelihood_field(grid)
    assert field is not None

    # 그 복도 한가운데서 벽 두 줄을 보는 스캔
    pose = (0.0, 0.0, 0.0)
    angles = common.lidar_angles()
    ranges = np.full(config.LIDAR_RESOLUTION, np.inf)
    for i, a in enumerate(angles):
        if abs(math.sin(a)) > 1e-3:
            d = 0.6 / abs(math.sin(a))
            if d <= config.LIDAR_MAX_RANGE:
                ranges[i] = d

    fixed, moved = scanmatch.match(pose, ranges, field)
    assert moved <= common.cells_to_metres(1), \
        f"특징 없는 복도에서 pose 를 밀면 안 된다: {moved:.3f} m 옮김 -> {fixed}"


def test_scan_matching_fixes_sideways_but_not_along_the_corridor():
    """옆으로는 고치고, 진행 방향으로는 밀지 않는다.

    ⚠️ 회귀 방지. 이득 검사가 **벡터 전체** 를 통과시켰다. 복도에서 옆으로 어긋나
       있으면 그 성분에는 근거가 있어 합계가 문턱을 넘는데, 그때 근거가 없는
       진행 방향 성분까지 같이 적용된다. 0.128초마다 조금씩, 156번이면 1 m 다.
       실측(corridor2): 20초에 오차 0.2 cm → 40초에 100.3 cm (= 정확히 20칸),
       지도가 망가져 61초에 0/3. 위 검사는 로봇을 복도 **정중앙** 에 두므로
       전체 이득이 0 이어서 이 경우를 잡지 못했다.
    """
    import numpy as np

    import mapping
    import scanmatch

    grid = mapping.new_map()
    r0, c0 = common.to_cell(-3.0, -0.6)
    r1, c1 = common.to_cell(3.0, 0.6)
    grid[r0:r1, c0:c1] = config.LOG_ODDS_MIN
    for y in (-0.6, 0.6):
        wr, wc0 = common.to_cell(-3.0, y)
        _, wc1 = common.to_cell(3.0, y)
        grid[wr, min(wc0, wc1):max(wc0, wc1)] = config.LOG_ODDS_MAX

    field = scanmatch.likelihood_field(grid)
    assert field is not None

    # 스캔은 복도 정중앙에서 찍고, pose 만 옆(y)으로 어긋나 있다고 주장한다.
    off = 0.10
    angles = common.lidar_angles()
    ranges = np.full(config.LIDAR_RESOLUTION, np.inf)
    for i, a in enumerate(angles):
        if abs(math.sin(a)) > 1e-3:
            d = 0.6 / abs(math.sin(a))
            if d <= config.LIDAR_MAX_RANGE:
                ranges[i] = d

    x0 = 0.0
    fixed, _moved = scanmatch.match((x0, off, 0.0), ranges, field)
    # 옆으로는 중앙(y=0) 쪽으로 고쳐야 한다
    assert abs(fixed[1]) < off, \
        f"옆으로 어긋난 것을 못 고쳤다: y {off} -> {fixed[1]}"
    # 진행 방향으로는 근거가 없으니 움직이지 않아야 한다
    assert abs(fixed[0] - x0) <= common.cells_to_metres(1), \
        f"진행 방향으로 근거 없이 밀렸다: x {x0} -> {fixed[0]}"


def test_scan_matching_is_not_dragged_back_by_unmapped_space_ahead():
    """앞이 아직 지도에 없다는 이유로 뒤로 끌려가면 안 된다.

    ⚠️ 회귀 방지. likelihood_field 는 distanceTransform(~occupied) 이라 **미탐색
       칸을 "벽에서 먼 곳"** 으로 계산한다. 앞쪽 벽이 아직 지도에 없으면 그쪽
       스캔점의 비용이 높고, pose 를 뒤로 밀면 이미 지도에 있는 벽 위로 옮겨져
       비용이 내려간다 — **진짜 이득이 있는** 후진 편향이라 성분별 이득 검사로도
       안 걸린다. 실측(corridor2): 20~40초에 로봇은 앞으로 0.70 m 갔는데 추정은
       뒤로 0.75 m 가서 오차 145.6 cm, 61초에 0/3.
       → known_cells 로 **아는 칸에 떨어진 점만** 쓰면 사라진다.
    """
    import numpy as np

    import mapping
    import scanmatch

    grid = mapping.new_map()
    # 복도 벽은 x = -3..3 에 걸쳐 실제로 존재하지만,
    # **지도에는 로봇 뒤쪽(x <= 0)만** 들어와 있다.
    for y in (-0.6, 0.6):
        wr, wc0 = common.to_cell(-3.0, y)
        _, wc_mid = common.to_cell(0.0, y)
        grid[wr, min(wc0, wc_mid):max(wc0, wc_mid)] = config.LOG_ODDS_MAX
    r0, c0 = common.to_cell(-3.0, -0.6)
    r1, c_mid = common.to_cell(0.0, 0.6)
    grid[min(r0, r1):max(r0, r1), min(c0, c_mid):max(c0, c_mid)] = config.LOG_ODDS_MIN

    field = scanmatch.likelihood_field(grid)
    known = scanmatch.known_cells(grid)
    assert field is not None

    # 로봇은 지도 끝(x = 0) 에 서서 복도 양옆 벽을 본다 (앞쪽은 미탐색)
    angles = common.lidar_angles()
    ranges = np.full(config.LIDAR_RESOLUTION, np.inf)
    for i, a in enumerate(angles):
        if abs(math.sin(a)) > 1e-3:
            d = 0.6 / abs(math.sin(a))
            if d <= config.LIDAR_MAX_RANGE:
                ranges[i] = d

    x0 = 0.0
    fixed, _moved = scanmatch.match((x0, 0.0, 0.0), ranges, field, known=known)
    assert fixed[0] >= x0 - common.cells_to_metres(1), \
        f"미탐색 구간 때문에 뒤로 끌렸다: x {x0} -> {fixed[0]}"
