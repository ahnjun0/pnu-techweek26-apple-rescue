"""경로 추종과 안전 로직을 Webots 없이 확인한다."""

import math

import numpy as np
import pytest

from sar import common
from sar import config
from sar import follower


"""주행기 보조 함수들을 본다. DWA 자체는 tests/test_dwa.py 에서 본다."""


def clear(distance=3.0):
    """사방이 distance 만큼 트여 있는 가짜 LiDAR."""
    return np.full(config.LIDAR_RESOLUTION, distance, dtype=np.float64)


def with_obstacle(angle, distance, width=0.25):
    """angle 방향 ±width 에만 장애물이 있는 가짜 LiDAR."""
    ranges = clear()
    angles = common.lidar_angles()
    ranges[np.abs(common.wrap_angle(angles - angle)) <= width] = distance
    return ranges


def single_ray(angle, distance, background=np.inf):
    """딱 한 광선만 장애물을 본 가짜 LiDAR.

    "띠" 로 놓으면 호(arc) 때문에 정면 거리가 미묘하게 달라져 검사가 헷갈린다.
    거리를 정확히 따질 때는 광선 하나만 쓴다.
    """
    ranges = np.full(config.LIDAR_RESOLUTION, background, dtype=np.float64)
    angles = common.lidar_angles()
    ranges[int(np.argmin(np.abs(common.wrap_angle(angles - angle))))] = distance
    return ranges


def at_offset(ahead, sideways, background=np.inf):
    """로봇 앞 ahead [m], 옆 sideways [m] 지점에 점 하나가 있는 가짜 LiDAR."""
    return single_ray(math.atan2(sideways, ahead), math.hypot(ahead, sideways),
                      background)


# --- 방향별 거리 재기 ---------------------------------------------------------

def test_sector_min_finds_the_obstacle_in_that_direction():
    ranges = with_obstacle(0.0, 0.5)
    assert follower.sector_min(ranges, 0.0, 0.3) == pytest.approx(0.5)


def test_sector_min_ignores_other_directions():
    ranges = with_obstacle(math.pi, 0.5)      # 뒤에만 장애물
    assert follower.sector_min(ranges, 0.0, 0.3) == pytest.approx(3.0)


def test_sector_min_returns_inf_when_nothing_is_there():
    ranges = np.full(config.LIDAR_RESOLUTION, np.inf)
    assert follower.sector_min(ranges, 0.0, 0.3) == math.inf


def test_sector_min_ignores_readings_closer_than_the_sensor_minimum():
    ranges = clear()
    ranges[180] = config.LIDAR_MIN_RANGE / 2.0    # 못 믿을 값
    assert follower.sector_min(ranges, 0.0, 0.3) == pytest.approx(3.0)


def test_sector_min_adapts_to_a_different_lidar_length():
    """당일 로봇의 LiDAR 해상도가 다를 수 있다. 터지지 않아야 한다."""
    ranges = np.full(720, 2.0)
    assert follower.sector_min(ranges, 0.0, 0.3) == pytest.approx(2.0)


# --- 직진 통로 검사 -----------------------------------------------------------

def test_drives_forward_along_a_straight_path():
    """곧은 길에서는 곧장 나아간다 (돌지 않는다).

    ⚠️ 정지 상태에서 한 틱만에 "주행" 이 되지는 않는다. 가속 한계가 있어서
       16 ms 에 낼 수 있는 속도가 0.6 x 0.016 = 0.0096 m/s 뿐이고, 이는 주행
       판정 문턱(0.01)에 못 미친다. 그래서 몇 틱 굴려 놓고 본다.
    """
    speed = 0.0
    for _ in range(5):
        speed, turn, status, _ = follower.step(
            (0.0, 0.0, 0.0), [(0.0, 0.0), (2.0, 0.0)], clear(),
            current_speed=speed)
    assert speed > 0 and abs(turn) < 1e-6 and status == follower.DRIVING


def test_turns_the_short_way_round():
    """왼쪽에 있는 목표에는 왼쪽(+)으로 돌아야 한다."""
    _, turn, _, _ = follower.step((0.0, 0.0, 0.0),
                                  [(0.0, 0.0), (0.0, 2.0)], clear())
    assert turn > 0
    _, turn, _, _ = follower.step((0.0, 0.0, 0.0),
                                  [(0.0, 0.0), (0.0, -2.0)], clear())
    assert turn < 0


def test_reports_arrival_at_the_end_of_the_path():
    speed, turn, status, _ = follower.step((2.0, 0.0, 0.0),
                                           [(0.0, 0.0), (2.0, 0.0)], clear())
    assert (speed, turn) == (0.0, 0.0) and status == follower.ARRIVED


def test_empty_path_means_no_command():
    speed, turn, status, _ = follower.step((0.0, 0.0, 0.0), [], clear())
    assert (speed, turn) == (0.0, 0.0) and status == follower.NO_PATH


def test_does_not_drive_into_an_obstacle_on_the_path():
    """경로보다 안전이 우선이다 — 장애물 쪽으로 전진하지 않는다.

    ⚠️ "정지" 라고 못박지 않는다. 최후의 여유(DWA_SQUEEZE_MARGIN)가 살아 있으면
       느린 후진 같은 다른 길을 찾아낼 수 있고, 그게 멈춰 서는 것보다 낫다.
    """
    ranges = with_obstacle(0.0, config.ROBOT_RADIUS + 0.05)
    speed, _, status, _ = follower.step((0.0, 0.0, 0.0),
                                        [(0.0, 0.0), (2.0, 0.0)], ranges)
    assert speed <= 0.0, f"장애물 쪽으로 전진했다 (v={speed:.3f})"


def test_never_freezes_when_blocked():
    """앞이 막혀도 얼어붙으면 안 된다.

    회전까지 멈추면 비스듬히 걸렸을 때 방향을 못 고쳐 그 자리에서 굳는다.
    ⚠️ 회전 방향까지 못박지 않는다 — 후진을 고르면 같은 방향으로 틀어도
       몸은 반대로 간다. 여기서 보려는 것은 "무언가 한다" 이다.
    """
    ranges = with_obstacle(0.0, config.ROBOT_RADIUS + 0.05)
    speed, turn, status, _ = follower.step((0.0, 0.0, 0.0),
                                           [(0.0, 0.0), (0.0, 2.0)], ranges)
    assert speed <= 0.0, f"장애물 쪽으로 전진했다 (v={speed:.3f})"
    assert abs(turn) > 0.0 or speed < 0.0, "아무것도 하지 않고 굳었다"


def test_speed_never_exceeds_the_configured_maximum():
    for theta in np.linspace(-0.6, 0.6, 13):
        speed, _, _, _ = follower.step((0.0, 0.0, float(theta)),
                                       [(0.0, 0.0), (5.0, 0.0)], clear())
        assert speed <= config.FOLLOW_MAX_SPEED + 1e-9


def test_turn_never_exceeds_the_configured_maximum():
    for theta in np.linspace(-math.pi, math.pi, 37):
        _, turn, _, _ = follower.step((0.0, 0.0, float(theta)),
                                      [(0.0, 0.0), (5.0, 0.0)], clear())
        assert abs(turn) <= config.FOLLOW_MAX_TURN + 1e-9


def test_advance_skips_waypoints_already_passed():
    path = [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0)]
    assert follower.advance(path, (1.0, 0.0, 0.0), 0) == 2


def test_advance_never_goes_past_the_last_waypoint():
    path = [(0.0, 0.0), (1.0, 0.0)]
    assert follower.advance(path, (1.0, 0.0, 0.0), 0) == 1


def test_lookahead_picks_a_point_further_ahead():
    path = [(x / 10.0, 0.0) for x in range(30)]
    target = follower.lookahead_point(path, (0.0, 0.0, 0.0), 0)
    assert common.distance(0.0, 0.0, *target) == \
        pytest.approx(config.FOLLOW_LOOKAHEAD, abs=1e-6)


def test_lookahead_follows_the_line_not_just_the_corners():
    """경로를 몇 개 점으로 줄여 놓아도 선분 위를 따라가야 한다.

    꼭짓점만 보면 "저 멀리 꺾인 뒤의 점" 을 향해 직진해서 모퉁이를 잘라 버린다.
    실제로 이것 때문에 로봇이 문틀을 긁었다.
    """
    path = [(0.0, 0.0), (2.0, 0.0), (2.0, 2.0)]     # ㄱ 자 모양, 점 3개
    target = follower.lookahead_point(path, (0.0, 0.0, 0.0), 0)
    assert target == pytest.approx((config.FOLLOW_LOOKAHEAD, 0.0), abs=1e-6), \
        "첫 선분 위에 있어야지, 꺾인 뒤의 점을 보면 안 된다"


def test_lookahead_rounds_the_corner():
    """모퉁이 직전에서는 목표점이 모퉁이 너머로 넘어가야 부드럽게 돈다."""
    path = [(0.0, 0.0), (1.0, 0.0), (1.0, 2.0)]
    target = follower.lookahead_point(path, (0.9, 0.0, 0.0), 0)
    assert target[0] == pytest.approx(1.0, abs=1e-6)
    assert target[1] > 0.0


def test_closest_point_is_on_the_line_not_a_corner():
    _, point = follower.closest_on_path([(0.0, 0.0), (4.0, 0.0)],
                                        (2.0, 0.5, 0.0))
    assert point == pytest.approx((2.0, 0.0))


def test_follower_steers_back_onto_the_path_when_pushed_off():
    """경로에서 옆으로 밀려나면 다시 경로 쪽으로 붙어야 한다."""
    path = [(0.0, 0.0), (4.0, 0.0)]
    _, turn, _, _ = follower.step((2.0, -0.3, 0.0), path, clear())
    assert turn > 0, "경로가 왼쪽에 있으니 왼쪽으로 붙어야 한다"


def test_lookahead_falls_back_to_the_last_point():
    path = [(0.0, 0.0), (0.1, 0.0)]
    assert follower.lookahead_point(path, (0.0, 0.0, 0.0), 0) == (0.1, 0.0)


# --- 범용성 -------------------------------------------------------------------

def test_a_slower_robot_obeys_its_own_limit(monkeypatch):
    monkeypatch.setattr(config, "FOLLOW_MAX_SPEED", 0.05)
    speed, _, _, _ = follower.step((0.0, 0.0, 0.0),
                                   [(0.0, 0.0), (5.0, 0.0)], clear())
    assert speed <= 0.05 + 1e-9


def test_nearest_obstacle_finds_direction_and_distance():
    distance, angle = follower.nearest_obstacle(single_ray(0.8, 0.4, background=3.0))
    assert distance == pytest.approx(0.4)
    assert angle == pytest.approx(0.8, abs=0.02)


def test_nearest_obstacle_of_empty_space():
    distance, _ = follower.nearest_obstacle(
        np.full(config.LIDAR_RESOLUTION, np.inf))
    assert distance == math.inf


def pinned_at(angle, distance=None):
    """angle 방향에 몸이 닿을 만큼 가까운 벽이 있는 가짜 LiDAR."""
    distance = (config.SAFETY_PINNED_DISTANCE - 0.02 if distance is None
                else distance)
    return with_obstacle(angle, distance, width=0.3)


def test_is_pinned_only_when_really_close():
    assert follower.is_pinned(pinned_at(0.0))
    assert not follower.is_pinned(clear(1.0))
    assert not follower.is_pinned(
        with_obstacle(0.0, config.SAFETY_PINNED_DISTANCE + 0.1))


def test_pinned_distance_is_smaller_than_the_stop_distance():
    """"눌렸다" 는 "멈춰야 한다" 보다 더 심각한 상태여야 한다."""
    assert config.SAFETY_PINNED_DISTANCE < config.SAFETY_STOP_DISTANCE


def test_escape_backs_away_from_a_wall_in_front():
    speed, _ = follower.escape_command(pinned_at(0.0))
    assert speed < 0.0


def test_escape_drives_forward_from_a_wall_behind():
    speed, _ = follower.escape_command(pinned_at(math.pi))
    assert speed > 0.0


@pytest.mark.parametrize("side,expected_turn", [
    (math.pi / 2, +1),     # 왼쪽이 벽 → 후진하며 코를 왼쪽으로 = 몸은 오른뒤로 빠진다
    (-math.pi / 2, -1),    # 오른쪽이 벽 → 반대
])
def test_escape_backs_out_from_a_wall_on_the_side(side, expected_turn):
    """차동구동은 옆으로 못 간다. 후진하면서 엉덩이를 트인 쪽으로 돌려 빠져나온다."""
    speed, turn = follower.escape_command(pinned_at(side))
    assert speed < 0.0, "옆으로 눌렸어도 전진하면 더 깊이 박힌다"
    assert turn * expected_turn > 0


def test_escape_turns_decisively_when_cornered():
    """앞뒤가 대칭으로 막히면 "도망갈 방향" 이 0 이 되어 아무것도 안 할 수 있다.

    그러면 구석에서 영원히 굳는다. 좌우 중 트인 쪽으로 확실히 돌아야 한다.
    """
    ranges = np.minimum(pinned_at(0.0), pinned_at(math.pi))
    # 오른쪽(-90°)에 0.5 m 벽을 둔다 → 왼쪽이 더 트였다
    right = np.abs(common.wrap_angle(common.lidar_angles() + math.pi / 2)) < 0.5
    ranges[right] = 0.5

    speed, turn = follower.escape_command(ranges)
    assert speed == 0.0
    assert turn > 0.0, "왼쪽이 더 트였으니 왼쪽으로 돌아야 한다"

    # 반대로 놓으면 반대로 돌아야 한다
    ranges = np.minimum(pinned_at(0.0), pinned_at(math.pi))
    left = np.abs(common.wrap_angle(common.lidar_angles() - math.pi / 2)) < 0.5
    ranges[left] = 0.5
    assert follower.escape_command(ranges)[1] < 0.0


def test_escape_does_not_reverse_into_a_wall_behind():
    """앞이 막혔다고 뒤로 빼다가 뒤쪽 벽을 들이받으면 안 된다.

    실제로 후진 중에 장애물까지 1.8 cm 까지 다가갔다 (시각 114.8s 에 측정).
    """
    ranges = np.minimum(pinned_at(0.0), pinned_at(math.pi))   # 앞도 뒤도 막힘
    speed, turn = follower.escape_command(ranges)
    assert speed == 0.0, "앞뒤가 다 막혔으면 움직이지 말고 돌아야 한다"
    assert abs(turn) > 0.0


def test_escape_goes_forward_when_only_the_rear_is_blocked():
    speed, _ = follower.escape_command(pinned_at(math.pi))
    assert speed > 0.0


def test_travel_clearance_measures_room_to_move():
    """로봇을 원으로 보고 "몸이 닿기까지 갈 수 있는 거리" 를 잰다."""
    ahead = single_ray(0.0, 0.5, background=3.0)
    assert follower.travel_clearance(ahead) == \
        pytest.approx(0.5 - config.ROBOT_RADIUS, abs=0.01)
    assert follower.travel_clearance(ahead, backward=True) > 1.0


def test_travel_clearance_ignores_what_slides_past_the_side():
    """옆으로 반지름보다 멀리 비켜 있으면 똑바로 가는 동안 절대 안 닿는다.

    이걸 안 빼면, 옆을 스쳐 지나갈 뿐인 벽 때문에 "뒤가 막혔다" 고 판단해서
    옆이 눌린 상황에서 로봇이 아예 못 움직이게 된다.
    """
    beside = at_offset(-0.05, config.ROBOT_RADIUS + 0.02, background=3.0)
    assert follower.travel_clearance(beside, backward=True) > 1.0


def test_travel_clearance_is_zero_when_already_overlapping():
    """몸 안쪽에 들어와 있으면 그 방향으로는 갈 수 없다.

    LiDAR 는 LIDAR_MIN_RANGE 보다 가까운 것은 못 보므로,
    "몸에 닿았다" 를 나타낼 수 있는 구간은 minRange ~ ROBOT_RADIUS 뿐이다.
    """
    assert config.LIDAR_MIN_RANGE < config.ROBOT_RADIUS, \
        "LiDAR 최소거리가 로봇 반경보다 크면 접촉을 아예 못 본다"
    touching = single_ray(0.0, config.LIDAR_MIN_RANGE + 0.004, background=3.0)
    assert follower.travel_clearance(touching) == 0.0


def test_escape_always_moves_when_one_side_is_open():
    """한쪽만 눌렸으면 반드시 "움직이는" 명령이어야 한다.

    제자리 회전만 시키면, 회전이 물리적으로 막힌 상황에서 영원히 갇힌다.
    실제로 벽 속에 0.08 m 박힌 상태에서 같은 회전 명령만 260 초를 반복했다.
    """
    for angle in np.linspace(-math.pi, math.pi, 37):
        speed, _ = follower.escape_command(pinned_at(float(angle)))
        assert abs(speed) > 0.0, f"{math.degrees(angle):.0f}° 에서 전진도 후진도 안 한다"


def test_escape_never_exceeds_the_speed_limits():
    for angle in np.linspace(-math.pi, math.pi, 25):
        speed, turn = follower.escape_command(pinned_at(float(angle)))
        assert abs(speed) <= config.FOLLOW_MAX_SPEED + 1e-9
        assert abs(turn) <= config.FOLLOW_MAX_TURN + 1e-9


def test_escape_backs_off_when_nothing_is_nearby():
    """가까운 데 아무것도 없는데 못 움직인다면 바퀴가 헛도는 것이다. 물러난다."""
    speed, turn = follower.escape_command(clear(3.0))
    assert speed < 0.0 and turn == 0.0


def test_escape_handles_empty_readings():
    speed, turn = follower.escape_command(
        np.full(config.LIDAR_RESOLUTION, np.inf))
    assert abs(speed) <= config.FOLLOW_MAX_SPEED
    assert abs(turn) <= config.FOLLOW_MAX_TURN


def test_a_single_stray_ray_is_not_being_pinned():
    """⚠️ 회귀 방지: 광선 하나가 짧게 나왔다고 눌린 것이 아니다.

    실측(한 판 전체): 0.2 m 이내 광선이 1개뿐이던 30틱에서, 사람과의 실제
    거리는 중앙값 339 cm 였다 — 아무것도 없었다. 진짜로 눌리면 몸 옆으로
    광선이 여러 개 걸린다 (2개 이상이면 실제 거리가 30~36 cm).
    이걸 안 걸렀더니 멀쩡히 달리다 1초씩 후진하는 일이 한 판에 세 번 났다.
    """
    lone = clear()
    lone[len(lone) // 3] = config.SAFETY_PINNED_DISTANCE - 0.05
    assert not follower.is_pinned(lone), "광선 하나짜리 잡음에 속았다"

    real = with_obstacle(0.0, config.SAFETY_PINNED_DISTANCE - 0.05, width=0.3)
    assert follower.is_pinned(real), "진짜 눌림을 놓쳤다"


def test_standing_still_is_penalised_unless_people_are_near():
    """가만히 있는 후보에는 벌점이 있다 — 단, 사람이 보이면 끈다.

    ⚠️ 속도 0 후보는 궤적이 "점" 이라 장애물 여유 점수가 늘 만점이라, 목표가 멀면
       "가만히 있기" 가 이긴다. 계측: 사람 없는 지도에서 출력 (0,0) 인 틱이
       전체의 39.6%(356.7초)였고 그중 임무 분기에서 나온 것은 4틱뿐이었다.
       반대로 사람 근처에서는 가만히 있는 편이 낫다 (억지로 움직이면 상대운동이
       늘어 닿은시간이 12.8 -> 23.0초로 나빠졌다). 그래서 끄고 켠다.
    """
    assert config.DWA_IDLE_PENALTY > 0, "이 시험은 벌점이 켜져 있을 때의 것이다"

    ranges = np.full(config.LIDAR_RESOLUTION, 3.0)
    pose = (0.0, 0.0, 0.0)
    path = [(0.0, 0.0), (2.0, 0.0)]          # 앞이 트였고 목표도 앞이다

    moving, _, _, _ = follower.step(pose, path, ranges, 0,
                                    current_speed=0.0, current_turn=0.0, dt=0.064)
    assert moving > 0.01, "트인 곳에서 목표가 앞이면 움직여야 한다"

    # 사람이 보이면 같은 상황에서도 가만히 있을 수 있어야 한다 (벌점 꺼짐)
    idle_ok, _, _, _ = follower.step(pose, path, ranges, 0,
                                     current_speed=0.0, current_turn=0.0,
                                     dt=0.064, allow_idle=True)
    assert idle_ok >= 0.0, "사람이 있을 때도 유효한 명령이 나와야 한다"


def test_squeeze_opens_when_only_forward_is_blocked():
    """전진할 데가 없으면 좁은 여유 비상구가 열려야 한다.

    ⚠️ 회귀 방지. 비상구 조건이 "안전한 후보가 없다"(safe.any()) 였는데, 제자리
       후보는 궤적이 "점" 이라 여유가 늘 만점이라 그 조건을 언제나 만족시킨다.
       그래서 전진할 데가 없는데도 비상구가 **영영 발동하지 않았다.**
       실측: comb 지도에서 최근접 0.34 m(DWA 요구 0.33 m)인 자리에 200초 갇혀
       제자리 회전만 했다. 비상구는 코드에 있었지만 열린 적이 없었다.
    """
    # 정면에 평소 여유로는 못 가고 좁은 여유로는 갈 수 있는 벽을 세운다
    # ⚠️ 벽 위치를 마진에서 유도하면, 마진을 줄일 때 벽이 SAFETY_STOP_DISTANCE
    #    안쪽으로 미끄러진다. 그러면 "이미 정지거리 안이면 전진으로 파고들지
    #    않는다" 는 규칙이 **옳게** 거부하는데 검사는 실패한다 — 동작이 아니라
    #    검사의 전제가 무너진 것이다 (DWA_CLEARANCE_MARGIN 0.20 -> 0.15 때 겪었다).
    #    그래서 "좁지만 **정지거리 밖**" 이라는 상황을 명시적으로 세운다.
    wall = (config.SAFETY_STOP_DISTANCE
            + config.ROBOT_RADIUS + config.DWA_CLEARANCE_MARGIN) / 2.0
    assert config.SAFETY_STOP_DISTANCE < wall, "정지거리 안이면 전진 금지가 맞다"
    assert (config.ROBOT_RADIUS + config.DWA_SQUEEZE_MARGIN
            < wall < config.ROBOT_RADIUS + config.DWA_CLEARANCE_MARGIN)

    angles = common.lidar_angles()
    ranges = np.full(config.LIDAR_RESOLUTION, 3.0)
    ranges[np.abs(angles) < 0.25] = wall

    speed, _turn, _status, _i = follower.step(
        (0.0, 0.0, 0.0), [(0.0, 0.0), (2.0, 0.0)], ranges, 0,
        current_speed=0.0, current_turn=0.0, dt=0.064)
    assert speed > 0.01, \
        "좁은 여유로는 갈 수 있는데 제자리에 머물면 안 된다 (비상구가 안 열렸다)"


def test_far_people_are_not_predicted():
    """멀리 있는 사람의 미래 위치는 계산하지 않는다 (비용만 든다).

    ⚠️ 회귀 방지. 예측을 켠 뒤 comb0/comb2 가 1100초 벽시계로 완주를 못 했다
       (680초까지밖에 못 감). 사람이 **없는** 월드에서도 유령 검출이 0.10/틱
       남아 있어 매 틱 예측 점을 만들고 DWA 장애물 집합을 불렸기 때문이다.
       예측 시간 안에 우리 궤적과 만날 수 없는 사람은 계산이 낭비다.
    """
    assert config.DWA_PERSON_REACH > 0, "이 시험은 필터가 켜져 있을 때의 것이다"

    ranges = np.full(config.LIDAR_RESOLUTION, 5.0)
    pose = (0.0, 0.0, 0.0)
    path = [(0.0, 0.0), (2.0, 0.0)]

    far = config.DWA_PERSON_REACH + 2.0
    speed_far, _, _, _ = follower.step(
        pose, path, ranges, 0, current_speed=0.0, current_turn=0.0, dt=0.064,
        people=[(far, 0.0, -0.5, 0.0)])
    speed_none, _, _, _ = follower.step(
        pose, path, ranges, 0, current_speed=0.0, current_turn=0.0, dt=0.064,
        people=[])
    assert abs(speed_far - speed_none) < 1e-9, \
        f"{far:.1f} m 밖의 사람이 명령을 바꾸면 안 된다"
