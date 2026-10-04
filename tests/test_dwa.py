"""DWA 주행기를 Webots 없이 확인한다.

DWA 는 (전진속도, 회전속도) 후보를 뿌려 몇 초 앞을 모의주행해 보고 고른다.
여기서 보려는 것: 부딪히는 후보를 고르지 않는가 / 가속 한계를 지키는가 /
목표 쪽으로 가는가 / 막혔을 때 얼어붙지 않는가.
"""

import math

import numpy as np
import pytest

import common
import config
import follower


def clear(distance=3.0):
    return np.full(config.LIDAR_RESOLUTION, distance, dtype=np.float64)


def wall_at(angle, distance, width=0.4):
    ranges = clear()
    angles = common.lidar_angles()
    ranges[np.abs(common.wrap_angle(angles - angle)) <= width] = distance
    return ranges


# --- 장애물 점 만들기 ---------------------------------------------------------

def test_obstacle_points_are_in_robot_coordinates():
    ranges = np.full(config.LIDAR_RESOLUTION, np.inf)
    ranges[180] = 1.0                       # 정면 (측정으로 확정된 인덱스)
    points = follower.obstacle_points(ranges, stride=1)
    assert len(points) == 1
    assert points[0] == pytest.approx((1.0, 0.0), abs=1e-6)


def test_obstacle_points_drop_unusable_readings():
    ranges = np.full(config.LIDAR_RESOLUTION, np.inf)
    assert len(follower.obstacle_points(ranges)) == 0


def test_obstacle_points_thin_out_with_stride():
    dense = follower.obstacle_points(clear(), stride=1)
    sparse = follower.obstacle_points(clear(), stride=6)
    assert len(sparse) < len(dense)


# --- 동적 창 (가속 한계) ------------------------------------------------------

def test_window_respects_acceleration_limits():
    """한 번의 결정으로 속도가 확 튀면 주행이 거칠어진다.

    ⚠️ 한계는 **제어 주기** 가 아니라 **결정 지평** (DWA_ACCEL_HORIZON) 분이다.
       한 틱(16 ms)으로 잡으면 폭이 0.0096 m/s 라 후진 중에 전진 후보가 창에
       없어 후진 벌점이 죽는다 (config 참고).
    """
    h = config.DWA_ACCEL_HORIZON
    speeds, turns = follower.dynamic_window(0.0, 0.0, dt=0.032)
    assert speeds.max() <= config.DWA_MAX_ACCEL * h + 1e-9
    assert turns.max() <= config.DWA_MAX_ANG_ACCEL * h + 1e-9


def test_window_reaches_forward_from_deepest_reverse():
    """후진 벌점이 작동할 수 있어야 한다 — 창에 전진 후보가 있어야 한다.

    실측 근거: 이것이 깨져 있어 comb0 에서 틱의 70.5% 가 후진이었다.
    """
    speeds, _ = follower.dynamic_window(-config.DWA_REVERSE_SPEED, 0.0,
                                        dt=config.TIME_STEP / 1000.0)
    assert (speeds > config.DWA_IDLE_SPEED).any(), \
        "가장 깊은 후진에서 창이 전진에 닿지 않는다 — 후진 벌점이 죽는다"


def test_window_never_exceeds_the_robot_limits():
    speeds, turns = follower.dynamic_window(config.FOLLOW_MAX_SPEED,
                                            config.FOLLOW_MAX_TURN, dt=1.0)
    assert speeds.max() <= config.FOLLOW_MAX_SPEED + 1e-9
    assert turns.max() <= config.FOLLOW_MAX_TURN + 1e-9
    assert turns.min() >= -config.FOLLOW_MAX_TURN - 1e-9


def test_window_allows_a_little_reverse():
    speeds, _ = follower.dynamic_window(0.0, 0.0, dt=1.0)
    assert speeds.min() < 0.0, "후진 후보가 있어야 끼였을 때 빠져나온다"


# --- 고르기 -------------------------------------------------------------------

def test_drives_toward_a_goal_straight_ahead():
    speed, turn, status, _ = follower.step((0.0, 0.0, 0.0),
                                           [(0.0, 0.0), (2.0, 0.0)], clear(),
                                           current_speed=0.1, dt=0.1)
    assert speed > 0.0
    assert abs(turn) < 0.2
    assert status == follower.DRIVING


@pytest.mark.parametrize("goal,expected_sign", [((0.0, 2.0), +1), ((0.0, -2.0), -1)])
def test_turns_toward_the_goal(goal, expected_sign):
    _, turn, _, _ = follower.step((0.0, 0.0, 0.0), [(0.0, 0.0), goal], clear(),
                                  current_speed=0.05, dt=0.1)
    assert turn * expected_sign > 0


def test_never_picks_a_trajectory_that_hits_a_wall():
    """제일 중요한 성질. 앞이 막혔는데도 목표 쪽이라고 그대로 가면 안 된다."""
    ranges = wall_at(0.0, 0.30, width=0.8)
    speed, _, _, _ = follower.step((0.0, 0.0, 0.0), [(0.0, 0.0), (3.0, 0.0)],
                                   ranges, current_speed=config.FOLLOW_MAX_SPEED,
                                   dt=0.1)
    # 1.2 초 모의주행 중 몸이 닿지 않을 만큼만 허용되어야 한다
    reach = speed * config.DWA_SIM_TIME
    assert reach < 0.30 - config.ROBOT_RADIUS + 1e-6, \
        f"전진 {speed:.3f} m/s 로 {config.DWA_SIM_TIME}s 가면 벽을 지나친다"


def test_prefers_the_roomier_side():
    """양쪽으로 갈 수 있으면 넓은 쪽을 고른다 — 안전거리가 점수에 들어 있다."""
    ranges = clear()
    angles = common.lidar_angles()
    ranges[np.abs(common.wrap_angle(angles - 0.6)) < 0.3] = 0.45   # 왼쪽이 좁다
    _, turn, _, _ = follower.step((0.0, 0.0, 0.0), [(0.0, 0.0), (2.0, 0.0)],
                                  ranges, current_speed=0.1, dt=0.1)
    assert turn <= 0.0, "좁은 왼쪽으로 꺾으면 안 된다"


def test_spins_instead_of_freezing_when_everything_is_blocked():
    """사방이 막혀도 얼어붙으면 안 된다. 원형 로봇은 제자리 회전이 언제나 안전하다."""
    ranges = np.full(config.LIDAR_RESOLUTION, config.ROBOT_RADIUS + 0.01)
    speed, turn, status, _ = follower.step((0.0, 0.0, 0.0),
                                           [(0.0, 0.0), (2.0, 0.0)], ranges,
                                           current_speed=0.0, dt=0.1)
    assert speed == 0.0
    assert abs(turn) > 0.0
    assert status == follower.STOPPED


def test_prefers_turning_around_over_reversing():
    """목표가 뒤에 있으면 돌아서야 한다. 뒤로 기어가면 한없이 느리다.

    실제로 복귀 중에 뒤로만 기어가다 100초에 1.6 m 밖에 못 가 시간 초과로 실패했다.
    """
    speed, turn, _, _ = follower.step((0.0, 0.0, 0.0), [(0.0, 0.0), (-2.0, 0.0)],
                                      clear(), current_speed=0.0, dt=0.1)
    assert speed >= 0.0, "목표가 뒤에 있다고 후진하면 안 된다"
    assert abs(turn) > 0.0, "돌아서야 한다"


def test_still_reverses_when_that_is_the_only_safe_option():
    """앞이 완전히 막혔으면 후진이라도 해야 한다 (벌점이 금지는 아니다)."""
    ranges = wall_at(0.0, config.ROBOT_RADIUS + 0.02, width=1.2)
    speed, _, _, _ = follower.step((0.0, 0.0, 0.0), [(0.0, 0.0), (2.0, 0.0)],
                                   ranges, current_speed=0.0, dt=1.0)
    assert speed <= 0.0


def test_reports_arrival():
    _, _, status, _ = follower.step((2.0, 0.0, 0.0), [(0.0, 0.0), (2.0, 0.0)],
                                    clear(), dt=0.1)
    assert status == follower.ARRIVED


def test_no_path_means_no_command():
    assert follower.step((0.0, 0.0, 0.0), [], clear(), dt=0.1)[:2] == (0.0, 0.0)


def test_speed_and_turn_stay_within_limits_in_many_situations():
    for angle in np.linspace(-math.pi, math.pi, 13):
        for distance in (0.3, 0.6, 1.5):
            speed, turn, _, _ = follower.step(
                (0.0, 0.0, 0.0), [(0.0, 0.0), (2.0, 0.0)],
                wall_at(float(angle), distance),
                current_speed=0.1, current_turn=0.0, dt=0.1)
            assert -config.DWA_REVERSE_SPEED - 1e-9 <= speed \
                <= config.FOLLOW_MAX_SPEED + 1e-9
            assert abs(turn) <= config.FOLLOW_MAX_TURN + 1e-9


def test_commands_change_smoothly_over_ticks():
    """가속 한계 덕에 명령이 한 번에 조금씩만 바뀌어야 한다.

    ⚠️ 경계가 **제어 주기** 분에서 **결정 지평** 분으로 느슨해졌다. 이것은 교환이다:
       틱당 0.0096 m/s 였던 변화 한계가 0.08 m/s 가 되었다. 그 대가로 후진 벌점이
       살아났다 (config.DWA_ACCEL_HORIZON 의 주석 참고). 실행 궤적이 평가 궤적과
       같아야 하므로 출력을 따로 묶지는 않는다.
    """
    ranges = clear()
    path = [(0.0, 0.0), (2.0, 0.0)]
    speed = turn = 0.0
    dt = 0.032
    for _ in range(20):
        new_speed, new_turn, _, _ = follower.step((0.0, 0.0, 0.0), path, ranges,
                                                  current_speed=speed,
                                                  current_turn=turn, dt=dt)
        assert abs(new_speed - speed) <= (config.DWA_MAX_ACCEL
                                          * config.DWA_ACCEL_HORIZON) + 1e-9
        assert abs(new_turn - turn) <= (config.DWA_MAX_ANG_ACCEL
                                        * config.DWA_ACCEL_HORIZON) + 1e-9
        speed, turn = new_speed, new_turn


def test_dwa_is_fast_enough_for_one_tick():
    import time
    ranges = clear()
    path = [(0.0, 0.0), (2.0, 0.0)]
    start = time.perf_counter()
    for _ in range(30):
        follower.step((0.0, 0.0, 0.0), path, ranges, current_speed=0.1, dt=0.032)
    per_tick = (time.perf_counter() - start) / 30 * 1000
    assert per_tick < config.TIME_STEP / 2, \
        f"한 틱에 {per_tick:.1f} ms 는 예산 {config.TIME_STEP} ms 에 비해 너무 크다"



def test_only_picks_commands_the_wheels_can_do():
    """바퀴가 못 내는 명령을 고르면, 실행 단계에서 줄어들어 궤적이 달라진다.

    실제로 그 때문에 추종 오차가 1.0 cm 에서 6.5 cm (최대 37.9 cm) 로 뛰었다.
    """
    limit = config.MAX_WHEEL_SPEED
    half = config.WHEEL_BASE / 2.0
    for goal in [(2.0, 0.0), (0.0, 2.0), (0.0, -2.0), (-2.0, 0.0)]:
        speed, turn, _, _ = follower.step((0.0, 0.0, 0.0), [(0.0, 0.0), goal],
                                          clear(), current_speed=0.1, dt=1.0)
        left = (speed - turn * half) / config.WHEEL_RADIUS
        right = (speed + turn * half) / config.WHEEL_RADIUS
        assert max(abs(left), abs(right)) <= limit + 1e-6, \
            f"바퀴가 못 내는 명령을 골랐다 (v={speed:.3f}, w={turn:.3f})"


def test_clip_to_wheels_keeps_turn_radius():
    """바퀴 한계를 넘으면 줄이되, 회전 반경(=궤적 모양)은 그대로여야 한다."""
    limit = config.MAX_WHEEL_SPEED * config.WHEEL_RADIUS
    speeds = np.array([0.0, limit * 0.5, limit])
    turns = np.array([config.FOLLOW_MAX_TURN, 0.0, config.FOLLOW_MAX_TURN])

    out_v, out_w = follower.clip_to_wheels(speeds, turns)

    half = config.WHEEL_BASE / 2.0
    biggest = np.maximum(np.abs(out_v - out_w * half),
                         np.abs(out_v + out_w * half)) / config.WHEEL_RADIUS
    assert (biggest <= config.MAX_WHEEL_SPEED + 1e-9).all(), "바퀴 한계를 넘었다"

    assert out_v[1] == pytest.approx(speeds[1]), "낼 수 있는 값은 건드리면 안 된다"
    # 3번째는 줄어들되, v/w 비율(=회전 반경)은 보존돼야 한다
    assert out_v[2] < speeds[2], "한계를 넘는데 안 줄었다"
    assert out_v[2] / out_w[2] == pytest.approx(speeds[2] / turns[2])


def test_dwa_can_still_turn_hard_while_moving():
    """⚠️ 회귀 방지: 못 내는 후보를 "버리면" 달리면서 크게 돌 수 없어 갇힌다.

    바퀴 상한이 0.22 m/s 라, 0.20 m/s 로 달릴 땐 회전이 0.25 rad/s 까지밖에
    안 남는다. 못 내는 후보를 버리던 구현에서는 로봇이 좁은 구석에서 벽에
    눌렸다 빠져나오기를 반복하며 200 초 동안 갇혔다.
    비례로 줄이면 "느리지만 크게 도는" 후보가 살아남는다.
    """
    speeds, turns = follower.dynamic_window(config.FOLLOW_MAX_SPEED, 0.0, dt=1.0)
    grid_v, grid_w = np.meshgrid(speeds, turns, indexing="ij")
    out_v, out_w = follower.clip_to_wheels(grid_v.ravel(), grid_w.ravel())

    moving_and_turning = (out_v > 0.05) & (np.abs(out_w) > 0.8)
    assert moving_and_turning.any(), \
        "달리면서 크게 도는 후보가 하나도 없다 — 좁은 구석에서 갇힌다"


def test_spins_the_short_way_toward_the_goal():
    """⚠️ 회귀 방지: 앞이 막혔을 때 '트인 쪽' 이 아니라 '가고 싶은 쪽' 으로 돈다.

    한때 좌우 중 트인 쪽으로 돌았더니, 목표가 왼쪽 99도에 있는데 오른쪽이
    더 트였다는 이유로 258도를 돌아갔다 (사람을 마주칠 때마다 한 바퀴씩 돌았다).
    제자리 회전은 원형 로봇에게 어느 쪽이든 안전하므로 짧은 쪽이 언제나 낫다.
    """
    # 목표는 왼쪽(+y). 오른쪽이 더 트여 있어도 왼쪽으로 돌아야 한다.
    angles = common.lidar_angles()
    ranges = np.full(config.LIDAR_RESOLUTION, config.ROBOT_RADIUS + 0.02)
    ranges[np.abs(angles + math.pi / 2) < 0.6] = 3.0      # 오른쪽만 뻥 뚫림

    speed, turn, status, _ = follower.step(
        (0.0, 0.0, 0.0), [(0.0, 0.0), (0.0, 2.0)], ranges,
        current_speed=0.0, dt=0.1)

    assert speed == 0.0 and status == follower.STOPPED
    assert turn > 0.0, f"목표가 왼쪽인데 오른쪽으로 돌았다 (w={turn:.2f})"
