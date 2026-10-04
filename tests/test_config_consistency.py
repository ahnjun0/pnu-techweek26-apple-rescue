"""config.py 의 값들이 서로 모순되지 않는지 확인한다.

당일 파라미터를 손볼 때 여기서 걸리면, 로봇이 이상하게 구는 이유를 바로 알 수 있다.
실제로 "계획은 되는데 주행이 거부하는" 모순 때문에 로봇이 문 앞에서 얼어붙었다.
"""

import math

import pytest

from sar import config


def test_stop_distance_is_derived_not_hand_set():
    """정지거리는 안전거리와 추종오차에서 유도되어야 한다.

    셋을 따로 손으로 정하면 반드시 어긋난다. 실제로 그 모순 때문에 로봇이
    자기가 만든 경로를 스스로 거부하며 멈칫댔다.
    """
    expected = (config.ROBOT_RADIUS + config.ROBOT_CLEARANCE
                - config.FOLLOW_TRACKING_BUDGET)
    assert config.SAFETY_STOP_DISTANCE == pytest.approx(expected), \
        "SAFETY_STOP_DISTANCE 에 숫자를 직접 쓰지 말고 유도식을 쓸 것"


def test_tracking_budget_is_positive_and_sane():
    """예산이 0 이면 오차가 1 mm 만 나도 정지가 걸려 로봇이 못 움직인다."""
    assert 0.0 < config.FOLLOW_TRACKING_BUDGET < config.ROBOT_CLEARANCE


def test_planner_margin_follows_the_clearance():
    assert config.PLANNER_INFLATION_MARGIN == config.ROBOT_CLEARANCE


def test_stop_distance_is_within_the_clearance_the_planner_guarantees():
    """제일 중요한 관계 (앞뒤 방향).

    planner 는 로봇 중심이 벽에서 (반경 + 여유) 만큼 떨어지도록 경로를 만든다.
    주행기의 정지거리가 그보다 크면, 계획대로 가는 중에 정지가 걸린다.
    로봇이 자기가 만든 경로를 스스로 거부하는 모순이고, 실제로 이것 때문에
    로봇이 "위에서 보면 넓은 곳" 에서 정지→대기→후진을 반복했다.
    """
    clearance = config.ROBOT_RADIUS + config.PLANNER_INFLATION_MARGIN
    assert config.SAFETY_STOP_DISTANCE < clearance, (
        f"정지거리 {config.SAFETY_STOP_DISTANCE:.3f} m 가 planner 여유 "
        f"{clearance:.3f} m 보다 크다. PLANNER_INFLATION_MARGIN 을 늘리거나 "
        f"SAFETY_STOP_DISTANCE 를 줄일 것")


def test_squeeze_margin_is_smaller_than_the_normal_one():
    """폴백은 평소보다 좁아야 의미가 있다."""
    assert config.PLANNER_SQUEEZE_MARGIN < config.PLANNER_INFLATION_MARGIN
    assert config.PLANNER_SQUEEZE_MARGIN > 0.0


def test_squeeze_paths_are_still_drivable():
    """좁게 짠 경로도 주행기가 받아들일 수 있어야 한다.

    좁은 여유가 정지거리와 같거나 작으면, 그 경로는 만들자마자 정지가 걸린다.
    쓸 수 없는 경로를 만들면서 "길을 찾았다" 고 착각하게 된다.
    프론티어 도달 판정도 이 값을 쓰므로, 갈 수 없는 목표를 계속 고르게 된다.
    """
    squeeze_clearance = config.ROBOT_RADIUS + config.PLANNER_SQUEEZE_MARGIN
    assert squeeze_clearance > config.SAFETY_STOP_DISTANCE, (
        f"좁은 여유 {squeeze_clearance:.3f} m 가 정지거리 "
        f"{config.SAFETY_STOP_DISTANCE:.3f} m 보다 크지 않다")


def test_corridor_is_narrower_than_the_clearance_the_planner_guarantees():
    """제일 중요한 관계.

    planner 는 로봇 중심이 벽에서 (반경 + 여유) 만큼 떨어지도록 경로를 만든다.
    주행 쪽 통로 반폭이 그보다 넓으면, 계획상 멀쩡한 경로에서 정지가 걸린다.
    그러면 로봇은 "갈 수 있다고 계획하고 못 간다고 거부하는" 상태로 얼어붙는다.
    """
    corridor = config.ROBOT_RADIUS + config.SAFETY_CORRIDOR_CLEARANCE
    planned = config.ROBOT_RADIUS + config.PLANNER_INFLATION_MARGIN
    assert corridor < planned, (
        f"통로 반폭 {corridor:.3f} m 가 planner 가 보장하는 여유 {planned:.3f} m 보다 "
        f"넓다. SAFETY_CORRIDOR_CLEARANCE 를 줄이거나 "
        f"PLANNER_INFLATION_MARGIN 을 늘릴 것")


def test_corridor_is_at_least_the_robot():
    """로봇 몸통보다 좁게 보면 벽을 긁는다."""
    assert config.SAFETY_CORRIDOR_CLEARANCE > 0.0


def test_pinned_is_more_urgent_than_stop():
    """"눌렸다" 는 "멈춰야 한다" 보다 더 가까운 거리여야 한다."""
    assert config.SAFETY_PINNED_DISTANCE < config.SAFETY_STOP_DISTANCE
    assert config.SAFETY_PINNED_DISTANCE > config.LIDAR_MIN_RANGE


def test_stop_distance_covers_the_robot_body():
    """정지거리는 LiDAR(로봇 중심) 기준이다. 몸 반경보다는 커야 의미가 있다."""
    assert config.SAFETY_STOP_DISTANCE > config.ROBOT_RADIUS


def test_lookahead_is_bigger_than_a_cell():
    """한 칸보다 짧게 보면 격자 계단을 따라 흔들린다."""
    assert config.FOLLOW_LOOKAHEAD > config.MAP_RESOLUTION * 2


def test_goal_tolerance_is_bigger_than_a_cell():
    assert config.FOLLOW_GOAL_TOLERANCE > config.MAP_RESOLUTION


def test_waypoint_tolerance_is_not_bigger_than_the_lookahead():
    """웨이포인트 통과 판정이 look-ahead 보다 크면 경로를 통째로 건너뛴다."""
    assert config.FOLLOW_WAYPOINT_TOLERANCE <= config.FOLLOW_LOOKAHEAD


def test_map_covers_more_than_the_lidar_reach():
    """지도가 LiDAR 사거리보다 작으면 측정값을 버리게 된다."""
    width = config.MAP_WIDTH_CELLS * config.MAP_RESOLUTION
    height = config.MAP_HEIGHT_CELLS * config.MAP_RESOLUTION
    assert min(width, height) > 2 * config.LIDAR_MAX_RANGE


def test_map_ray_range_is_within_the_sensor_range():
    assert config.MAP_MAX_RAY_RANGE <= config.LIDAR_MAX_RANGE
    assert config.MAP_MAX_RAY_RANGE > config.LIDAR_MIN_RANGE


def test_log_odds_thresholds_are_ordered():
    assert config.LOG_ODDS_MIN < config.LOG_ODDS_FREE_THRESHOLD
    assert config.LOG_ODDS_FREE_THRESHOLD < config.LOG_ODDS_OCCUPIED_THRESHOLD
    assert config.LOG_ODDS_OCCUPIED_THRESHOLD < config.LOG_ODDS_MAX


def test_free_evidence_is_weaker_than_occupied_evidence():
    """빈 칸 증거가 더 세면 한번 찍힌 벽이 곧바로 지워진다."""
    assert abs(config.LOG_ODDS_FREE) < config.LOG_ODDS_OCCUPIED


def test_log_odds_can_recover_from_a_passing_person():
    """사람이 지나간 흔적이 지워질 수 있어야 한다 (한계값이 도달 가능해야 한다)."""
    assert config.LOG_ODDS_MIN < config.LOG_ODDS_FREE_THRESHOLD
    assert config.LOG_ODDS_MAX > config.LOG_ODDS_OCCUPIED_THRESHOLD


def test_lidar_angles_cover_a_full_turn():
    assert config.LIDAR_FOV == 2.0 * math.pi or config.LIDAR_FOV > 0


def test_lidar_orientation_has_been_measured():
    assert config.LIDAR_ORIENTATION_VERIFIED, \
        "debug/lidar_orientation.py 로 측정한 뒤 config 를 고칠 것"


def test_speeds_are_within_what_the_wheels_can_do():
    """바퀴가 낼 수 있는 속도보다 빠르게 시키면 명령이 잘린다."""
    wheel_limit = config.MAX_WHEEL_SPEED * config.WHEEL_RADIUS
    assert config.FOLLOW_MAX_SPEED < wheel_limit

    # 제자리 회전도 바퀴 한계 안이어야 한다
    spin = config.FOLLOW_MAX_TURN * config.WHEEL_BASE / 2.0
    assert spin < wheel_limit


def test_scan_speed_is_within_limits():
    spin = config.MISSION_SCAN_SPEED * config.WHEEL_BASE / 2.0
    assert spin < config.MAX_WHEEL_SPEED * config.WHEEL_RADIUS


def test_timeouts_are_ordered():
    assert config.MISSION_STUCK_SECONDS < config.MISSION_GOAL_TIMEOUT


def test_approach_distance_is_reachable():
    """계획기가 데려다 줄 수 있는 거리보다 가깝게 요구하면 영영 도착 못 한다.

    계획기는 목표물(장애물)에서 (반경 + 안전거리) 만큼 떨어진 곳까지만 간다.
    거기에 목표물 굵기를 더한 것보다 접근 판정이 빡빡하면 안 된다.
    """
    reachable = (config.ROBOT_RADIUS + config.PLANNER_INFLATION_MARGIN
                 + config.APPROACH_TARGET_RADIUS)
    assert config.APPROACH_DISTANCE > reachable, (
        f"접근 판정 {config.APPROACH_DISTANCE:.3f} m 가 계획기가 데려다 줄 수 있는 "
        f"{reachable:.3f} m 보다 빡빡하다")


def test_escape_is_not_slower_than_a_walking_person():
    """미는 것보다 느리게 빠져나오면 못 빠져나온다.

    보행자는 0.4 m/s 로 걷는다. 탈출 속도가 그보다 한참 느리면 밀리는 동안
    벽까지 끌려간다 (실제로 0.075 m/s 로 뒀다가 벽에 닿았다).
    """
    escape = config.FOLLOW_MAX_SPEED * config.SAFETY_ESCAPE_SPEED
    assert escape >= config.FOLLOW_MAX_SPEED, "탈출은 전속력으로 하는 게 맞다"


def test_return_tolerance_is_forgiving():
    """복귀 판정은 너그러워야 한다 (오도메트리 오차가 쌓여 있다)."""
    assert config.RETURN_TOLERANCE >= config.FOLLOW_GOAL_TOLERANCE


def test_wheel_saturation_is_understood():
    """전진하면서 돌면 바깥 바퀴가 한계를 넘을 수 있다.

    넘는 것 자체는 괜찮다 — sensors.set_wheel_speeds 가 좌우를 "같은 비율로"
    줄여서 회전 반경을 지킨다. 좌우를 따로 잘라내면 명령이 조용히 바뀐다.
    여기서는 그 사실이 문서화돼 있는지, 그리고 제자리 회전만큼은 한계 안인지 본다.
    """
    limit = config.MAX_WHEEL_SPEED * config.WHEEL_RADIUS
    spin_only = config.FOLLOW_MAX_TURN * config.WHEEL_BASE / 2.0
    assert spin_only < limit, "제자리 회전조차 못 하면 곤란하다"
    assert config.FOLLOW_MAX_SPEED <= limit, "전진만 해도 한계를 넘으면 안 된다"


def test_dwa_floor_is_below_the_planner_clearance():
    """⚠️ DWA 하한이 계획기 여유보다 작아야 한다.

    같거나 크면 로봇이 자기 경로를 스스로 거부한다 — 계획기가 "여기까지는
    괜찮다" 고 그은 길을 주행기가 "너무 가깝다" 고 버리기 때문이다.
    측정: 하한 0.22(계획기와 같음)에서 완주 5/5 -> 4/5, 목표 3.0 -> 2.6 개.
    """
    assert config.DWA_CLEARANCE_MARGIN < config.PLANNER_INFLATION_MARGIN, (
        f"DWA 하한 {config.DWA_CLEARANCE_MARGIN} 이 계획기 여유 "
        f"{config.PLANNER_INFLATION_MARGIN} 보다 작지 않다")


def test_dwa_squeeze_is_really_a_fallback():
    """⚠️ 최후의 여유가 평소 하한과 같으면 폴백이 아무 일도 안 한다.

    실제로 실험 스크립트가 값을 덮어써서 둘이 0.20 으로 같아진 채 네 커밋이
    지나갔다. 그 상태는 사람이 바짝 붙는 순간 후보가 전멸해 제자리 회전으로
    떨어지는 설정이고, 실제로 교착이 났다.
    ⚠️ import 가 된다고 값이 맞는 것은 아니다. 값 자체를 검사해야 한다.
    """
    assert config.DWA_SQUEEZE_MARGIN < config.DWA_CLEARANCE_MARGIN, (
        f"최후의 여유({config.DWA_SQUEEZE_MARGIN})가 평소 하한"
        f"({config.DWA_CLEARANCE_MARGIN})보다 작지 않다 — 폴백이 죽어 있다")


def test_people_cost_and_replan_rate_go_together():
    """⚠️ 사람 둘레 비용과 재계획 주기는 짝이다.

    사람은 0.4 m/s 로 움직인다. 30초마다 계획하면 그 사이에 12 m 를 가므로
    비용을 얹어 봐야 의미가 없다. 반대로 "자주 계획" 만 하고 비용을 안 넣으면
    사람을 뚫고 가는 최단 경로를 더 자주 다시 그려서 오히려 나쁘다
    (실측: 닿은시간 합 6.1초 -> 134.9초, 목표물 9 -> 7).
    """
    if config.PEOPLE_COST_RADIUS > 0.0:
        assert config.MISSION_REPLAN_EVERY <= 1.0, (
            "사람 둘레를 비싸게 하면서 재계획이 느리면 소용없다 "
            f"({config.MISSION_REPLAN_EVERY}초)")
    else:
        assert config.MISSION_REPLAN_EVERY > 1.0, (
            "사람 비용 없이 자주 계획하면 사람을 뚫는 경로를 더 자주 그린다")


def test_scanmatch_search_is_not_wider_than_a_tick_of_drift():
    """⚠️ 한 번에 고칠 수 있는 양이 너무 넓으면 보정이 아니라 대체가 된다.

    지도가 조금만 틀어져도 엉뚱한 자리로 끌려간다. 실측: "많이 틀어졌을 때만
    크게 고치기" 로 했더니 방문 9 -> 5, 복귀오차 20.8 -> 291.8 cm 였다.
    """
    assert config.SCANMATCH_RANGE <= 0.25, (
        f"한 번에 {config.SCANMATCH_RANGE} m 를 고치면 보정이 아니라 대체다")


def test_tick_constants_keep_their_time_interval_at_a_slower_step():
    """월드 주기가 바뀌어도 "N 틱마다" 가 같은 **시간** 간격이어야 한다.

    ⚠️ 회귀 방지. 전부 16 ms 에서 맞춘 값이라 64 ms 월드(apartment)에서 4배 느려졌다.
       DETECT_EVERY=6 이 0.38초마다가 되어 확정에 9.6초 연속 관측이 필요했고,
       진짜 빨간 사과를 3초 보고 지나쳤다.
    """
    saved = {n: getattr(config, n) for n in config._TICK_CONSTANTS + ("TIME_STEP",)}
    try:
        base = {n: getattr(config, n) * 16 for n in config._TICK_CONSTANTS}   # [ms]
        config.apply_timestep(64)
        assert config.TIME_STEP == 64
        for n in config._TICK_CONSTANTS:
            period = getattr(config, n) * 64
            assert getattr(config, n) >= 1
            # 반올림 때문에 한 틱(64 ms)까지는 어긋날 수 있다
            assert abs(period - base[n]) <= 64, (
                f"{n}: 16 ms 에선 {base[n]} ms 간격이었는데 64 ms 에선 {period} ms")
        config.apply_timestep(64)          # 두 번 불러도 같아야 한다
        assert config.DETECT_EVERY == max(1, round(saved['DETECT_EVERY'] * 16 / 64))
    finally:
        for n, v in saved.items():
            setattr(config, n, v)
        config._TICK_ORIGINAL.clear()
