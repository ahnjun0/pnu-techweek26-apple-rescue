"""세워 둔 경로를 따라가는 조종 + 부딪히기 전에 멈추는 안전 로직.

받는 것: pose (x, y, theta), 웨이포인트 목록, LiDAR 거리 배열.
내놓는 것: (전진속도 v [m/s], 회전속도 omega [rad/s], 지금 무슨 상태인지 문자열).
핵심 아이디어: 경로 위 "조금 앞" 점을 보고 그쪽으로 돈다(look-ahead).
그리고 LiDAR 로 정면·좌우를 보고 가까우면 감속, 아주 가까우면 정지한다.
순수 numpy — Webots 없이 pytest 로 돈다.
"""

import math

import numpy as np

from . import common
from . import config

# 상태 문자열 (화면과 상태머신이 같이 읽는다)
DRIVING = "주행"
TURNING = "제자리 회전"
SLOWING = "감속"
STOPPED = "정지 (장애물)"
ARRIVED = "도착"
NO_PATH = "경로 없음"

# ── 진단 계수기 ───────────────────────────────────────────────────────────
# ⚠️ 주행 로직이 아니다. 왜 후진을 고르는지 재기 위한 관찰용이고, 아무 결정에도
#    쓰이지 않는다 — 센서 밖의 정보를 읽지 않는다. debug/mission_check.py 가
#    끝에 한 번 출력한다.
COUNT = {}


# 직전 DWA 틱의 요약 — 채점 스크립트가 기록한다 (관찰용, 결정에는 쓰지 않는다).
LAST = {"fwd": 0, "safe_fwd": 0, "squeeze": 0}


def _tally(key):
    COUNT[key] = COUNT.get(key, 0) + 1


def sector_min(ranges, centre, half_width):
    """로봇 기준 centre 방향 ±half_width 안에서 가장 가까운 거리 [m].

    아무것도 못 맞혔으면 inf 를 돌려준다.
    LiDAR 인덱스와 각도의 관계는 common.lidar_angles() 하나만 쓴다.
    """
    ranges = np.asarray(ranges, dtype=np.float64)
    angles = _angles_for(ranges)

    inside = np.abs(common.wrap_angle(angles - centre)) <= half_width
    values = ranges[inside]
    values = values[np.isfinite(values) & (values >= config.LIDAR_MIN_RANGE)]
    return float(values.min()) if len(values) else math.inf


def nearest_obstacle(ranges):
    """가장 가까운 장애물의 (거리 [m], 방향 [rad]). 없으면 (inf, 0.0)."""
    ranges = np.asarray(ranges, dtype=np.float64)
    angles = _angles_for(ranges)
    usable = np.isfinite(ranges) & (ranges >= config.LIDAR_MIN_RANGE)
    if not usable.any():
        return math.inf, 0.0
    index = int(np.argmin(np.where(usable, ranges, np.inf)))
    return float(ranges[index]), float(angles[index])


def travel_clearance(ranges, backward=False):
    """그 방향으로 몸통이 부딪히기 전까지 갈 수 있는 거리 [m]. 없으면 inf.

    로봇을 반지름 ROBOT_RADIUS 인 원으로 보고 정확히 계산한다.
    장애물이 로봇 옆 (|옆거리| >= 반지름) 에 있으면 똑바로 가는 동안에는
    절대 닿지 않으므로 아예 세지 않는다.

    ⚠️ 이 "옆으로 비켜 있으면 안 센다" 가 핵심이다. 단순한 통로 검사로 뒤를
    봤더니, 옆에 바짝 붙어 스쳐 지나갈 뿐인 벽까지 "뒤가 막혔다" 로 세어서
    옆이 눌린 상황에서 로봇이 아예 못 움직이게 됐다.
    """
    radius = config.ROBOT_RADIUS
    ranges = np.asarray(ranges, dtype=np.float64)
    angles = _angles_for(ranges)

    usable = np.isfinite(ranges) & (ranges >= config.LIDAR_MIN_RANGE)
    safe = np.where(usable, ranges, 0.0)
    forward = safe * np.cos(angles)
    lateral = safe * np.sin(angles)
    if backward:
        forward = -forward

    # 옆으로 반지름보다 멀리 비켜 있으면 직진 중에는 절대 안 닿는다.
    beside = np.abs(lateral)
    inside = usable & (beside < radius)
    if not inside.any():
        return math.inf

    # 반지름 radius 인 원이 그 점에 닿을 때까지 갈 수 있는 거리:
    #   가는 방향 거리 forward, 옆 거리 beside 일 때
    #   원의 앞머리는 sqrt(radius^2 - beside^2) 만큼 앞서 있으므로
    #   갈 수 있는 거리 = forward - 그 값
    cap = np.sqrt(np.maximum(radius ** 2 - beside ** 2, 0.0))
    ahead = inside & (forward > 0.0)      # 가는 방향에 있는 것만 센다
    if not ahead.any():
        return math.inf
    room = forward[ahead] - cap[ahead]
    return float(max(room.min(), 0.0))    # 음수면 이미 겹친 것 → 갈 수 없다


def is_pinned(ranges):
    """벽이나 사람에게 눌려 있는가. 그렇다면 경로고 뭐고 일단 빠져나와야 한다.

    ⚠️ "가장 가까운 광선 하나" 로 판단하면 안 된다. LiDAR 가 이따금 광선 하나만
       엉뚱하게 짧은 값을 준다. 실측으로 갈린다 (한 판 전체, 0.2 m 이내 광선 수):
         광선 1개  → 30틱, 그때 사람과의 실제 거리 중앙값 339 cm  (아무것도 없다)
         광선 2개  →  3틱,                              30 cm
         광선 3개↑ → 82틱,                           33~36 cm
       진짜로 눌리면 몸 옆으로 광선이 여러 개 걸린다. 하나짜리는 잡음이다.
       이걸 안 걸렀더니 멀쩡히 달리다 1초씩 후진하는 일이 한 판에 세 번 났다.
    """
    values = np.asarray(ranges)
    close = np.isfinite(values) & (values >= config.LIDAR_MIN_RANGE) \
        & (values <= config.SAFETY_PINNED_DISTANCE)
    return int(close.sum()) >= config.SAFETY_PINNED_RAYS


def escape_command(ranges):
    """눌렸을 때 빠져나오는 명령 (v, omega).

    순서:
      1. 가장 가까운 장애물의 반대쪽을 "가고 싶은 방향" 으로 삼는다.
      2. 그쪽이 앞이면 전진, 뒤면 후진을 먼저 시도한다.
      3. 그 방향에 몸이 들어갈 공간이 없으면 반대 방향을 시도한다.
      4. 앞뒤 다 막혔으면(구석) 제자리에서 등을 돌린다.

    ⚠️ 제자리 회전만 시키면 안 된다. 벽 속에 박히면 회전 자체가 막혀서
       같은 명령만 260 초를 반복한 적이 있다. 그래서 회전은 마지막 수단이다.
    ⚠️ 반대로 공간 확인 없이 무작정 후진해도 안 된다. 뒤쪽 장애물까지
       1.8 cm 까지 다가간 적이 있다. 그래서 travel_clearance 로 확인한다.
    """
    obstacle_distance, obstacle_angle = nearest_obstacle(ranges)
    speed = config.FOLLOW_MAX_SPEED * config.SAFETY_ESCAPE_SPEED
    room_needed = config.SAFETY_ESCAPE_MIN_ROOM

    # 가까운 데 아무것도 없는데 못 움직이고 있다면 (바퀴가 헛도는 등) 이유를
    # 알 수 없다. 그럴 땐 일단 왔던 길로 물러난다.
    if obstacle_distance > config.SAFETY_STOP_DISTANCE:
        return -speed, 0.0

    away = common.wrap_angle(obstacle_angle + math.pi)
    forward_turn = _clamp_turn(config.FOLLOW_TURN_GAIN * away)
    backward_turn = _clamp_turn(
        config.FOLLOW_TURN_GAIN * common.wrap_angle(away - math.pi))

    go_forward = (speed, forward_turn)
    go_backward = (-speed, backward_turn)

    # 도망갈 쪽이 앞이면 전진부터, 아니면 후진부터 시도한다.
    if abs(away) < math.pi / 3.0:
        first, second = go_forward, go_backward
        first_room = travel_clearance(ranges)
        second_room = travel_clearance(ranges, backward=True)
    else:
        first, second = go_backward, go_forward
        first_room = travel_clearance(ranges, backward=True)
        second_room = travel_clearance(ranges)

    if first_room >= room_needed:
        return first
    if second_room >= room_needed:
        return second

    # 구석에 몰렸다. 제자리 회전밖에 할 게 없다.
    # ⚠️ 이때 away 로 돌면 안 된다. 앞뒤가 대칭으로 막힌 경우 away 가 0 이 되어
    #    회전도 0, 즉 아무것도 안 하게 된다. 좌우 중 트인 쪽으로 확실히 돈다.
    left = sector_min(ranges, math.pi / 2, math.pi / 3)
    right = sector_min(ranges, -math.pi / 2, math.pi / 3)
    return 0.0, config.FOLLOW_MAX_TURN * (1.0 if left >= right else -1.0)


def _clamp_turn(value):
    return max(-config.FOLLOW_MAX_TURN, min(config.FOLLOW_MAX_TURN, value))


def _angles_for(ranges):
    """이 길이의 LiDAR 배열에 맞는 각도 배열."""
    angles = common.lidar_angles()
    if len(angles) != len(ranges):
        angles = np.linspace(-math.pi, math.pi, len(ranges), endpoint=False)
    return angles


def advance(path, pose, start_index=0):
    """이미 지나친 웨이포인트를 건너뛴다. 새 인덱스를 돌려준다.

    두 가지 경우를 모두 넘긴다:
      (1) 지금 웨이포인트에 충분히 가까워졌다
      (2) 다음 웨이포인트가 지금 것보다 더 가깝다 = 이미 지나쳤다
    (2) 가 없으면, 한 번 크게 지나쳐 버렸을 때 뒤를 향해 되돌아가려 한다.
    """
    x, y, _ = pose
    index = start_index
    while index < len(path) - 1:
        here = common.distance(x, y, *path[index])
        nxt = common.distance(x, y, *path[index + 1])
        if here < config.FOLLOW_WAYPOINT_TOLERANCE or nxt <= here:
            index += 1
        else:
            break
    return index


def _closest_on_segment(a, b, point):
    """선분 a-b 위에서 point 에 가장 가까운 점."""
    ax, ay = a
    bx, by = b
    dx, dy = bx - ax, by - ay
    length_squared = dx * dx + dy * dy
    if length_squared < 1e-12:
        return a
    t = ((point[0] - ax) * dx + (point[1] - ay) * dy) / length_squared
    t = max(0.0, min(1.0, t))
    return (ax + t * dx, ay + t * dy)


def closest_on_path(path, pose, index=0):
    """경로 "선" 위에서 로봇에 가장 가까운 점과 그 선분 번호.

    ⚠️ 웨이포인트(꼭짓점)가 아니라 선분 위의 점이어야 한다.
    planner.simplify 가 경로를 몇 개 점으로 줄여 놓기 때문에, 꼭짓점만 보면
    "2 m 앞 점" 을 향해 직진하게 되어 경로 모양을 통째로 무시한다.
    """
    if len(path) == 1:
        return 0, path[0]

    here = (pose[0], pose[1])
    best_distance = math.inf
    best = (index, path[index])
    for i in range(index, len(path) - 1):
        point = _closest_on_segment(path[i], path[i + 1], here)
        gap = common.distance(here[0], here[1], *point)
        if gap < best_distance:
            best_distance = gap
            best = (i, point)
    return best


def lookahead_point(path, pose, index=0, distance=None):
    """경로를 따라 distance [m] 만큼 "걸어간" 자리의 점.

    너무 가까우면 좌우로 흔들리고, 너무 멀면 모퉁이를 자른다.
    길이가 모자라면 경로의 마지막 점을 돌려준다.
    distance 를 안 주면 config.FOLLOW_LOOKAHEAD 를 쓴다.
    """
    if len(path) == 1:
        return path[0]

    segment, point = closest_on_path(path, pose, index)
    remaining = config.FOLLOW_LOOKAHEAD if distance is None else distance

    for i in range(segment, len(path) - 1):
        nxt = path[i + 1]
        span = common.distance(point[0], point[1], *nxt)
        if span >= remaining:
            if span < 1e-12:
                return nxt
            ratio = remaining / span
            return (point[0] + (nxt[0] - point[0]) * ratio,
                    point[1] + (nxt[1] - point[1]) * ratio)
        remaining -= span
        point = nxt
    return path[-1]


def step(pose, path, ranges, index=0, current_speed=0.0, current_turn=0.0,
         dt=None, allow_idle=False, people=()):
    """한 틱 분량의 조종 명령을 만든다. (v, omega, 상태, 새 인덱스)

    ⚠️ 한때 주행기가 두 벌이었다 (look-ahead 추종 + 안전정지 / DWA).
       DWA 가 모든 항목에서 이겨서 추종 쪽을 통째로 들어냈다
       (171초 -> 124초, 정지 23% -> 10%, 벽 여유 2.9cm -> 8.5cm).
       자세한 것은 docs/측정_기록.md.
    """
    return dwa_step(pose, path, ranges, index, current_speed, current_turn, dt,
                    allow_idle=allow_idle, people=people)


def obstacle_points(ranges, stride=None):
    """LiDAR 를 로봇 기준 (x, y) 점 목록으로. 모의주행 충돌 검사에 쓴다."""
    stride = config.DWA_RAY_STRIDE if stride is None else stride
    ranges = np.asarray(ranges, dtype=np.float64)
    angles = _angles_for(ranges)

    usable = np.isfinite(ranges) & (ranges >= config.LIDAR_MIN_RANGE)
    picked = np.zeros(len(ranges), dtype=bool)
    picked[::max(stride, 1)] = True
    keep = usable & picked
    if not keep.any():
        return np.zeros((0, 2))
    return np.stack([ranges[keep] * np.cos(angles[keep]),
                     ranges[keep] * np.sin(angles[keep])], axis=1)


def dynamic_window(current_speed, current_turn, dt):
    """지금 속도에서 **결정 지평** 안에 도달할 수 있는 (전진, 회전) 후보들.

    ⚠️ 폭을 dt 로 잡으면 안 된다. 16 ms 면 폭이 0.0096 m/s 뿐이라 후진 중에
       전진 후보가 창에 없고, 후진 벌점이 걸릴 자리가 사라진다 (config 참고).
       그래서 **고르는 창** 은 DWA_ACCEL_HORIZON 으로 넓히고, **내보내는 값** 은
       clamp_accel 로 한 틱의 가속 한계에 묶는다 — 부드러움은 거기서 지킨다.
    """
    # ⚠️ 두 축을 다 넓힌다. 회전축만 되돌려 보니 오도메트리 오차는 그대로였고 목표물만 잃었다.
    horizon = max(dt, config.DWA_ACCEL_HORIZON)
    speed_span = config.DWA_MAX_ACCEL * horizon
    turn_span = config.DWA_MAX_ANG_ACCEL * horizon

    low = max(-config.DWA_REVERSE_SPEED, current_speed - speed_span)
    high = min(config.FOLLOW_MAX_SPEED, current_speed + speed_span)
    speeds = np.linspace(low, high, config.DWA_SPEED_SAMPLES)

    turns = np.linspace(max(-config.FOLLOW_MAX_TURN, current_turn - turn_span),
                        min(config.FOLLOW_MAX_TURN, current_turn + turn_span),
                        config.DWA_TURN_SAMPLES)
    return speeds, turns


def clip_to_wheels(speeds, turns):
    """바퀴 한계를 넘는 (전진, 회전) 을 "같은 비율로" 줄여서 돌려준다.

    sensors.set_wheel_speeds 가 실제로 하는 일과 똑같다. 후보를 모의주행하기
    전에 여기를 통과시켜야 "예측한 궤적" 과 "실제 궤적" 이 같아진다.

    ⚠️ 한계를 넘는 후보를 "버리면" 안 된다. 바퀴 상한이 0.22 m/s 라서
       0.20 m/s 로 달릴 땐 회전이 0.25 rad/s 까지밖에 안 남는다. 버리는 순간
       로봇은 달리면서 크게 돌 방법이 없어져, 좁은 구석에서 벽에 눌렸다
       빠져나오기를 반복하며 갇힌다 (실측: 한 자리에서 200 초).
       비례로 줄이면 회전 반경은 그대로고 그 궤적을 더 천천히 지날 뿐이다.
    """
    half = config.WHEEL_BASE / 2.0
    left = (speeds - turns * half) / config.WHEEL_RADIUS
    right = (speeds + turns * half) / config.WHEEL_RADIUS
    biggest = np.maximum(np.abs(left), np.abs(right))
    limit = config.MAX_WHEEL_SPEED * 0.999   # sensors.set_wheel_speeds 와 같은 여유
    scale = np.where(biggest > limit, limit / np.maximum(biggest, 1e-9), 1.0)
    return speeds * scale, turns * scale


def _simulate(speeds, turns):
    """후보마다 로봇 기준으로 몇 초 앞까지의 자취를 만든다. 모양 (후보수, 점수, 2)."""
    steps = np.arange(1, config.DWA_SIM_STEPS + 1) * \
        (config.DWA_SIM_TIME / config.DWA_SIM_STEPS)

    speed_grid, turn_grid = np.meshgrid(speeds, turns, indexing="ij")
    speed_flat = speed_grid.reshape(-1, 1)
    turn_flat = turn_grid.reshape(-1, 1)
    # 바퀴가 못 내는 조합은 실행될 때 줄어든다. 줄어든 값으로 모의주행해야
    # 예측과 실행이 같아진다.
    speed_flat, turn_flat = clip_to_wheels(speed_flat, turn_flat)

    theta = turn_flat * steps                     # (후보, 점)
    # 회전이 0 에 가까우면 원호 공식이 0/0 이 된다. 직선으로 계산한다.
    straight = np.abs(turn_flat) < 1e-6
    radius = np.where(straight, 1.0, speed_flat / np.where(straight, 1.0, turn_flat))
    x = np.where(straight, speed_flat * steps, radius * np.sin(theta))
    y = np.where(straight, 0.0, radius * (1.0 - np.cos(theta)))
    return np.stack([x, y], axis=2), speed_flat.ravel(), turn_flat.ravel()


def dwa_step(pose, path, ranges, index=0, current_speed=0.0, current_turn=0.0,
             dt=None, allow_idle=False, people=()):
    """DWA 한 틱. (v, omega, 상태, 새 인덱스) — follower.step 과 같은 모양.

    """
    dt = config.TIME_STEP / 1000.0 if dt is None else dt

    if not path:
        return 0.0, 0.0, NO_PATH, index

    x, y, theta = pose
    if common.distance(x, y, *path[-1]) < config.FOLLOW_GOAL_TOLERANCE:
        return 0.0, 0.0, ARRIVED, len(path) - 1

    index = advance(path, pose, index)
    # ⚠️ 조준점은 "모의주행으로 가 볼 거리" 보다 가까우면 안 된다.
    #    1.2 초 동안 0.24 m 를 가는데 0.20 m 앞을 조준하면, 지나칠 지점을
    #    목표로 삼는 셈이라 궤적이 휜다 (추종 오차가 1 cm 에서 11 cm 로 뛰었다).
    #    속도가 바뀌어도 알아서 맞도록 여기서 계산한다.
    aim = max(config.FOLLOW_LOOKAHEAD,
              config.FOLLOW_MAX_SPEED * config.DWA_SIM_TIME)
    target = lookahead_point(path, pose, index, aim)

    # 목표점을 로봇 기준 좌표로 옮긴다
    dx, dy = target[0] - x, target[1] - y
    target_local = np.array([dx * math.cos(-theta) - dy * math.sin(-theta),
                             dx * math.sin(-theta) + dy * math.cos(-theta)])

    speeds, turns = dynamic_window(current_speed, current_turn, dt)
    tracks, speed_flat, turn_flat = _simulate(speeds, turns)

    obstacles = obstacle_points(ranges)

    # 움직이는 것은 **갈 곳** 도 장애물로 둔다. 현재 위치만 피하면 우리 쪽으로
    # 걸어오는 사람을 막을 수 없다 (Webots Pedestrian 은 기구학적이라 궤적을
    # 그대로 밀고 지나간다). 실측: 물리적으로 밀리는 mover 둘은 0.54/1.73 m 로
    # 완벽히 피했는데 보행자만 0.029 m 까지 닿았다.
    # ⚠️ 가까운 사람만 본다. 멀리 있는 사람(이나 유령)의 미래 위치는 우리 궤적과
    #    만날 수 없는데, 그걸 계산해 장애물에 더하면 **비용만** 든다.
    #    실측: 사람이 **없는** 월드에서도 유령 검출이 남아 매 틱 예측 점을 만들어 느려졌다.
    #    (allow_idle 때와 같은 처방이다: 검출 유무가 아니라 **거리** 로 거른다.)
    reach = config.DWA_PERSON_REACH
    if reach > 0.0 and people:
        people = [p for p in people
                  if math.hypot(p[0] - pose[0], p[1] - pose[1]) <= reach]
    ghosts = _predicted_points(pose, people)
    if len(ghosts):
        obstacles = np.vstack([obstacles, ghosts]) if len(obstacles) else ghosts

    if len(obstacles):
        # (후보, 점, 장애물) 거리를 한 번에 — 반복문으로 돌면 너무 느리다
        gaps = np.linalg.norm(tracks[:, :, None, :] - obstacles[None, None, :, :],
                              axis=3)
        clearance = gaps.min(axis=(1, 2))
    else:
        clearance = np.full(len(speed_flat), math.inf)

    safe = clearance > config.ROBOT_RADIUS + config.DWA_CLEARANCE_MARGIN

    # ⚠️ 비상구 조건은 "안전한 후보가 없다" 가 아니라 "안전한 **전진** 후보가
    #    없다" 여야 한다. 제자리 후보는 궤적이 "점" 이라 여유가 늘 만점이므로
    #    safe.any() 를 언제나 만족시킨다 — 그래서 전진할 데가 없는데도 좁은
    #    여유 비상구가 영영 발동하지 않았다.
    #    실측: 요구 여유를 겨우 넘는 자리에 **200초** 갇혀 제자리 회전만 했다.
    forward = speed_flat > config.DWA_IDLE_SPEED
    LAST["fwd"] = int(forward.sum())
    LAST["safe_fwd"] = int((safe & forward).sum())
    LAST["squeeze"] = 0
    # 계수는 서로 겹쳐도 되게 독립적으로 센다 (elif 로 묶으면 원인이 가려진다)
    _tally("틱")
    if not forward.any():
        _tally("창에 전진 후보 없음")
    if forward.any() and not (safe & forward).any():
        _tally("전진 후보가 모두 위험")
    behind = bool(target_local[0] < 0.0)
    if behind:
        _tally("조준점이 뒤에 있음")
    cs = float(current_speed)
    _tally("들어온 속도: " + ("후진" if cs < -0.005 else
                            "거의0" if cs <= 0.005 else
                            "최고속" if cs >= 0.19 else "느린전진"))
    # ⚠️ 조준점이 뒤에 있으면 제자리에서 그쪽으로 돈다. 점수는 궤적 "끝점" 만 보므로 뒤의 점까지
    #    거리가 전진 후보끼리 거의 같고(1.2초에 3 cm 차이), 옆 장애물 여유가 그 차이를 이긴다 —
    #    로봇이 조준점을 등진 채 0.02~0.03 m/s 로 거의 돌지도 않고 기어갔다.
    #    대회 월드 2026-10-06 녹화: 693.6초 중 121초 (10초 넘게 이어진 것만 6번).
    #    제자리 회전은 원형 로봇에게 어느 쪽이든 안전하다 (_spin_toward 설명).
    if behind:
        return 0.0, _spin_toward(target_local, ranges), TURNING, index
    if not (safe & forward).any():
        # ⚠️ 원하는 여유로 갈 데가 없다고 곧바로 포기하면 안 된다.
        #    사람이 그 여유 안으로 들어온 순간 후보가 전멸해 제자리 회전으로
        #    떨어지는데, 사람이 밀고 있는 상태에서 제자리 회전은 바퀴가 헛돌아
        #    오도메트리가 폭주한다 (실제로 사람과 뒤엉켜 교착에 빠졌다).
        #    계획기가 "평소 여유 → 안 되면 좁게" 로 두 번 시도하는 것과 같이,
        #    여기서도 최소 여유로 한 번 더 본다.
        squeeze = clearance > config.ROBOT_RADIUS + config.DWA_SQUEEZE_MARGIN
        LAST["squeeze"] = 1
        # ⚠️ 비상구가 후진·제자리회전을 열어 주는 것은 옳지만, **전진** 까지
        #    무조건 열면 정면의 벽으로 기어든다. 창을 넓히기 전에는 창이 좁아
        #    전진 후보를 표현조차 못 해 이 구멍이 드러나지 않았다.
        #    좁은 틈 통과(0.245 m 에서 전진해야 한다)와 벽에 기어들기(0.18 m 에서
        #    전진하면 안 된다)는 기하가 같고 거리로만 갈린다. 그 경계는 이미
        #    이름이 있다 — SAFETY_STOP_DISTANCE (= 0.23 m, "이보다 가까우면 멈춘다").
        here = (float(np.linalg.norm(obstacles, axis=1).min())
                if len(obstacles) else math.inf)
        if here <= config.SAFETY_STOP_DISTANCE:
            squeeze &= ~forward
        safe = squeeze

    # ⚠️ "제자리 회전(속도 0)을 후보에서 빼고 반드시 움직이게 하기" 를 넣었다가
    #    재 보고 뺐다. 속도 0 후보는 궤적이 "점" 이라 안전거리 점수가 늘 최고라
    #    사람이 근처면 가만히 있는 쪽이 이기는데, 그게 문제로 보였기 때문이다.
    #    그런데 Webots 에서 보행자 속도 3가지로 재 보니 오히려 나빴다:
    #      그대로 두면   사람과 닿은시간 합 12.8초 / 최악 거리 3.4 cm
    #      움직이게 강제 사람과 닿은시간 합 23.0초 / 최악 거리 2.8 cm
    #    사람 근처에서 억지로 움직이면 상대운동이 늘어난다. 가만히 있으면
    #    적어도 내가 다가가지는 않는다. docs/측정_기록.md 참고.

    if not safe.any():
        # 그래도 없다. 제자리 회전은 원형 로봇에게 어느 쪽이든 안전하므로,
        # "트인 쪽" 이 아니라 "가고 싶은 쪽" 으로 짧게 돈다.
        return 0.0, _spin_toward(target_local, ranges), STOPPED, index

    goal_gap = np.linalg.norm(tracks[:, -1, :] - target_local, axis=1)

    # 점수 항은 둘뿐이다: "목표점에 가까워지는가" 와 "장애물에서 먼가".
    # ⚠️ "빨리 가는가" 항도 있었는데, 하나씩 꺼 보니 없는 쪽이 나았다:
    #    완주 5/5 그대로, 목표물 3/3 그대로, 시간 132 -> 136초 (3% 손해),
    #    최악 벽 여유 32.6 -> 35.4 cm, 접촉 42 -> 18회.
    #    속도는 목표점 항이 알아서 끌어낸다 (빨리 가면 목표에 더 가까워진다).
    score = (config.DWA_WEIGHT_GOAL * _normalise(-goal_gap, safe)
             + config.DWA_WEIGHT_CLEARANCE
             * _normalise(np.minimum(clearance, config.DWA_CLEARANCE_CAP), safe))

    # 후진에는 벌점을 준다. 목표가 뒤에 있으면 돌아서야지, 뒤로 기어가면 안 된다.
    # (앞이 전부 막혀 안전한 후보가 후진뿐이면 그때는 후진이 선택된다.)
    score -= config.DWA_REVERSE_PENALTY * (speed_flat < 0.0)

    # 아무것도 안 하는 후보에 벌점. 사람이 보이면(allow_idle) 벌점을 끈다 —
    # 사람 근처에서는 가만히 있는 것이 옳다. 자세한 근거는 config 의 주석.
    if config.DWA_IDLE_PENALTY and not allow_idle:
        # ⚠️ 한때 "가만히 + 안 돎" 에만 벌점을 줬더니, 로봇이 **제자리 회전** 으로
        #    피해 갔다 (완전 정지 252초 -> 15초, 대신 제자리 회전 442 -> 776초).
        #    회전 궤적도 사실상 "점" 이라 여유 점수가 만점이기 때문이다.
        #    벌점은 "전진이 없는" 후보 전체에 걸어야 한다.
        #    계측 근거: 안전한 전진 후보가 틱당 평균 15개 있는데도 88% 의 틱에서
        #    전진 0 을 골랐다 — 갈 데가 없어서가 아니라 점수가 그랬다.
        idle = np.abs(speed_flat) <= config.DWA_IDLE_SPEED
        score -= config.DWA_IDLE_PENALTY * idle
    score[~safe] = -math.inf

    best = int(np.argmax(score))
    speed, turn = float(speed_flat[best]), float(turn_flat[best])
    # ⚠️ 고른 값을 여기서 "한 틱 가속 한계" 로 묶어 봤다가 되돌렸다. v 만 줄고
    #    ω 는 그대로면 회전 반경 v/ω 가 모의주행한 것보다 **조여져서**, 평가하지
    #    않은 궤적으로 벽에 휘어 든다 — 안전 판정이 거짓이 된다.
    #    실측: 목표물을 하나 잃고 벽 여유가 0.3 cm 까지 떨어졌다.
    #    그래서 표준 DWA 대로 고른 값을 그대로 내보낸다 (평가 궤적 = 실행 궤적).
    if speed < 0.0:
        _tally("후진을 골랐다")
        _tally("후진 & 조준점 " + ("뒤" if behind else "앞"))
        # 창 안에서 더 깊은 후진을 골랐나, 얕은 쪽(회복)을 골랐나
        low, high = float(speed_flat.min()), float(speed_flat.max())
        if speed <= low + 1e-6:
            _tally("후진 — 창의 최저(더 깊이)")
        elif speed >= high - 1e-6:
            _tally("후진 — 창의 최고(회복 중)")
    # ⚠️ 속도가 0.01 만 넘어도 "주행" 이라고 찍으면 거짓말이 된다. v = 0.02 m/s 로
    #    기어가며 제자리에서 도는 것을 화면에서는 회전으로 보는데 로그는 "주행"
    #    이라고 해서, 사용자가 본 것과 로그가 어긋났다 ("여전히 제자리 회전이
    #    남아 있는데 로그에는 EXPLORE — 주행 이라고 뜬다").
    #    최고속의 1/4 에 못 미치면서 크게 돌고 있으면 그것은 사실상 회전이다.
    if speed <= config.DWA_IDLE_SPEED:
        status = TURNING
    elif (speed < config.FOLLOW_MAX_SPEED * 0.25
            and abs(turn) > config.FOLLOW_MAX_TURN * 0.5):
        status = SLOWING          # 기어가며 크게 돌고 있다
    else:
        status = DRIVING
    return speed, turn, status, index


def _predicted_points(pose, people):
    """사람들이 앞으로 있을 자리를 로봇 기준 좌표 점으로 만든다.

    DWA 가 후보 궤적을 앞으로 모의주행하므로, 사람도 같은 시간만큼 앞으로
    보내 두면 "지금은 비었지만 곧 사람이 올 자리" 를 피하게 된다.
    """
    if not len(people) or config.DWA_PERSON_LOOKAHEAD <= 0.0:
        return np.empty((0, 2))
    x, y, theta = pose
    cos_t, sin_t = math.cos(-theta), math.sin(-theta)
    out = []
    steps = max(1, config.DWA_PERSON_STEPS)
    for person in people:
        px, py = person[0], person[1]
        vx, vy = (person[2], person[3]) if len(person) >= 4 else (0.0, 0.0)
        for k in range(1, steps + 1):
            t = config.DWA_PERSON_LOOKAHEAD * k / steps
            wx, wy = px + vx * t, py + vy * t
            dx, dy = wx - x, wy - y
            out.append((dx * cos_t - dy * sin_t, dx * sin_t + dy * cos_t))
    return np.asarray(out, dtype=np.float64)


def _normalise(values, mask):
    """고른 후보들 사이에서 0~1 로 편다. 가중치끼리 비교가 되게 하려는 것."""
    picked = values[mask]
    if len(picked) == 0:
        return np.zeros_like(values)
    low, high = picked.min(), picked.max()
    if not np.isfinite(low) or not np.isfinite(high) or high - low < 1e-9:
        return np.zeros_like(values)
    return np.clip((values - low) / (high - low), 0.0, 1.0)


def _spin_toward(target_local, ranges):
    """앞으로 갈 수 있는 후보가 하나도 없을 때, 목표 쪽으로 제자리 회전한다.

    ⚠️ 한때 "좌우 중 트인 쪽" 으로 돌았는데, 그러면 목표가 왼쪽 99도에 있어도
       오른쪽이 더 트였다는 이유로 258도를 돌아갔다 (사용자가 GUI 에서
       "사람을 마주치면 한 바퀴 돈다" 고 지적한 것이 이것이다).
       제자리 회전은 원형 로봇에게 어느 쪽이든 안전하다. 그러니 트인 쪽을
       고를 이유가 없고, 목표 쪽으로 짧게 도는 것이 언제나 낫다.
    """
    want = math.atan2(target_local[1], target_local[0])
    if abs(want) < math.radians(10):
        # 목표가 거의 정면인데 앞이 막혔다. 어느 쪽이든 틀어야 하므로
        # 그때만 "트인 쪽" 을 본다.
        left = sector_min(ranges, math.pi / 2, math.pi / 3)
        right = sector_min(ranges, -math.pi / 2, math.pi / 3)
        return config.FOLLOW_MAX_TURN * (1.0 if left >= right else -1.0)
    # 남은 각도가 작으면 살살 돈다 (지나치면 반대로 다시 돌아야 한다).
    rate = min(config.FOLLOW_MAX_TURN, abs(want) * 2.0)
    return math.copysign(rate, want)
