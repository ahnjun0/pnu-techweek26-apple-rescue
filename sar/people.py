"""LiDAR 한 스캔에서 '움직이는 것'(사람)의 위치를 찾는다.

받는 것: 로봇 pose, LiDAR 거리 배열, 지금까지의 지도.
내놓는 것: 사람 후보의 월드 좌표 목록 [(x, y), ...].
핵심 아이디어: 지도가 "빈 칸" 이라 아는 자리에서 뭔가 보이면 그건 움직이는 것이다.
광선을 '끊긴 곳' 에서 조각내고, 조각의 폭으로 다리인지 걸러 둘씩 묶는다
(Arras et al., ICRA 2007 의 단순판 — 부스팅 대신 폭 하나만 쓴다).
여러 스캔에 걸쳐 본 것만 믿는다 (시간 투표). 한 스캔에는 사람 몸에 점이 몇 개
안 찍히고, 벽 조각이 다리처럼 보이는 일도 있다. 연속해서 같은 자리에 보이는
것만 사람으로 치면 둘 다 나아진다 — 최신 연구도 "여러 스캔을 합치는" 방향이다
(DR-SPAAM, Jia et al. RA-L 2021).
사람은 16 ms 에 0.64 cm 밖에 못 움직이므로, 몇 스캔 합쳐도 3 cm 남짓 번진다
(우리 검출 오차 23 cm 보다 훨씬 작다).

순수 numpy — Webots 없이 pytest 로 돈다.

⚠️ 위치만 쓴다. 속도는 못 쓴다 — 실측 위치 오차 23 cm 를 틱 간격으로 나누면
   속도 잡음이 사람 속도보다 크다. docs/측정_기록.md 참고.
"""
import math
import numpy as np

from . import common
from . import config
from . import mapping
from . import planner


# ── 진단 계수기 ───────────────────────────────────────────────────────────
# ⚠️ 주행 로직이 아니다. 재현율 손실이 **후보 생성** 에서 나는지 **투표** 에서
#    나는지 가르기 위한 관찰용이고, 아무 결정에도 쓰이지 않는다.
COUNT = {}


def _tally(key, n=1):
    COUNT[key] = COUNT.get(key, 0) + n


def segments(pose, ranges, jump=0.13, min_points=3):
    """광선을 '거리가 뚝 끊기는 곳' 에서 잘라 조각 목록으로. [(xs, ys), ...]

    jump: 이웃 광선의 거리가 이보다 많이 벌어지면 다른 물체로 본다 [m].
    """
    angles = common.lidar_angles()
    good = np.isfinite(ranges) & (ranges > config.LIDAR_MIN_RANGE) \
        & (ranges < config.LIDAR_MAX_RANGE)
    idx = np.nonzero(good)[0]
    if len(idx) < min_points:
        return []

    world = angles[idx] + pose[2]
    xs = pose[0] + ranges[idx] * np.cos(world)
    ys = pose[1] + ranges[idx] * np.sin(world)

    # 인덱스가 건너뛰거나 거리가 크게 벌어지면 끊는다 (원형이라 끝과 처음도 이웃)
    step = np.hypot(np.diff(xs), np.diff(ys))
    cut = np.nonzero((step > jump) | (np.diff(idx) > 2))[0] + 1
    out = []
    for piece_x, piece_y in zip(np.split(xs, cut), np.split(ys, cut)):
        if len(piece_x) >= min_points:
            out.append((piece_x, piece_y))
    return out


def shape(xs, ys):
    """조각의 기하 특징. (폭, 굽음, 점개수)

    폭   : 양 끝점 사이 거리 [m]
    굽음 : 양 끝을 잇는 선에서 가장 많이 벗어난 거리 [m] (다리는 볼록하다)
    """
    width = math.hypot(xs[-1] - xs[0], ys[-1] - ys[0])
    if width < 1e-6:
        return width, 0.0, len(xs)
    # 끝점을 잇는 직선에서의 수직거리
    dx, dy = (xs[-1] - xs[0]) / width, (ys[-1] - ys[0]) / width
    off = np.abs(-dy * (xs - xs[0]) + dx * (ys - ys[0]))
    return width, float(off.max()), len(xs)


def legs(pose, ranges, grid=None, width_range=None):
    """다리로 보이는 조각들의 중심. [(x, y), ...]

    grid 를 주면 "지도가 빈 칸이라고 아는 자리" 의 조각만 남긴다 (고정물 제외).
    """
    width_range = config.PEOPLE_WIDTH_RANGE if width_range is None else width_range
    found = []
    for xs, ys in segments(pose, ranges, jump=config.PEOPLE_SEGMENT_JUMP):
        width, _, _ = shape(xs, ys)
        if not (width_range[0] <= width <= width_range[1]):
            continue
        cx, cy = float(xs.mean()), float(ys.mean())
        if grid is not None:
            row, col = common.to_cell(cx, cy)
            if not common.in_bounds(row, col) or not mapping.is_free(grid)[row, col]:
                continue
        found.append((cx, cy))
    return found


def moving_blobs(pose, ranges, grid):
    """지도에 없는 물체들의 중심. [(x, y), ...] — 모양도 색도 묻지 않는다.

    원리: 점유격자가 "빈 칸" 이라고 아는 자리에서 광선이 돌아오면, 거기 지도에
    없는 무언가가 있다는 뜻이다. 통짜 원기둥이든 코트를 입은 사람이든 카트든
    똑같이 잡힌다.

    ⚠️ 왜 다리 검출(legs)로는 모자랐나: 판정을 움직임으로 바꿔도 **후보를 만드는
       단계** 가 여전히 모양 문턱(다리 폭 0.04~0.35 m)이면, 문턱을 통과 못 한
       물체는 움직임을 볼 기회조차 없다. 실측: 사거리 안의 진짜
       물체를 틱당 평균 1.13개 놓쳤고, 그래서 충돌이 났다 (닿은 시간 7.3초).

    ⚠️ 지도는 같은 LiDAR 로 만들므로 움직이는 물체도 언젠가 지도에 찍힌다.
       그래서 이것만으로는 모자라고, Watcher 의 "움직였는가" 투표가 함께 걸린다.
    """
    free = mapping.is_free(grid)
    # ⚠️ 벽 표면의 양자화 잡음을 걸러야 한다. 광선이 벽에 맞는 칸이 한 칸씩
    #    흔들리면서, 이전에 "빈 칸" 으로 찍힌 자리에 반사가 생긴다. 그것을 그대로
    #    두었더니 **사람이 없는 미로에서 유령이 틱당 0.44개** 나왔고(헛것 100%),
    #    그 유령이 계획기에 사회적 비용을 얹어 주행 거리가 2.7배가 됐다.
    #    사람은 트인 공간에 있고 이 잡음은 벽에 붙어 있으므로, 벽에서 떨어진 것만 남긴다.
    near_wall = planner.inflate(grid, margin=config.PEOPLE_WALL_CLEARANCE
                                - config.ROBOT_RADIUS)
    out = []
    for xs, ys in segments(pose, ranges, jump=config.PEOPLE_SEGMENT_JUMP):
        if len(xs) < config.PEOPLE_MIN_BLOB_POINTS:
            continue
        # 조각의 점들이 "지도가 비었다고 아는 칸" 에 얼마나 떨어졌나
        off_map = 0
        for x, y in zip(xs, ys):
            row, col = common.to_cell(float(x), float(y))
            if common.in_bounds(row, col) and free[row, col]:
                off_map += 1
        if off_map < len(xs) * config.PEOPLE_OFF_MAP_SHARE:
            continue
        cx, cy = float(xs.mean()), float(ys.mean())
        cell = common.to_cell(cx, cy)
        if not common.in_bounds(*cell) or near_wall[cell]:
            continue                # 벽에 붙어 있다 = 표면 잡음이다
        out.append((cx, cy))
    return out


def people(pose, ranges, grid=None, pair_within=None, **kw):
    """다리를 둘씩 묶어 사람 위치로. [(x, y), ...]

    다리 하나만 보이는 경우(가려짐)도 사람으로 친다 — Arras 논문도 그렇게 한다.
    """
    pair_within = config.PEOPLE_PAIR_WITHIN if pair_within is None else pair_within
    spots = legs(pose, ranges, grid, **kw)
    used = [False] * len(spots)
    out = []
    for i, (x, y) in enumerate(spots):
        if used[i]:
            continue
        mate = None
        for j in range(i + 1, len(spots)):
            if used[j]:
                continue
            if math.hypot(spots[j][0] - x, spots[j][1] - y) <= pair_within:
                mate = j
                break
        used[i] = True
        if mate is None:
            out.append((x, y))
        else:
            used[mate] = True
            out.append(((x + spots[mate][0]) / 2.0, (y + spots[mate][1]) / 2.0))
    return out


class Watcher:
    """여러 스캔에 걸쳐 본 것만 사람으로 친다 (시간 투표).

    쓰는 법: 매 틱 see(pose, ranges, grid) 를 부르면 사람 목록을 돌려준다.
    ⚠️ 상태를 들고 있으므로 임무마다 하나씩 새로 만든다.
    """

    def __init__(self):
        self.recent = []          # [[(x, y), ...], ...] 최근 스캔들의 후보

    def see(self, pose, ranges, grid=None):
        """이번 스캔의 후보를 넣고, **움직인** 것만 돌려준다.

        ⚠️ 예전 규칙은 "여러 스캔에서 같은 자리에 보이면 사람" 이었고, 주석에
           "벽 조각은 한 번만 보인다" 고 적혀 있었다. **그 가정이 틀렸다.**
           문틈이나 벽 끝은 매 스캔 같은 자리에 보인다 — 오히려 이 규칙에
           가장 잘 맞는다. 그래서 사람이 **없는** 미로에서 유령 사람이 끊임없이
           잡혔다 (계측: 900초 중 335초의 틱에서 검출).
           그 유령은 계획기에 사회적 비용(반경 1.2 m, 배수 4.0)을 얹어 경로를
           왜곡했고, 주행기의 "가만히 있어도 된다" 도 계속 켜 두었다.

        그래서 판단 기준을 **모양이 아니라 움직임** 으로 바꾼다. 세계 좌표에서
        고정물은 로봇이 움직여도 제자리에 있고, 사람은 옮겨 간다.
        이 기준은 모양·색을 가정하지 않으므로 통짜 원기둥이든 코트를 입었든
        똑같이 듣는다 (docs 의 "다리가 아니라 사람" 장 참고).

        ⚠️ 맞바꾼 것: **가만히 서 있는 사람은 못 잡는다.** 다만 그때는 사실상
           고정 장애물이라 점유격자와 DWA 가 이미 피한다. 대회 공고도
           "**움직이는** 사람" 이라고 쓰고 있다.
        """
        # 후보 생성: 지도에 없는 덩어리(모양 무관) 또는 다리 모양.
        # 둘 다 쓰면 재현율이 올라간다 — 어느 쪽이든 잡히면 움직임 투표로 거른다.
        found = []
        _tally("틱")
        if grid is not None:
            blobs = moving_blobs(pose, ranges, grid)
            _tally("후보: 지도차이", len(blobs))
            found.extend(blobs)
        legs = people(pose, ranges, grid)
        _tally("후보: 다리모양", len(legs))
        found.extend(legs)
        if not found:
            _tally("후보가 0개인 틱")
        self.recent.append(found)
        if len(self.recent) > config.PEOPLE_VOTE_SCANS:
            self.recent.pop(0)
        if len(self.recent) < config.PEOPLE_VOTE_NEEDED:
            return []

        # 어느 스캔에서 나온 점인지 같이 들고 다닌다 (움직임을 재려면 필요하다).
        spots = [(p[0], p[1], k)
                 for k, scan in enumerate(self.recent) for p in scan]
        if not spots:
            return []

        # ⚠️ 문턱은 **실제로 본 구간** 으로 재야 한다. 창 전체(PEOPLE_VOTE_SCANS)로
        #    재면, 창의 일부에만 걸린 덩어리는 그 동안 갈 수 없는 거리를 요구받아
        #    "안 움직였다" 로 기각된다 — **진짜 사람이 걸러진다.**
        #    계산: 창 9틱 = 0.144 s 이면 문턱 2.2 cm 인데, 3틱(0.048 s)만 본
        #    그룹에서 0.5 m/s 로 걷는 사람은 1.6 cm 밖에 못 간다 → 기각.
        #    실측: 그 상태에서 볼 수 있었을 때의 재현율이 53% 였다.
        tick_seconds = config.TIME_STEP / 1000.0

        used = [False] * len(spots)
        out = []
        for i, (x, y, k) in enumerate(spots):
            if used[i]:
                continue
            group = [(x, y, k)]
            used[i] = True
            for j in range(i + 1, len(spots)):
                if used[j]:
                    continue
                if math.hypot(spots[j][0] - x, spots[j][1] - y) <= config.PEOPLE_VOTE_RADIUS:
                    used[j] = True
                    group.append(spots[j])

            scans = {g[2] for g in group}
            _tally("그룹")
            if len(scans) < config.PEOPLE_VOTE_NEEDED:
                _tally("기각: 스캔 수 부족")
                continue            # 몇 번은 봐야 믿는다 (한 번 반짝한 잡음 제외)

            if config.PEOPLE_MIN_SPEED > 0.0:
                first, last = min(scans), max(scans)
                if first == last:
                    continue
                start = _centre([g for g in group if g[2] == first])
                end = _centre([g for g in group if g[2] == last])
                span_seconds = (last - first) * tick_seconds
                moved_enough = config.PEOPLE_MIN_SPEED * span_seconds
                if math.hypot(end[0] - start[0], end[1] - start[1]) < moved_enough:
                    _tally("기각: 안 움직였다")
                    continue        # 안 움직였다 = 고정물이다

            # 속도도 같이 낸다. 이미 움직임을 재고 있으니 공짜다.
            # ⚠️ 왜 필요한가: 사람의 **현재** 위치만 피하면, 사람이 우리 쪽으로
            #    걸어오는 경우를 막을 수 없다. Webots Pedestrian 은 기구학적이라
            #    궤적을 그대로 밀고 지나간다 — 실측: 움직이는 물체
            #    둘은 0.54 m / 1.73 m 로 완벽히 피하는데 보행자만 0.029 m 까지
            #    닿았다 (5.9초). 피하려면 갈 곳을 알아야 한다.
            vx = vy = 0.0
            if len(scans) > 1:
                first, last = min(scans), max(scans)
                a = _centre([g for g in group if g[2] == first])
                b = _centre([g for g in group if g[2] == last])
                span = (last - first) * config.TIME_STEP / 1000.0
                if span > 0:
                    vx, vy = (b[0] - a[0]) / span, (b[1] - a[1]) / span
            cx, cy = _centre(group)
            # ⚠️ **몇 스캔에서 봤는지** 를 같이 낸다. 이게 확신도다.
            #    유령의 해악은 두 경로로 들어오는데 값이 다르다:
            #      DWA(국소 회피) — 유령을 피해도 잠깐 돌아가면 그만이다. 싸다.
            #      계획기(사회적 비용) — 전역 경로를 왜곡한다. 비싸다.
            #    그래서 회피에는 낮은 문턱을, 계획에는 높은 문턱을 쓴다
            #    (config.PEOPLE_PLANNER_SCANS). 양자택일하지 않아도 된다.
            out.append((cx, cy, vx, vy, len(scans)))
        return out


def _centre(group):
    """(x, y, 스캔번호) 들의 무게중심 (x, y)."""
    return (sum(g[0] for g in group) / len(group),
            sum(g[1] for g in group) / len(group))


class Tracker:
    """사람 한 명마다 등속(CV) 칼만 필터 하나. 상태 [x, y, vx, vy].

    쓰는 법: 매 틱 update(detections, dt) — detections 는 Watcher.see() 결과.
    돌려주는 형식도 같다: (x, y, vx, vy, 갱신 횟수). 갱신 횟수가 확신도다.
    """

    def __init__(self):
        self.tracks = []    # {"x": (4,), "P": (4,4), "hits": int, "unseen": s}

    def update(self, detections, dt):
        q = config.PEOPLE_KF_ACCEL_STD ** 2
        r = config.PEOPLE_KF_MEAS_STD ** 2
        F = np.array([[1, 0, dt, 0], [0, 1, 0, dt], [0, 0, 1, 0], [0, 0, 0, 1]], float)
        # 가속도 잡음을 위치·속도에 나눠 싣는 표준형 (구간 상수 가속도 모델)
        G = np.array([[dt * dt / 2, 0], [0, dt * dt / 2], [dt, 0], [0, dt]])
        Q = G @ G.T * q
        H = np.array([[1, 0, 0, 0], [0, 1, 0, 0]], float)
        R = np.eye(2) * r

        # 예측
        for t in self.tracks:
            t["x"] = F @ t["x"]
            t["P"] = F @ t["P"] @ F.T + Q
            t["unseen"] += dt

        # 가까운 것부터 짝짓는다 (탐욕적 — 사람이 한두 명이면 충분하다)
        pairs = sorted(
            (math.hypot(d[0] - t["x"][0], d[1] - t["x"][1]), i, j)
            for i, t in enumerate(self.tracks) for j, d in enumerate(detections))
        used_t, used_d = set(), set()
        for gap, i, j in pairs:
            if gap > config.PEOPLE_KF_GATE or i in used_t or j in used_d:
                continue
            used_t.add(i); used_d.add(j)
            t = self.tracks[i]
            z = np.array(detections[j][:2], float)
            S = H @ t["P"] @ H.T + R
            K = t["P"] @ H.T @ np.linalg.inv(S)
            t["x"] = t["x"] + K @ (z - H @ t["x"])
            t["P"] = (np.eye(4) - K @ H) @ t["P"]
            t["hits"] += 1
            t["unseen"] = 0.0
        for j, d in enumerate(detections):
            if j not in used_d:
                # 처음 보는 사람: 속도는 모른다 — 크게 불확실하게 시작한다
                P = np.diag([r, r, 1.0, 1.0])
                self.tracks.append({"x": np.array([d[0], d[1], 0.0, 0.0]),
                                    "P": P, "hits": 1, "unseen": 0.0})
        self.tracks = [t for t in self.tracks
                       if t["unseen"] <= config.PEOPLE_KF_FORGET]
        return [(float(t["x"][0]), float(t["x"][1]), float(t["x"][2]),
                 float(t["x"][3]), t["hits"])
                for t in self.tracks
                if t["hits"] >= config.PEOPLE_KF_MIN_HITS
                and t["unseen"] <= config.PEOPLE_KF_COAST]


# ── 다가오는 사람 — "가만히 있으면 부딪히는가" ─────────────────────────────────
# 송지윤(yun110w)의 song-jiyun 브랜치(279d5d5)에서 옮겼다.
def person_threat(pose, people, horizon=None):
    """내가 **가만히 있을 때** 사람이 금지 반경 안으로 들어오고, 나를 향해 오는가.

    받는 것: pose (x, y, theta), people = [(x, y, vx, vy[, 확신도]), ...]
             (Watcher.see / Tracker.update 가 내놓는 형식 그대로).
    내놓는 것: 위협이 되는 사람 하나 (그 튜플). 없으면 None — `if person_threat(...)` 로 쓴다.

    예측: 사람은 지금 속도 그대로 걷는다고 보고 horizon 초 앞까지 0.1 초 간격으로
    거리를 잰다. 먼 미래일수록 예측이 틀리므로 금지 반경을 시간에 비례해 넓힌다
    (EVADE_UNCERTAINTY_GROWTH [m/s]).
      d(τ) = |사람(τ) - 나| - EVADE_KEEP_DISTANCE - 불확실성·τ
    d < 0 이 되는 순간이 있고, 사람이 나를 향해 오고 있으면 위협이다.

    ⚠️ 서 있는 사람(EVADE_MIN_PERSON_SPEED 미만)은 여기서 다루지 않는다 — 가만히
       있는 사람은 나를 칠 수 없고, 계획기의 사람 둘레 비용이 우회시킨다.
    ⚠️ 나에게서 멀어지는 사람은 위협이 아니다 (뒤따라가는 상황). 그걸 위협으로
       치면 사람 뒤에서 계속 비키느라 앞으로 못 간다.
    """
    horizon = config.EVADE_HORIZON if horizon is None else horizon
    step = 0.1
    for person in people:
        px, py, vx, vy = person[:4]
        if math.hypot(vx, vy) < config.EVADE_MIN_PERSON_SPEED:
            continue
        rx, ry = px - pose[0], py - pose[1]
        coming = rx * vx + ry * vy < 0.0      # 상대 위치와 속도가 반대 방향 = 다가온다
        if not coming:
            continue
        for k in range(int(round(horizon / step)) + 1):
            tau = k * step
            gap = (math.hypot(rx + vx * tau, ry + vy * tau)
                   - config.EVADE_KEEP_DISTANCE
                   - config.EVADE_UNCERTAINTY_GROWTH * tau)
            if gap < 0.0:
                return person
    return None
