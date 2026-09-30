"""카메라로 빨간 목표물을 찾고, LiDAR 거리와 합쳐 월드 좌표를 추정한다.

받는 것: 카메라 BGR 영상, LiDAR 거리 배열, 지금 pose.
내놓는 것: 찾은 목표물 목록 [(x, y), ...] 과 방문 여부.
핵심 아이디어: 카메라가 "어느 쪽" 을, LiDAR 가 "얼마나 멀리" 를 담당한다.
같은 물체를 여러 번 보므로, 가까운 것끼리 묶고 여러 번 본 것만 인정한다.
⚠️ LiDAR 는 평면 한 층만 본다 — 거리를 못 재는 탐지가 실제로 존재한다.
"""

import math

import cv2
import numpy as np

import common
import config


def red_mask(image_bgr):
    """빨간 픽셀만 남긴 흑백 마스크.

    빨강은 HSV 색상환의 양끝(0 근처와 179 근처)에 걸쳐 있어 두 구간으로 잡는다.
    채도 하한이 제일 중요하다 — 바닥 나뭇결이 "빨강" 으로 잡히는 것을 막는다.
    임계값의 근거는 config.py 의 DETECT_* 주석 참고 (측정해서 정했다).
    """
    hsv = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2HSV)
    low_end = cv2.inRange(
        hsv,
        np.array([0, config.DETECT_SAT_MIN, config.DETECT_VALUE_MIN]),
        np.array([config.DETECT_HUE_LOW, 255, 255]))
    high_end = cv2.inRange(
        hsv,
        np.array([config.DETECT_HUE_HIGH, config.DETECT_SAT_MIN,
                  config.DETECT_VALUE_MIN]),
        np.array([179, 255, 255]))
    return low_end | high_end


def column_bearings(width, fov):
    """화면의 각 세로줄이 로봇 기준 몇 rad 방향인가.

    핀홀 카메라:  angle = atan( (u - 중심) / (폭/2) * tan(fov/2) )
    화면 오른쪽(u 증가)은 로봇 기준 오른쪽(음의 각도)이라 부호를 뒤집는다.
    ⚠️ 이 부호는 추측이 아니라 실측이다 — worlds/camera_test.wbt 에서
       왼쪽에 둔 기둥의 실제 방향 +22.9° vs 측정 +22.9° (차이 0.0°).
    """
    columns = np.arange(width)
    normalised = ((columns - (width - 1) / 2.0) / (width / 2.0)
                  * math.tan(fov / 2.0))
    return -np.arctan(normalised)


def blobs(image_bgr, fov):
    """영상에서 빨간 덩어리를 찾는다.

    돌려주는 것: [(픽셀수, 방위각 [rad], 화면상 각폭 [rad]), ...] 큰 것부터.
    각폭은 "그 거리에 그만한 물체가 맞는가" 를 확인하는 데 쓴다.
    """
    mask = red_mask(image_bgr)
    count, _, stats, centroids = cv2.connectedComponentsWithStats(mask, 8)
    bearings = column_bearings(image_bgr.shape[1], fov)

    found = []
    for label in range(1, count):
        area = int(stats[label, cv2.CC_STAT_AREA])
        if area < config.DETECT_MIN_BLOB_PIXELS:
            continue
        centre_x = float(centroids[label][0])
        bearing = float(np.interp(centre_x, np.arange(len(bearings)), bearings))

        left = int(stats[label, cv2.CC_STAT_LEFT])
        right = left + int(stats[label, cv2.CC_STAT_WIDTH]) - 1
        span = abs(float(bearings[max(0, left)] - bearings[min(right, len(bearings) - 1)]))
        found.append((area, bearing, span))
    found.sort(key=lambda item: -item[0])
    return found


def blob_boxes(image_bgr):
    """빨간 덩어리를 **상자** 로 돌려준다 (카메라만으로 거리를 잴 때 쓴다).

    [{area, cx, left, top, w, h, edge}, ...] 큰 것부터. edge 는 화면 가장자리에 잘렸는가
    — 잘린 덩어리는 크기가 틀리므로 거리 계산에 쓰지 않는다.
    """
    mask = red_mask(image_bgr)
    height, width = mask.shape[:2]
    count, _, stats, centroids = cv2.connectedComponentsWithStats(mask, 8)
    found = []
    for label in range(1, count):
        left, top, w, h, area = (int(v) for v in stats[label])
        if area < config.DETECT_MIN_BLOB_PIXELS:
            continue
        found.append({
            "area": area, "cx": float(centroids[label][0]),
            "left": left, "top": top, "w": w, "h": h,
            "edge": left <= 0 or top <= 0 or left + w >= width or top + h >= height,
        })
    found.sort(key=lambda b: -b["area"])
    return found


def camera_range(box, width, height, fov):
    """카메라에서 목표물 중심까지의 수평 거리를 **두 가지로** 잰다. (크기, 바닥) [m].

    크기: 반지름 TARGET_RADIUS 인 공은 각반지름 asin(R/d) 로 보인다 → d = R / sin(반각).
    바닥: 공이 바닥에 놓여 있으면 밑동은 카메라보다 CAMERA_HEIGHT 아래다 →
          d = CAMERA_HEIGHT / tan(밑동의 내려본 각).
    밑동이 수평선 위면 바닥에 놓인 물체가 아니다 → None.

    ⚠️ 두 값이 서로 맞는지가 핵심이다. 사과보다 큰 빨간 물체(소화기)는 크기로 재면
       실제보다 가깝게 나와 바닥 거리와 어긋난다 (기록: 81%). 진짜 사과는 1~7%.
    ⚠️ 두 값 모두 진짜 거리보다 15~20% 짧게 나왔다 (표본 3개). 둘 다 초점거리에
       비례하므로 시야각 해석이 원인일 수 있으나 확정 못 했다 — 판별에는 지장 없다.
    """
    focal = (width / 2.0) / math.tan(fov / 2.0)
    half = math.atan((box["w"] / 2.0) / focal)
    d_size = config.TARGET_RADIUS / math.sin(half) if half > 0 else math.inf
    centre_row = (height - 1) / 2.0
    bottom = box["top"] + box["h"] - 0.5
    below = math.atan((bottom - centre_row) / focal)
    if below <= 0.0:
        return None
    d_ground = config.CAMERA_HEIGHT / math.tan(below)
    return d_size, d_ground


# 관찰용 계수기 — YOLO 가 몇 번 받고, 버리고, 못 봤는지 (결정에는 쓰지 않는다)
YOLO_COUNT = {}


def yolo_verdict(box, detections):
    """빨간 덩어리 상자 하나를 YOLO 결과로 판정한다: "accept" / "reject" / "unknown".

    덩어리 중심이 들어 있는 YOLO 상자의 라벨로 정한다. 사과 계열이 하나라도 있으면
    받는다 (병과 공이 겹쳐 잡혀도 사과를 잃지 않는 쪽).
    detections: [(x1, y1, x2, y2, label, conf), ...]
    """
    cx = box["left"] + box["w"] / 2.0
    cy = box["top"] + box["h"] / 2.0
    labels = {label for x1, y1, x2, y2, label, _ in detections
              if x1 <= cx <= x2 and y1 <= cy <= y2}
    if labels & set(config.YOLO_ACCEPT):
        return "accept"
    if labels & set(config.YOLO_REJECT):
        return "reject"
    return "unknown"


def _scan_camera(image_bgr, pose, fov, target_list, lead_list=None, classify=None):
    """카메라만으로 목표물 위치를 낸다 (DETECT_RANGING == "camera").

    classify 를 주면 모양 검사를 통과한 덩어리에 한해 YOLO 에 묻는다 (한 장에 한 번).
    """
    detections = None
    height, width = image_bgr.shape[:2]
    bearings = column_bearings(width, fov)
    x0, y0, theta = pose
    cam_x = x0 + config.CAMERA_FORWARD * math.cos(theta)
    cam_y = y0 + config.CAMERA_FORWARD * math.sin(theta)
    placed = rejected = 0
    for box in blob_boxes(image_bgr):
        bearing = float(np.interp(box["cx"], np.arange(len(bearings)), bearings))
        if box["edge"]:
            # 잘려서 크기를 모른다 — 방위만 남기고 가까이 가서 다시 본다
            if lead_list is not None:
                lead_list.add(pose, bearing)
            rejected += 1
            continue
        if box["h"] > config.DETECT_MAX_ASPECT * box["w"]:
            rejected += 1                    # 세로로 길다 (소화기 등)
            continue
        if box["h"] < config.DETECT_MIN_ASPECT * box["w"]:
            rejected += 1                    # 납작하다 (누운 캔 등)
            continue
        measured = camera_range(box, width, height, fov)
        if measured is None:
            rejected += 1                    # 바닥에 안 놓였다 (선반 위 등)
            continue
        d_size, d_ground = measured
        if abs(d_size - d_ground) > config.DETECT_RANGE_AGREEMENT * d_ground:
            rejected += 1                    # 사과 크기가 아니다 (소화기 등)
            continue
        distance = (d_size + d_ground) / 2.0
        if distance > config.DETECT_MAX_RANGE:
            if lead_list is not None:
                lead_list.add(pose, bearing)
            rejected += 1
            continue
        if classify is not None:
            if detections is None:
                detections = classify(image_bgr)     # 빨간 게 있을 때만 돈다
            verdict = yolo_verdict(box, detections)
            YOLO_COUNT[verdict] = YOLO_COUNT.get(verdict, 0) + 1
            if verdict == "reject":
                rejected += 1                # 병·컵 (캔)
                continue
            if verdict == "unknown":
                if lead_list is not None:
                    lead_list.add(pose, bearing)   # 가까이 가서 다시 본다
                rejected += 1
                continue
        x = cam_x + distance * math.cos(theta + bearing)
        y = cam_y + distance * math.sin(theta + bearing)
        if not position_is_sane(x, y):
            rejected += 1
            continue
        target_list.add(x, y)
        placed += 1
    return placed, rejected


def size_is_consistent(span, distance, radius=None):
    """그 거리에 있는 물체가 화면에서 그만한 각폭으로 보이는 게 맞는가.

    반지름 r 인 물체를 거리 d 에서 보면 각폭은 대략 2*atan(r/d) 다.
    카메라가 본 각폭이 그것과 크게 어긋나면, 방위각과 거리가 서로 다른 물체를
    가리키고 있다는 뜻이다 (예: 멀리 있는 목표물 앞으로 사람이 지나갈 때).
    """
    radius = config.APPROACH_TARGET_RADIUS if radius is None else radius
    if distance <= 0.0 or span <= 0.0:
        return False
    expected = 2.0 * math.atan(radius / distance)
    ratio = span / expected
    return 1.0 / config.DETECT_SIZE_TOLERANCE <= ratio <= config.DETECT_SIZE_TOLERANCE


def range_at(ranges, bearing, span=None):
    """그 방위를 보는 LiDAR 거리 [m]. 못 재면 None.

    덩어리 중심 방향 하나만 보면 광선이 물체 옆을 스칠 수 있으므로 주변을 함께 본다.

    ⚠️ 예전에는 그 창 안의 **최솟값** 을 썼다. 창 안에 목표물보다 가까운 것이
       하나라도 있으면 거리가 짧게 나오고, 위치가 **로봇과 목표물 사이 빈 공간** 에
       찍힌다 — 아무것도 없는 자리에 유령 목표물이 생긴다.
       실측(무작위 월드 rand3): 검출 4개 중 2개가 빈 공간의 가짜였고, 그 둘을
       확정·방문하고 3개를 채웠다고 판단해 76초 만에 복귀했다. 진짜 목표물 하나는
       아예 못 찾았다. `size_is_consistent` 도 못 걸렀다 — 관용 2.5 는 거리가 2배
       틀려도 통과시킨다 (비율 0.5 > 1/2.5).

    그래서 **중앙값** 을 쓴다. 카메라가 덩어리의 각폭(span)을 알려주므로 그 폭 안만
    본다. 폭 안은 대부분 목표물 표면이므로 중앙값은 목표물 거리에 붙고, 스치는
    광선 한둘에 끌려가지 않는다.
    """
    ranges = np.asarray(ranges, dtype=np.float64)
    angles = common.lidar_angles()
    if len(angles) != len(ranges):
        angles = np.linspace(-math.pi, math.pi, len(ranges), endpoint=False)

    # 창은 덩어리 각폭의 절반. 너무 좁으면 광선이 한 개도 안 들어오므로 하한을 둔다.
    half = math.radians(config.DETECT_BEARING_WINDOW)
    if span is not None and span > 0.0:
        half = max(half, span / 2.0)
    near = np.abs(common.wrap_angle(angles - bearing)) <= half
    values = ranges[near]
    values = values[np.isfinite(values)
                    & (values >= config.LIDAR_MIN_RANGE)
                    & (values <= config.DETECT_MAX_RANGE)]
    return float(np.median(values)) if len(values) else None


def world_position(pose, bearing, distance):
    """로봇 기준 (방위, 거리) → 월드 좌표 (x, y)."""
    x, y, theta = pose
    angle = theta + bearing
    return (x + distance * math.cos(angle), y + distance * math.sin(angle))


class Target:
    """찾은 목표물 하나. 같은 물체를 여러 번 보므로 평균을 내며 다듬는다."""

    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.sightings = 1
        self.visited = False

    @property
    def position(self):
        return (self.x, self.y)

    @property
    def confirmed(self):
        """여러 번 본 것만 진짜로 친다 (한 번 반짝한 잡음을 거르려는 것)."""
        return self.sightings >= config.DETECT_MIN_SIGHTINGS

    def merge(self, x, y):
        """같은 물체를 또 봤다. 위치를 평균 쪽으로 조금 당긴다."""
        self.sightings += 1
        # ⚠️ 추정이 얼마나 움직였는지 누적해 둔다. 목표물 직전에서 주저하는 것이
        #    "로봇이 망설인다" 가 아니라 "목표물이 움직인다" 인지 가르려면 필요하다.
        self.wobble = getattr(self, "wobble", 0.0) + math.hypot(x - self.x,
                                                                y - self.y)
        weight = 1.0 / self.sightings
        self.x += (x - self.x) * weight
        self.y += (y - self.y) * weight


class TargetList:
    """목표물 목록. 가까운 탐지끼리 묶고, 방문 여부를 기억한다."""

    def __init__(self, merge_radius=None, limit=None):
        self.radius = (config.DETECT_MERGE_RADIUS if merge_radius is None
                       else merge_radius)
        self.targets = []
        # 찾아야 할 개수를 알면 그보다 많이 확정하지 않는다 (아래 confirmed 참고).
        self.limit = limit

    # --- 읽기 ---------------------------------------------------------

    @property
    def confirmed(self):
        """확정된 목표물. **개수를 알면 그보다 많이 내주지 않는다.**

        ⚠️ 왜 필요한가: 헛것 하나가 확정 문턱(DETECT_MIN_SIGHTINGS)을 넘으면 진짜
           목표물로 등록된다. 실측(corridor2): 같은 목표물이 위치 드리프트 때문에
           0.52 m 떨어진 **아레나 밖**(-3.23, +2.37)에 한 번 더 찍혀 **4/3** 이
           됐다. 대회에서는 "없는 것을 찾았다" 가 된다.
           선행 대회도 같은 문제를 같은 방식으로 다룬다 — DARPA SubT 는 팀마다
           **보고 횟수를 정해 두어** 오검출을 억제했고, RoboCup Rescue 는
           오검출에 감점을 준다.
        방문한 것은 이미 점수이므로 남기고, 나머지는 **본 횟수가 많은 순** 으로
        자른다 — 많이 본 것일수록 진짜일 가능성이 높다.
        """
        hits = [t for t in self.targets if t.confirmed]
        if self.limit is None or len(hits) <= self.limit:
            return hits
        visited = [t for t in hits if t.visited]
        rest = sorted((t for t in hits if not t.visited),
                      key=lambda t: t.sightings, reverse=True)
        room = max(self.limit - len(visited), 0)
        return visited + rest[:room]

    def unvisited(self):
        return [t for t in self.confirmed if not t.visited]

    def positions(self):
        """화면 표시용 — 확인된 목표물 좌표 목록."""
        return [t.position for t in self.confirmed]

    def best_unconfirmed(self, minimum=None):
        """확정엔 못 미쳤지만 "뭔가 있다" 싶은 후보 중 가장 많이 본 것.

        ⚠️ 왜 필요한가: 확정 문턱이 DETECT_MIN_SIGHTINGS(25)라, 지나가며 몇 번만
           본 목표물은 임무 판단에 **아예 안 보인다**. 그래서 로봇이 "저기 뭔가
           있다" 는 것을 알면서도 다 찾았다고 판단하고 복귀했다.
           실측: corridor 지도에서 후보 3개 중 2개만 확정하고 DONE.
           확신이 없으면 가까이 가서 더 보면 된다 (능동 인지).
        """
        minimum = (config.DETECT_VERIFY_SIGHTINGS if minimum is None
                   else minimum)
        maybe = [t for t in self.targets
                 if not t.confirmed and not t.visited
                 and t.sightings >= minimum]
        if not maybe:
            return None
        return max(maybe, key=lambda t: t.sightings)

    def nearest_unvisited(self, x, y):
        """아직 안 가 본 것 중 제일 가까운 것. 없으면 None."""
        candidates = self.unvisited()
        if not candidates:
            return None
        return min(candidates, key=lambda t: common.distance(x, y, t.x, t.y))

    # --- 쓰기 ---------------------------------------------------------

    def add(self, x, y):
        """탐지 하나를 목록에 반영한다. 해당 Target 을 돌려준다."""
        for target in self.targets:
            if common.distance(x, y, target.x, target.y) <= self.radius:
                target.merge(x, y)
                return target
        fresh = Target(x, y)
        self.targets.append(fresh)
        return fresh

    def mark_visited(self, x, y):
        """그 근처 목표물을 방문 처리한다. 처리했으면 True."""
        for target in self.confirmed:
            if common.distance(x, y, target.x, target.y) <= self.radius:
                target.visited = True
                return True
        return False


class LeadList:
    """거리를 못 잰 관측 = **방위만 아는 단서**.

    ⚠️ 왜 필요한가: scan() 은 카메라가 빨간 덩어리를 찾아도 그 방위의 LiDAR
       거리를 못 재면 그 관측을 **버렸다**. 주석에는 "봤지만 아직 모른다 로 두고
       가까이 가서 다시 본다" 고 적혀 있었는데 **그런 장치가 없었다** — 의도만
       있고 구현이 없었다.
       거리를 못 재는 경우는 실제로 흔하다: 3 m(DETECT_MAX_RANGE) 밖, LiDAR 평면
       위/아래, 아직 지도에 안 찍힌 물체.
       실측(comb0): 오른쪽 아래 구석의 목표물을 카메라로 봤지만 아무 기록도
       남지 않아, 로봇은 LiDAR 지도상 7% 를 "갈 수 없는 곳" 으로 판정하고
       554초에 2/3 으로 복귀했다. 그 구석에 1.87 m 보다 가까이 간 적이 없다.

    그래서 방위만이라도 남긴다. 그 방향으로 다가가면 거리를 잴 수 있게 되고,
    그때부터는 기존 검출 경로가 확정하거나 기각한다 (능동 인지).
    """

    def __init__(self):
        self.leads = []          # [{x, y, bearing, sightings, tries}]

    def add(self, pose, bearing):
        """(로봇 위치, 방위) 단서를 남긴다. 비슷한 방향이면 합친다."""
        x, y, theta = pose
        ray = common.wrap_angle(theta + bearing)   # 월드 기준 방위
        for lead in self.leads:
            near = common.distance(x, y, lead["x"], lead["y"]) <= config.LEAD_MERGE_DISTANCE
            same = abs(common.wrap_angle(ray - lead["ray"])) <= math.radians(
                config.LEAD_MERGE_DEGREES)
            if near and same:
                lead["sightings"] += 1
                lead["x"], lead["y"], lead["ray"] = x, y, ray
                return lead
        fresh = {"x": x, "y": y, "ray": ray, "sightings": 1, "tries": 0}
        self.leads.append(fresh)
        return fresh

    def best(self, minimum=None):
        """쫓을 만한 단서 — 여러 번 본 것부터. 없으면 None."""
        minimum = (config.LEAD_MIN_SIGHTINGS if minimum is None else minimum)
        ready = [l for l in self.leads
                 if l["sightings"] >= minimum
                 and l["tries"] < config.LEAD_MAX_TRIES]
        if not ready:
            return None
        return max(ready, key=lambda l: l["sightings"])

    def look_point(self, lead, distance=None):
        """그 단서를 **가까이서 볼 수 있는** 자리 (월드 좌표).

        단서는 방위만 아니까, 그 방향으로 얼마쯤 나아간 점을 목표로 삼는다.
        거리를 잴 수 있는 범위(DETECT_MAX_RANGE) 안으로 들어가는 것이 목적이다.
        """
        step = config.LEAD_STEP_DISTANCE if distance is None else distance
        return (lead["x"] + step * math.cos(lead["ray"]),
                lead["y"] + step * math.sin(lead["ray"]))

    def give_up(self, lead):
        """한 번 쫓아 봤다고 기록한다 (무한 추격 방지)."""
        lead["tries"] += 1

    def drop_near(self, x, y, radius=None):
        """그 자리 근처에서 생긴 단서를 버린다 (이미 가까이 가 봤다는 뜻)."""
        radius = (config.LEAD_MERGE_DISTANCE if radius is None else radius)
        before = len(self.leads)
        self.leads = [l for l in self.leads
                      if common.distance(x, y, l["x"], l["y"]) > radius]
        return before - len(self.leads)


def position_is_sane(x, y, grid=None):
    """그 자리에 물체가 있을 수 있는가.

    지도 밖이면 말이 안 된다. 오도메트리가 틀어진 채 탐지하면 아레나 밖에
    목표물이 등록되고, 로봇이 갈 수 없는 그곳에 영원히 매달린다
    (실제로 (-3.8, 3.0) 같은 곳에 등록돼 900초를 허비했다).
    """
    row, col = common.to_cell(x, y)
    return bool(common.in_bounds(row, col))


def scan(image_bgr, ranges, pose, fov, target_list, lead_list=None, classify=None):
    """영상 한 장을 훑어 목표물 목록을 갱신한다.

    돌려주는 것: (이번에 위치까지 알아낸 탐지 수, 버린 탐지 수)
    버리는 경우가 실제로 있다:
      - LiDAR 평면보다 낮거나 높은 물체, 사거리 밖의 물체 → 거리를 못 잰다
      - 방향과 거리가 서로 다른 물체를 가리킬 때 → 크기가 안 맞아 걸러진다
    그때는 "봤지만 아직 모른다" 로 두고 가까이 가서 다시 본다.
    """
    if image_bgr is None:
        return 0, 0
    if config.DETECT_RANGING == "camera":
        return _scan_camera(image_bgr, pose, fov, target_list, lead_list, classify)

    placed = 0
    rejected = 0
    for _, bearing, span in blobs(image_bgr, fov):
        distance = range_at(ranges, bearing, span)
        if distance is None:
            # 봤지만 거리를 모른다 — **방위만이라도 남긴다** (LeadList 설명 참고).
            if lead_list is not None:
                lead_list.add(pose, bearing)
            rejected += 1
            continue
        if not size_is_consistent(span, distance):
            rejected += 1
            continue
        x, y = world_position(pose, bearing, distance)
        if not position_is_sane(x, y):
            rejected += 1
            continue
        target_list.add(x, y)
        placed += 1
    return placed, rejected
