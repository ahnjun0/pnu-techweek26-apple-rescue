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

from . import common
from . import config


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


def floor_position(bottom_row, centre_col, pose, width, height, fov):
    """영상에서 바닥에 닿은 점(밑동 행, 가운데 열) → 월드 좌표. 수평선 위면 None.

    거리는 camera_range 의 "바닥" 식과 같다: d = CAMERA_HEIGHT / tan(내려본 각).
    """
    focal = (width / 2.0) / math.tan(fov / 2.0)
    below = math.atan((bottom_row - (height - 1) / 2.0) / focal)
    if below <= 0.0:
        return None
    distance = config.CAMERA_HEIGHT / math.tan(below)
    if distance > config.LOW_MAX_RANGE:
        return None
    bearings = column_bearings(width, fov)
    bearing = float(np.interp(centre_col, np.arange(len(bearings)), bearings))
    x0, y0, theta = pose
    cam_x = x0 + config.CAMERA_FORWARD * math.cos(theta)
    cam_y = y0 + config.CAMERA_FORWARD * math.sin(theta)
    return (cam_x + distance * math.cos(theta + bearing),
            cam_y + distance * math.sin(theta + bearing))


class LowObstacles:
    """LiDAR 에 안 보이는 낮은 물체의 위치 목록 (config 의 '낮은 물체' 설명 참고)."""

    def __init__(self):
        self.points = []          # [[x, y, 본 횟수], ...]

    def add(self, x, y):
        for p in self.points:
            if math.hypot(p[0] - x, p[1] - y) <= config.LOW_MERGE_RADIUS:
                p[2] += 1
                p[0] += (x - p[0]) / p[2]
                p[1] += (y - p[1]) / p[2]
                return
        self.points.append([x, y, 1])

    def positions(self):
        return [(p[0], p[1]) for p in self.points if p[2] >= config.LOW_MIN_SIGHTINGS]


def yolo_low_obstacles(detections, pose, width, height, fov, low_list):
    """YOLO 결과 중 바닥의 작은 물체를 low_list 에 넣는다."""
    for x1, y1, x2, y2, label, _ in detections:
        if label not in config.YOLO_LOW_CLASSES:
            continue
        spot = floor_position(y2 - 0.5, (x1 + x2) / 2.0, pose, width, height, fov)
        if spot is not None:
            low_list.add(*spot)


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


def _scan_camera(image_bgr, pose, fov, target_list, lead_list=None, classify=None,
                 low_list=None):
    """카메라만으로 목표물 위치를 낸다.

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
        if low_list is not None:
            # 바닥에 닿은 빨간 것은 사과든 캔이든 **부딪히면 안 되는 물체** 다.
            # ⚠️ 모양 검사보다 **먼저** 적는다 — 누운 캔은 모양 검사에서 걸러지기 때문이다.
            spot = floor_position(box["top"] + box["h"] - 0.5, box["cx"], pose, width, height, fov)
            if spot is not None:
                low_list.add(*spot)
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
           목표물로 등록된다. 실측: 같은 목표물이 위치 드리프트 때문에 0.52 m 떨어진
           곳에 한 번 더 찍혀 목표물 수보다 많이 확정됐다 — "없는 것을 찾았다" 가 된다.
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
           실측: 후보 3개 중 2개만 확정하고 끝냈다.
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
       위치를 못 정하는 경우는 실제로 흔하다: 3 m(DETECT_MAX_RANGE) 밖, 화면 가장자리에
       잘린 덩어리, YOLO 가 아무것도 못 본 덩어리.
       실측: 구석의 목표물을 카메라로 보고도 기록이 남지 않아, 그 구석에 가까이 가 보지
       않고 복귀했다.

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


def scan(image_bgr, ranges, pose, fov, target_list, lead_list=None, classify=None,
         low_list=None):
    """영상 한 장을 훑어 목표물 목록을 갱신한다. 거리는 카메라만으로 잰다 (_scan_camera).

    돌려주는 것: (이번에 위치까지 알아낸 탐지 수, 버린 탐지 수)
    ⚠️ 사과(지름 0.10 m)는 LiDAR 평면(0.173 m)보다 낮아 LiDAR 로는 거리를 잴 수 없다.
    ranges 는 쓰지 않는다 (호출 형식을 지키려고 받는다).
    """
    if image_bgr is None:
        return 0, 0
    return _scan_camera(image_bgr, pose, fov, target_list, lead_list, classify, low_list)
