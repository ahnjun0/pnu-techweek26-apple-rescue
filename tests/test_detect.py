"""카메라 목표물 탐지를 Webots 없이 확인한다.

영상은 numpy 로 지어낸다. 색 임계값·방위각 부호는 실제 측정으로 정한 값이라,
여기서는 "그 규약대로 동작하는가" 를 본다.
"""

import math

import numpy as np
import pytest

import common
import config
import detect


def blank(width=128, height=96, colour=(40, 40, 40)):
    """BGR 영상 한 장 (기본은 어두운 회색)."""
    image = np.zeros((height, width, 3), dtype=np.uint8)
    image[:, :] = colour
    return image


def paint_red(image, x0, x1, y0=30, y1=60):
    image[y0:y1, x0:x1] = (0, 0, 255)      # BGR 에서 순수 빨강
    return image


def clear_lidar(distance=1.5):
    return np.full(config.LIDAR_RESOLUTION, distance, dtype=np.float64)


FOV = 1.0      # practice.wbt 의 카메라와 같은 값


# --- 색 걸러내기 -------------------------------------------------------------

def test_pure_red_is_detected():
    mask = detect.red_mask(paint_red(blank(), 50, 70))
    assert mask[45, 60] > 0


def test_dark_grey_is_not_red():
    assert detect.red_mask(blank()).sum() == 0


def from_hsv(hue, saturation, value, width=128, height=96):
    """HSV 로 칠한 영상 한 장. 실제로 측정한 색을 그대로 쓰려는 것."""
    import cv2
    patch = np.full((height, width, 3), (hue, saturation, value), dtype=np.uint8)
    return cv2.cvtColor(patch, cv2.COLOR_HSV2BGR)


def test_the_measured_floor_colour_is_not_red():
    """바닥 나뭇결이 빨강으로 잡히면 온 바닥이 목표물이 된다.

    실제로 그랬다 — 바닥 픽셀의 29.9% 가 오검출됐다.
    debug/camera_lidar_check.py 로 잰 값을 그대로 쓴다:
        진짜 바닥  H 9,  S  98, V 173
        진짜 목표물 H 179, S 219, V 207
    채도(S) 하한이 이 둘을 가른다.
    """
    assert detect.red_mask(from_hsv(9, 98, 173)).sum() == 0, \
        "측정한 바닥 색이 빨강으로 잡힌다 — DETECT_SAT_MIN 을 올릴 것"


def test_the_measured_target_colour_is_red():
    assert detect.red_mask(from_hsv(179, 219, 207)).all(), \
        "측정한 목표물 색이 안 잡힌다 — 임계값이 너무 빡빡하다"


def test_saturation_threshold_sits_between_the_two_measurements():
    """임계값이 두 측정값 사이에 있어야 한다. 한쪽에 붙으면 곧 깨진다."""
    assert 98 < config.DETECT_SAT_MIN < 219


def test_bright_blue_is_not_red():
    assert detect.red_mask(blank(colour=(255, 0, 0))).sum() == 0


# --- 화면 위치 → 방위각 -------------------------------------------------------

def test_centre_column_is_straight_ahead():
    bearings = detect.column_bearings(128, FOV)
    assert bearings[63] == pytest.approx(0.0, abs=0.01)


def test_left_of_image_is_positive_bearing():
    """왼쪽이 + 다. 실측으로 확정했다 (실제 +22.9° vs 측정 +22.9°)."""
    bearings = detect.column_bearings(128, FOV)
    assert bearings[0] > 0 > bearings[-1]


def test_edges_match_the_field_of_view():
    bearings = detect.column_bearings(128, FOV)
    assert abs(bearings[0]) == pytest.approx(FOV / 2, rel=0.02)


def test_blob_reports_its_angular_width():
    narrow = detect.blobs(paint_red(blank(), 60, 66), FOV)[0]
    wide = detect.blobs(paint_red(blank(), 40, 90), FOV)[0]
    assert wide[2] > narrow[2]


# --- 방향과 거리가 같은 물체를 가리키는가 --------------------------------------

def test_size_matches_at_the_right_distance():
    """반지름 r 물체를 거리 d 에서 보면 각폭은 2*atan(r/d) 다."""
    radius = config.APPROACH_TARGET_RADIUS
    for distance in (0.6, 1.2, 2.5):
        span = 2 * math.atan(radius / distance)
        assert detect.size_is_consistent(span, distance)


def test_size_rejects_a_near_reading_for_a_far_object():
    """멀리 있는 목표물 앞으로 사람이 지나가면 LiDAR 가 짧은 거리를 준다.

    그대로 믿으면 목표물이 사람 자리에 찍힌다 — 실제로 가짜 목표물 2개가 생겼다.
    화면에서 작게 보이는데 거리가 가깝다고 하면 앞뒤가 안 맞는다.
    """
    far_span = 2 * math.atan(config.APPROACH_TARGET_RADIUS / 3.0)
    assert not detect.size_is_consistent(far_span, 1.0)


def test_size_rejects_a_far_reading_for_a_near_object():
    near_span = 2 * math.atan(config.APPROACH_TARGET_RADIUS / 0.5)
    assert not detect.size_is_consistent(near_span, 3.0)


def test_size_check_handles_degenerate_input():
    assert not detect.size_is_consistent(0.0, 1.0)
    assert not detect.size_is_consistent(0.1, 0.0)


def test_scan_rejects_an_inconsistent_fusion():
    """방향은 먼 목표물, 거리는 가까운 사람 → 목표물로 받아들이면 안 된다."""
    targets = detect.TargetList()
    image = paint_red(blank(), 62, 66)          # 아주 작게 보인다 = 멀다
    close = clear_lidar(0.5)                    # 그런데 거리는 0.5 m 라고 한다
    placed, rejected = detect.scan(image, close, (0.0, 0.0, 0.0), FOV, targets)
    assert (placed, rejected) == (0, 1)
    assert targets.targets == []


def test_blob_bearing_follows_where_it_is_painted():
    left = detect.blobs(paint_red(blank(), 5, 25), FOV)
    right = detect.blobs(paint_red(blank(), 100, 120), FOV)
    assert len(left) == 1 and len(right) == 1
    assert left[0][1] > 0 > right[0][1]


def test_tiny_speck_is_ignored():
    image = blank()
    image[50, 60] = (0, 0, 255)          # 한 픽셀
    assert detect.blobs(image, FOV) == []


def test_two_blobs_are_reported_separately():
    image = paint_red(blank(), 10, 30)
    paint_red(image, 90, 110)
    assert len(detect.blobs(image, FOV)) == 2


# --- LiDAR 거리 붙이기 --------------------------------------------------------

def test_range_at_reads_the_lidar_in_that_direction():
    ranges = np.full(config.LIDAR_RESOLUTION, np.inf)
    angles = common.lidar_angles()
    ranges[np.abs(angles) < 0.1] = 1.2
    assert detect.range_at(ranges, 0.0) == pytest.approx(1.2)


def test_range_at_returns_none_when_lidar_cannot_see_it():
    """LiDAR 는 평면 한 층만 본다. 낮거나 높거나 먼 물체는 거리를 못 잰다.

    worlds/camera_test.wbt 로 실증한 실제 상황이다.
    """
    assert detect.range_at(np.full(config.LIDAR_RESOLUTION, np.inf), 0.0) is None


def test_range_at_ignores_readings_beyond_the_useful_range():
    far = np.full(config.LIDAR_RESOLUTION, config.DETECT_MAX_RANGE + 1.0)
    assert detect.range_at(far, 0.0) is None


# --- 월드 좌표 ---------------------------------------------------------------

@pytest.mark.parametrize("theta,bearing,expected", [
    (0.0, 0.0, (1.0, 0.0)),                    # 동쪽을 보고 정면
    (0.0, math.pi / 2, (0.0, 1.0)),            # 동쪽을 보고 왼쪽 → 북쪽
    (math.pi / 2, 0.0, (0.0, 1.0)),            # 북쪽을 보고 정면
    (math.pi, 0.0, (-1.0, 0.0)),               # 서쪽을 보고 정면
])
def test_world_position(theta, bearing, expected):
    got = detect.world_position((0.0, 0.0, theta), bearing, 1.0)
    assert got == pytest.approx(expected, abs=1e-9)


def test_world_position_accounts_for_robot_offset():
    got = detect.world_position((2.0, -1.0, 0.0), 0.0, 0.5)
    assert got == pytest.approx((2.5, -1.0))


# --- 목표물 목록 -------------------------------------------------------------

def test_new_detection_creates_a_target():
    targets = detect.TargetList()
    targets.add(1.0, 1.0)
    assert len(targets.targets) == 1


def test_nearby_detections_merge_into_one():
    """같은 물체를 여러 번 본다. 매번 새 목표물이 되면 개수가 폭발한다."""
    targets = detect.TargetList(merge_radius=0.4)
    for offset in (0.0, 0.05, -0.08, 0.1):
        targets.add(1.0 + offset, 1.0)
    assert len(targets.targets) == 1
    assert targets.targets[0].sightings == 4


def test_distant_detections_stay_separate():
    targets = detect.TargetList(merge_radius=0.4)
    targets.add(1.0, 1.0)
    targets.add(3.0, 1.0)
    assert len(targets.targets) == 2


def test_a_single_sighting_is_not_confirmed():
    """한 번 반짝한 것은 잡음일 수 있다."""
    targets = detect.TargetList()
    targets.add(1.0, 1.0)
    assert targets.confirmed == []


def test_repeated_sightings_confirm_it():
    targets = detect.TargetList()
    for _ in range(config.DETECT_MIN_SIGHTINGS):
        targets.add(1.0, 1.0)
    assert len(targets.confirmed) == 1


def test_merging_pulls_the_position_toward_the_average():
    targets = detect.TargetList(merge_radius=1.0)
    targets.add(1.0, 0.0)
    targets.add(1.2, 0.0)
    assert 1.0 < targets.targets[0].x < 1.2


def test_nearest_unvisited_picks_the_closest():
    targets = detect.TargetList(merge_radius=0.3)
    for _ in range(config.DETECT_MIN_SIGHTINGS):
        targets.add(1.0, 0.0)
        targets.add(5.0, 0.0)
    assert targets.nearest_unvisited(0.0, 0.0).x == pytest.approx(1.0)


def test_visited_targets_are_not_offered_again():
    targets = detect.TargetList(merge_radius=0.3)
    for _ in range(config.DETECT_MIN_SIGHTINGS):
        targets.add(1.0, 0.0)
    targets.mark_visited(1.0, 0.0)
    assert targets.nearest_unvisited(0.0, 0.0) is None


def test_mark_visited_needs_to_be_near():
    targets = detect.TargetList(merge_radius=0.3)
    for _ in range(config.DETECT_MIN_SIGHTINGS):
        targets.add(1.0, 0.0)
    assert targets.mark_visited(5.0, 5.0) is False


# --- 한 장 훑기 --------------------------------------------------------------

def test_scan_places_a_target_in_front():
    targets = detect.TargetList()
    # 1.5 m 거리의 반지름 0.1 m 물체는 화면에서 약 7.6도 = 약 17 픽셀
    image = paint_red(blank(), 55, 72)
    placed, rejected = detect.scan(image, clear_lidar(1.5), (0.0, 0.0, 0.0),
                                   FOV, targets)
    assert placed == 1 and rejected == 0
    assert targets.targets[0].x == pytest.approx(1.5, abs=0.2)


def test_scan_counts_detections_it_cannot_range():
    """카메라엔 보이는데 LiDAR 가 거리를 못 재는 경우가 실제로 있다."""
    targets = detect.TargetList()
    image = paint_red(blank(), 55, 72)
    placed, rejected = detect.scan(image, np.full(config.LIDAR_RESOLUTION, np.inf),
                                   (0.0, 0.0, 0.0), FOV, targets)
    assert (placed, rejected) == (0, 1)
    assert targets.targets == []


def test_position_outside_the_map_is_rejected():
    """오도메트리가 틀어지면 아레나 밖에 목표물이 등록된다.

    로봇은 거기 갈 수 없으므로 영원히 매달리게 된다 (실제로 900초를 허비했다).
    """
    assert detect.position_is_sane(0.0, 0.0)
    assert not detect.position_is_sane(99.0, 99.0)


def test_scan_drops_a_detection_that_lands_outside_the_map():
    targets = detect.TargetList()
    image = paint_red(blank(), 55, 72)
    far_pose = (config.MAP_ORIGIN_X - 5.0, 0.0, 0.0)
    placed, rejected = detect.scan(image, clear_lidar(1.5), far_pose, FOV, targets)
    assert (placed, rejected) == (0, 1)


def test_scan_without_an_image_does_nothing():
    targets = detect.TargetList()
    assert detect.scan(None, clear_lidar(), (0.0, 0.0, 0.0), FOV, targets) == (0, 0)


def test_repeated_scans_of_the_same_object_confirm_one_target():
    """로봇이 조금씩 움직이며 같은 기둥을 계속 본다 → 목표물 하나여야 한다."""
    targets = detect.TargetList()
    image = paint_red(blank(), 55, 72)
    for step in range(config.DETECT_MIN_SIGHTINGS + 2):
        pose = (0.0 + step * 0.02, 0.0, 0.0)
        detect.scan(image, clear_lidar(1.5), pose, FOV, targets)
    assert len(targets.confirmed) == 1


def test_a_stray_close_ray_does_not_pull_the_target_nearer():
    """목표물 방위에 스치는 가까운 광선 하나가 위치를 끌어당기면 안 된다.

    ⚠️ 회귀 방지. range_at() 이 창 안의 **최솟값** 을 썼다. 그러면 스치는 광선
       하나 때문에 거리가 짧게 나오고, 목표물 위치가 **로봇과 목표물 사이 빈 공간**
       에 찍힌다 — 아무것도 없는 자리에 유령 목표물이 생긴다.
       실측(무작위 월드 rand3): 검출 4개 중 2개가 빈 공간의 가짜였고, 그 둘을
       확정·방문하고 다 찾았다고 판단해 76초 만에 복귀했다 (진짜 하나는 못 찾음).
    """
    angles = common.lidar_angles()
    ranges = np.full(config.LIDAR_RESOLUTION, 10.0)

    bearing = 0.0
    span = math.radians(6.0)          # 목표물이 6도로 보인다
    inside = np.abs(common.wrap_angle(angles - bearing)) <= span / 2.0
    ranges[inside] = 2.0              # 목표물은 2 m 에 있다

    # 그 창 가장자리에 스치는 광선 하나만 0.8 m
    edge = int(np.argmax(inside))
    ranges[edge] = 0.8

    got = detect.range_at(ranges, bearing, span)
    assert got is not None
    assert abs(got - 2.0) < 0.2, \
        f"스치는 광선 하나에 끌려가면 안 된다: {got:.2f} m (진짜 2.0 m)"


# --- 방위만 아는 단서 (lead) --------------------------------------------------

def test_sighting_without_range_is_kept_as_a_lead():
    """거리를 못 잰 관측을 버리지 않고 방위로 남긴다.

    ⚠️ 회귀 방지. scan() 은 카메라가 빨간 덩어리를 찾아도 그 방위의 LiDAR 거리를
       못 재면 그 관측을 **버렸다**. 주석에는 "가까이 가서 다시 본다" 고 적혀
       있었으나 그런 장치가 없었다.
       실측(comb0): 오른쪽 아래 구석 목표물을 보고도 기록이 없어 554초에 2/3 으로
       복귀했다 — 그 구석에 1.87 m 보다 가까이 간 적이 없다.
    """
    import numpy as np

    import config
    import detect

    # 빨간 덩어리가 정면에 보이는 영상
    image = np.zeros((96, 128, 3), dtype=np.uint8)
    image[30:70, 54:74] = (0, 0, 255)          # BGR 빨강

    # LiDAR 는 그 방향에서 아무것도 못 잰다 (전부 사거리 밖)
    ranges = np.full(config.LIDAR_RESOLUTION, np.inf)

    targets = detect.TargetList()
    leads = detect.LeadList()
    placed, rejected = detect.scan(image, ranges, (0.0, 0.0, 0.0), 1.0,
                                   targets, leads)

    assert placed == 0, "거리를 모르는데 위치를 찍으면 안 된다"
    assert rejected >= 1
    assert len(leads.leads) >= 1, "봤다는 사실이 사라졌다 (단서가 안 남았다)"


def test_lead_is_chased_only_after_enough_sightings_and_then_given_up():
    """한 번 반짝한 것은 안 쫓고, 몇 번 본 것만 쫓되 무한히 쫓지 않는다."""
    import math

    import config
    import detect

    leads = detect.LeadList()
    pose = (0.0, 0.0, 0.0)
    leads.add(pose, 0.0)
    assert leads.best() is None, "한 번 본 것을 쫓으면 헛걸음이 된다"

    for _ in range(config.LEAD_MIN_SIGHTINGS):
        leads.add(pose, 0.0)
    lead = leads.best()
    assert lead is not None

    # 그 방향으로 나아간 점이 목표가 된다
    px, py = leads.look_point(lead)
    assert px > 0.0 and abs(py) < 1e-6, f"정면 단서인데 목표가 ({px}, {py})"
    assert abs(math.hypot(px, py) - config.LEAD_STEP_DISTANCE) < 1e-6

    for _ in range(config.LEAD_MAX_TRIES):
        leads.give_up(lead)
    assert leads.best() is None, "같은 단서를 무한히 쫓으면 안 된다"


def test_more_confirmed_than_expected_keeps_only_the_best_seen():
    """찾아야 할 개수보다 많이 확정되면, 본 횟수 상위만 남긴다.

    ⚠️ 헛것 하나가 확정 문턱을 넘으면 진짜 목표물로 등록된다.
       실측(corridor2): 같은 목표물이 위치 드리프트 때문에 0.52 m 떨어진
       **아레나 밖**(-3.23, +2.37)에 한 번 더 찍혀 **4/3** 이 됐다. 대회에서는
       "없는 것을 찾았다" 가 된다.
       선행 대회도 같은 방식이다 — DARPA SubT 는 보고 횟수를 정해 오검출을
       억제했고, RoboCup Rescue 는 오검출에 감점을 준다.
    """
    import config
    import detect

    store = detect.TargetList(limit=3)
    for i, seen in enumerate([40, 35, 30, 28]):
        for _ in range(seen):
            store.add(i * 1.0, 0.0)

    assert len(store.confirmed) == 3, (
        f"개수를 알면서 {len(store.confirmed)}개를 확정했다")
    assert sorted((t.sightings for t in store.confirmed), reverse=True) == [40, 35, 30], (
        "본 횟수가 적은 것을 남겼다")

    # 방문한 것은 이미 점수이므로 잘라내면 안 된다
    store2 = detect.TargetList(limit=2)
    for i, seen in enumerate([10 + config.DETECT_MIN_SIGHTINGS,
                              config.DETECT_MIN_SIGHTINGS,
                              5 + config.DETECT_MIN_SIGHTINGS]):
        for _ in range(seen):
            store2.add(i * 1.0, 0.0)
    least = min(store2.targets, key=lambda t: t.sightings)
    least.visited = True
    assert least in store2.confirmed, "방문한 것을 잘라냈다 — 이미 점수다"


# --- 카메라만으로 거리 재기 (apartment 의 사과) ----------------------------------

def _apple_image(distance, width=640, height=480, fov=1.0472):
    """바닥에 놓인 반지름 TARGET_RADIUS 공을 거리 distance 에서 찍은 합성 영상."""
    import cv2
    focal = (width / 2.0) / math.tan(fov / 2.0)
    radius_px = focal * math.tan(math.asin(config.TARGET_RADIUS / distance))
    centre_drop = config.CAMERA_HEIGHT - config.TARGET_RADIUS      # 카메라보다 이만큼 아래
    centre_row = (height - 1) / 2.0 + focal * centre_drop / distance
    img = np.zeros((height, width, 3), np.uint8)
    cv2.circle(img, (width // 2, int(round(centre_row))), int(round(radius_px)),
               (0, 0, 255), -1)
    return img


def test_camera_range_agrees_for_an_apple_on_the_floor():
    """바닥에 놓인 사과는 크기 거리와 바닥 거리가 서로, 그리고 진짜와 맞는다."""
    for d in (0.8, 1.5, 2.5):
        img = _apple_image(d)
        box = detect.blob_boxes(img)[0]
        d_size, d_ground = detect.camera_range(box, 640, 480, 1.0472)
        assert abs(d_size - d) < 0.1 * d, (d, d_size)
        assert abs(d_ground - d) < 0.15 * d, (d, d_ground)
        assert abs(d_size - d_ground) < config.DETECT_RANGE_AGREEMENT * d_ground


def test_camera_scan_rejects_a_tall_red_object(monkeypatch):
    """사과보다 큰 빨간 물체(소화기)는 두 거리가 어긋나 목표물이 되지 않는다.

    ⚠️ 회귀 방지. apartment 첫 완주에서 시작점 옆 소화기를 사과로 확정·방문했다
       (실제 점수 0/2). 기록된 프레임에서 소화기는 두 거리가 81% 어긋났다.
    """
    import cv2
    monkeypatch.setattr(config, "DETECT_RANGING", "camera")
    img = np.zeros((480, 640, 3), np.uint8)
    cv2.rectangle(img, (300, 120), (340, 300), (0, 0, 255), -1)   # 좁고 긴 빨간 기둥
    targets = detect.TargetList()
    placed, rejected = detect.scan(img, np.full(360, 5.0), (0.0, 0.0, 0.0),
                                   1.0472, targets)
    assert placed == 0 and rejected == 1, (placed, rejected)


def test_camera_scan_places_an_apple_where_it_is(monkeypatch):
    """카메라만으로 잰 사과 위치가 진짜 위치와 맞는다 (LiDAR 는 뒤의 벽을 본다)."""
    monkeypatch.setattr(config, "DETECT_RANGING", "camera")
    d = 1.5
    targets = detect.TargetList()
    # LiDAR 는 사과를 못 보고 3 m 뒤 벽을 본다 — 그래도 위치가 맞아야 한다
    placed, _ = detect.scan(_apple_image(d), np.full(360, 3.0), (0.0, 0.0, 0.0),
                            1.0472, targets)
    assert placed == 1
    t = targets.targets[0]
    expect = d + config.CAMERA_FORWARD
    assert abs(t.x - expect) < 0.15 and abs(t.y) < 0.05, (t.x, t.y)


def test_camera_scan_rejects_a_far_tall_object_whose_ranges_almost_agree(monkeypatch):
    """멀리 있는 소화기는 두 거리가 29% 만 어긋나 거리 검사를 통과했다 — 모양으로 거른다.

    ⚠️ 회귀 방지. apartment 195초: 상자 23x72, 크기로 2.4 m, 바닥으로 3.4 m.
    """
    import cv2
    monkeypatch.setattr(config, "DETECT_RANGING", "camera")
    img = np.zeros((480, 640, 3), np.uint8)
    cv2.rectangle(img, (300, 180), (322, 251), (0, 0, 255), -1)    # 23x72, 밑동 251행
    box = detect.blob_boxes(img)[0]
    d_size, d_ground = detect.camera_range(box, 640, 480, 1.0472)
    assert abs(d_size - d_ground) <= config.DETECT_RANGE_AGREEMENT * d_ground, \
        "전제: 거리 검사만으로는 못 거르는 상자여야 한다"
    placed, _ = detect.scan(img, np.full(360, 5.0), (0.0, 0.0, 0.0), 1.0472,
                            detect.TargetList())
    assert placed == 0


def _fake_yolo(label):
    """화면 전체를 덮는 상자 하나를 label 로 내놓는 가짜 YOLO. 불린 횟수를 센다."""
    calls = []

    def classify(image):
        calls.append(1)
        return [(0, 0, image.shape[1], image.shape[0], label, 0.5)] if label else []
    classify.calls = calls
    return classify


@pytest.mark.parametrize("label, placed_expected, lead_expected", [
    ("sports ball", 1, 0),      # 사과 계열 → 받는다
    ("bottle", 0, 0),           # 캔 → 버린다 (단서도 안 남긴다)
    (None, 0, 1),               # 못 봄 → 가까이 가서 다시 보도록 단서만
])
def test_yolo_decides_what_a_red_blob_is(monkeypatch, label, placed_expected, lead_expected):
    monkeypatch.setattr(config, "DETECT_RANGING", "camera")
    classify = _fake_yolo(label)
    targets, leads = detect.TargetList(), detect.LeadList()
    placed, _ = detect.scan(_apple_image(1.5), np.full(360, 3.0), (0.0, 0.0, 0.0),
                            1.0472, targets, leads, classify=classify)
    assert placed == placed_expected
    assert len(leads.leads) == lead_expected
    assert len(classify.calls) == 1


def test_yolo_is_not_run_when_nothing_red_is_seen(monkeypatch):
    """YOLO 는 빨간 덩어리가 모양 검사를 통과했을 때만 돈다."""
    monkeypatch.setattr(config, "DETECT_RANGING", "camera")
    classify = _fake_yolo("sports ball")
    detect.scan(np.zeros((480, 640, 3), np.uint8), np.full(360, 3.0), (0.0, 0.0, 0.0),
                1.0472, detect.TargetList(), classify=classify)
    assert classify.calls == []


def test_yolo_keeps_an_apple_even_if_a_bottle_box_also_covers_it():
    box = {"left": 100, "top": 100, "w": 20, "h": 20}
    both = [(0, 0, 640, 480, "bottle", 0.3), (90, 90, 130, 130, "sports ball", 0.2)]
    assert detect.yolo_verdict(box, both) == "accept"


def test_camera_scan_rejects_a_flat_red_object(monkeypatch):
    """바닥에 누운 캔(26x11)은 사과 크기지만 납작하다 — 모양으로 거른다 (apartment 캔 오탐)."""
    import cv2
    monkeypatch.setattr(config, "DETECT_RANGING", "camera")
    img = np.zeros((480, 640, 3), np.uint8)
    cv2.rectangle(img, (505, 247), (530, 257), (0, 0, 255), -1)     # 26x11, 녹화 그대로
    placed, _ = detect.scan(img, np.full(360, 5.0), (0.0, 0.0, 0.0), 1.0472,
                            detect.TargetList())
    assert placed == 0


def test_floor_position_of_an_object_straight_ahead(monkeypatch):
    """화면 가운데 열, 수평선 아래 행 → 로봇 앞쪽 바닥의 점 (거리 = 높이 / tan(내려본 각))."""
    focal = 320.0 / math.tan(1.0472 / 2.0)
    d = 1.0
    row = 239.5 + focal * config.CAMERA_HEIGHT / d
    x, y = detect.floor_position(row, 319.5, (0.0, 0.0, 0.0), 640, 480, 1.0472)
    assert abs(x - (d + config.CAMERA_FORWARD)) < 0.02 and abs(y) < 0.01
    assert detect.floor_position(200, 320, (0.0, 0.0, 0.0), 640, 480, 1.0472) is None, \
        "수평선 위는 바닥이 아니다"


def test_low_obstacles_need_several_sightings_and_merge_nearby():
    low = detect.LowObstacles()
    for k in range(config.LOW_MIN_SIGHTINGS - 1):
        low.add(1.0 + 0.05 * k, 1.0)
    assert low.positions() == [], "몇 번 더 봐야 믿는다"
    low.add(1.0, 1.0)
    (x, y), = low.positions()
    assert abs(x - 1.0) < 0.1 and y == 1.0


def test_a_flat_red_can_on_the_floor_becomes_a_low_obstacle(monkeypatch):
    """누운 캔은 사과는 아니지만(모양 검사) 부딪히면 안 되는 물체로는 남는다."""
    import cv2
    monkeypatch.setattr(config, "DETECT_RANGING", "camera")
    img = np.zeros((480, 640, 3), np.uint8)
    cv2.rectangle(img, (505, 247), (530, 257), (0, 0, 255), -1)
    low = detect.LowObstacles()
    monkeypatch.setattr(config, "LOW_MAX_RANGE", 10.0)     # 이 합성 영상의 캔은 1.2 m 보다 멀다
    for _ in range(config.LOW_MIN_SIGHTINGS):
        placed, _ = detect.scan(img, np.full(360, 5.0), (0.0, 0.0, 0.0), 1.0472,
                                detect.TargetList(), low_list=low)
        assert placed == 0
    assert len(low.positions()) == 1
