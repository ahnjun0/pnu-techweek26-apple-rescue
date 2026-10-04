"""카메라가 보는 것과 LiDAR 가 재는 것이 같은 방향을 가리키는지 확인한다.

받는 것: worlds/camera_test.wbt (일부러 "카메라에만 보이는" 물체를 섞어 놓았다).
내놓는 것: 방향별 대조표와 debug/out/camera_lidar.png
핵심 아이디어: 화면의 가로 픽셀 → 방위각 → 그 방위의 LiDAR 거리.
              Phase 5 의 detect.py 가 하려는 계산을 그대로, 눈에 보이게 한 것이다.
⚠️ 카메라 픽셀 → 각도 부호도 추측하지 않는다. 왼쪽에 놓은 물체로 실증한다.
"""

import math
import os
import sys

import cv2
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from sar import common
from sar import config
from sar import viz  # noqa: F401  (한글 폰트 설정을 위해 import 한다)
from sar import sensors as sensors_mod
from debug import truth as truth_mod

OUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "out")

# 월드에 놓아 둔 것들 (이름, x, y, 겉면까지 빼 줄 반지름, 설명)
# 반지름을 빼는 이유: LiDAR 는 물체 "표면" 까지의 거리를 잰다. 중심까지 거리와
# 비교하면 늘 반지름만큼 차이가 나서, 멀쩡한 측정을 오차로 오해하게 된다.
PLACED = [
    ("A 정면 기둥", 1.5, 0.0, 0.08, "높이 0.4 — 둘 다 봐야 한다"),
    ("B 왼쪽 기둥", 1.3, 0.55, 0.08, "높이 0.4 — 둘 다 봐야 한다 (부호 확인용)"),
    ("C 납작한 것", 1.0, -0.35, 0.08, "높이 0.10 — LiDAR 평면(0.173) 아래"),
    ("D 뜬 것", 2.0, 0.35, 0.10, "z 0.5~0.7 — LiDAR 평면 위"),
    ("E 먼 기둥", 4.5, -0.55, 0.08, "LiDAR 사거리(3.5) 밖"),
    ("F 회색 상자", 2.0, -1.2, 0.10, "화각 밖 — LiDAR 만 본다"),
]

def red_mask(image_bgr):
    """빨간 픽셀만 남긴다. 임계값은 config 에 있다 (측정해서 정한 값)."""
    hsv = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2HSV)
    low = np.array([0, config.DETECT_SAT_MIN, config.DETECT_VALUE_MIN])
    return (cv2.inRange(hsv, low, np.array([config.DETECT_HUE_LOW, 255, 255]))
            | cv2.inRange(hsv,
                          np.array([config.DETECT_HUE_HIGH,
                                    config.DETECT_SAT_MIN,
                                    config.DETECT_VALUE_MIN]),
                          np.array([179, 255, 255])))


def column_bearings(width, fov):
    """화면의 각 세로줄이 로봇 기준 몇 도 방향인지.

    핀홀 카메라: 화면 중심에서 u 픽셀 떨어진 곳의 각도는
        atan( (u - (W-1)/2) / (W/2) * tan(fov/2) )
    화면의 오른쪽(u 증가)은 로봇 기준 오른쪽(음의 각도)이므로 부호를 뒤집는다.
    ⚠️ 이 부호가 맞는지는 왼쪽에 놓은 B 기둥으로 확인한다.
    """
    u = np.arange(width)
    normalised = (u - (width - 1) / 2.0) / (width / 2.0) * math.tan(fov / 2.0)
    return -np.arctan(normalised)


def lidar_at(ranges, bearing):
    """그 방위각을 보는 LiDAR 광선의 거리. 없으면 inf."""
    angles = common.lidar_angles()
    index = int(np.argmin(np.abs(common.wrap_angle(angles - bearing))))
    value = ranges[index]
    return float(value) if np.isfinite(value) else math.inf


def blobs_from(mask, bearings):
    """빨간 덩어리들을 찾아 (픽셀수, 중심 방위각, 화면 x) 로 돌려준다."""
    count, labels, stats, centroids = cv2.connectedComponentsWithStats(mask, 8)
    found = []
    for label in range(1, count):
        area = stats[label, cv2.CC_STAT_AREA]
        if area < config.DETECT_MIN_BLOB_PIXELS:      # 잡음
            continue
        centre_x = float(centroids[label][0])
        bearing = float(np.interp(centre_x, np.arange(len(bearings)), bearings))
        found.append((int(area), bearing, centre_x))
    found.sort(key=lambda item: -item[0])
    return found


def main():
    from controller import Supervisor

    robot = Supervisor()
    timestep = int(robot.getBasicTimeStep())
    sensors = sensors_mod.Sensors(robot)
    truth = truth_mod.Truth(robot)   # 정답 위치 (debug/truth.py)

    for _ in range(40):          # LiDAR 가 한 바퀴 돌고 카메라가 채워질 때까지
        robot.step(timestep)

    image = sensors.read_camera_bgr()
    ranges = sensors.read_lidar()
    width = sensors.camera.getWidth()
    height = sensors.camera.getHeight()
    fov = sensors.camera.getFov()
    robot_x, robot_y = truth.xy()

    bearings = column_bearings(width, fov)
    mask = red_mask(image)

    print("=" * 76)
    print("[camera_lidar_check] 카메라와 LiDAR 가 같은 것을 보고 있는가")
    print("=" * 76)
    print(f"  카메라 {width}x{height}, 수평 FOV {math.degrees(fov):.1f}°"
          f"  → 화면 왼쪽끝 {math.degrees(bearings[0]):+.1f}°,"
          f" 오른쪽끝 {math.degrees(bearings[-1]):+.1f}°")
    print(f"  LiDAR  {len(ranges)}선, 수평 360°, 사거리"
          f" {config.LIDAR_MIN_RANGE}~{config.LIDAR_MAX_RANGE} m, 평면 한 층")
    print(f"  높이: LiDAR z=0.173 m / 카메라 z=0.213 m  (4 cm 차이)")

    # --- 1. 월드에 놓아 둔 것들을 하나씩 대조 -----------------------------
    print("\n" + "-" * 76)
    print("  놓아 둔 물체별로: 실제 방향/거리 vs LiDAR 가 그 방향에서 잰 값")
    print("-" * 76)
    print(f"  {'물체':<14s} {'실제방향':>8s} {'실제거리':>8s} {'LiDAR':>8s}"
          f" {'차이':>8s}  {'카메라FOV':>8s}   비고")
    for name, x, y, radius, note in PLACED:
        dx, dy = x - robot_x, y - robot_y
        true_bearing = math.atan2(dy, dx)
        true_range = math.hypot(dx, dy) - radius      # 표면까지의 거리
        measured = lidar_at(ranges, true_bearing)
        in_camera = abs(true_bearing) <= fov / 2.0
        if math.isfinite(measured):
            gap = f"{(measured - true_range) * 100:+6.1f}cm"
            seen = "LiDAR ✅" if abs(measured - true_range) < 0.25 else "LiDAR ❌ 딴 것"
        else:
            gap = "     —"
            seen = "LiDAR ❌ 못 봄"
        print(f"  {name:<14s} {math.degrees(true_bearing):+7.1f}°"
              f" {true_range:7.2f}m {measured:7.2f}m {gap}"
              f" {'안에' if in_camera else '밖':>8s}   {seen} / {note}")

    # --- 2. 카메라가 실제로 찾은 빨간 덩어리 ------------------------------
    found = blobs_from(mask, bearings)
    print("\n" + "-" * 76)
    print(f"  카메라가 찾은 빨간 덩어리 {len(found)} 개")
    print("-" * 76)
    print(f"  {'크기':>6s} {'화면x':>6s} {'방위각':>8s} {'그 방향 LiDAR':>14s}"
          f"   추정 월드좌표")
    for area, bearing, centre_x in found:
        distance = lidar_at(ranges, bearing)
        if math.isfinite(distance):
            wx = robot_x + distance * math.cos(bearing)
            wy = robot_y + distance * math.sin(bearing)
            guess = f"({wx:+.2f}, {wy:+.2f})"
        else:
            guess = "거리를 모른다 → 좌표 못 구함"
        print(f"  {area:6d} {centre_x:6.1f} {math.degrees(bearing):+7.1f}°"
              f" {distance:13.2f}m   {guess}")

    # --- 3. 부호 실증 -----------------------------------------------------
    print("\n" + "-" * 76)
    print("  카메라 좌우 부호 실증 (B 기둥을 로봇 왼쪽에 놓았다)")
    print("-" * 76)
    left_pillar = next(p for p in PLACED if p[0].startswith("B"))
    expected = math.atan2(left_pillar[2] - robot_y, left_pillar[1] - robot_x)
    assert abs(expected) < fov / 2.0, \
        "부호 검증용 물체가 카메라 화각 밖이다 — 월드를 고칠 것"
    if found:
        best = min(found, key=lambda item: abs(item[1] - expected))
        error = math.degrees(abs(common.angle_diff(best[1], expected)))
        print(f"    실제 방향 {math.degrees(expected):+.1f}°"
              f"  vs 카메라가 말한 {math.degrees(best[1]):+.1f}°"
              f"  차이 {error:.1f}°"
              f"  → {'✅ 부호가 맞다' if error < 5 else '❌ 좌우가 뒤집혔다'}")
    print("=" * 76)
    sys.stdout.flush()

    report_colours(image, mask)
    cv2.imwrite(os.path.join(OUT_DIR, "camera_raw.png"), image)
    draw(image, mask, bearings, ranges, fov, found)

    print("[[DONE]]")          # run_headless.sh 가 이걸 보고 끝낸다
    sys.stdout.flush()

    while robot.step(timestep) != -1:
        pass


def report_colours(image, mask):
    """실제 화면의 HSV 값을 재서, 목표물과 바닥을 색으로 가를 수 있는지 본다.

    바닥(Parquetry 체크무늬)이 적갈색이라 "빨강" 범위에 걸려 들어온다.
    추측으로 임계값을 정하지 말고, 여기서 나온 숫자를 보고 정한다.
    """
    os.makedirs(OUT_DIR, exist_ok=True)
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    height = image.shape[0]

    print("\n" + "-" * 76)
    print("  실제 HSV 값 — 목표물과 바닥을 가를 수 있는가")
    print("-" * 76)

    regions = {
        "화면 위쪽 절반 (물체·하늘)": hsv[: height // 2],
        "화면 맨 아래 6줄 (순수 바닥)": hsv[-6:],
        "지금 마스크가 빨강이라 한 곳": hsv[mask > 0],
    }
    for label, pixels in regions.items():
        flat = pixels.reshape(-1, 3)
        if len(flat) == 0:
            continue
        h, s_, v = flat[:, 0], flat[:, 1], flat[:, 2]
        print(f"    {label}")
        print(f"      H 중앙값 {np.median(h):5.0f}  S 중앙값 {np.median(s_):5.0f}"
              f"  V 중앙값 {np.median(v):5.0f}"
              f"   (S 90퍼센타일 {np.percentile(s_, 90):.0f},"
              f" V 90퍼센타일 {np.percentile(v, 90):.0f})")

    # 맨 아래 몇 줄만 본다. 1/3 을 다 보면 낮은 물체(C)가 섞여 들어와
    # "바닥이 오검출된다" 는 잘못된 결론이 나온다.
    floor = hsv[-6:].reshape(-1, 3)
    floor_red = (((floor[:, 0] <= config.DETECT_HUE_LOW)
                  | (floor[:, 0] >= config.DETECT_HUE_HIGH))
                 & (floor[:, 1] >= config.DETECT_SAT_MIN)
                 & (floor[:, 2] >= config.DETECT_VALUE_MIN))
    share = floor_red.mean() * 100
    print(f"\n    지금 임계값으로 바닥 픽셀의 {share:.1f}% 가 "
          f"'빨강' 으로 잡힌다 → {'❌ 오검출' if share > 5 else '✅ 괜찮다'}")
    if floor_red.any():
        red_floor = floor[floor_red]
        print(f"      그 바닥 픽셀들의 S 중앙값 {np.median(red_floor[:, 1]):.0f},"
              f" V 중앙값 {np.median(red_floor[:, 2]):.0f}")
    sys.stdout.flush()


def draw(image, mask, bearings, ranges, fov, found):
    os.makedirs(OUT_DIR, exist_ok=True)
    figure, axes = plt.subplots(3, 1, figsize=(11, 10),
                                gridspec_kw={"height_ratios": [3, 1, 3]})

    axes[0].imshow(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
    axes[0].set_title("카메라가 보는 것")
    axes[0].set_xticks([])
    axes[0].set_yticks([])
    for _, bearing, centre_x in found:
        axes[0].axvline(centre_x, color="lime", linewidth=1.2)

    axes[1].imshow(mask, cmap="gray", aspect="auto")
    axes[1].set_title("빨간색만 남긴 것 (HSV)")
    axes[1].set_xticks([])
    axes[1].set_yticks([])

    degrees = np.degrees(bearings)
    in_view = [lidar_at(ranges, b) for b in bearings]
    axes[2].plot(degrees, in_view, "-", color="tab:blue",
                 label="LiDAR 거리")
    for _, bearing, _ in found:
        axes[2].axvline(math.degrees(bearing), color="lime", linewidth=1.2)
    axes[2].set_xlim(degrees[0], degrees[-1])
    axes[2].set_ylim(0, config.LIDAR_MAX_RANGE + 0.3)
    axes[2].invert_xaxis()      # 화면 왼쪽이 + 각도이므로 축을 맞춰 준다
    axes[2].set_xlabel("방위각 [°]  (왼쪽이 +)")
    axes[2].set_ylabel("LiDAR 거리 [m]")
    axes[2].set_title("같은 방향의 LiDAR 거리  (초록 선 = 카메라가 빨강을 본 방향)")
    axes[2].grid(alpha=0.3)
    axes[2].legend(loc="upper right", fontsize=8)

    figure.tight_layout()
    path = os.path.join(OUT_DIR, "camera_lidar.png")
    figure.savefig(path, dpi=100, bbox_inches="tight")
    print(f"  그림을 저장했다: {path}")
    sys.stdout.flush()
