"""LiDAR 배열의 인덱스 0 이 어느 방향인지, 인덱스가 늘면 어느 쪽으로 도는지 측정한다.

받는 것: worlds/lidar_test.wbt — 상자를 정면 1.0 m / 왼쪽 1.5 m / 오른쪽 2.0 m 에 놓았다.
내놓는 것: config.LIDAR_ANGLE_OFFSET 과 LIDAR_ANGLE_SIGN 에 적을 값.
핵심 아이디어: 방향을 아는 물체 3개를 찾아 그 인덱스를 역산한다. 문서를 믿지 않는다.
확인 방법: 앞 2개로 공식을 세우고, 남은 1개(오른쪽)로 예측이 맞는지 검산한다.
⚠️ 로봇이 바뀌면 이 실험을 다시 해야 한다.
"""

import math
import sys

import numpy as np

from sar import common
from sar import config
from sar import sensors as sensors_mod

# lidar_test.wbt 에 놓은 상자들. (이름, 진짜 방향 [rad], 대략 거리 [m])
BOXES = [
    ("정면", 0.0, 0.9),
    ("왼쪽", math.pi / 2, 1.4),
    ("오른쪽", -math.pi / 2, 1.9),
]


def find_box_indices(ranges):
    """가까운 것부터 골라 상자 3개의 인덱스를 찾는다.

    상자는 아레나 벽(3 m)보다 가깝고 서로 거리가 다르므로,
    제일 가까운 최소점부터 차례로 집으면 정면 → 왼쪽 → 오른쪽 순서가 된다.
    """
    work = np.array(ranges, dtype=np.float64)
    work[~np.isfinite(work)] = np.inf
    found = []
    for _ in range(len(BOXES)):
        index = int(np.argmin(work))
        found.append((index, float(work[index])))
        # 같은 상자를 또 집지 않도록 주변을 지운다 (상자 하나가 여러 광선에 걸린다).
        span = len(work) // 12          # ±30도쯤
        for offset in range(-span, span + 1):
            work[(index + offset) % len(work)] = np.inf
    return found


def main():
    from controller import Robot

    robot = Robot()
    timestep = int(robot.getBasicTimeStep())
    sensors = sensors_mod.Sensors(robot)   # GPS 안 씀

    # LiDAR 가 한 바퀴 다 돌아 값이 채워질 때까지 기다린다.
    for _ in range(40):
        robot.step(timestep)

    ranges = sensors.read_lidar()
    n = len(ranges)
    step_rad = 2.0 * math.pi / n

    print("=" * 70)
    print("[lidar_orientation] Phase 2 — LiDAR 인덱스 방향 측정")
    print("=" * 70)
    print(f"  배열 길이 : {n}  (config.LIDAR_RESOLUTION = {config.LIDAR_RESOLUTION})")
    print(f"  한 칸 각도 : {math.degrees(step_rad):.3f}°")
    finite = np.isfinite(ranges)
    print(f"  유효 측정  : {finite.sum()} / {n} 개, "
          f"최소 {np.min(ranges[finite]):.3f} m / 최대 {np.max(ranges[finite]):.3f} m")

    found = find_box_indices(ranges)
    print("\n  찾은 물체 (가까운 순):")
    for (index, dist), (label, true_angle, expect_dist) in zip(found, BOXES):
        print(f"    {label:<4s} 예상 {expect_dist:.1f} m → 측정 {dist:.3f} m"
              f"   인덱스 {index:3d}   (실제 방향 {math.degrees(true_angle):+.0f}°)")

    i_front = found[0][0]
    i_left = found[1][0]
    i_right = found[2][0]

    # --- 부호: 정면에서 왼쪽(+90도)으로 갈 때 인덱스가 늘었나 줄었나 -------
    forward_steps = (i_left - i_front) % n        # 인덱스를 늘려서 왼쪽까지 가는 거리
    quarter = n // 4
    if abs(forward_steps - quarter) <= abs(forward_steps - 3 * quarter):
        sign = +1      # 인덱스 +1 칸 = 반시계 +1 칸
        gap = forward_steps
    else:
        sign = -1      # 인덱스 +1 칸 = 시계 방향
        gap = n - forward_steps

    print("\n  " + "-" * 66)
    print(f"  정면 인덱스 {i_front} → 왼쪽 인덱스 {i_left} 까지 인덱스를 "
          f"{forward_steps} 칸 늘려야 한다 (한 바퀴 {n} 칸)")
    print(f"  왼쪽은 +90°, 즉 {quarter} 칸이어야 하므로 → "
          f"{'인덱스가 늘면 반시계' if sign > 0 else '인덱스가 늘면 시계'} 방향이다")
    print(f"  실제 잰 90° = {gap} 칸 (이론값 {quarter} 칸, "
          f"오차 {abs(gap - quarter)} 칸 = {abs(gap - quarter) * math.degrees(step_rad):.1f}°)")

    # --- 오프셋: 인덱스 0 이 가리키는 각도 --------------------------------
    offset = common.wrap_angle(-sign * i_front * step_rad)

    print(f"\n  인덱스 0 이 가리키는 각도 = {math.degrees(offset):+.2f}° "
          f"(정면 인덱스 {i_front} 가 0° 가 되도록 역산)")

    # --- 검산: 남겨 둔 오른쪽 상자로 예측이 맞는지 본다 --------------------
    def angle_of(index):
        return common.wrap_angle(offset + sign * index * step_rad)

    predicted = angle_of(i_right)
    error_deg = abs(math.degrees(common.angle_diff(predicted, -math.pi / 2)))
    ok = error_deg < 5.0

    print("\n  " + "-" * 66)
    print("  검산 (공식을 세울 때 쓰지 않은 오른쪽 상자로 확인)")
    print(f"    인덱스 {i_right} 의 예측 각도 : {math.degrees(predicted):+.2f}°")
    print(f"    실제 방향                  : -90.00°")
    print(f"    차이                       : {error_deg:.2f}°  "
          f"→ {'✅ 공식이 맞다' if ok else '❌ 맞지 않는다'}")

    # --- 결과 -------------------------------------------------------------
    print("\n" + "=" * 70)
    print("config.py 에 이렇게 적을 것")
    print("=" * 70)
    print(f"  LIDAR_ANGLE_OFFSET = {offset:.6f}   # = {math.degrees(offset):+.1f}°")
    print(f"  LIDAR_ANGLE_SIGN = {sign:+d}")
    print(f"  LIDAR_ORIENTATION_VERIFIED = {ok}")
    print("=" * 70)

    # 지금 config 값과 비교해서 고쳐야 하는지 알려 준다.
    same = (abs(common.angle_diff(offset, config.LIDAR_ANGLE_OFFSET)) < 1e-6
            and sign == config.LIDAR_ANGLE_SIGN)
    if same and config.LIDAR_ORIENTATION_VERIFIED:
        print("\n  현재 config.py 값과 같다. 고칠 것 없음. ✅")
    else:
        print(f"\n  ⚠️ 현재 config.py 는 OFFSET={config.LIDAR_ANGLE_OFFSET:.6f}, "
              f"SIGN={config.LIDAR_ANGLE_SIGN:+d}, "
              f"VERIFIED={config.LIDAR_ORIENTATION_VERIFIED} 이다. 위 값으로 고칠 것.")

    # --- 참고용: 주요 방향의 인덱스와 거리 ---------------------------------
    print("\n  참고 — 주요 방향이 몇 번 인덱스인가 (config 값을 고친 뒤 다시 보면 좋다):")
    for label, angle in [("정면   0°", 0.0), ("왼쪽 +90°", math.pi / 2),
                         ("뒤쪽 180°", math.pi), ("오른쪽 -90°", -math.pi / 2)]:
        index = int(round(common.wrap_angle(angle - offset) / (sign * step_rad))) % n
        print(f"    {label} → 인덱스 {index:3d}, 그 방향 거리 {ranges[index]:.3f} m")
    print("=" * 70)
    sys.stdout.flush()

    print("[[DONE]]")          # run_headless.sh 가 이걸 보고 끝낸다
    sys.stdout.flush()

    while robot.step(timestep) != -1:
        pass
