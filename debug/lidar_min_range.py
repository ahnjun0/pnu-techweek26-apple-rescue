"""LiDAR 가 **최소 측정거리보다 가까운** 것을 어떤 값으로 돌려주는지 측정한다.

왜 필요한가: `follower.obstacle_points` 는 `ranges >= config.LIDAR_MIN_RANGE`
인 광선만 장애물로 쓴다. 만약 너무 가까운 것이 inf 나 0 으로 돌아오면 그 광선은
버려지고, DWA 는 그 방향에 **장애물이 없다고** 본다. 그런데 LiDAR 는 원점보다
3 cm 뒤에 있고 (TurtleBot3Burger.proto:43-46) 몸체 뒤쪽 모서리는 LiDAR 기준
0.084 m 에 있으므로, 벽이 원점에서 0.110~0.150 m 사이일 때가 바로 그 구간이다 —
**몸이 닿기 직전인데 안 보이는 사각지대** 가 된다.

실측 근거(final6): comb0 이 (-2.11, +2.79) 에서 외벽에 몸체 여유 -0.0 cm 로 닿았고
그때 "LiDAR 최솟값 근처 반환" 은 0 개였다 (= 값이 아예 버려졌다는 뜻).

받는 것: worlds/min_range_test.wbt (빈 아레나, 벽 내면 x = +2.95)
내놓는 것: 거리별로 그 방향 광선이 무엇으로 돌아오는지 표.
⚠️ 로봇이나 LiDAR 가 바뀌면 다시 해야 한다.
"""

import math
import sys

import numpy as np

import common
import config
import sensors as sensors_mod

# 벽면의 정확한 x 를 **가정하지 않는다** (RectangleArena 의 벽이 경계선에 걸쳐
# 있는지 안쪽에 있는지 모른다). 대신 같은 자리에서 앞/뒤로 돌려 재면
#   앞 = (X - x) + d,   뒤 = (X - x) - d   →   앞 - 뒤 = 2d
# 로 센서 오프셋 d 를 벽 위치와 무관하게 얻는다.
PROBE_X = 2.0            # 오프셋 측정용 위치 (벽에서 넉넉히 떨어진 곳)
WALL_X_GUESS = 2.95
# 뒤쪽 사각지대 확인용: 원점~벽면 거리를 줄여 간다 [m]
GAPS = [0.40, 0.30, 0.25, 0.22, 0.20, 0.18, 0.16, 0.14, 0.120, 0.110, 0.100]


def _read_dir(robot, sensors, timestep, angles, want, half_deg=3.0):
    """로봇 기준 want 방향 광선의 최솟값. (값 또는 None, 유효수, 쓰이는 수)."""
    for _ in range(40):
        if robot.step(timestep) == -1:
            return None, 0, 0
    ranges = np.asarray(sensors.read_lidar(), dtype=np.float64)
    pick = np.abs(common.wrap_angle(angles - want)) < math.radians(half_deg)
    vals = ranges[pick]
    finite = np.isfinite(vals)
    n_kept = int((finite & (vals >= config.LIDAR_MIN_RANGE)).sum())
    best = float(np.min(vals[finite])) if finite.any() else None
    return best, int(finite.sum()), n_kept


def main():
    from controller import Supervisor

    robot = Supervisor()
    timestep = int(robot.getBasicTimeStep())
    sensors = sensors_mod.Sensors(robot)
    me = robot.getSelf()
    trans = me.getField("translation")
    rot = me.getField("rotation")
    angles = common.lidar_angles()

    def place(x):
        trans.setSFVec3f([x, 0.0, 0.0])
        rot.setSFRotation([0.0, 0.0, 1.0, 0.0])
        me.resetPhysics()

    print("=" * 78)
    print("[lidar_min_range] 센서 오프셋과 뒤쪽 사각지대")
    print("=" * 78)
    print(f"  config.LIDAR_MIN_RANGE = {config.LIDAR_MIN_RANGE} m")
    print(f"  몸체 외접 반경         = {config.ROBOT_BODY_RADIUS:.4f} m")
    print()

    # --- 1) 센서 오프셋 -------------------------------------------------
    # 아레나는 대칭이다. 중앙(x=0)에서 앞/뒤 광선을 같이 읽으면
    #   앞 = X - d,   뒤 = X + d      (d = 센서의 x, 음수면 뒤)
    # 이므로 d = (뒤 - 앞) / 2. 벽 위치 X 를 몰라도 된다.
    place(0.0)
    fwd, _, _ = _read_dir(robot, sensors, timestep, angles, 0.0)
    bwd, _, _ = _read_dir(robot, sensors, timestep, angles, math.pi)
    if fwd is None or bwd is None:
        print(f"  ❌ 유효한 값을 못 얻었다 (앞 {fwd}, 뒤 {bwd})")
        return
    d = (bwd - fwd) / 2.0
    wall_x = (fwd + bwd) / 2.0
    print(f"  x=0 에서  앞 {fwd:.4f} m / 뒤 {bwd:.4f} m")
    print(f"  → 센서의 x = {d:+.4f} m (음수면 원점보다 뒤)")
    print(f"     PROTO 의 Pose translation 은 -0.03 — 차이는 RobotisLds01 내부 오프셋")
    print(f"  → 벽면 |x| ≈ {wall_x:.4f} m")
    print()

    # --- 2) 뒤쪽 사각지대 ------------------------------------------------
    # heading 0 을 유지하고 -x 벽 쪽으로 붙인다. 뒤 광선이 그 벽을 본다.
    print("  -x 벽에 **뒤로** 붙여 간다 (heading 0, 뒤 광선을 읽는다):")
    print(f"  {'원점~벽면':>9} {'센서~벽면':>10} {'읽은 값':>10}"
          f" {'유효':>5} {'쓰임':>5}  몸체여유")
    print("  " + "-" * 64)
    for gap in GAPS:
        place(-(wall_x - gap))
        val, n_finite, n_kept = _read_dir(robot, sensors, timestep, angles,
                                          math.pi)
        sensor_gap = gap + d          # 센서가 원점보다 뒤(d<0)면 벽에 더 가깝다
        body = gap - config.ROBOT_BODY_RADIUS
        shown = f"{val:.4f}" if val is not None else "비유효"
        print(f"  {gap:9.3f} {sensor_gap:10.3f} {shown:>10}"
              f" {n_finite:5d} {n_kept:5d}  {body * 100:+6.1f} cm"
              f"{'   ← 안 보인다' if n_kept == 0 else ''}")

    print()
    print("  판단: '쓰임 0' 인 줄이 사각지대다. 그 줄의 몸체여유가 양수라면,")
    print("        로봇은 뒤로 벽에 닿기 **전에** 이미 벽을 못 보고 있다.")
    sys.stdout.flush()


if __name__ == "__main__":
    main()
