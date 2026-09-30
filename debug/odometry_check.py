"""오도메트리가 얼마나 맞는지 정답 위치(Supervisor)와 대조해 숫자로 보여 준다. (Phase 1 검증)

받는 것: empty.wbt 의 로봇. 정해진 동작(직진 1 m / 360도 회전 / 정사각형)을 스스로 한다.
내놓는 것: 구간마다 추정 pose · 정답 위치 · 오차 [cm] 표.
핵심 아이디어: theta 를 나침반/자이로/엔코더로 각각 구한 추정기 3개를 동시에 돌려
                 어느 쪽이 얼마나 정확한지 눈으로 비교한다.
⚠️ 여기는 debug/ 다. 정답 위치(debug/truth.py)를 읽는 것이 허용되는 곳이다 (CLAUDE.md 규칙 1).
"""

import math
import sys

import common
import config
import localization
import sensors as sensors_mod
from debug import truth as truth_mod

# 검증 주행은 느리고 조심스럽게 한다.
DRIVE_SPEED = 0.10      # [m/s]
TURN_SPEED = 0.60       # [rad/s]


class Check:
    """로봇 한 대와 추정기 3개를 들고 다니는 상자."""

    def __init__(self, robot):
        self.robot = robot
        self.timestep = int(robot.getBasicTimeStep())
        self.dt = self.timestep / 1000.0
        self.sensors = sensors_mod.Sensors(robot)
        self.truth = truth_mod.Truth(robot)   # 정답 위치 (debug/truth.py)

        # 같은 엔코더 값을 먹이고 theta 소스만 다르게 한 추정기 3개.
        start = (config.START_X, config.START_Y, config.START_THETA)
        self.est = {
            "compass": localization.Odometry(*start),
            "gyro": localization.Odometry(*start),
            "encoder": localization.Odometry(*start),
        }

        # 센서가 첫 값을 내놓을 때까지 몇 틱 돌린다.
        for _ in range(5):
            self.robot.step(self.timestep)

        # 시작 위치를 정답 위치으로 맞춘다 (월드와 config 가 어긋나도 비교가 성립하게).
        gx, gy = self.truth.xy()
        for odo in self.est.values():
            odo.x, odo.y = gx, gy
        self.start_gps = (gx, gy)

    # ------------------------------------------------------------------

    def tick(self):
        """한 틱 진행하고 추정기 3개를 모두 갱신한다."""
        if self.robot.step(self.timestep) == -1:
            sys.exit(0)
        left, right = self.sensors.read_encoders()
        compass = self.sensors.read_compass()
        gyro_z = self.sensors.read_gyro_z()

        self.est["compass"].update(left, right, self.dt, compass_values=compass)
        self.est["gyro"].update(left, right, self.dt, gyro_z=gyro_z)
        self.est["encoder"].update(left, right, self.dt)

    @property
    def primary(self):
        """대표 추정기 = 나침반 기반 (localization.py 의 1순위와 같다)."""
        return self.est["compass"]

    # --- 동작 ---------------------------------------------------------

    def wait(self, seconds):
        self.sensors.stop()
        for _ in range(int(seconds / self.dt)):
            self.tick()

    def go_straight(self, distance):
        """추정 이동거리가 distance 가 될 때까지 직진한다."""
        x0, y0, _ = self.primary.pose
        self.sensors.drive(DRIVE_SPEED, 0.0)
        while common.distance(x0, y0, self.primary.x, self.primary.y) < distance:
            self.tick()
        self.sensors.stop()

    def turn_by(self, angle):
        """추정 각도가 angle [rad] 만큼 변할 때까지 제자리에서 돈다. + 는 반시계."""
        turned = 0.0
        previous = self.primary.theta
        self.sensors.drive(0.0, math.copysign(TURN_SPEED, angle))
        while abs(turned) < abs(angle):
            self.tick()
            turned += common.angle_diff(self.primary.theta, previous)
            previous = self.primary.theta
        self.sensors.stop()

    # --- 보고 ---------------------------------------------------------

    def report(self, label, expected=None):
        """현재 추정값들을 정답 위치과 나란히 찍는다."""
        gx, gy = self.truth.xy()
        print(f"\n── {label}")
        print(f"   정답 위치       : ({gx:+.3f}, {gy:+.3f}) m")
        if expected is not None:
            ex, ey = expected
            print(f"   원래 가야 할 곳 : ({ex:+.3f}, {ey:+.3f}) m"
                  f"   (실제와 {common.distance(gx, gy, ex, ey) * 100:5.1f} cm 차이"
                  f" — 이건 주행 오차지 오도메트리 오차가 아니다)")
        for name, odo in self.est.items():
            err = common.distance(gx, gy, odo.x, odo.y) * 100.0
            print(f"   [{name:<7s}] 추정 ({odo.x:+.3f}, {odo.y:+.3f}) m"
                  f"  theta {math.degrees(odo.theta):+7.2f}°"
                  f"   오차 {err:6.2f} cm")
        sys.stdout.flush()
        return gx, gy


def verify_compass_formula(check, before, after):
    """정답 이동 방향과 나침반 각도가 같은지 확인한다 (공식 실증).

    직진 구간의 시작·끝 정답 좌표를 이으면 그게 로봇이 실제로 향한 방향이다.
    그 방향과 localization.heading_from_compass 의 결과가 일치해야 공식이 맞는 것이다.
    """
    dx = after[0] - before[0]
    dy = after[1] - before[1]
    gps_heading = math.atan2(dy, dx)
    compass_heading = localization.heading_from_compass(check.sensors.read_compass())
    diff = math.degrees(abs(common.angle_diff(gps_heading, compass_heading)))

    print("\n" + "=" * 68)
    print("나침반 각도 공식 실증  theta = atan2(values[0], values[1])")
    print("=" * 68)
    print(f"   정답 이동방향  : {math.degrees(gps_heading):+7.2f}°")
    print(f"   나침반이 말한 값: {math.degrees(compass_heading):+7.2f}°")
    print(f"   차이           : {diff:.2f}°  →  "
          f"{'✅ 공식이 맞다' if diff < 5.0 else '❌ 공식이 틀렸다. 부호나 인수 순서를 의심할 것'}")
    sys.stdout.flush()


def main():
    from controller import Supervisor

    robot = Supervisor()
    check = Check(robot)

    print("=" * 68)
    print("[odometry_check] Phase 1 — 오도메트리 vs 정답")
    print("=" * 68)
    print("  empty.wbt 에서 직진 1 m → 360° 회전 → 1 m 정사각형 순서로 주행한다.")
    print(f"  시작 정답: ({check.start_gps[0]:+.3f}, {check.start_gps[1]:+.3f}) m")
    sys.stdout.flush()

    check.wait(0.5)
    origin = check.report("0. 시작")

    # --- 1. 직진 1 m ----------------------------------------------------
    check.go_straight(1.0)
    check.wait(0.5)
    after = check.report("1. 직진 1 m",
                         expected=(origin[0] + 1.0, origin[1]))
    verify_compass_formula(check, origin, after)

    # --- 2. 제자리 360° 회전 --------------------------------------------
    # 위치는 그대로여야 하고, theta 는 한 바퀴 돌아 제자리로 와야 한다.
    check.turn_by(2.0 * math.pi)
    check.wait(0.5)
    check.report("2. 제자리 360° 회전 (위치는 1번과 같아야 한다)", expected=after)

    # --- 3. 1 m 정사각형 ------------------------------------------------
    square_start = check.truth.xy()
    for i in range(4):
        check.go_straight(1.0)
        check.turn_by(math.pi / 2)
        check.wait(0.3)
        check.report(f"3-{i + 1}. 정사각형 {i + 1}/4 변")
    final = check.report("3. 정사각형 완료 (출발점으로 돌아와야 한다)",
                         expected=square_start)

    # --- 요약 -----------------------------------------------------------
    print("\n" + "=" * 68)
    print("요약")
    print("=" * 68)
    loop_error = common.distance(*square_start, *final) * 100.0
    print(f"  정사각형 실제 복귀 오차 (정답 기준) : {loop_error:6.2f} cm")
    print("   ※ 이건 주행 제어 오차다. 아래가 오도메트리 오차다.\n")
    for name, odo in check.est.items():
        err = common.distance(final[0], final[1], odo.x, odo.y) * 100.0
        verdict = "✅ 좋다" if err < 10.0 else ("△ 쓸 만하다" if err < 25.0 else "❌ 크다")
        print(f"  [{name:<7s}] 최종 추정 오차 : {err:6.2f} cm   {verdict}")
    print("\n  나침반이 제일 작아야 정상이다. 엔코더만 쓴 것이 제일 클 것이다.")
    print("=" * 68)
    sys.stdout.flush()

    print("[[DONE]]")          # run_headless.sh 가 이걸 보고 끝낸다
    sys.stdout.flush()

    check.sensors.stop()
    while robot.step(check.timestep) != -1:
        pass
