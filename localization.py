"""로봇이 "나는 지금 어디에 있는가" 를 추정하는 곳 (오도메트리).

받는 것: 바퀴 엔코더 누적각 [rad], 나침반 벡터, 자이로 z [rad/s], 경과시간 dt [s].
내놓는 것: pose = (x, y, theta). 좌표 규약은 common.py 상단 참고.
핵심 아이디어: x,y 는 항상 바퀴 엔코더로 적분한다. theta 만 센서를 골라 쓴다
(나침반 > 자이로 적분 > 엔코더 차이). 나침반은 누적오차가 없어 제일 믿을 만하다.
이 파일은 Webots 를 import 하지 않는다 — pytest 로 단독 테스트된다.
"""

import math

import common
import config


def heading_from_compass(values):
    """나침반 원시 벡터 → 로봇 heading theta [rad] (+x 축이 0, 반시계가 +).

    유도 과정 (추측이 아니라 계산이다):
      - compass.md:164 — 기본 좌표계 ENU 에서 북쪽은 월드 +y 축이다.
      - compass.md:142 — 나침반은 그 북쪽 벡터를 "디바이스 좌표계" 로 돌려준다.
      - 로봇이 theta 만큼 돌아 있으면 월드→로봇 변환은 R(-theta) 이므로
            R(-theta) @ (0, 1) = (sin theta, cos theta)
        즉  values[0] = sin(theta),  values[1] = cos(theta)
      - 따라서 theta = atan2(values[0], values[1]) 이다.

    ⚠️ 문서에 나오는 bearing 예제는 atan2(north[1], north[0]) 로 인수 순서가 다르다.
       그건 "북쪽 기준 시계방향 방위각" 이라 우리 규약과 다르다. 헷갈리지 말 것.
       이 공식이 맞는지는 debug/odometry_check.py 가 GPS 이동방향과 대조해 실증한다.
    """
    return common.wrap_angle(math.atan2(values[0], values[1]))


class Odometry:
    """엔코더 기반 위치 추정기. 상태는 pose 와 직전 엔코더값뿐이다."""

    def __init__(self, x=None, y=None, theta=None):
        self.x = config.START_X if x is None else x
        self.y = config.START_Y if y is None else y
        self.theta = config.START_THETA if theta is None else theta
        self.wheel_turn_rate = 0.0   # 바퀴로 계산한 회전 속도 [rad/s] (미끄러짐 감지용)
        self._prev_left = None    # 직전 엔코더 [rad]. None 이면 아직 기준이 없다
        self._prev_right = None

    @property
    def pose(self):
        """(x, y, theta) — 규약대로 길이 3 tuple 로만 주고받는다."""
        return (self.x, self.y, self.theta)

    def correct(self, pose):
        """바깥에서 고쳐 준 위치를 받아들인다 (스캔 정합용).

        ⚠️ 엔코더 누적을 "덮어쓰는" 것이지 되돌리는 것이 아니다. 다음 틱부터는
           이 자리에서 다시 누적한다. 바퀴가 헛돌아 생긴 오차를 끊는 유일한 길이다.
        """
        self.x, self.y, self.theta = pose
        return self.pose

    def update(self, left_rad, right_rad, dt, compass_values=None, gyro_z=None):
        """엔코더 한 틱치를 반영해 pose 를 갱신하고 돌려준다.

        left_rad/right_rad 는 "이번 틱의 변화량" 이 아니라 "누적 회전각" 이다.
        첫 호출은 기준점만 잡고 pose 를 바꾸지 않는다.
        """
        # NaN (아직 측정 전) 은 무시한다.
        if math.isnan(left_rad) or math.isnan(right_rad):
            return self.pose

        if self._prev_left is None:
            self._prev_left = left_rad
            self._prev_right = right_rad
            # 나침반이 있으면 시작 자세도 실제 값으로 맞춰 둔다.
            if compass_values is not None:
                self.theta = heading_from_compass(compass_values)
            return self.pose

        # --- 바퀴가 굴러간 거리 [m] ------------------------------------
        d_left = (left_rad - self._prev_left) * config.WHEEL_RADIUS_ODOM
        d_right = (right_rad - self._prev_right) * config.WHEEL_RADIUS_ODOM
        self._prev_left = left_rad
        self._prev_right = right_rad

        d_center = 0.5 * (d_left + d_right)
        if dt > 0.0:
            self.wheel_turn_rate = (d_right - d_left) / config.WHEEL_BASE_ODOM / dt

        # --- theta 갱신: 믿을 만한 순서대로 하나만 고른다 ---------------
        theta_old = self.theta
        if compass_values is not None:
            # 1순위. 절대 각도라 누적오차가 없다.
            theta_new = heading_from_compass(compass_values)
        elif gyro_z is not None:
            # 2순위. 각속도를 적분한다. 드리프트가 천천히 쌓인다.
            theta_new = common.wrap_angle(theta_old + gyro_z * dt)
        else:
            # 3순위. 바퀴 미끄러짐이 그대로 각도 오차가 된다.
            # 기하학적 WHEEL_BASE 가 아니라 측정으로 보정한 WHEEL_BASE_ODOM 을 쓴다
            # (이유와 측정법은 config.py 의 해당 줄 주석 참고).
            theta_new = common.wrap_angle(
                theta_old + (d_right - d_left) / config.WHEEL_BASE_ODOM)

        # --- x, y 갱신 -------------------------------------------------
        # 틱 시작 각도가 아니라 "틱 중간 각도" 로 전진시킨다.
        # 곡선 주행에서 오차가 눈에 띄게 줄어든다 (중점법).
        theta_mid = theta_old + 0.5 * common.angle_diff(theta_new, theta_old)
        self.x += d_center * math.cos(theta_mid)
        self.y += d_center * math.sin(theta_mid)
        self.theta = theta_new

        return self.pose
