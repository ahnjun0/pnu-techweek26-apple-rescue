"""Webots 디바이스를 열고 원시값을 읽어 오는 곳.

받는 것: Webots Robot 인스턴스.
내놓는 것: 엔코더 [rad], LiDAR 거리 [m] 배열, 나침반 벡터, 자이로 [rad/s], 카메라 BGR.
핵심 아이디어: Webots 에 의존하는 코드를 여기 한 곳에 가둔다. 다른 모듈은 숫자만 받는다.
"""

import math

import numpy as np
from controller import Robot  # noqa: F401  (타입 힌트 겸 import 확인용)

from . import config


class Sensors:
    """디바이스 핸들을 담아 두는 단순한 상자. 상속·계층 없음."""

    def __init__(self, robot):
        self.robot = robot
        self.timestep = int(robot.getBasicTimeStep())

        # --- 바퀴 모터: 속도 제어 모드로 둔다 ---------------------------
        # setPosition(inf) 를 해야 setVelocity 가 "목표 속도" 로 동작한다.
        self.left_motor = robot.getDevice(config.LEFT_MOTOR_NAME)
        self.right_motor = robot.getDevice(config.RIGHT_MOTOR_NAME)
        for motor in (self.left_motor, self.right_motor):
            motor.setPosition(float("inf"))
            motor.setVelocity(0.0)

        # --- 엔코더 ---------------------------------------------------
        self.left_encoder = robot.getDevice(config.LEFT_ENCODER_NAME)
        self.right_encoder = robot.getDevice(config.RIGHT_ENCODER_NAME)
        self.left_encoder.enable(self.timestep)
        self.right_encoder.enable(self.timestep)

        # --- LiDAR ----------------------------------------------------
        self.lidar = robot.getDevice(config.LIDAR_NAME)
        self.lidar.enable(self.timestep)
        self.lidar.enablePointCloud()  # 쓰지는 않지만 켜 두면 GUI 에서 광선이 보인다

        # --- 자세 센서 (PROTO 에 내장돼 있다) ---------------------------
        self.compass = robot.getDevice(config.COMPASS_NAME)
        self.compass.enable(self.timestep)
        self.gyro = robot.getDevice(config.GYRO_NAME)
        self.gyro.enable(self.timestep)

        # --- 카메라 ---------------------------------------------------
        self.camera = robot.getDevice(config.CAMERA_NAME)
        self.camera.enable(self.timestep)


    # ------------------------------------------------------------------
    # 읽기
    # ------------------------------------------------------------------

    def read_encoders(self):
        """(왼쪽, 오른쪽) 바퀴 누적 회전각 [rad].

        시뮬레이션 첫 틱에는 NaN 이 나올 수 있으므로 호출한 쪽에서 확인할 것.
        """
        return self.left_encoder.getValue(), self.right_encoder.getValue()

    def read_lidar(self):
        """LiDAR 거리 배열 [m]. 길이 = config.LIDAR_RESOLUTION.

        측정 실패 지점은 inf 로 들어온다. 그대로 돌려주고 판단은 쓰는 쪽에서 한다.
        """
        return np.array(self.lidar.getRangeImage(), dtype=np.float32)

    def read_compass(self):
        """나침반 원시 3D 벡터. 각도 변환은 localization.heading_from_compass 가 한다."""
        return self.compass.getValues()

    def read_gyro_z(self):
        """z 축(수직축) 각속도 [rad/s]. 반시계가 + ."""
        return self.gyro.getValues()[2]

    def read_camera_bgr(self):
        """카메라 영상을 OpenCV 가 바로 쓰는 BGR (height, width, 3) uint8 로."""
        raw = self.camera.getImage()
        if raw is None:
            return None
        h = self.camera.getHeight()
        w = self.camera.getWidth()
        # Webots 는 BGRA 순서로 준다 (camera.md 의 getImage 설명).
        image = np.frombuffer(raw, dtype=np.uint8).reshape((h, w, 4))
        return image[:, :, :3]

    # ------------------------------------------------------------------
    # 쓰기
    # ------------------------------------------------------------------

    def verify_against_config(self):
        """실제 디바이스 스펙과 config.py 를 대조해서 다른 점을 찍어 준다.

        당일 로봇이 바뀌면 config 를 고쳐야 하는데, 안 고치고 돌리면 지도가
        조용히 틀어진다. 그걸 시작하자마자 눈에 띄게 하려고 만든 함수다.
        돌려주는 값: 어긋난 항목 목록 (비어 있으면 다 맞는 것).
        """
        checks = [
            ("LIDAR_RESOLUTION", config.LIDAR_RESOLUTION,
             self.lidar.getHorizontalResolution()),
            ("LIDAR_FOV", config.LIDAR_FOV, self.lidar.getFov()),
            ("LIDAR_MIN_RANGE", config.LIDAR_MIN_RANGE, self.lidar.getMinRange()),
            ("LIDAR_MAX_RANGE", config.LIDAR_MAX_RANGE, self.lidar.getMaxRange()),
            ("MAX_WHEEL_SPEED", config.MAX_WHEEL_SPEED,
             self.left_motor.getMaxVelocity()),
        ]

        mismatches = []
        print("\n  config.py vs 실제 디바이스:")
        for name, expected, actual in checks:
            same = abs(float(expected) - float(actual)) < 1e-3
            mark = "OK  " if same else "다름"
            print(f"    {mark} {name:<18s} config {expected:<10.4g} 실제 {actual:.4g}")
            if not same:
                mismatches.append((name, expected, actual))

        layers = self.lidar.getNumberOfLayers()
        if layers != 1:
            print(f"    다름 LiDAR 층수            1 이 아니라 {layers} 다 "
                  f"— mapping.py 는 1층 LiDAR 를 가정한다")
            mismatches.append(("LIDAR_LAYERS", 1, layers))

        # 이 둘은 코드로 읽을 방법이 없다. PROTO 를 보거나 직접 재야 한다.
        print(f"    (측정) WHEEL_RADIUS      {config.WHEEL_RADIUS} m"
              f"  / WHEEL_BASE {config.WHEEL_BASE} m"
              f"  — PROTO 를 확인할 것, 코드로는 못 읽는다")
        if not config.LIDAR_ORIENTATION_VERIFIED:
            print("    ⚠️ LiDAR 방향이 아직 측정되지 않았다 "
                  "— debug/lidar_orientation.py 를 먼저 돌릴 것")
            mismatches.append(("LIDAR_ORIENTATION", True, False))

        if mismatches:
            print(f"\n  ⚠️ {len(mismatches)} 개가 어긋난다. config.py 를 고칠 것.")
        else:
            print("\n  ✅ config.py 가 실제 로봇과 맞는다.")
        return mismatches

    def set_wheel_speeds(self, left, right):
        """바퀴 각속도 [rad/s]. 한계를 넘으면 "양쪽을 같은 비율로" 줄인다.

        ⚠️ 좌우를 따로 잘라내면 안 된다. 그러면 좌우 비율이 바뀌어
           회전 반경이 달라진다 — 주행기가 고른 (전진, 회전) 이 조용히
           다른 명령으로 바뀌는 것이다.
           전진하면서 최대로 돌 때 바깥 바퀴가 한계를 12~44% 넘으므로,
           이 일이 실제로 자주 일어난다.
           같은 비율로 줄이면 궤적(회전 반경)은 그대로 두고 느려지기만 한다.
        """
        # 한계에 "정확히" 맞추면 부동소수점 반올림으로 마지막 자리가 넘어가서
        # Webots 가 maxVelocity 초과 경고를 뱉는다. 0.1% 만 여유를 둔다.
        limit = config.MAX_WHEEL_SPEED * 0.999
        biggest = max(abs(left), abs(right))
        if biggest > limit:
            scale = limit / biggest
            left *= scale
            right *= scale
        self.left_motor.setVelocity(left)
        self.right_motor.setVelocity(right)

    def stop(self):
        self.set_wheel_speeds(0.0, 0.0)

    def drive(self, v, omega):
        """(전진속도 v [m/s], 회전속도 omega [rad/s]) → 좌우 바퀴 각속도로 변환해 넣는다.

        차동 구동 표준식:  v_wheel = v ± omega * B / 2,  그리고 rad/s = v_wheel / R
        """
        half = omega * config.WHEEL_BASE / 2.0
        self.set_wheel_speeds((v - half) / config.WHEEL_RADIUS,
                              (v + half) / config.WHEEL_RADIUS)
