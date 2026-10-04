"""시험용 작은 아레나 — tests/fake_world.py 의 원점 중심 8×8 m 세계에 맞춘 config 값.

config 의 기본값은 대회(apartment: 16×16 m 지도, 시작 (-0.3, -7.5), 64 ms)다.
가짜 월드는 그보다 작고 16 ms 를 전제로 쓰였으므로 테스트에서만 이 값으로 바꾼다.
⚠️ 기하·주기·바퀴 반지름 같은 **입력** 만 바꾼다. 판단 로직은 대회 그대로 시험한다.
⚠️ conftest 가 **import 될 때** 넣는다 — 테스트 모듈이 import 시점에 config 로 계산하는 값이 있다.
"""
from sar import config

VALUES = {
    "TIME_STEP": 16,
    "START_X": -2.5,
    "START_Y": -2.5,
    "START_THETA": 0.0,
    "MAP_ORIGIN_X": -4.0,
    "MAP_ORIGIN_Y": -4.0,
    "MAP_WIDTH_CELLS": 160,
    "MAP_HEIGHT_CELLS": 160,
    "MISSION_TARGET_COUNT": 3,
}


def install():
    for name, value in VALUES.items():
        assert hasattr(config, name), f"config 에 {name} 이 없다"
        setattr(config, name, value)
    # 합성 엔코더 시험이다 — apartment 바닥 보정(÷0.983) 없이 기하 반지름을 쓴다.
    config.WHEEL_RADIUS_ODOM = config.WHEEL_RADIUS
