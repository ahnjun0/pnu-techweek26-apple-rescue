"""크기·속도를 마음대로 정할 수 있는 '움직이는 장애물'.

받는 것: --trajectory="x1 y1, x2 y2" 와 --speed=<m/s> (월드의 controllerArgs).
하는 일: 그 선분 위를 왕복한다. Webots 의 Pedestrian 과 같은 방식으로,
         Supervisor 로 자기 translation 을 직접 옮긴다 (물리 없는 운동학 물체).

왜 만들었나: Pedestrian PROTO 에는 크기 필드가 없다 (색만 바꿀 수 있다).
대회의 "움직이는 사람" 이 우리 연습용과 같은 크기라는 보장이 없으므로,
작은 것·큰 것·빠른 것을 두고 우리 코드가 견디는지 보려고 만들었다.

⚠️ 이 컨트롤러는 자기 자신만 움직인다. 로봇의 판단에는 아무 영향도 주지 않는다
   (로봇 입장에서는 그냥 LiDAR 에 잡히는 물체다). CLAUDE.md 규칙 1 과 무관하다.
"""

import math
import sys

from controller import Supervisor


def parse_args(argv):
    """--trajectory 와 --speed 를 읽는다. 없으면 제자리에 선다."""
    points, speed = [], 0.4
    for arg in argv:
        if arg.startswith("--trajectory="):
            for pair in arg.split("=", 1)[1].split(","):
                nums = pair.split()
                if len(nums) == 2:
                    points.append((float(nums[0]), float(nums[1])))
        elif arg.startswith("--speed="):
            speed = float(arg.split("=", 1)[1])
    return points, speed


def main():
    robot = Supervisor()
    timestep = int(robot.getBasicTimeStep())
    points, speed = parse_args(sys.argv[1:])

    node = robot.getSelf()
    translation = node.getField("translation")
    height = translation.getSFVec3f()[2]

    if len(points) < 2 or speed <= 0.0:
        print("[mover] --trajectory 와 --speed 를 줘야 움직인다. 제자리에 선다.")
        while robot.step(timestep) != -1:
            pass
        return

    (x0, y0), (x1, y1) = points[0], points[1]
    length = math.hypot(x1 - x0, y1 - y0)

    while robot.step(timestep) != -1:
        # 삼각파로 왕복한다 (Pedestrian 과 같은 방식).
        phase = (robot.getTime() * speed / length) % 2.0
        ratio = phase if phase < 1.0 else 2.0 - phase
        translation.setSFVec3f([x0 + ratio * (x1 - x0),
                                y0 + ratio * (y1 - y0), height])

