"""로봇의 진짜 위치(정답)를 Supervisor 로 읽는다. **debug/ 전용** (CLAUDE.md 규칙 1).

주최 측 controllers/tb3_ground_truth/tb3_ground_truth.py:28,59-65 와 같은 방식이다:
    position = robot_node.getPosition()
    orientation = robot_node.getOrientation()
    theta = math.atan2(orientation[3], orientation[0])
(getOrientation 은 3x3 회전행렬을 행 우선 9개로 준다 → [0]=cosθ, [3]=sinθ.)

예전에는 GPS 디바이스를 월드 사본에 달아 읽었다. 둘 다 정답이지만 하나로 통일했다 —
Supervisor 는 방향까지 주고, 월드에 디바이스를 더 달 필요가 없다.
주행 판단에는 절대 쓰지 않는다. 채점(오차·방문·접촉)에만 쓴다.
"""
import math


class Truth:
    def __init__(self, robot):
        if not robot.getSupervisor():
            raise RuntimeError(
                "진짜 위치를 읽으려면 월드의 로봇에 supervisor TRUE 가 필요하다 (채점용 사본 월드).")
        self.node = robot.getSelf()

    def pose(self):
        """(x, y, theta) — theta 는 +x 축이 0, 반시계가 + [rad]."""
        x, y, _ = self.node.getPosition()
        r = self.node.getOrientation()
        return x, y, math.atan2(r[3], r[0])

    def xy(self):
        x, y, _ = self.pose()
        return x, y
