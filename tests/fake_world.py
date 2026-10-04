"""Webots 없이 로봇을 굴려 보기 위한 아주 단순한 가짜 월드.

받는 것: 벽 목록 (월드 좌표 [m]).
내놓는 것: 어느 pose 에서의 가짜 LiDAR 측정값, 그리고 차동구동 한 틱 이동.
핵심 아이디어: 전체 루프(지도→프론티어→A*→추종)를 pytest 안에서 통째로 돌려 본다.
Webots 를 켜기 전에 "혼자 탐색하고 멈추는가" 를 여기서 먼저 확인한다.
※ 테스트 전용이다. 로봇 코드에서 import 하지 말 것.
"""

import math

import cv2
import numpy as np

from sar import common
from sar import config

# 벽 격자는 지도보다 곱게 쓴다 (측정값이 지도 해상도에 딱 맞아떨어지지 않게).
TRUTH_RESOLUTION = 0.02
# 실제 practice.wbt 의 벽 두께와 맞춘다. 얇으면 LiDAR 가 비스듬히 볼 때 자꾸 놓친다.
WALL_THICKNESS = 0.10


class FakeWorld:
    """벽으로 둘러싸인 방. 로봇 한 대가 산다.

    targets 를 주면 그 자리에 "빨간 기둥" 이 선다. LiDAR 에도 잡히고,
    fake_camera() 로 카메라 영상도 흉내 낼 수 있다.
    """

    def __init__(self, walls, size=6.0, noise=0.0, seed=0, targets=(),
                 walker=None, walker_speed=0.4, walker_phase=0.0):
        """walls: [(x0, y0, x1, y1), ...] 선분 목록. size: 정사각 아레나 한 변 [m].

        walker 를 (x0, y0, x1, y1) 로 주면 그 선분을 왕복하는 "사람" 이 생긴다.
        LiDAR 에 잡히고, 로봇과 겹치면 충돌로 센다.
        walker_phase 로 시작 위치를 바꿔 여러 조건에서 재 볼 수 있다.
        """
        self.targets = [tuple(t) for t in targets]
        self.walker = tuple(walker) if walker else None
        self.walker_speed = walker_speed
        self.walker_phase = walker_phase
        self.walker_radius = 0.18       # 사람 몸통 굵기
        self.time = 0.0
        self.half = size / 2.0
        self.cells = int(round(size / TRUTH_RESOLUTION))
        self.occupied = np.zeros((self.cells, self.cells), dtype=bool)
        self.noise = noise
        self.random = np.random.default_rng(seed)

        border = self.half - TRUTH_RESOLUTION
        for segment in [(-border, -border, border, -border),
                        (-border, border, border, border),
                        (-border, -border, -border, border),
                        (border, -border, border, border)]:
            self._draw(*segment)
        for segment in walls:
            self._draw(*segment)
        # 목표물도 LiDAR 에 잡히는 실제 물체다 (반지름 0.08 m 기둥)
        for tx, ty in self.targets:
            self._draw(tx - 0.08, ty, tx + 0.08, ty, thickness=0.16)

    # --- 내부 격자 -----------------------------------------------------

    def _index(self, value):
        return int(round((value + self.half) / TRUTH_RESOLUTION))

    def _draw(self, x0, y0, x1, y1, thickness=WALL_THICKNESS):
        """선분을 두께 있는 벽으로 칠한다.

        두께가 중요하다. 한 칸짜리 종잇장 벽으로 만들면 LiDAR 가 비스듬히 볼 때
        듬성듬성 맞아서 지도에 점선처럼 찍힌다. 실제 Webots 벽은 0.1 m 다.
        """
        steps = int(math.hypot(x1 - x0, y1 - y0) / (TRUTH_RESOLUTION / 2)) + 1
        half = int(round(thickness / 2.0 / TRUTH_RESOLUTION))
        for t in np.linspace(0.0, 1.0, steps):
            r = self._index(y0 + t * (y1 - y0))
            c = self._index(x0 + t * (x1 - x0))
            for dr in range(-half, half + 1):
                for dc in range(-half, half + 1):
                    rr, cc = r + dr, c + dc
                    if 0 <= rr < self.cells and 0 <= cc < self.cells:
                        self.occupied[rr, cc] = True

    def _hits(self, xs, ys):
        """월드 좌표 배열 → 그 자리가 벽인가 (배열 밖은 벽으로 친다)."""
        rows = np.round((ys + self.half) / TRUTH_RESOLUTION).astype(int)
        cols = np.round((xs + self.half) / TRUTH_RESOLUTION).astype(int)
        inside = ((rows >= 0) & (rows < self.cells)
                  & (cols >= 0) & (cols < self.cells))
        result = np.ones(xs.shape, dtype=bool)
        result[inside] = self.occupied[rows[inside], cols[inside]]
        return result

    # --- 걸어다니는 사람 ------------------------------------------------

    def walker_position(self):
        """지금 사람이 어디 있나. 사람이 없으면 None."""
        if self.walker is None:
            return None
        x0, y0, x1, y1 = self.walker
        length = math.hypot(x1 - x0, y1 - y0)
        if length < 1e-9:
            return (x0, y0)
        # 삼각파로 왕복시킨다
        period = 2.0 * length / self.walker_speed
        phase = ((self.time + self.walker_phase * period) % period) / period
        ratio = 2.0 * phase if phase < 0.5 else 2.0 * (1.0 - phase)
        return (x0 + ratio * (x1 - x0), y0 + ratio * (y1 - y0))

    def advance_time(self, dt):
        self.time += dt

    def _walker_hits(self, xs, ys):
        """사람 몸통에 닿는 점들 (원으로 본다)."""
        where = self.walker_position()
        if where is None:
            return np.zeros(np.shape(xs), dtype=bool)
        return np.hypot(xs - where[0], ys - where[1]) <= self.walker_radius

    # --- 센서 ----------------------------------------------------------

    def lidar(self, pose):
        """그 pose 에서 보이는 LiDAR 거리 배열. 못 맞히면 inf."""
        x, y, theta = pose
        angles = common.lidar_angles() + theta
        steps = np.arange(config.LIDAR_MIN_RANGE, config.LIDAR_MAX_RANGE,
                          TRUTH_RESOLUTION)

        # (광선 수, 걸음 수) 를 한 번에 계산한다. 하나씩 돌면 너무 느리다.
        sample_x = x + np.outer(np.cos(angles), steps)
        sample_y = y + np.outer(np.sin(angles), steps)
        blocked = self._hits(sample_x, sample_y) | self._walker_hits(sample_x, sample_y)

        ranges = np.full(len(angles), np.inf, dtype=np.float64)
        any_hit = blocked.any(axis=1)
        first = blocked.argmax(axis=1)
        ranges[any_hit] = steps[first[any_hit]]

        if self.noise:
            ranges[any_hit] += self.random.normal(0.0, self.noise, any_hit.sum())
        return ranges

    # --- 움직임 --------------------------------------------------------

    def collides(self, x, y, radius=None):
        """그 자리에 로봇(원)이 들어갈 수 있는가."""
        radius = config.ROBOT_RADIUS if radius is None else radius
        offsets = np.linspace(-radius, radius, 9)
        grid_x, grid_y = np.meshgrid(offsets, offsets)
        inside = grid_x ** 2 + grid_y ** 2 <= radius ** 2
        points_x = x + grid_x[inside]
        points_y = y + grid_y[inside]
        return bool(self._hits(points_x, points_y).any()
                    or self._walker_hits(points_x, points_y).any())

    @staticmethod
    def clip_to_wheels(speed, turn):
        """바퀴가 낼 수 있는 범위로 줄인다 (좌우 비율은 지킨다).

        실제 로봇은 sensors.set_wheel_speeds 가 이렇게 한다.
        여기서도 같게 해야 측정이 실제와 맞는다.
        """
        left = (speed - turn * config.WHEEL_BASE / 2) / config.WHEEL_RADIUS
        right = (speed + turn * config.WHEEL_BASE / 2) / config.WHEEL_RADIUS
        biggest = max(abs(left), abs(right))
        if biggest > config.MAX_WHEEL_SPEED:
            scale = config.MAX_WHEEL_SPEED / biggest
            speed *= scale
            turn *= scale
        return speed, turn

    def move(self, pose, speed, turn, dt):
        """차동구동 한 틱. 벽에 부딪히면 제자리에 머문다 (실제로 끼이는 상황 재현).

        사람이 로봇을 덮치면 **밀어낸다**. Webots 의 Pedestrian 은 physics 가 없는
        운동학 물체라 로봇을 무한한 힘으로 민다 — 벽 쪽으로 밀리면 로봇이 벽에
        끼고 오도메트리가 망가진다. 그 위험을 여기서도 재현해야 판단이 맞는다.

        돌려주는 것: (새 pose, 부딪혔는가)
        """
        speed, turn = self.clip_to_wheels(speed, turn)
        x, y, theta = pose
        new_theta = common.wrap_angle(theta + turn * dt)
        mid = theta + 0.5 * common.angle_diff(new_theta, theta)
        new_x = x + speed * math.cos(mid) * dt
        new_y = y + speed * math.sin(mid) * dt

        touched = False
        if self.collides(new_x, new_y):
            # ⚠️ 이미 겹쳐 있는 상태라면 움직임을 막으면 안 된다.
            #    실제 물리에서는 벽에 눌린 로봇도 후진해서 빠져나올 수 있다.
            #    여기서 다 막았더니 한 번 박힌 로봇이 영원히 못 나와,
            #    "탈출 동작이 소용없다" 는 잘못된 결론이 나왔다.
            if not self.collides(x, y):
                new_x, new_y = x, y             # 멀쩡한 자리에서 벽으로는 못 간다
            touched = True

        # 사람이 덮쳤으면 밀어낸다 (벽이 있으면 못 밀려나고 끼인다)
        where = self.walker_position()
        if where is not None:
            overlap = (self.walker_radius + config.ROBOT_RADIUS
                       - math.hypot(new_x - where[0], new_y - where[1]))
            if overlap > 0.0:
                touched = True
                away = math.atan2(new_y - where[1], new_x - where[0])
                pushed_x = new_x + overlap * math.cos(away)
                pushed_y = new_y + overlap * math.sin(away)
                if not self._hits(np.array([pushed_x]), np.array([pushed_y]))[0]:
                    new_x, new_y = pushed_x, pushed_y
        return (new_x, new_y, new_theta), touched


    # --- 카메라 흉내 ---------------------------------------------------

    def camera(self, pose, fov=1.0, width=128, height=96):
        """그 pose 에서 보이는 목표물(바닥에 놓인 지름 2·TARGET_RADIUS 공)을 그린 가짜 BGR 영상.

        detect.camera_range 와 같은 핀홀 모형이다: 초점거리 f = (폭/2)/tan(fov/2),
        카메라는 CAMERA_FORWARD 앞·CAMERA_HEIGHT 위, 수평을 본다.
        공의 밑동 행 = 가운데 행 + f·CAMERA_HEIGHT/d, 반지름 = f·tan(asin(R/d)).
        그래서 크기로 잰 거리와 바닥으로 잰 거리가 서로 맞는다 (진짜 사과처럼).
        """
        image = np.full((height, width, 3), 50, dtype=np.uint8)
        x, y, theta = pose
        cam_x = x + config.CAMERA_FORWARD * math.cos(theta)
        cam_y = y + config.CAMERA_FORWARD * math.sin(theta)
        focal = (width / 2.0) / math.tan(fov / 2.0)
        centre_row = (height - 1) / 2.0
        centre_col = (width - 1) / 2.0
        for tx, ty in self.targets:
            gap = math.hypot(tx - cam_x, ty - cam_y)
            if gap > config.LIDAR_MAX_RANGE or gap <= config.TARGET_RADIUS:
                continue
            bearing = common.wrap_angle(math.atan2(ty - cam_y, tx - cam_x) - theta)
            if abs(bearing) > fov / 2:
                continue
            # 가리는 것이 있으면 안 보인다 (벽 뒤의 목표물)
            if self._blocked_between(x, y, tx, ty):
                continue
            radius = focal * math.tan(math.asin(config.TARGET_RADIUS / gap))
            column = centre_col - math.tan(bearing) * focal
            bottom = centre_row + focal * config.CAMERA_HEIGHT / gap
            cv2.circle(image, (int(round(column)), int(round(bottom - radius))),
                       max(1, int(round(radius))), (0, 0, 255), thickness=-1)
        return image

    def _blocked_between(self, x0, y0, x1, y1):
        """두 점 사이에 벽이 있는가 (목표물 자신은 빼고 본다)."""
        steps = int(math.hypot(x1 - x0, y1 - y0) / TRUTH_RESOLUTION) + 1
        ts = np.linspace(0.0, 0.85, max(steps, 2))     # 목표물 직전까지만
        xs = x0 + ts * (x1 - x0)
        ys = y0 + ts * (y1 - y0)
        return bool(self._hits(xs, ys).any())


# --------------------------------------------------------------------------
# 미리 만들어 둔 월드들
# --------------------------------------------------------------------------

def two_rooms():
    """가운데 벽에 문 하나. practice.wbt 를 닮았지만 별개다."""
    return FakeWorld(walls=[
        (0.0, -2.9, 0.0, -0.5),
        (0.0, 0.5, 0.0, 2.9),
    ])


def four_rooms():
    """방이 여러 개라 한참 돌아다녀야 다 본다."""
    return FakeWorld(walls=[
        (0.0, -2.9, 0.0, -0.6),
        (0.0, 0.6, 0.0, 2.9),
        (-2.9, 0.0, -0.8, 0.0),
        (0.8, 0.0, 2.9, 0.0),
    ])


def open_room():
    """장애물 없는 빈 방."""
    return FakeWorld(walls=[])


def crowded_world(phase=0.0):
    """목표물 3개 + 걸어다니는 사람 1명. 사람을 피하는 능력을 재는 데 쓴다.

    사람이 목표물 하나로 가는 길을 가로지른다 — practice.wbt 와 같은 어려움이다.
    """
    return FakeWorld(
        walls=[(0.0, -2.9, 0.0, -0.5), (0.0, 0.5, 0.0, 2.9)],
        targets=[(-2.0, 2.0), (2.0, 2.0), (2.0, -2.0)],
        walker=(-2.4, 0.8, 0.4, 0.8), walker_phase=phase)


def rescue_world():
    """목표물 3개가 있는 방 — 탐색·접근·복귀를 통째로 확인하는 데 쓴다."""
    return FakeWorld(
        walls=[(0.0, -2.9, 0.0, -0.5), (0.0, 0.5, 0.0, 2.9)],
        targets=[(-2.0, 2.0), (2.0, 2.0), (2.0, -2.0)])
