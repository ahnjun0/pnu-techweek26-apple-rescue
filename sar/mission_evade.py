"""임무의 비키기 쪽 — 다가오는 사람을 비켜 선다 (people.person_threat).

송지윤(yun110w)의 song-jiyun 브랜치(279d5d5)에서 옮겼다.
mission.Mission 이 이 mixin 을 물려받는다. 상태(self.*)는 Mission.__init__ 이 만든다.
"""

import math

from . import common
from . import config
from . import follower
from . import people as people_mod
from .mission_state import EVADE, EXPLORE, _tally


class EvadeMixin:
    """EVADE 상태 — 사람이 지나가면 하던 일로 돌아간다."""

    def _enter_evade(self):
        self._evade_resume = self.state
        self._evade_left = config.EVADE_MAX_SECONDS
        self._evade_calm = 0.0
        self.state = EVADE
        _tally(f"EVADE 진입 ({self._evade_resume})")

    def _leave_evade(self):
        self.state = self._evade_resume or EXPLORE
        self._evade_resume = None
        self._clear_path()          # 비키느라 자리가 바뀌었다 — 경로를 다시 짠다

    def _evade(self, pose, ranges, dt):
        """사람의 진행 방향에서 옆으로(또는 반대로) 비켜 선다.

        1. 눌렸으면 그게 먼저다 (_escape_if_pinned — 모든 상태 공통).
        2. 위협이 EVADE_CALM_SECONDS 동안 없으면, 또는 EVADE_MAX_SECONDS 가 지나면 복귀.
        3. 방향은 _evade_heading 이 고른다 — 사람이 걷는 줄에서 옆으로 벗어나는 쪽.
        4. 트인 쪽이 없으면 그 자리에 선다. 보행자는 로봇을 넘어뜨리지 않으므로
           벽에 몸을 비비는 것보다 서서 지나가게 두는 편이 낫다.
        """
        escape = self._escape_if_pinned(ranges, dt)
        if escape is not None:
            return escape

        self._evade_left -= dt
        threat = people_mod.person_threat(pose, self._people)
        self._evade_calm = 0.0 if threat is not None else self._evade_calm + dt
        if self._evade_calm >= config.EVADE_CALM_SECONDS or self._evade_left <= 0.0:
            _tally("EVADE 끝 (시간 초과)" if self._evade_left <= 0.0 else "EVADE 끝")
            self._leave_evade()
            return 0.0, 0.0
        if threat is None:
            self.status = "EVADE — 사람이 지나갔다, 잠깐 지켜본다"
            return 0.0, 0.0

        heading = self._evade_heading(pose, ranges, threat)
        if heading is None:
            _tally("EVADE 틱 — 비킬 곳 없음")
            self.status = "EVADE — 비킬 곳이 없다, 제자리에서 기다림"
            return 0.0, 0.0

        error = common.wrap_angle(heading - pose[2])
        turn = max(-config.FOLLOW_MAX_TURN,
                   min(config.FOLLOW_MAX_TURN, config.FOLLOW_TURN_GAIN * error))
        if abs(error) > math.radians(config.EVADE_ALIGN_DEGREES):
            self.status = "EVADE — 비킬 쪽으로 도는 중"
            return 0.0, turn
        # 가는 쪽이 정지거리 안으로 막혀 오면 멈춘다 (후보를 고른 뒤 상황이 바뀔 수 있다).
        room = follower.sector_min(ranges, 0.0,
                                   math.radians(config.EVADE_SECTOR_DEGREES))
        if room < config.SAFETY_STOP_DISTANCE + config.ROBOT_RADIUS:
            self.status = "EVADE — 앞이 막혀 멈춤"
            return 0.0, turn
        self.status = "EVADE — 사람을 비켜 가는 중"
        return config.FOLLOW_MAX_SPEED, turn

    def _evade_heading(self, pose, ranges, person):
        """비킬 방향 (월드 각 [rad]). 트인 후보가 없으면 None.

        1순위 — 사람 진행 방향의 왼쪽/오른쪽 직각. LiDAR 로 EVADE_MIN_ROOM 만큼 트인 쪽 중,
                 EVADE_HORIZON 동안 가서 섰을 때 **사람이 걷는 줄에서 더 멀어지는** 쪽.
                 (이미 줄에서 비켜 있는 쪽이 저절로 뽑힌다.)
        2순위 — 양옆이 막혔으면 사람 반대쪽으로 물러난다.
        ⚠️ "사람과의 최근접 거리가 큰 쪽" 으로 고르면 정면에서 오는 사람에게서 **같은 줄로
           뒤로 도망가는** 쪽이 이긴다 (3초 동안 1.0 m 유지 vs 옆으로 0.71 m).
           그러면 사람이 계속 따라오는 꼴이다 — 비키는 게 아니다. 그래서 줄에서의 거리로 본다.
        """
        px, py, vx, vy = person[:4]
        speed = math.hypot(vx, vy)
        ux, uy = vx / speed, vy / speed              # 사람이 걷는 방향 (단위벡터)
        walk = math.atan2(uy, ux)
        half = math.radians(config.EVADE_SECTOR_DEGREES)

        def room_toward(heading):
            return follower.sector_min(ranges, common.wrap_angle(heading - pose[2]), half)

        best, best_offset = None, -math.inf
        for heading in (walk + math.pi / 2.0, walk - math.pi / 2.0):
            room = room_toward(heading)
            if room < config.EVADE_MIN_ROOM:
                continue
            moved = min(config.FOLLOW_MAX_SPEED * config.EVADE_HORIZON,
                        room - config.EVADE_MIN_ROOM)
            ex = pose[0] + moved * math.cos(heading) - px
            ey = pose[1] + moved * math.sin(heading) - py
            offset = abs(ux * ey - uy * ex)          # 걷는 줄에서 떨어진 거리
            if offset > best_offset:
                best, best_offset = heading, offset
        if best is not None:
            return best
        away = math.atan2(pose[1] - py, pose[0] - px)
        return away if room_toward(away) >= config.EVADE_MIN_ROOM else None
