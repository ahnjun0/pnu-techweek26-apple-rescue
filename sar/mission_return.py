"""임무의 복귀 쪽 — 시작점으로 돌아가기, 지나온 길 되짚기, 복귀 마감.

mission.Mission 이 이 mixin 을 물려받는다. 상태(self.*)는 Mission.__init__ 이 만든다.
"""

import math

import numpy as np

from . import common
from . import config
from . import follower
from . import planner
from .mission_state import DONE, EXPLORE, _tally


def retrace(crumbs, join=None):
    """지나온 길(crumbs, 오래된 것부터)을 거꾸로 — 지금 자리에서 시작점까지.

    같은 자리를 두 번 지났으면(고리) 그 사이는 건너뛴다. 매 점에서 **join 안에
    있는 가장 오래된 점** 으로 넘어간다. 건너뛰는 거리는 join 이하라서 새로
    생기는 직선 구간도 이미 지나온 자리 바로 옆이다.
    """
    if join is None:
        join = config.RETURN_CRUMB_JOIN
    if not crumbs:
        return []
    pts = np.asarray(crumbs, dtype=np.float64)
    out = []
    i = len(pts) - 1
    while True:
        out.append((float(pts[i, 0]), float(pts[i, 1])))
        if i == 0:
            return out
        near = np.flatnonzero(np.hypot(pts[:i, 0] - pts[i, 0],
                                       pts[:i, 1] - pts[i, 1]) <= join)
        i = int(near[0]) if near.size else i - 1


def _first_metres(points, metres):
    """꺾은선 points 의 앞에서부터 길이 metres 까지. 마지막 구간은 그 길이에서 자른다.

    ⚠️ 마지막 구간을 통째로 넣었더니, 복귀 중에는 지나온 길을 안 적어서 첫 구간이 복귀를 시작한
       자리까지 6.7 m 였고 로봇이 그만큼 거슬러 가 복귀 시간이 끝났다 (2026-10-06 지연 3초).
    """
    out = [points[0]]
    total = 0.0
    for a, b in zip(points, points[1:]):
        step = common.distance(*a, *b)
        if total + step >= metres:
            ratio = (metres - total) / step if step > 0.0 else 0.0
            out.append((a[0] + (b[0] - a[0]) * ratio, a[1] + (b[1] - a[1]) * ratio))
            return out
        out.append(b)
        total += step
    return out


class ReturnMixin:
    """RETURN 상태와 복귀 마감."""

    # ------------------------------------------------------------------
    # RETURN — 시작 지점으로 돌아간다
    # ------------------------------------------------------------------

    def _return(self, pose, ranges, dt):
        home = self.start_pose[:2]
        gap = common.distance(pose[0], pose[1], *home)
        if gap <= config.RETURN_TOLERANCE:
            self.state = DONE
            self.status = f"복귀 완료 (오차 {gap * 100:.0f} cm)"
            return 0.0, 0.0

        self._return_age += dt      # 탈출 중에도 시계는 돈다
        if self._return_age > config.RETURN_TIMEOUT:
            # 영원히 시도하느니 멈추는 게 낫다. 어디서 멈췄는지 남긴다.
            self.state = DONE
            self.status = f"복귀 시간 초과 — {gap:.2f} m 남기고 멈춘다"
            return 0.0, 0.0

        escape = self._escape_if_pinned(ranges, dt)
        if escape is not None:
            return escape

        self._since_replan += dt
        if self.goal is None:
            self.goal = home
            self._replan(pose)
        if self._since_replan >= config.MISSION_REPLAN_EVERY \
                or planner.path_is_blocked(self.path, self.plan_grid):
            self._replan(pose)

        # 지도에 길은 있는데 한 자리에서 맴돌면 들어온 길을 조금 되짚어 나온다
        # (config.RETURN_STALL_TIME 설명 참고). 들어온 길은 이미 한 번 지나간 길이다.
        if self._unstick:
            return self._back_out(pose, ranges, dt, gap)
        if self._stalled(pose, dt) and self.path and len(self._crumbs) >= 2:
            trail = [pose[:2]] + retrace(self._crumbs)
            self._unstick = _first_metres(trail, config.RETURN_UNSTICK_DISTANCE)
            self._unstick_index = 0
            self._stall_anchor = None       # 되짚는 동안의 맴돎은 새로 잰다
            _tally("복귀: 맴돌아서 들어온 길을 조금 되짚음")
            return self._back_out(pose, ranges, dt, gap)

        if not self.path:
            # 계획이 안 된다. 가만히 서서 다시 시도만 하면 영영 못 돌아간다
            # (실제로 300초를 그대로 서 있었다).
            # 집 쪽으로 직선을 하나 긋고 DWA 에게 맡긴다 — DWA 는 부딪히는
            # 명령을 애초에 안 내므로 벽을 피해 가며 조금씩이라도 다가간다.
            self._no_path_age += dt
            if self._no_path_age < config.RETURN_DIRECT_AFTER:
                self.status = "복귀 경로를 찾는 중"
                self._clear_path()
                return 0.0, 0.0
            if len(self._crumbs) >= 2:
                if not self._retrace:
                    self._retrace = [pose[:2]] + retrace(self._crumbs)
                    self._retrace_index = 0
                    _tally("복귀: 지나온 길 되짚기 시작")
                self.status = (f"복귀 — 지도에 길이 없어 지나온 길을 되짚는다"
                               f" ({gap:.2f} m 남음)")
                speed, turn, _, self._retrace_index = follower.step(
                    pose, self._retrace, ranges, self._retrace_index,
                    current_speed=self.last_speed, current_turn=self.last_turn,
                    dt=dt, allow_idle=self._person_is_close(pose),
                    people=self._people + self._low_ghosts())
                return speed, turn
            self.status = f"복귀 — 경로 없음, 직선 접근 ({gap:.2f} m 남음)"
            direct = [pose[:2], home]
            speed, turn, _, _ = follower.step(
                pose, direct, ranges, 0,
                current_speed=self.last_speed, current_turn=self.last_turn, dt=dt,
            allow_idle=self._person_is_close(pose), people=self._people + self._low_ghosts())
            return speed, turn

        self._no_path_age = 0.0
        retraced = bool(self._retrace)
        self._retrace = []          # 지도 경로가 다시 생겼다. 되짚기는 버린다

        if self._resume_after_escape:
            # 갇혀서 시작한 복귀다. 집까지 길이 생겼으면 빠져나온 것이다.
            self._resume_after_escape = False
            if self._targets_missing() and self.elapsed < self._return_deadline(pose):
                # ⚠️ 되짚어 빠져나온 것만 센다. 되짚기 없이 길이 바로 생겼으면 잠깐
                #    막혔던 것이다 — 대회 월드(2026-10-05)에서 67·70초의 0.2초짜리
                #    막힘 두 번에 한도를 다 써서, 575초에 정말 갇혔을 때 못 빠져나왔다.
                if retraced:
                    self._escape_resumes += 1
                self._return_age = 0.0
                self._clear_goal()
                self.state = EXPLORE
                self.status = "빠져나왔다 — 탐색을 이어 간다"
                _tally("갇힘: 빠져나와 탐색을 이어 감")
                return 0.0, 0.0

        speed, turn, status, self.path_index = follower.step(
            pose, self.path, ranges, self.path_index,
            current_speed=self.last_speed, current_turn=self.last_turn, dt=dt,
            allow_idle=self._person_is_close(pose), people=self._people + self._low_ghosts())
        # 경로 끝에 왔는데 집이 아직 멀면 계획기가 더 가까이 못 데려다 주는 것이다 (집 칸이 팽창 안).
        # APPROACH 와 같이 한 번 다시 짜 보고, 그래도 같으면 그 자리를 집으로 친다.
        # ⚠️ 2026-10-06: 추정 위치가 35 cm 틀어져 로봇이 아는 집이 벽 팽창 안에 들어갔고, 집 0.41 m
        #    앞에서 20초를 서 있다가 되짚기로 빠졌다 (진짜로는 집 7 cm 앞이었다). 옛 실행 셋에서 같은 일.
        if status == follower.ARRIVED:
            if not self._home_arrival_retried:
                self._home_arrival_retried = True
                self._replan(pose)
                return 0.0, 0.0
            self.state = DONE
            self.status = f"복귀 완료 — 시작점 {gap * 100:.0f} cm 앞, 계획기가 더 못 다가간다"
            return 0.0, 0.0
        self._home_arrival_retried = False
        self.status = f"RETURN — {status} ({gap:.2f} m 남음)"
        return speed, turn

    def _stalled(self, pose, dt):
        """RETURN_STALL_RADIUS 안에 RETURN_STALL_TIME 넘게 머물렀나."""
        if (self._stall_anchor is None
                or common.distance(*self._stall_anchor, *pose[:2]) > config.RETURN_STALL_RADIUS):
            self._stall_anchor = pose[:2]
            self._stall_age = 0.0
            return False
        self._stall_age += dt
        return self._stall_age > config.RETURN_STALL_TIME

    def _back_out(self, pose, ranges, dt, gap):
        """들어온 길을 조금 되짚는다. 끝에 닿거나 거기서도 막히면 다시 짠다."""
        speed, turn, status, self._unstick_index = follower.step(
            pose, self._unstick, ranges, self._unstick_index,
            current_speed=self.last_speed, current_turn=self.last_turn, dt=dt,
            allow_idle=self._person_is_close(pose), people=self._people + self._low_ghosts())
        if status == follower.ARRIVED or self._stalled(pose, dt):
            self._unstick = []
            self._stall_anchor = None
            self._replan(pose)
            return 0.0, 0.0
        self.status = f"복귀 — 막혀서 들어온 길을 조금 되짚는다 ({gap:.2f} m 남음)"
        return speed, turn

    def _home_reachable(self, pose):
        """지금 자리에서 집까지 계획기로 길이 나오나 (_replan 과 같은 순서: 평소, 안 되면 좁게).

        ⚠️ exact=True 다. 아니면 plan() 이 막힌 목표를 근처 칸으로 바꿔 치워 "길이 있다" 고
           답한다 (planner.plan 의 설명 참고) — 갇힌 로봇도 언제나 집에 갈 수 있게 보인다.
        """
        home = self.start_pose[:2]
        return bool(planner.plan(self.plan_grid, pose[:2], home,
                                 people=self._people_xy, exact=True)
                    or planner.plan(self.plan_grid, pose[:2], home,
                                    margin=config.PLANNER_SQUEEZE_MARGIN,
                                    people=self._people_xy, exact=True))

    # ------------------------------------------------------------------

    def _return_deadline(self, pose):
        """이 시각을 넘기면 복귀를 시작해야 한다 [s].

        ⚠️ 처음엔 "제한 시간의 75%" 라는 고정 비율이었다. 근거가 없었고,
           그 선에 걸려 목표물을 둘 잃은 실행이 있었다.
           실제로 필요한 것은 **시작점까지 돌아갈 시간** 이므로 거기서 거꾸로 센다.
           시작점 근처면 거의 끝까지 탐색하고, 멀리 있으면 일찍 돌아선다.
        """
        if not config.MISSION_TIME_LIMIT:
            return math.inf
        home = (self.start_pose[:2] if self.start_pose
                else (config.START_X, config.START_Y))

        # ⚠️ 처음엔 **직선거리 / 최대속도** 로 쟀다. 두 가정이 다 틀렸다:
        #    ① 실제 경로는 직선보다 훨씬 길다 (빗살·복도를 돌아나가야 한다)
        #    ② 실제 평균 속도는 최대속도의 절반도 안 된다 (느린 주행이 78.5% 인 실행이 있었다)
        #    그래서 복귀를 **5.88 m 남기고** 시간이 끝났다.
        #    둘 다 추측 대신 **가진 정보** 로 바꾼다 — 경로는 계획기에게 묻고,
        #    속도는 지금까지 실제로 낸 평균을 쓴다.
        # ⚠️ 이 계산을 **매 틱** 했더니 시뮬레이션이 크게 느려졌다 (A* 가 2 ms).
        #    1100초 벽시계로 900초를 돌던 것이 580초까지밖에 못 갔다.
        #    복귀 마감은 매 틱 정확할 필요가 없으므로 가끔만 다시 센다.
        if (self._home_gap is None
                or self._ticks - self._home_gap_tick
                >= config.MISSION_RETURN_RECHECK):
            path = planner.plan(self.plan_grid, pose[:2], home)
            self._home_gap = (
                sum(common.distance(*a, *b) for a, b in zip(path, path[1:]))
                if path else common.distance(pose[0], pose[1], *home))
            self._home_gap_tick = self._ticks
        gap = self._home_gap

        speed = config.FOLLOW_MAX_SPEED
        if self.elapsed > 1.0 and self._travelled > 0.0:
            speed = max(self._travelled / self.elapsed, speed * 0.25)

        need = max(gap / speed * config.MISSION_RETURN_SLACK,
                   config.MISSION_RETURN_MIN)
        return config.MISSION_TIME_LIMIT - need
