"""임무의 접근 쪽 — 찾은 목표물로 다가가 방문 판정을 받는다.

mission.Mission 이 이 mixin 을 물려받는다. 상태(self.*)는 Mission.__init__ 이 만든다.
"""

from . import common
from . import config
from . import follower
from . import planner
from .mission_state import APPROACH, _tally


class ApproachMixin:
    """APPROACH 상태."""

    # ------------------------------------------------------------------
    # APPROACH — 찾은 목표물로 다가간다
    # ------------------------------------------------------------------

    def _enter_approach(self, pose):
        self.state = APPROACH
        self._clear_goal()
        self._goal_age = 0.0
        self._arrival_retried = False

    def _approach(self, pose, ranges, dt):
        # ⚠️ 시계는 탈출 중에도 돌아야 한다. 탈출할 때 시간을 안 세면,
        #    계속 눌려 있는 동안 시간 초과가 영영 안 걸려 그 목표물에
        #    영원히 매달린다 (실제로 900초를 APPROACH 상태로 보냈다).
        self._goal_age += dt

        escape = self._escape_if_pinned(ranges, dt)
        if escape is not None:
            return escape

        target = self.targets.nearest_unvisited(*pose[:2])
        if target is None:
            # 확정 전 후보도 목적지로 쓴다. 다가가는 동안 본 횟수가 쌓이면 확정된다.
            # ⚠️ 예전에는 확인하러 온 경우(_verify_tried)에만 그랬다. 그래서 탐색 중
            #    끼어든 APPROACH 는 다음 틱에 곧바로 EXPLORE 로 나갔고, 후보 추정이
            #    조금씩 움직여 _interrupted_for 에도 안 걸려 또 끼어들었다 — 상태줄만
            #    "본 것부터 확인한다" 였다. 대회 월드(2026-10-05): 화장실 사과를 22회
            #    보고(확정 25회) 떠났다. 배준호(bae-junho 브랜치)가 짚은 고침이다.
            target = self.targets.best_unconfirmed()
        if target is None:
            self._leave_approach(pose)
            return 0.0, 0.0
        gap = common.distance(pose[0], pose[1], *target.position)

        # 충분히 다가갔으면 방문으로 친다.
        if gap <= config.APPROACH_DISTANCE:
            target.visited = True
            self.status = (f"목표물 방문 ({target.x:+.2f}, {target.y:+.2f}) "
                           f"— {len(self.targets.unvisited())} 개 남음")
            self._leave_approach(pose)
            return 0.0, 0.0

        # 너무 오래 걸리면 포기한다 (벽 뒤에 있는 목표물 등).
        if self._goal_age > config.APPROACH_TIMEOUT:
            target.visited = True          # 더 매달리지 않는다
            self.status = "목표물 접근 시간 초과 — 건너뛴다"
            self._leave_approach(pose)
            return 0.0, 0.0

        # ⚠️ 목표물 추정이 움직이면 경로를 새로 짠다. 그런데 추정은 관측을
        #    weight = 1/sightings 로 섞으므로 **관측이 적을 때 크게 흔들린다.**
        #    그 흔들림마다 경로와 조준점이 바뀌면 목표물 직전에서 주저하는 것처럼
        #    보인다 (사용자가 GUI 에서 짚었다). 얼마나 자주 일어나는지 센다.
        # ⚠️ 여기서 목표를 "판정을 만족하는 설 자리" 로 옮겨 봤다가 **되돌렸다.**
        #    겨냥을 옮겨도 주저가 전혀 안 줄었다 (목표물 근처 18.4초/85% ->
        #    18.9초/84%). 느린 띠는 **어디를 겨누느냐가 아니라 장애물에 얼마나
        #    가까우냐** 로 정해지는데, 방문 판정을 받으려면 로봇이 물리적으로
        #    그 띠 안에 들어가야 하기 때문이다.
        #    게다가 nearest_free 가 로봇 반대편 자리를 골라 우회가 생겼다.
        #    제대로 하려면 겨냥이 아니라 **DWA 장애물 집합에서 그 목표물을 빼야** 한다.
        want = target.position
        if self.goal is None or common.distance(*self.goal, *want) > 0.1:
            if self.goal is not None:
                _tally("APPROACH: 추정이 움직여 재계획")
            self.goal = want
            self._replan(pose)

        if self._since_replan >= config.MISSION_REPLAN_EVERY \
                or planner.path_is_blocked(self.path, self.plan_grid):
            self._replan(pose)

        if not self.path:
            target.visited = True          # 갈 수 없는 목표물
            self.status = "목표물까지 길이 없다 — 건너뛴다"
            self._leave_approach(pose)
            return 0.0, 0.0

        speed, turn, status, self.path_index = follower.step(
            pose, self.path, ranges, self.path_index,
            current_speed=self.last_speed, current_turn=self.last_turn, dt=dt,
            allow_idle=self._person_is_close(pose), people=self._people + self._low_ghosts())

        # ⚠️ 주행기가 "경로 끝에 왔다" 고 하는데 목표물은 아직 멀다면, 계획기가
        #    더 가까이 데려다 줄 수 없다는 뜻이다 (목표물이 벽에 붙어 있으면
        #    팽창 때문에 이런 일이 생긴다). 그대로 서 있어 봐야 아무것도 안 바뀐다.
        #    한 번은 다시 계획해 보고 (지도가 그새 넓어졌을 수 있다),
        #    그래도 같으면 "갈 수 있는 데까지 갔다" 로 치고 넘어간다.
        #    이걸 안 했더니 0.58 m 앞에서 판정 기준 0.57 m 를 1 cm 차로 못 넘겨
        #    시간 초과까지 31 초를 서 있었다.
        if status == follower.ARRIVED:
            if not self._arrival_retried:
                self._arrival_retried = True
                self._replan(pose)
                return 0.0, 0.0
            target.visited = True
            self.status = (f"목표물 앞까지 갔다 ({gap:.2f} m) — "
                           f"계획기가 더 못 다가간다, 방문으로 친다")
            self._leave_approach(pose)
            return 0.0, 0.0

        # ⚠️ 목표물 직전에서 주저하는 것을 **숫자로** 센다. 추정 흔들림이 원인이라고
        #    봤는데 계측이 아니라고 했다 (재계획 0회). 그러면 남은 후보는 목표물이
        #    장애물이라 DWA 의 여유 판정에 걸려 기어가는 것이다.
        _tally("APPROACH 틱")
        if abs(speed) < 0.05:
            _tally("APPROACH 거의 멈춤")
        if gap < config.APPROACH_DISTANCE * 1.5:
            _tally("APPROACH 목표물 근처 틱")
            if abs(speed) < 0.05:
                _tally("APPROACH 목표물 근처에서 거의 멈춤")
        self.status = f"APPROACH — {status} ({gap:.2f} m 남음)"
        return speed, turn

    def _leave_approach(self, pose):
        self._clear_goal()
        self._decide_next(pose, exploring_possible=True)
