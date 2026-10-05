"""임무의 탐색 쪽 — 시작 스캔, 프론티어 탐색, 둘러보기(SWEEP), 다음 할 일 정하기.

mission.Mission 이 이 mixin 을 물려받는다. 상태(self.*)는 Mission.__init__ 이 만든다.
"""

import math

import cv2
import numpy as np

from . import common
from . import config
from . import exploration
from . import follower
from . import mapping
from . import planner
from .mission_state import APPROACH, COUNT, EXPLORE, RETURN, SCAN, SWEEP, _tally


class ExploreMixin:
    """SCAN · EXPLORE · SWEEP 상태와 목표 고르기."""

    # ------------------------------------------------------------------

    def _scan(self, pose, dt):
        """제자리에서 한 바퀴 돌고 EXPLORE 로 넘어간다.

        ⚠️ 여기 원래 "주변 **지도** 를 만든다" 고 적혀 있었는데 **틀린 설명이다.**
           우리 LiDAR 는 360° 라 매 틱 사방을 이미 본다 — 제자리 회전으로 새로
           그려지는 지도는 없다. (사용자가 "LiDAR 360도라며?" 하고 짚어 줬다.)

        그래도 남기는 이유: 껐더니 나빠졌다. 지도 4종에서 재 봤다 —
            켬 1.15바퀴 : 207.0 m,  목표물 12/12
            끔 0바퀴    : 279.5 m,  목표물 11/12
        왜 좋은지는 **확실하지 않다.** 짚이는 것은 카메라가 57° 밖에 못 본다는 점이다
           (LiDAR 는 360°). 시작 지점 주변을 카메라로 훑어 두면 목표물을 일찍 본다.
           다만 위 네 월드는 시작점 근처에 목표물이 없어 그 가설을 시험하지 못했다.
        비용은 2π×1.15/0.9 ≈ 8초로 작고, 빼면 나빠지므로 남긴다.
        """
        theta = pose[2]
        if self._last_theta is not None:
            self._scanned += abs(common.angle_diff(theta, self._last_theta))
        self._last_theta = theta

        target = 2.0 * math.pi * config.MISSION_SCAN_TURNS
        if self._scanned >= target:
            self.state = EXPLORE
            self.status = "시작 스캔 끝"
            return 0.0, 0.0

        self.status = f"시작 스캔 {self._scanned / target * 100 // 10 * 10:.0f}%"
        return 0.0, config.MISSION_SCAN_SPEED

    def _explore(self, pose, ranges, dt):
        # ⚠️ 시계는 탈출하거나 기다리는 동안에도 돌아야 한다 (APPROACH 와 같은 이유).
        #    안 그러면 벽에 눌렸다 빠져나오길 반복하는 내내 시간 초과가 안 걸린다.
        self._goal_age += dt

        escape = self._escape_if_pinned(ranges, dt)
        if escape is not None:
            return escape

        self._since_replan += dt

        # --- 가는 길에 "뭔가 봤다" 면 들러서 확인한다 --------------------
        # ⚠️ **확정된** 목표물은 이 자리에 올 수 없다 — _step 이 그보다 먼저
        #    무조건 APPROACH 로 보낸다 (mission.py 의 "EXPLORE 중이라도" 검사).
        #    문제는 **미확정** 후보다. 확정 문턱이 DETECT_MIN_SIGHTINGS(25)회라,
        #    지나가며 몇 번만 본 목표물은 임무 판단에 **아예 안 보인다.**
        #    실측: 목표물 옆을 지나 다른 곳에 먼저 갔다가 되돌아왔다 (GUI 에서 짚었다:
        #    "분명히 카메라에서 목표물을 보았음에도 패스함").
        #    이 장치는 _decide_next 에도 있지만 거기는 **포기하기 직전** 이라
        #    탐색이 남아 있는 동안에는 열리지 않는다 — 자리가 문제였다.
        #
        # ⚠️ 한때 "들렀다 가는 것이 **오히려 짧을 때**" 만 끼어들었다
        #    (to_target + APPROACH_DISTANCE < to_goal). 그 조건을 **없앴다.**
        #    목표물과 프론티어를 같은 자격으로 비교한 것이 잘못이다:
        #      목표물은 **확실한 점수**, 프론티어는 **추측** 이다.
        #    그리고 언젠가는 그 목표물에 가야 하므로, 지금 지나치면 왕복이 추가된다.
        #    (GUI 에서 짚었다: "검출했으면 그 쪽으로 가는 게 맞지 않아?")
        #    같은 후보로는 한 번만 끼어든다 — 그래야 EXPLORE 와 APPROACH 를
        #    왕복하며 카메라가 덜덜 떠는 일이 없다.
        if self.goal is not None:
            maybe = self.targets.best_unconfirmed()
            if maybe is not None and maybe.position not in self._interrupted_for:
                to_target = common.distance(*pose[:2], *maybe.position)
                self._interrupted_for.append(maybe.position)
                self.state = APPROACH
                self._goal_age = 0.0
                self._clear_goal()
                self.status = (f"본 것부터 확인한다"
                               f" ({maybe.x:+.2f}, {maybe.y:+.2f})"
                               f" — {maybe.sightings}회 봄, {to_target:.2f} m")
                return 0.0, 0.0

        # --- 목표를 새로 잡아야 하는가 --------------------------------
        if self._glance_heading is not None:
            return self._glance(pose)
        if self._needs_new_goal(pose):
            # 떠나기 전에, 바로 옆인데 카메라가 못 본 곳이 시야 밖이면 고개부터 돌린다.
            if self._start_glance(pose):
                return self._glance(pose)
            # 가까운데 카메라가 못 본 곳이 있으면 먼저 본다.
            if self._start_nearby_look(pose):
                return 0.0, 0.0
            if not self._pick_goal(pose):
                # 더 볼 곳이 없다.
                self._clear_goal()
                self._decide_next(pose, exploring_possible=False)
                return 0.0, 0.0
            if self.state == SCAN:             # 포기 대신 다시 스캔하기로 했다
                return 0.0, 0.0

        # --- 경로가 낡았으면 다시 계획 --------------------------------
        if (self._since_replan >= config.MISSION_REPLAN_EVERY
                or planner.path_is_blocked(self.path, self.plan_grid)):
            self._replan(pose)
            if not self.path:
                self._fail_goal("경로를 못 찾음")
                return 0.0, 0.0

        # --- 따라가기 -------------------------------------------------
        speed, turn, status, self.path_index = follower.step(
            pose, self.path, ranges, self.path_index,
            current_speed=self.last_speed, current_turn=self.last_turn, dt=dt,
            allow_idle=self._person_is_close(pose), people=self._people + self._low_ghosts())

        if status == follower.STOPPED:
            # 앞으로 갈 수 있는 후보가 없다 = 주행기가 목표 쪽으로 제자리 회전을
            # 내보낸 것이다. 그대로 따른다. 몸이 진짜로 눌리면 위쪽 is_pinned 가,
            # 그래도 안 풀리면 바퀴 멈춤이 잡는다.
            # ⚠️ 여기 "몇 번 막히면 기다렸다가 후진한다" 가 있었는데 지웠다.
            #    DWA 는 멈출 때 항상 회전 명령(0.52 rad/s 이상)을 같이 내보내므로
            #    그 분기의 조건(회전 0.15 이하)이 결코 참이 될 수 없었다 —
            #    죽은 코드였다. docs/측정_기록.md 참고.
            self.status = f"{self.state} — 정지, 방향 조정 중"
            return 0.0, turn

        if status == follower.ARRIVED:
            # ⚠️ "도착" 은 주행기가 **경로의 끝** 에 닿았다는 뜻이지, 목표에 갔다는
            #    뜻이 아니다. planner 는 목표 칸이 팽창으로 막혀 있으면 근처의 갈 수
            #    있는 칸으로 바꿔서 경로를 낸다 (planner.nearest_free). 그래서 목표가
            #    1 m 떨어져 있어도 "도착" 이 나온다.
            #
            #    이걸 성공으로 치면 목표만 지워지고 블랙리스트에 안 들어가서, 다음
            #    틱에 같은 목표를 또 고른다 — 속도 0 으로 영원히 도는 고리가 된다.
            #    실제로 사람 없는 월드에서 (+2.14,-1.88) 에 **800초** 를 서 있었다
            #    (목표 (+2.78,-2.62), 0.98 m 떨어짐). 900초 실패의 진짜 원인이었다.
            #
            #    벽에서 ROBOT_RADIUS + 여유(0.35 m) 안으로는 못 들어가므로 구석은
            #    영영 미지로 남고 프론티어를 계속 만든다. 즉 "못 가는 목표" 는
            #    예외가 아니라 항상 생긴다. 반드시 실패로 처리해 블랙리스트에 넣어야
            #    한다.
            # ⚠️ 경로 끝이 목표에 붙어 있으면(FOLLOW_GOAL_TOLERANCE 안) 경로 끝 도착이 곧 목표
            #    도착이다. 주행기는 경로 끝 0.2 m 안이면 "도착" 이라 하므로, 그 끝이 목표에서
            #    0.15 m 만 떨어져 있어도 로봇은 목표 0.35 m 앞에 선다 — 그걸 실패로 쳐서
            #    블랙리스트 횟수가 쌓였다 (2026-10-05 녹화 재생: 이 실패 11번 중 8번).
            #    끝이 멀리 바뀐 경우(위 800초 고리)는 그대로 실패다.
            end_next_to_goal = (self.goal is not None and self.path and common.distance(
                *self.path[-1], *self.goal) <= config.FOLLOW_GOAL_TOLERANCE)
            if self.goal is not None and not end_next_to_goal and \
                    common.distance(*pose[:2], *self.goal) > config.FOLLOW_GOAL_TOLERANCE:
                self._fail_goal("경로는 끝났는데 목표에 못 닿았다")
            elif self.goal is not None and \
                    exploration.frontier_near(self.plan_grid, *self.goal):
                # 좁은 문 바로 앞에서는 LiDAR 가 안쪽 모서리를 못 봐 경계가 남는다 — 실패로
                # 치기 전에 한 번만 조금 더 들어가 본다 (config.EXPLORE_PUSH_DISTANCE).
                if self._push_deeper(pose):
                    return 0.0, 0.0
                self._fail_goal("도착했는데 프론티어가 그대로다")
            else:
                self.status = f"목표 도착 ({self.goal[0]:+.2f}, {self.goal[1]:+.2f})"
                self._clear_goal()
            return 0.0, 0.0

        self._check_stuck(pose, speed, dt)
        self.status = f"{self.state} — {status}"
        return speed, turn

    # ------------------------------------------------------------------
    # 목표 고르기 / 실패 처리
    # ------------------------------------------------------------------

    def _goal_patience(self):
        """이 목표를 얼마나 기다려 줄 것인가 [s].

        ⚠️ 고정값이면 **거리와 무관** 해서, 가까운 목표에는 후하고 먼 목표에는
           가혹하다. 그리고 시간 초과는 즉시 블랙리스트에 올리므로 대가가 크다.
           실측: 뱀길을 돌아 6 m 를 가야 하는 목표가 45초에 걸려 매번 절반쯤에서
           버려졌고, 블랙리스트가 리셋될 때마다 같은 일이 반복됐다.
        복귀 마감과 같은 방식으로 **지금 들고 있는 경로 길이** 와 **실측 평균
        속도** 에서 유도한다. 경로가 없으면 하한을 쓴다.
        """
        floor = config.MISSION_GOAL_TIMEOUT
        if not self.path:
            return floor
        metres = sum(common.distance(*a, *b)
                     for a, b in zip(self.path, self.path[1:]))
        speed = config.FOLLOW_MAX_SPEED
        if self.elapsed > 1.0 and self._travelled > 0.0:
            speed = max(self._travelled / self.elapsed, speed * 0.25)
        return max(floor, metres / speed * config.MISSION_GOAL_TIMEOUT_SLACK)

    def _needs_new_goal(self, pose):
        _tally("틱")
        if self.goal is None:
            _tally("교체: 목표가 없었다")
            return True
        if self._goal_age > self._goal_patience():
            _tally("교체: 시간 초과")
            # 시간 초과는 곧바로 제외한다 (위 record_failure 설명 참고).
            self._fail_goal("시간 초과", immediate=True)
            return True
        if not common.in_bounds(*common.to_cell(*self.goal)):
            _tally("교체: 지도 밖")
            self._clear_goal()
            return True
        # 가는 동안 LiDAR 가 그 근처를 다 봐 버렸을 수 있다.
        # 그러면 굳이 거기까지 갈 이유가 없다 — 실패가 아니라 "목적 달성" 이다.
        # 이 확인이 없으면 벽 앞까지 기어가서 멈췄다 기다렸다를 무한 반복한다.
        if not exploration.frontier_near(self.plan_grid, *self.goal):
            _tally("교체: 주변을 이미 다 봤다")
            self.status = "목표 주변을 이미 다 봤다 — 다음 목표로"
            self._clear_goal()
            return True
        return False

    def _decide_next(self, pose, exploring_possible):
        """다음에 무엇을 할지 한 곳에서 정한다.

        ⚠️ 이 판단이 여러 곳에 흩어져 있으면 반드시 하나가 어긋난다.
           실제로 SWEEP 이 세 번째 목표물을 찾고도 방문하지 않고 복귀해 버렸다.

        우선순위:
          1. 안 가 본 목표물이 있으면 → 간다
          2. 목표물을 다 찾아 다 가 봤으면 → 복귀 (더 탐색할 이유가 없다)
          3. 탐색할 곳이 남았으면 → 탐색
          4. 목표물이 모자라고 더 둘러볼 곳이 있으면 → 둘러본다
          5. 찾은 것이 있으면 → 복귀, 없으면 → 끝

        ⚠️ 3 과 4 의 순서가 중요하다. 둘러보기(SWEEP)는 "지도를 다 그렸는데도
           목표물이 모자랄 때" 의 최후 수단이다. 순서를 반대로 뒀더니, 목표물
           하나를 방문하자마자 탐색을 그만두고 둘러보러 다녔다
           (전체 시간의 64% 를 SWEEP 에, 2.9% 만 EXPLORE 에 썼다).
        """
        if self.targets.nearest_unvisited(*pose[:2]) is not None:
            self.state = APPROACH
            self._goal_age = 0.0
            return
        if not self._targets_missing() and self.targets.confirmed:
            # 다 찾고 다 가 봤다. 지도를 더 그릴 이유가 없다.
            self.state = RETURN
            self.status = "목표물을 다 찾았다 — 시작 지점으로 복귀"
            return
        if exploring_possible:
            self.state = EXPLORE
            return
        # ⚠️ 여기에 "카메라가 안 본 곳으로 간다" 는 마지막 수단이 있었다 — 연습 월드에서
        #    근거가 없어 뺐다 (docs/측정_기록.md).

        # 확정은 못 했지만 "뭔가 봤다" 는 후보가 있으면, 포기하기 전에 가서 본다.
        # ⚠️ 확정 문턱(25회)에 못 미친 후보는 nearest_unvisited 에 안 잡혀서
        #    로봇이 그 정보를 쥐고도 복귀했다 (후보 3, 확정 2 에서 끝냈다).
        #    가까이 가면 카메라가 더 보게 되고, 진짜면 확정되어 APPROACH 로 넘어간다.
        #    가짜면 25회를 못 채우고, 아래 _verify_tried 가 같은 곳을 두 번 안 가게 막는다.
        if self._targets_missing():
            maybe = self.targets.best_unconfirmed()
            if maybe is not None and maybe.position not in self._verify_tried:
                self._verify_tried.append(maybe.position)
                self.state = APPROACH
                self._goal_age = 0.0
                self._clear_goal()
                self.status = (f"확정 못 한 후보를 확인하러 간다"
                               f" ({maybe.x:+.2f}, {maybe.y:+.2f})"
                               f" — {maybe.sightings}회 봄")
                return

        # 카메라로는 **봤는데** 거리를 못 잰 방향이 남았으면 그쪽으로 가 본다.
        # ⚠️ 이것이 없던 동안 그 관측은 그냥 버려졌다 (detect.LeadList 설명 참고).
        #    실측: 구석의 목표물을 보고도 기록이 없어 그 구석에 가 보지 않고 복귀했다.
        if self._targets_missing():
            lead = self.leads.best()
            if lead is not None:
                self.leads.give_up(lead)      # 쫓아 봤다고 먼저 기록한다
                spot = self.leads.look_point(lead)
                self.goal = spot
                self._goal_age = 0.0
                self._replan(pose)
                if not self.path:
                    # ⚠️ 카메라가 **그 방향으로 뭔가를 봤다** 는 것은 그쪽으로
                    #    시선이 뚫려 있다는 뜻이다 — 적어도 그만큼은 빈 공간이다.
                    #    그런데 그 구역이 지도에 아직 미탐색이면 계획기가 거부한다
                    #    (PLANNER_ALLOW_UNKNOWN = False, 그 기본값 자체는 옳다).
                    #    단서는 근거가 다르므로 **이 경우에만** 미탐색 통과를
                    #    허용한다. (사용자 지적: "검출했으면 그쪽으로 가는 게 맞지
                    #    않아? 길은 뚫려 있다는 것이잖아.")
                    self.path = planner.plan(self.plan_grid, pose[:2], spot,
                                             allow_unknown=True,
                                             people=self._people_xy) or []
                if self.path:
                    self.state = EXPLORE
                    self.status = (f"카메라가 본 방향으로 가 본다"
                                   f" ({spot[0]:+.2f}, {spot[1]:+.2f})"
                                   f" — {lead['sightings']}회 봄")
                    return
                self.goal = None

        if self._targets_missing() and self._start_sweep(pose):
            return
        # 갈 경계가 안 보이는데 집까지도 길이 없으면 "다 봤다" 가 아니라 "갇혔다" 다.
        # ⚠️ 대회 월드(2026-10-05): 좁은 문으로 들어간 방에서 그 문이 지도에 좁은 여유로도
        #    못 지나가게 칠해졌다. 방 밖이 전부 "갈 수 없음" 이 되어 422초에 탐색을 끝냈다
        #    (478초 남음, 사과 1/2). 빠져나오는 일은 복귀의 되짚기가 한다 — 길이 다시
        #    생기면 _return 이 탐색으로 돌려보낸다.
        if (self._targets_missing()
                and self._escape_resumes < config.EXPLORE_ESCAPE_RESUMES
                and not self._home_reachable(pose)):
            self._resume_after_escape = True
            self.state = RETURN
            self.status = "갇혔다 — 지나온 길로 빠져나간 뒤 다시 탐색한다"
            return
        # ⚠️ 목표물을 못 찾았어도 **돌아온다.** 과제는 "찾아가서 시작점으로 돌아오기" 다.
        #    예전에는 그 자리에서 DONE 이었다 — apartment 에서 61초에 탐색을 포기하고
        #    시작점에서 4.55 m 떨어진 곳에 선 채 끝났다.
        self.state = RETURN
        self.status = ("할 일을 마쳤다 — 시작 지점으로 복귀" if self.targets.confirmed
                       else "목표물을 못 찾았다 — 그래도 시작 지점으로 복귀")

    # ------------------------------------------------------------------
    # SWEEP — 지도는 다 그렸는데 목표물이 모자랄 때 몇 곳에서 둘러본다
    # ------------------------------------------------------------------
    # 왜 필요한가: LiDAR 는 360도를 보지만 카메라는 앞쪽 57도만 본다.
    # 지도를 다 그려도 목표물 쪽을 한 번도 안 봤을 수 있다 (실제로 3개 중 2개만 찾았다).

    def _targets_missing(self):
        return (self.target_count > 0
                and len(self.targets.confirmed) < self.target_count)

    def _start_sweep(self, pose):
        """둘러볼 자리를 정하고 SWEEP 으로 넘어간다. 더 볼 자리가 없으면 False."""
        if len(self._sweep_points) >= config.MISSION_SWEEP_POINTS:
            return False
        spot = self._pick_sweep_point(pose)
        if spot is None:
            return False
        # ⚠️ **여기서** 기록한다 (도착한 뒤가 아니다). 못 가는 자리를 기록하지
        #    않으면 다음 선택에서 또 같은 곳이 뽑혀 영원히 반복한다.
        self._sweep_points.append(spot)
        _tally("둘러보기 시작")
        self._begin_sweep(pose, spot)
        self.status = f"둘러보러 간다 ({spot[0]:+.2f}, {spot[1]:+.2f})"
        return True

    def _begin_sweep(self, pose, spot):
        self.state = SWEEP
        self.goal = spot
        self._goal_age = 0.0
        self._sweep_turned = 0.0
        self._sweep_theta = None
        self._replan(pose)

    def _start_glance(self, pose):
        """바로 옆(GLANCE_RADIUS)에 카메라가 못 본 빈칸이 시야 밖에 있으면 돌아볼 방향을 정한다."""
        if not self._targets_missing() or not self._camera_fov:
            return False
        if any(common.distance(pose[0], pose[1], *p) < config.GLANCE_SPACING
               for p in self._glance_points):
            return False
        k = common.to_cells(config.GLANCE_RADIUS)
        r0, c0 = common.to_cell(pose[0], pose[1])
        rows = slice(max(0, r0 - k), r0 + k + 1)
        cols = slice(max(0, c0 - k), c0 + k + 1)
        seen = self.camera_seen[rows, cols].astype(np.uint8)
        s = config.GLANCE_SEEN_SLACK
        seen = cv2.dilate(seen, np.ones((2 * s + 1, 2 * s + 1), np.uint8)) > 0
        todo = mapping.is_free(self.plan_grid[rows, cols]) & ~seen
        rr, cc = np.nonzero(todo)
        if len(rr) == 0:
            return False
        xs, ys = common.to_world(rr + rows.start, cc + cols.start)
        near = np.hypot(xs - pose[0], ys - pose[1]) <= config.GLANCE_RADIUS
        if near.sum() * common.cell_area() < config.GLANCE_MIN_AREA:
            return False
        heading = math.atan2(ys[near].mean() - pose[1], xs[near].mean() - pose[0])
        if abs(common.wrap_angle(heading - pose[2])) <= self._camera_fov / 2.0 * 0.6:
            return False                  # 이미 시야 안이다
        self._glance_points.append((pose[0], pose[1]))
        self._glance_heading = heading
        _tally("고개 돌리기")
        return True

    def _glance(self, pose):
        error = common.wrap_angle(self._glance_heading - pose[2])
        if abs(error) <= config.GLANCE_DONE_ANGLE:
            self._glance_heading = None
            self.status = "돌아봤다 — 탐색을 이어 간다"
            return 0.0, 0.0
        turn = max(-config.FOLLOW_MAX_TURN,
                   min(config.FOLLOW_MAX_TURN, config.FOLLOW_TURN_GAIN * error))
        self.status = "카메라가 못 본 옆자리를 돌아본다"
        return 0.0, turn

    def _start_nearby_look(self, pose):
        """탐색 중, 가까운 "카메라가 못 본 주머니" 를 떠나기 전에 둘러본다 (config.EXPLORE_LOOK_*).

        끝난 뒤 둘러보기(_start_sweep)와 같은 SWEEP 을 쓰되, 끝나면 탐색으로 돌아간다.
        """
        if not self._targets_missing() or len(self._look_points) >= config.EXPLORE_LOOK_MAX:
            return False
        spot = self._pick_look_point(pose)
        if spot is None:
            return False
        self._look_points.append(spot)      # 못 가는 자리를 또 고르지 않게 미리 적는다
        self._look_sweep = True
        _tally("탐색 중 둘러보기 시작")
        self._begin_sweep(pose, spot)
        self.status = f"카메라가 못 본 곳을 둘러보러 간다 ({spot[0]:+.2f}, {spot[1]:+.2f})"
        return True

    def _pick_look_point(self, pose):
        """가까운 큰 주머니(빈 칸 & 카메라 못 봄 & 경계에서 떨어짐)의 중심에 가장 가까운 설 자리."""
        todo = (mapping.is_free(self.plan_grid) & ~self.camera_seen
                & ~planner.inflate(self.plan_grid))
        frontier = exploration.frontier_mask(self.plan_grid)
        if frontier.any():
            k = common.to_cells(config.EXPLORE_LOOK_FRONTIER_CLEARANCE)
            disk = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * k + 1, 2 * k + 1))
            todo &= cv2.dilate(frontier.astype(np.uint8), disk) == 0
        count, labels, stats, centres = cv2.connectedComponentsWithStats(
            todo.astype(np.uint8), connectivity=8)
        min_cells = config.EXPLORE_LOOK_POCKET / common.cell_area()
        ban = config.FOLLOW_GOAL_TOLERANCE * 2.0
        best = None
        for k in range(1, count):
            if stats[k, cv2.CC_STAT_AREA] < min_cells:
                continue
            cx, cy = common.to_world(centres[k][1], centres[k][0])   # 중심은 (열, 행) 순서다
            far = common.distance(pose[0], pose[1], cx, cy)
            if far > config.EXPLORE_LOOK_DISTANCE:
                continue
            if any(common.distance(cx, cy, *p) <= ban for p in self._look_points):
                continue
            if best is None or far < best[0]:
                best = (far, k, cx, cy)
        if best is None:
            return None
        _, k, cx, cy = best
        rows, cols = np.nonzero(labels == k)
        xs, ys = common.to_world(rows, cols)
        for i in np.argsort(np.hypot(xs - cx, ys - cy))[:config.MISSION_SWEEP_TRIES]:
            spot = (float(xs[i]), float(ys[i]))
            if (common.distance(pose[0], pose[1], *spot) <= config.FOLLOW_GOAL_TOLERANCE
                    or planner.plan(self.plan_grid, pose[:2], spot, exact=True)):
                return spot
        return None

    def _pick_sweep_point(self, pose):
        """**카메라가 아직 안 본** 칸에 가장 가까이 설 수 있는 자리를 고른다.

        ⚠️ 예전 규칙은 "이미 둘러본 자리에서 **가장 먼** 빈 칸" 이었다. 그런데
           SWEEP 의 존재 이유는 "LiDAR 는 360°인데 카메라는 57°뿐이라, 지도를 다
           그려도 목표물 쪽을 한 번도 안 봤을 수 있다" 다. 그러면 골라야 할 곳은
           **카메라가 안 본 곳** 인데 옛 규칙은 거리만 최대화했다.
           그 결과 맵을 가로지르며 왕복했다 — 180초 동안 탐색 면적이 96칸밖에 안 늘었다
           (GUI 에서 "왔던 곳을 재탐색한다" 고 짚은 것이 이것이다).
        """
        todo = (mapping.is_free(self.plan_grid) & ~self.camera_seen
                & ~planner.inflate(self.plan_grid))
        rows, cols = np.nonzero(todo)
        if len(rows) == 0:
            return None
        xs, ys = common.to_world(rows, cols)

        # ⚠️ 이미 시도한 자리는 뺀다. 안 빼면 못 간 자리를 영원히 다시 고른다
        #    (옛 코드는 **360°를 다 돈 뒤에만** 기록해서, 못 간 자리가 영원히
        #     "가장 먼 곳" 으로 남아 같은 지점을 두 번 목표로 잡았다).
        #
        # ⚠️ 배제 반경은 **작아야** 한다. 이 목록의 유일한 용도는 "못 간 자리를 또
        #    고르지 않기" 다 — 실제로 도착해 한 바퀴 돌면 camera_seen 이 채워져
        #    그 구역은 todo 에서 저절로 빠지므로 넓게 금지할 이유가 없다.
        #    실측: 여기에 MISSION_SWEEP_SPACING(1.5 m)을 썼더니 몇 번 실패한 뒤
        #    남은 미관측 칸이 전부 금지 구역에 들어가 둘러보기가 조용히 끝났다
        #    (갈 수 있는 미관측 칸 238개를 남기고 목표물 하나를 놓쳤다).
        #    도착 판정이 FOLLOW_GOAL_TOLERANCE 이므로 그보다 조금 크면 충분하다.
        ban = config.FOLLOW_GOAL_TOLERANCE * 2.0
        for sx, sy in self._sweep_points:
            keep = np.hypot(xs - sx, ys - sy) > ban
            xs, ys = xs[keep], ys[keep]
            if len(xs) == 0:
                return None

        # 가까운 것부터 보되, **갈 수 있는** 자리만 고른다.
        # ⚠️ 탐색 쪽(exploration.candidate_list)은 이미 A*(exact=True)로 도달성을
        #    확인하는데 둘러보기는 안 하고 있었다. 그래서 못 가는 자리를 고른 뒤
        #    "못 가겠으면 그 자리에서 둘러본다" 로 빠져 **엉뚱한 자리에서 360°를
        #    돌았다.** 남은 미관측 구역이 갈 수 없는 주머니면 그 짓을 무한히
        #    반복한다 (사용자: "무제한 동글동글 회전했어").
        #    같은 규칙을 쓰면 그 갈래가 아예 필요 없어진다.
        order = np.argsort(np.hypot(xs - pose[0], ys - pose[1]))
        for k in order[:config.MISSION_SWEEP_TRIES]:
            spot = (float(xs[k]), float(ys[k]))
            if planner.plan(self.plan_grid, pose[:2], spot, exact=True):
                return spot
        return None

    def _sweep_step(self, pose, ranges, dt):
        escape = self._escape_if_pinned(ranges, dt)
        if escape is not None:
            return escape

        # 가는 중이면 먼저 도착한다
        if self.goal is not None:
            self._goal_age += dt
            gap = common.distance(pose[0], pose[1], *self.goal)
            timed_out = self._goal_age > config.APPROACH_TIMEOUT
            if gap > config.FOLLOW_GOAL_TOLERANCE and not timed_out:
                if self._since_replan >= config.MISSION_REPLAN_EVERY \
                        or planner.path_is_blocked(self.path, self.plan_grid):
                    self._replan(pose)
                if not self.path:
                    # ⚠️ 여기서 "그 자리에서 둘러본다" 를 하고 있었다 (뺐다).
                    #    도착하지도 않은 자리에서 360°를 돌고, 그걸 "둘러봤다" 고
                    #    기록했다 — 두 번 틀린다. 이제는 자리를 고를 때 도달성을
                    #    확인하므로 여기 올 일이 드물고, 오면 다음 자리를 고른다.
                    return self._end_sweep(pose)
                self._since_replan += dt
                speed, turn, status, self.path_index = follower.step(
                    pose, self.path, ranges, self.path_index,
                    current_speed=self.last_speed, current_turn=self.last_turn,
                    dt=dt)
                # ⚠️ 주행기가 "도착" 이라고 하면 그 말을 믿어야 한다.
                #    계획기는 벽 때문에 목표보다 조금 앞에서 멈추게 데려다 줄 수 있다
                #    (0.27 m 남았는데 판정 기준은 0.20 m). 그 말을 무시했더니
                #    로봇이 v=0, w=0 으로 17 초를 그대로 서 있었다.
                if status != follower.ARRIVED:
                    self.status = f"둘러보러 가는 중 ({gap:.2f} m 남음)"
                    return speed, turn
            self.goal = None
            self._clear_path()

        # 도착했다. 제자리에서 한 바퀴 돈다.
        theta = pose[2]
        if self._sweep_theta is not None:
            self._sweep_turned += abs(common.angle_diff(theta, self._sweep_theta))
        self._sweep_theta = theta

        if self._sweep_turned < 2.0 * math.pi:
            _tally("둘러보며 도는 틱")
            done = self._sweep_turned / (2.0 * math.pi) * 100
            self.status = f"둘러보는 중 {done:.0f}%  (목표물 {len(self.targets.confirmed)} 개)"
            return 0.0, config.MISSION_SCAN_SPEED

        # 한 바퀴 다 돌았다. 둘러보다 찾은 목표물이 있으면 그쪽이 먼저다.
        # (자리는 _start_sweep 에서 이미 기록했다 — 여기서 또 넣으면 실제로 선
        #  자리까지 금지 구역이 되어 다음 후보가 과하게 줄어든다.)
        return self._end_sweep(pose)

    def _end_sweep(self, pose):
        # 탐색 중 둘러보기였으면 탐색으로 돌아간다 (끝난 뒤 둘러보기는 다음 할 일을 정한다).
        exploring = self._look_sweep
        self._look_sweep = False
        self._clear_goal()
        self._decide_next(pose, exploring_possible=exploring)
        return 0.0, 0.0

    def _why_no_goal(self, pose):
        """고를 후보가 없을 때, **그 순간** 의 사유를 찍는다 (한 번만).

        ⚠️ 끝난 뒤의 지도를 보고 원인을 추론하다 두 번 틀렸다. 로봇이 포기하는
           시점과 실행이 끝나는 시점의 지도가 다르기 때문이다 (시작 스캔 직후에
           포기한 실행은 그때 지도가 3.5 m² 뿐이었다).
           원인은 **결정하는 순간** 에 남겨야 한다.
        """
        if COUNT.get("포기 사유를 찍었다"):
            return
        _tally("포기 사유를 찍었다")
        blocked = (planner.inflate(self.plan_grid, config.PLANNER_INFLATION_MARGIN)
                   | mapping.is_unknown(self.plan_grid))
        lumps = exploration.cluster(exploration.frontier_mask(self.plan_grid))
        lines = [f"[포기] {self.elapsed:.1f}초, 빈칸 "
                 f"{int(mapping.is_free(self.plan_grid).sum())}, 덩어리 {len(lumps)}개"]
        for row, col, size in lumps:
            cell = (row, col)
            spot = common.to_world(row, col)
            if blocked[cell]:
                near = planner.nearest_free(blocked, cell)
                if near is None:
                    lines.append(f"  {size:3d}칸 @({spot[0]:+.2f},{spot[1]:+.2f})"
                                 f" -> 설 자리 없음")
                    continue
                cell = near
            x, y = common.to_world(*cell)
            moved = common.distance(x, y, *spot)
            gap = common.distance(x, y, *pose[:2])
            ok = bool(planner.plan(self.plan_grid, pose[:2], (x, y), exact=True))
            tight = bool(planner.plan(self.plan_grid, pose[:2], (x, y), exact=True,
                                      margin=config.PLANNER_SQUEEZE_MARGIN))
            banned = (self.blacklist is not None
                      and self.blacklist.contains(x, y))
            lines.append(
                f"  {size:3d}칸 @({spot[0]:+.2f},{spot[1]:+.2f})"
                f" -> 설자리({x:+.2f},{y:+.2f}) 이동{moved:.2f}m"
                f"{' [신선도초과]' if moved > config.FRONTIER_STALE_RADIUS else ''}"
                f" 로봇과{gap:.2f}m"
                f"{' [너무가까움]' if gap < exploration._worth_driving_to() else ''}"
                f" 평소={ok} 좁게={tight}"
                f"{' [블랙리스트]' if banned else ''}")
        print("\n".join(lines), flush=True)

    def _push_deeper(self, pose):
        """경계 목표에 왔는데 경계가 그대로면, 같은 방향으로 아는 빈칸을 따라 조금 더 들어간다.

        목표 하나에 한 번만 한다 (_pushed_from). 옮길 자리가 없거나 길이 없으면 False.
        ⚠️ 대회 월드(2026-10-05, 2/2 실행): 110초에 화장실 문 앞 경계에 도착했지만 경계가
           남아 떠났고, 화장실 사과는 558초 뒤에야 찾았다.
        """
        if self._pushed_from is not None:
            return False
        gx, gy = self.goal
        dx, dy = gx - pose[0], gy - pose[1]
        norm = math.hypot(dx, dy)
        if norm < 1e-6:
            dx, dy, norm = math.cos(pose[2]), math.sin(pose[2]), 1.0
        ux, uy = dx / norm, dy / norm
        free = mapping.is_free(self.plan_grid)
        blocked = planner.inflate(self.plan_grid, config.PLANNER_SQUEEZE_MARGIN)
        deeper = None
        step = common.cells_to_metres(1)       # 한 칸씩 더 들어가 본다
        s = step
        while s <= config.EXPLORE_PUSH_DISTANCE + 1e-9:
            spot = (gx + ux * s, gy + uy * s)
            cell = common.to_cell(*spot)
            if not common.in_bounds(*cell) or not free[cell] or blocked[cell]:
                break
            deeper = spot
            s += step
        if deeper is None or common.distance(*deeper, gx, gy) < config.FOLLOW_GOAL_TOLERANCE:
            return False
        self._pushed_from = self.goal
        self.goal = deeper
        self._goal_age = 0.0
        self._replan(pose)
        if not self.path:
            self.goal = self._pushed_from
            return False
        _tally("경계가 그대로라 조금 더 들어감")
        self.status = f"경계가 그대로다 — 조금 더 들어가 본다 ({deeper[0]:+.2f}, {deeper[1]:+.2f})"
        return True

    def _pick_goal(self, pose):
        """다음 프론티어를 고르고 경로를 세운다. 성공하면 True.

        "고를 게 없다" 와 "남았는데 전부 블랙리스트다" 를 구분한다.
        후자라면 블랙리스트를 한 번 비우고 다시 해 본다 —
        사람이 잠깐 막고 있었을 뿐일 수도 있기 때문이다.
        """
        for _ in range(8):      # A* 가 실패하면 다음 후보로 몇 번 더 시도
            # 갈 수 있는 프론티어 중 경로가 가장 짧은 곳. 규칙은 이 하나다.
            goal = exploration.choose(self.plan_grid, pose[:2], self.blacklist)
            # ⚠️ 여기에도 "카메라가 안 본 곳" 검사를 뒀다가 **뺐다.** 프론티어가
            #    잠깐 바닥날 때마다 발동해서, 정상 탐색이 곧 찾아낼 목표물까지
            #    가로채고 방을 한 번 더 훑었다 — 이득 없이 시간만 세 배가 됐다.
            #    마지막 수단은 마지막에만 써야 한다.
            if goal is None:
                break
            self.goal = goal
            self._goal_age = 0.0
            self._replan(pose)
            if self.path:
                _tally("새 목표를 세웠다")
                return True
            self.blacklist.record_failure(*goal)   # A* 실패
            self.goal = None

        self._why_no_goal(pose)

        # 목표물을 아직 다 못 찾았으면 더 끈질기게 매달린다. 채점 기준 1번이
        # "제한 시간 내에 모든 구조 대상을 식별" 이라, 덜 찾고 일찍 복귀하는 것은
        # 가장 크게 잃는 길이다. 다 찾았다면 굳이 더 헤맬 이유가 없다.
        limit = config.MISSION_MAX_BLACKLIST_RESETS
        if self._targets_missing():
            limit = config.MISSION_MAX_BLACKLIST_RESETS_SEARCHING

        if self._frontiers_remain(pose) and self._blacklist_resets < limit:
            self._blacklist_resets += 1
            self.blacklist.clear()
            self.status = "블랙리스트를 비우고 다시 시도"
            return self._pick_goal(pose)

        # ⚠️ 미끄러진 자리 표시가 통로를 막아 탐색을 포기하는 일이 있었다 (카펫 서쪽 가장자리
        #    3곳 → 폭 1 m 띠, 307초에 미탐색 29곳 전부 '갈 수 없음'). 포기하기 전에 풀어 본다.
        if self.slip_spots:
            self.slip_spots = []
            self._refresh_plan_grid()
            self.blacklist.clear()
            self.status = "미끄러진 자리 표시를 풀고 다시 시도"
            return self._pick_goal(pose)


        return False

    def _frontiers_remain(self, pose):
        """블랙리스트를 빼고 봤을 때, 아직 **갈 수 있는** 곳이 남아 있는가.

        ⚠️ 예전에는 frontier_mask() 를 기본값으로 불러 덩어리가 있는지만 봤다.
           그때는 candidate_list 를 따로 불러 엄격한 판정만 썼는데, choose() 에는
           그런 경우를 위한 느슨한 폴백이 따로 있었다. 즉 "갈 데가 있는데 없다고
           판단" 하고 복귀해 버렸다. (지금은 두 갈래가 하나로 합쳐졌다.)
           실측: 구석 주머니를 아예 안 가 보고 끝냈다 (LiDAR 90%, 목표물 하나를 놓쳤다).

           그래서 목록을 따로 계산하지 않고 **choose() 에게 직접 묻는다.**
           실제로 목표를 고르는 논리와 같은 답이 나와야 하기 때문이다.
        """
        return exploration.choose(self.plan_grid, pose[:2],
                                  blacklist=None) is not None
