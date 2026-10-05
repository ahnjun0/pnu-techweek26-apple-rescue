"""임무 전체를 굴리는 상태 머신.

받는 것: pose (x, y, theta), LiDAR 거리 배열, 경과시간 dt [s], (선택) 카메라 영상.
내놓는 것: (전진속도 v, 회전속도 omega). 지도·경로·목표물은 속성으로 꺼내 본다.
핵심 아이디어: "어디로 갈지" 는 여기서 정하고, "어떻게 갈지" 는 follower 가 한다.
상태: SCAN → EXPLORE ⇄ APPROACH → (SWEEP) → RETURN → DONE
      (그 사이 어디서든 사람이 다가오면 → EVADE → 원래 상태)
상태별 동작은 mission_explore / mission_approach / mission_return / mission_evade 의 mixin 에 있고,
여기에는 매 틱의 흐름(step)·안전·경로 계획·표시용 접근자가 있다.
순수 numpy — Webots 없이 pytest 로 돈다 (카메라는 안 주면 그냥 탐색만 한다).
"""

import numpy as np

from . import common
from . import config
from . import detect
from . import exploration
from . import follower
from . import mapping
from . import people as people_mod
from . import planner
from . import scanmatch
from .mission_approach import ApproachMixin
from .mission_evade import EvadeMixin
from .mission_explore import ExploreMixin
from .mission_return import ReturnMixin
from .mission_return import retrace  # noqa: F401  (밖에서 mission.retrace 로 쓴다)
# 상태 이름과 계수기는 mission_state 에 있다. 밖에서는 mission.EXPLORE, mission.COUNT 로 쓴다.
from .mission_state import APPROACH, COUNT, DONE, EVADE, EXPLORE, RETURN, SCAN, SWEEP, _tally  # noqa: F401


class Mission(ExploreMixin, ApproachMixin, ReturnMixin, EvadeMixin):
    """지도·목표·경로를 들고 있는 상자. 매 틱 step() 을 부르면 된다."""

    def __init__(self, start_pose=None):
        self.grid = mapping.new_map()
        # 카메라가 이미 훑은 칸. 목표물은 카메라로만 보이므로, 탐색을 끝낼지
        # 판단할 때 LiDAR 지도만 보면 안 된다 (config 의 CAMERA_COVER_* 주석 참고).
        self.camera_seen = np.zeros_like(self.grid, dtype=bool)
        self._camera_fov = None
        self._travelled = 0.0
        self._home_gap = None
        self._home_gap_tick = 0
        self.state = SCAN
        self.blacklist = exploration.Blacklist()
        # ⚠️ 찾아야 할 개수를 알려 준다 — 그보다 많이 확정하지 않게 한다
        #    (헛것이 문턱을 넘어 4/3 이 된 적이 있다. detect.TargetList 참고).
        self.targets = detect.TargetList(limit=config.MISSION_TARGET_COUNT or None)
        # 카메라로 봤지만 거리를 못 잰 관측 — 방위만 아는 단서 (detect.LeadList).
        self.leads = detect.LeadList()
        self.classify = None        # YOLO (yolo_check.load()). 없으면 모양 검사만 한다
        self.low = detect.LowObstacles()   # LiDAR 에 안 보이는 낮은 물체 (config 참고)
        self._camera_frames = 0
        self._slip_for = 0.0        # 바퀴·나침반 회전이 어긋난 채 이어진 시간 [s]
        self._slip_theta = None
        self.slip_spots = []        # 미끄러진 자리 (계획용 지도에 벽으로 찍는다)
        self.slip_count = 0         # 미끄러짐 감지 횟수 (표시를 풀어도 센다)
        self.plan_grid = self.grid  # 계획용 지도 = LiDAR 지도 + 낮은 물체 (_refresh_plan_grid)
        self.target_count = config.MISSION_TARGET_COUNT
        # 둘러보기 (SWEEP)
        self._sweep_points = []       # 이미 둘러본 자리
        self._look_points = []        # 탐색 중 둘러본 자리 (config.EXPLORE_LOOK_*)
        self._look_sweep = False      # 지금 SWEEP 이 탐색 중 둘러보기인가 (끝나면 탐색으로 돌아간다)
        self._verify_tried = []     # 확인하러 가 본 미확정 후보 (두 번 안 간다)
        self._interrupted_for = []  # 같은 목표물로 두 번 끼어들지 않는다 (떨림 방지)
        self._sweep_turned = 0.0
        self._sweep_theta = None

        # 움직이는 것 감지
        self._last_pose = start_pose

        # 복귀
        self._return_age = 0.0        # 복귀를 시작한 지 [s]
        self._no_path_age = 0.0       # 복귀 경로를 못 찾은 지 [s]
        self._crumbs = []             # 지나온 길 (config.RETURN_CRUMB_SPACING 간격)
        self._retrace = []            # 지도에 길이 없을 때 되짚을 길
        self._retrace_index = 0
        self._resume_after_escape = False   # 갇혀서 시작한 복귀 — 빠져나오면 탐색을 잇는다
        self._escape_resumes = 0            # 그렇게 탐색을 이어 간 횟수
        self._stall_anchor = None           # 복귀 중 맴돎 감시: 기준 자리
        self._stall_age = 0.0               # 그 자리 반경 안에 머문 시간 [s]
        self._unstick = []                  # 지도 길은 있는데 막혔을 때 조금 되짚을 길
        self._unstick_index = 0

        self.goal = None            # 지금 가려는 월드 좌표 (x, y)
        self.path = []              # 웨이포인트 목록
        self.path_index = 0
        self.status = ""            # 화면에 띄울 한 줄

        self.start_pose = start_pose
        self.elapsed = 0.0          # 전체 경과 [s]
        self._goal_age = 0.0        # 지금 목표를 잡은 지 [s]
        self._pinned_ticks = 0      # 연속으로 '눌렸다' 가 나온 틱 수
        self._people = []           # 이번 틱에 보이는 움직이는 것들 (x,y,vx,vy)
        self._people_xy = []        # 그중 위치만 (계획기용)
        self._watcher = people_mod.Watcher()   # 여러 스캔에 걸쳐 본 것만 믿는다
        self._tracker = people_mod.Tracker()   # 칼만 필터로 속도를 거른다
        self._evade_resume = None   # 비키기가 끝나면 돌아갈 상태
        self._evade_left = 0.0      # 비키기에 남은 시간 [s]
        self._evade_calm = 0.0      # 위협이 연속으로 없었던 시간 [s]
        self.pose_fix = None        # 스캔 정합이 고쳐 준 위치 (없으면 None)
        self._field = None          # 스캔 정합용 거리장 (지도 갱신 때 다시 만든다)
        self._known = None          # 그때 "아는 칸" 마스크 (거리장과 함께 갱신)
        self._escaping_for = 0.0    # 탈출이 연속된 시간 [s]
        self._frozen_left = 0.0     # 바퀴를 멈춰 둘 남은 시간 [s]
        self._arrival_retried = False   # APPROACH 에서 도착 후 한 번 재계획했는가
        self._look_age = 0.0            # 확정 전 후보를 바라본 시간 [s]
        self._since_replan = 0.0
        self._stuck_time = 0.0
        self._stuck_anchor = None   # 끼임 판정을 위한 기준 위치
        self._backup_left = 0.0     # 남은 후진 시간 [s]
        self._scanned = 0.0         # 시작 스캔에서 지금까지 돈 각도 [rad]
        self._last_theta = None
        self._blacklist_resets = 0
        self.squeezing = False      # 지금 경로가 "좁은 여유" 로 짜인 것인가 (화면 표시용)
        # DWA 는 "지금 속도에서 한 틱에 갈 수 있는 범위" 를 보므로 직전 명령이 필요하다.
        self.last_speed = 0.0
        self.last_turn = 0.0
        self._ticks = 0

    # ------------------------------------------------------------------

    def step(self, pose, ranges, dt, image=None, camera_fov=None, wheel_turn=None):
        """한 틱. (v, omega) 를 돌려준다.

        image/camera_fov 를 주면 목표물도 찾는다. 안 주면 탐색만 한다
        (그래야 카메라 없이도 pytest 로 돌릴 수 있다).
        """
        if camera_fov:
            self._camera_fov = camera_fov
        if image is not None and camera_fov is not None:
            detect.scan(image, ranges, pose, camera_fov, self.targets,
                        self.leads, classify=self.classify, low_list=self.low)
            self._camera_frames += 1
            if self.classify is not None and self._camera_frames % config.YOLO_LOW_EVERY == 0:
                height, width = image.shape[:2]
                detect.yolo_low_obstacles(self.classify(image), pose, width, height,
                                          camera_fov, self.low)
        self._check_slip(pose, wheel_turn, dt)
        speed, turn = self._step(pose, ranges, dt)
        self.last_speed, self.last_turn = speed, turn
        return speed, turn

    def _step(self, pose, ranges, dt):
        self._ticks += 1
        if self._last_pose is not None:
            self._travelled += common.distance(*self._last_pose[:2], *pose[:2])
        self._last_pose = pose
        self.elapsed += dt
        if self.state != RETURN and (
                not self._crumbs or common.distance(
                    *self._crumbs[-1], *pose[:2]) >= config.RETURN_CRUMB_SPACING):
            self._crumbs.append((pose[0], pose[1]))
        if self.start_pose is None:
            self.start_pose = pose

        # 지도는 매 틱이 아니라 가끔만 갱신한다 (성능 규칙 5).
        # 단 첫 틱은 반드시 한다 — 아무것도 모르는 채로 판단하면 안 된다.
        if self._ticks == 1 or self._ticks % config.MAP_UPDATE_EVERY == 0:
            mapping.update(self.grid, pose, ranges)
            if self._camera_fov:
                mapping.mark_camera_seen(
                    self.camera_seen, self.plan_grid, pose, self._camera_fov,
                    common.to_cells(config.DETECT_MAX_RANGE))
            self._field = None          # 지도가 바뀌었으니 거리장도 다시 만든다
        self._refresh_plan_grid()

        # 스캔 정합 — 바퀴가 헛돌아도 벽은 제자리에 있다.
        # ⚠️ 고친 값은 여기서 쓰지 않고 바깥(컨트롤러)이 오도메트리에 되먹인다.
        #    그래야 다음 틱부터 그 자리에서 다시 누적된다.
        self.pose_fix = None
        if self._ticks % config.SCANMATCH_EVERY == 0:
            if self._field is None:
                self._field = scanmatch.likelihood_field(self.grid)
                self._known = scanmatch.known_cells(self.grid)
            fixed, _ = scanmatch.match_fine(pose, ranges, self._field, known=self._known)
            self.pose_fix = fixed
            pose = fixed

        # 움직이는 것(사람)을 찾아 칼만 추적기로 속도를 거른다. 계획기가 그 둘레를 비싸게 보고
        # 경로를 우회시킨다. 지도를 갱신한 "뒤" 에 해야 방금 본 벽이 사람으로 안 잡힌다.
        # (x, y, vx, vy) 목록. 계획기는 위치만, 주행기는 속도까지 쓴다.
        self._people = self._tracker.update(self._watcher.see(pose, ranges, self.grid), dt)
        # ⚠️ 계획기에는 **확신이 서는 것만** 넘긴다 (config.PEOPLE_PLANNER_SCANS).
        #    DWA 에는 전부 넘긴다 — 국소 회피는 틀려도 싸다.
        self._people_xy = [(p[0], p[1]) for p in self._people
                           if len(p) < 5 or p[4] >= config.PEOPLE_PLANNER_SCANS]

        if self.state == DONE:
            self.status = "DONE — 더 볼 곳이 없다"
            return 0.0, 0.0

        if self.state == SCAN:
            return self._scan(pose, dt)

        # 사람이 나를 향해 오고, 가만히 있으면 부딪힌다 → 하던 일을 멈추고 비킨다
        # (송지윤의 EVADE, config 의 '비키기' 참고).
        # ⚠️ 상태와 무관하게 **매 틱** 본다 (아래 시간 예산 검사와 같은 이유 — 특정 상태
        #    함수 안에 두면 그 상태일 때만 불린다).
        if self.state == EVADE:
            return self._evade(pose, ranges, dt)
        if (config.EVADE_ENABLED and self.state != DONE
                and people_mod.person_threat(pose, self._people) is not None):
            self._enter_evade()
            return self._evade(pose, ranges, dt)
        # 시간이 얼마 안 남았으면 찾은 것만 들고 돌아간다.
        # ⚠️ 이 검사를 _decide_next() 안에 뒀다가 **한 번도 발동하지 않았다.**
        #    그 함수는 "다음에 무엇을 할지 정할 때" 만 불린다. 로봇이 EXPLORE 로
        #    한 목표를 붙들고 있으면 거기를 아예 안 거친다 — 예산을 한참 넘겼는데도
        #    복귀하지 않았다.
        #    (choose_unseen 때와 같은 실수다: 장치가 아니라 **묻는 자리** 가 문제.)
        #    그러니 상태와 무관하게 **매 틱** 본다.
        # (찾은 것이 없어도 돌아온다 — 과제의 절반은 복귀다.)
        if (config.MISSION_TIME_LIMIT
                and self.state not in (RETURN, DONE)
                and self.elapsed > self._return_deadline(pose)):
            self.state = RETURN
            self._clear_goal()
            self.status = "시간 예산 초과 — 찾은 것만 들고 복귀"

        if self.state == RETURN:
            return self._return(pose, ranges, dt)
        if self.state == APPROACH:
            return self._approach(pose, ranges, dt)
        if self.state == SWEEP:
            return self._sweep_step(pose, ranges, dt)

        # EXPLORE 중이라도 안 가 본 목표물이 보이면 그쪽을 먼저 처리한다.
        if self.targets.nearest_unvisited(*pose[:2]) is not None:
            self._enter_approach(pose)
            return self._approach(pose, ranges, dt)

        return self._explore(pose, ranges, dt)

    # ------------------------------------------------------------------

    def _escape_if_pinned(self, ranges, dt):
        """몸이 눌려 있거나 무언가 다가오면 빠져나오는 명령을 돌려준다. 아니면 None.

        어느 상태에 있든 이게 최우선이다. 보행자가 로봇을 벽으로 밀어붙이는 일이
        실제로 일어나고, 그대로 두면 계속 갈리면서 오도메트리가 망가진다
        (물리 엔진이 로봇을 아레나 밖으로 튕겨 낸 적도 있다 — 오차 25 m).
        ⚠️ 처음엔 EXPLORE 에만 넣었는데, APPROACH 중에 끼여서 그대로 튕겨 나갔다.
        """
        # ⚠️ 한 틱만 믿으면 안 된다. LiDAR 가 이따금 한 틱(32 ms)만 튄다:
        #    앞뒤로 70 cm 인데 그 틱만 13 cm 를 찍는다. 그걸 "눌렸다" 로 받으면
        #    1 초를 통째로 후진하고 경로를 버린다. 실측: 한 자리에서 40초 동안
        #    15번 연속으로 이 일이 났다 (진짜 최근접 거리는 내내 70 cm 였다).
        #    진짜 눌림은 여러 틱에 걸쳐 천천히 가까워진다
        #    (24 -> 22 -> 21 -> 20 -> 19 cm). 그래서 두 틱 연속일 때만 믿는다.
        if follower.is_pinned(ranges):
            self._pinned_ticks += 1
        else:
            self._pinned_ticks = 0
        # ⚠️ 탈출이 끝없이 이어지면 바퀴가 헛돌고 있는 것이다. 그대로 두면
        #    엔코더만 쌓여 추정 위치가 폭주한다 (실측: 840초 매달려 108 m).
        #    바퀴를 멈춰 추정 위치를 지킨다. 사람은 정해진 길을 걸으니 지나간다.
        # ⚠️ 정해진 시간만 멈췄다가 무조건 다시 시도하면 안 된다. 상황이 그대로면
        #    또 헛돌고, 9초 중 6초를 헛도는 꼴이 된다 (실측: 추정 위치가 -12 m).
        #    "상황이 실제로 바뀔 때까지" 멈춰 있는다 — 바뀐 것은 LiDAR 로 안다.
        if self._frozen_left > 0.0:
            self._frozen_left -= dt
            self._escaping_for = 0.0
            self._backup_left = 0.0
            nearest = follower.nearest_obstacle(ranges)[0]
            if nearest > config.SAFETY_PINNED_DISTANCE + config.MISSION_FREEZE_CLEAR:
                self._frozen_left = 0.0     # 트였다. 곧바로 다시 움직인다
                self._clear_path()
            else:
                self.status = (f"{self.state} — 안 빠져나와진다, 바퀴를 멈추고 "
                               f"기다림 ({nearest * 100:.0f}cm)")
                return 0.0, 0.0

        if self._pinned_ticks >= config.SAFETY_PINNED_TICKS:
            self._backup_left = max(self._backup_left,
                                    config.MISSION_BACKUP_SECONDS)
        if self._backup_left > 0.0:
            self._backup_left -= dt
            self._escaping_for += dt
            if self._escaping_for > config.MISSION_ESCAPE_GIVEUP:
                self._frozen_left = config.MISSION_FREEZE_SECONDS
                self._escaping_for = 0.0
                self._backup_left = 0.0
                self.status = f"{self.state} — 안 빠져나와진다, 바퀴를 멈추고 기다림"
                return 0.0, 0.0
            self.status = f"{self.state} — 눌렸다, 트인 쪽으로 빠져나오는 중"
            if self._backup_left <= 0.0:
                self._escaping_for = 0.0
                self._clear_path()          # 빠져나왔으니 다시 계획한다
            return follower.escape_command(ranges)

        # 아직 안 눌렸는데 무언가 다가온다면 — 멈춰서 지나가길 기다린다.
        # ⚠️ 뒤로 물러나 봐야 사람(0.4 m/s)보다 느려서 못 피하고, 수시로 물러나면
        #    탐색이 진행되지 않는다 (넓게 잡았을 때 탐색이 37% 에서 멈췄다).
        #    사람이 길을 건너갈 때까지 잠깐 서 있는 것이 사람의 행동이기도 하다.
        # ⚠️ "다가오면 멈춰서 기다린다" 는 기능이 있었는데, 재 보고 뺐다.
        #    Webots 보행자는 운동학 물체라 멈춰 선 로봇을 그냥 밀고 지나간다.
        #    비키기 끔 169초 / 사람과 닿은 시간 2.7초
        #    비키기 0.6 m  446~900초 / 40~141초 (제자리에서 바퀴만 헛돌아
        #                  추정 위치가 108 m 까지 폭주한 적도 있다)
        #    비키기 1.4 m  244초 / 7.4초
        #    멈춰 서는 것이 이 시뮬레이터에서는 "밟히기" 와 같다.
        #    docs/측정_기록.md 참고.
        return None

    # ------------------------------------------------------------------

    def _check_slip(self, pose, wheel_turn, dt):
        """바퀴 회전과 나침반 회전이 어긋나면 미끄러지는 중이다 (config 의 설명 참고)."""
        theta = pose[2]
        if self._slip_theta is None or wheel_turn is None or dt <= 0.0:
            self._slip_theta = theta
            return
        compass_turn = common.angle_diff(theta, self._slip_theta) / dt
        self._slip_theta = theta
        if self.state in (SCAN, DONE):
            self._slip_for = 0.0
            return
        if abs(wheel_turn - compass_turn) > config.SLIP_TURN_DIFF:
            self._slip_for += dt
        else:
            self._slip_for = 0.0
        if self._slip_for >= config.SLIP_SECONDS:
            _tally("미끄러짐 감지")
            self._slip_for = 0.0
            if config.SLIP_MARK_RADIUS > 0.0:
                self.slip_spots.append(tuple(pose[:2]))
            self.slip_count += 1
            self._backup_left = max(self._backup_left, config.MISSION_BACKUP_SECONDS)
            if self.goal is not None and self.state != RETURN:
                self._fail_goal("미끄러짐")
            self._refresh_plan_grid()

    def _low_points(self):
        """계획에서 피할 낮은 물체: 카메라로 본 것 + 확정한 목표물 (사과도 LiDAR 에 안 보인다)."""
        return self.low.positions() + [t.position for t in self.targets.confirmed]

    def _refresh_plan_grid(self):
        """LiDAR 지도에 낮은 물체·미끄러진 자리를 벽으로 찍은 **사본**. LiDAR 지도는 그대로."""
        marks = ([(p, config.LOW_OBSTACLE_RADIUS) for p in self._low_points()]
                 + [(p, config.SLIP_MARK_RADIUS) for p in self.slip_spots])
        if not marks:
            self.plan_grid = self.grid
            return
        grid = self.grid.copy()
        for (x, y), radius in marks:
            span = common.to_cells(radius)
            row, col = common.to_cell(x, y)
            r0, r1 = max(0, row - span), min(grid.shape[0], row + span + 1)
            c0, c1 = max(0, col - span), min(grid.shape[1], col + span + 1)
            if r0 < r1 and c0 < c1:
                grid[r0:r1, c0:c1] = config.LOG_ODDS_MAX
        self.plan_grid = grid

    def _low_ghosts(self):
        """DWA 에 넘길 정지 장애물 (사람과 같은 형식 x, y, vx, vy, 확신)."""
        return [(x, y, 0.0, 0.0, 99) for x, y in self._low_points()]

    def _replan(self, pose):
        """경로를 다시 세운다. 넉넉한 여유로 먼저, 안 되면 좁게 한 번 더.

        평소에는 안전거리를 넉넉히 두고 다니다가, 정말 그 길밖에 없을 때만
        (좁은 문 같은 곳) 여유를 줄인다. "안전하게 먼저, 안 되면 좁게" 순서다.
        """
        self._since_replan = 0.0
        self.path_index = 0
        self.squeezing = False
        if not self.goal:
            self.path = []
            return

        # ⚠️ planner.plan 은 실패하면 None 을 돌려준다. self.path 는 언제나
        #    "리스트" 여야 한다 (len() 을 쓰는 곳이 있다). 반드시 [] 로 바꿔 둔다.
        #
        # ⚠️ 여기서는 exact=True 를 쓰지 **않는다.** 한 번 써 봤다가 크게 나빠졌다:
        #    목표가 정확히 설 수 있는 칸이 아니면 곧바로 좁은 여유로 넘어가는데,
        #    그러면 **경로 전체** 가 벽에 붙어 짜여 DWA 와 싸우게 된다.
        #    실측(같은 시작점): 주행 거리가 1.6~1.9배가 됐다.
        #    "못 가는 목표" 문제는 목표를 **고르는 쪽**(exploration.choose)에서
        #    exact=True 로 이미 막았다. 길을 짤 때까지 엄격할 이유가 없다.
        # ⚠️ "평소 여유로 안 되면 좁은 여유로 한 번 더" — 아파트 문 때문이다
        #    (exploration.candidate_list 의 주석 참고). 후보 선택과 **같은 규칙** 이어야 한다.
        found = planner.plan(self.plan_grid, pose[:2], self.goal,
                             people=self._people_xy)
        if found and common.distance(*found[-1], *self.goal) > config.FOLLOW_GOAL_TOLERANCE:
            # ⚠️ 목표 칸이 평소 여유로 막혀 plan() 이 근처 칸으로 바꿔 줬다. 그 길 끝에서
            #    도착하면 "경로는 끝났는데 목표에 못 닿았다" 로 실패한다 (2026-10-05 2/2 실행:
            #    14~15번). 좁은 여유로 진짜 목표까지 닿으면 **마지막 구간만** 좁게 잇는다 —
            #    경로 전체를 좁게 짜면 벽에 붙는다 (위 주석의 1.6~1.9배).
            tail = planner.plan(self.plan_grid, found[-1], self.goal,
                                margin=config.PLANNER_SQUEEZE_MARGIN,
                                people=self._people_xy, exact=True)
            if tail:
                found = found + tail[1:]
                self.squeezing = True
        if not found:
            found = planner.plan(self.plan_grid, pose[:2], self.goal,
                                 margin=config.PLANNER_SQUEEZE_MARGIN,
                                 people=self._people_xy)
            self.squeezing = bool(found)
        self.path = found or []

    def _fail_goal(self, reason, immediate=False):
        if self.goal is not None:
            banned = self.blacklist.record_failure(*self.goal, immediate=immediate)
            self.status = (f"목표 실패 ({reason})"
                           + (" → 블랙리스트" if banned else ""))
        self._clear_goal()

    def _clear_goal(self):
        self.goal = None
        self._clear_path()
        self._goal_age = 0.0

    def _clear_path(self):
        self.path = []
        self.path_index = 0
        self._since_replan = config.MISSION_REPLAN_EVERY   # 다음 틱에 곧바로 재계획

    # ------------------------------------------------------------------
    # 끼임 감지 — Phase 2 에서 겪은 "바퀴가 헛돌아 오도메트리 폭주" 를 막는다
    # ------------------------------------------------------------------

    def _check_stuck(self, pose, commanded_speed, dt):
        if abs(commanded_speed) < 1e-3:
            self._stuck_time = 0.0
            self._stuck_anchor = None
            return

        if self._stuck_anchor is None:
            self._stuck_anchor = pose[:2]
            self._stuck_time = 0.0
            return

        self._stuck_time += dt
        moved = common.distance(*self._stuck_anchor, *pose[:2])
        if moved > config.MISSION_STUCK_DISTANCE:
            self._stuck_anchor = pose[:2]
            self._stuck_time = 0.0
        elif self._stuck_time > config.MISSION_STUCK_SECONDS:
            # 가라고 했는데 안 움직인다 = 어딘가 걸렸다.
            self._backup_left = config.MISSION_BACKUP_SECONDS
            self._stuck_time = 0.0
            self._stuck_anchor = None
            self._fail_goal("끼임")

    def _person_is_close(self, pose):
        """가까이에 사람이 있는가 — 그때만 "가만히 있어도 된다" 를 허용한다.

        ⚠️ 예전에는 `bool(self._people)` 였다. **유령 하나만 있어도 정지 벌점이
           통째로 꺼진다.** 검출은 완벽할 수 없는데 그 위에 중요한 안전장치를
           얹은 것이 잘못이었다.
           실측(사람이 **없는** 월드): 유령이 틱당 0.10회 잡혔고, 그 때문에 로봇이
           640초부터 끝까지 한자리에 붙박여 복귀에 실패했다.

        멀리 있는 사람(이나 유령) 때문에 멈춰 설 이유는 없다. 가만히 있는 것이
        옳은 경우는 **사람이 내 앞에 있을 때** 뿐이다.
        """
        if not self._people:
            return False
        reach = config.PEOPLE_IDLE_RADIUS
        return any(common.distance(pose[0], pose[1], p[0], p[1]) <= reach
                   for p in self._people)

    def people_points(self):
        """이번 틱에 보이는 움직이는 것들 (화면 표시용). [(x, y), ...]"""
        return [(p[0], p[1]) for p in self._people]

    def frontier_points(self):
        """화면 표시용."""
        return exploration.frontier_points(self.plan_grid)

    def target_points(self):
        """화면 표시용 — 확인된 목표물 좌표 (발견한 것 전부)."""
        return self.targets.positions()

    def found_points(self):
        """**발견했지만 아직 안 간** 목표물. 화면에서 도달한 것과 구별한다.

        ⚠️ 대회 기준은 "식별하여 각 객체 위치까지 **이동**" 이므로 점수는 방문이다.
           화면에서 둘이 같아 보이면 "3개 찾았는데 2개만 갔다" 를 눈으로 알 수 없다.
        """
        return [t.position for t in self.targets.confirmed if not t.visited]

    def visited_points(self):
        """**도달까지 마친** 목표물 = 실제 점수."""
        return [t.position for t in self.targets.confirmed if t.visited]
