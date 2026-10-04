"""임무 전체를 굴리는 상태 머신.

받는 것: pose (x, y, theta), LiDAR 거리 배열, 경과시간 dt [s], (선택) 카메라 영상.
내놓는 것: (전진속도 v, 회전속도 omega). 지도·경로·목표물은 속성으로 꺼내 본다.
핵심 아이디어: "어디로 갈지" 는 여기서 정하고, "어떻게 갈지" 는 follower 가 한다.
상태: SCAN → EXPLORE ⇄ APPROACH → RETURN → DONE
순수 numpy — Webots 없이 pytest 로 돈다 (카메라는 안 주면 그냥 탐색만 한다).
"""

import math

import numpy as np

import common
import config
import detect
import exploration
import follower
import mapping
import people as people_mod
import planner
import scanmatch

SCAN = "SCAN"          # 시작하자마자 제자리에서 한 바퀴 — 주변 지도부터 만든다
EXPLORE = "EXPLORE"    # 프론티어를 찾아 돌아다니며 지도를 넓힌다
SWEEP = "SWEEP"        # 지도는 다 그렸는데 목표물이 모자랄 때, 몇 곳에서 둘러본다
APPROACH = "APPROACH"  # 찾은 목표물로 다가간다
RETURN = "RETURN"      # 시작 지점으로 돌아간다
DONE = "DONE"          # 끝. 그 자리에 선다


# ⚠️ 관찰용 계수기. 목표가 얼마나 자주, 왜 바뀌는지 모르면 "구불구불하다" 와
#    "왔던 곳을 재탐색한다" 의 원인을 짚을 수 없다. 실측(diag): 목표가 5초마다
#    바뀌는 동안 탐색 면적이 420초간 멈춰 있었다.
COUNT = {}


def _tally(key, n=1):
    COUNT[key] = COUNT.get(key, 0) + n



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


class Mission:
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
        self._tracker = people_mod.Tracker()   # 칼만 필터로 속도를 거른다 (PEOPLE_KALMAN)
        self.pose_fix = None        # 스캔 정합이 고쳐 준 위치 (없으면 None)
        self._field = None          # 스캔 정합용 거리장 (지도 갱신 때 다시 만든다)
        self._known = None          # 그때 "아는 칸" 마스크 (거리장과 함께 갱신)
        self._escaping_for = 0.0    # 탈출이 연속된 시간 [s]
        self._frozen_left = 0.0     # 바퀴를 멈춰 둘 남은 시간 [s]
        self._arrival_retried = False   # APPROACH 에서 도착 후 한 번 재계획했는가
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
            if (config.LOW_OBSTACLES_ENABLED and self.classify is not None
                    and self._camera_frames % config.YOLO_LOW_EVERY == 0):
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
        if config.SCANMATCH_ENABLED and self._ticks % config.SCANMATCH_EVERY == 0:
            if self._field is None:
                self._field = scanmatch.likelihood_field(self.grid)
                self._known = scanmatch.known_cells(self.grid)
            fixed, _ = scanmatch.match_fine(pose, ranges, self._field, known=self._known)
            self.pose_fix = fixed
            pose = fixed

        # 움직이는 것(사람)을 찾아 둔다. 계획기가 그 둘레를 비싸게 보고 경로를
        # 우회시킨다. 지도를 갱신한 "뒤" 에 해야 방금 본 벽이 사람으로 안 잡힌다.
        # (x, y, vx, vy) 목록. 계획기는 위치만, 주행기는 속도까지 쓴다.
        # ⚠️ 스위치 하나로 끈다 (config.PEOPLE_ENABLED). 목록이 비면 하류 전체가
        #    저절로 무동작이 된다 — 계획기의 사회적 비용, DWA 의 유령 점,
        #    "가만히 있어도 된다" 허용까지.
        #    끈 근거: **1단계 월드에는 사람이 아예 없는데** 검출이 0.28/틱,
        #    헛것 비율 100% 였다 (maze0: 헛것 3237회). 후보의 26배가 "다리 모양"
        #    에서 나오는데 벽 끝·문틈·상자 모서리가 그 규칙에 가장 잘 맞고,
        #    "움직였나" 검사도 못 걸러낸다 — 로봇이 움직이면 고정물의 보이는
        #    끝점이 벽을 따라 미끄러져 세계 좌표에서 진짜로 이동한다.
        #    그 잡음이 계획기에 사회적 비용을 얹고 있었다.
        self._people = (self._watcher.see(pose, ranges, self.grid)
                        if config.PEOPLE_ENABLED else [])
        if config.PEOPLE_ENABLED and config.PEOPLE_KALMAN:
            self._people = self._tracker.update(self._people, dt)
        # ⚠️ 계획기에는 **확신이 서는 것만** 넘긴다 (config.PEOPLE_PLANNER_SCANS).
        #    DWA 에는 전부 넘긴다 — 국소 회피는 틀려도 싸다.
        self._people_xy = [(p[0], p[1]) for p in self._people
                           if len(p) < 5 or p[4] >= config.PEOPLE_PLANNER_SCANS]

        if self.state == DONE:
            self.status = "DONE — 더 볼 곳이 없다"
            return 0.0, 0.0

        if self.state == SCAN:
            return self._scan(pose, dt)
        # 시간이 얼마 안 남았으면 찾은 것만 들고 돌아간다.
        # ⚠️ 이 검사를 _decide_next() 안에 뒀다가 **한 번도 발동하지 않았다.**
        #    그 함수는 "다음에 무엇을 할지 정할 때" 만 불린다. 로봇이 EXPLORE 로
        #    한 목표를 붙들고 있으면 거기를 아예 안 거친다 — comb0 은 840초까지
        #    EXPLORE 였고 675초 예산을 한참 넘겼는데도 복귀하지 않았다.
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

    def _scan(self, pose, dt):
        """제자리에서 한 바퀴 돌고 EXPLORE 로 넘어간다.

        ⚠️ 여기 원래 "주변 **지도** 를 만든다" 고 적혀 있었는데 **틀린 설명이다.**
           우리 LiDAR 는 360° 라 매 틱 사방을 이미 본다 — 제자리 회전으로 새로
           그려지는 지도는 없다. (사용자가 "LiDAR 360도라며?" 하고 짚어 줬다.)

        그래도 남기는 이유: 껐더니 나빠졌다. 지도 4종에서 재 봤다 —
            켬 1.15바퀴 : 207.0 m,  목표물 12/12
            끔 0바퀴    : 279.5 m,  목표물 11/12  (comb0 에서 하나 잃었다)
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
        #    docs/무엇을-빼기로-했나.md 참고.
        return None

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
        #    실측(maze0): 40초에 남동 목표물 옆을 지나 북동으로 먼저 가고,
        #    100~120초에 남동으로 되돌아왔다. 그 목표물은 결국 127회로 확정됐다
        #    — 나중에 일부러 갔을 때다. (GUI 로 사용자가 짚어 줬다:
        #    "분명히 카메라에서 목표물을 보았음에도 패스함")
        #    이 장치는 _decide_next 에도 있지만 거기는 **포기하기 직전** 이라
        #    탐색이 남아 있는 동안에는 열리지 않는다 — 자리가 문제였다.
        #
        # ⚠️ 한때 "들렀다 가는 것이 **오히려 짧을 때**" 만 끼어들었다
        #    (to_target + APPROACH_DISTANCE < to_goal). 그 조건을 **없앴다.**
        #    목표물과 프론티어를 같은 자격으로 비교한 것이 잘못이다:
        #      목표물은 **확실한 점수**, 프론티어는 **추측** 이다.
        #    그리고 언젠가는 그 목표물에 가야 하므로, 지금 지나치면 왕복이 추가된다.
        #    실측(maze0): 40초에 남동 목표물 옆을 지나 북동으로 먼저 가고
        #    100~120초에 되돌아왔다. 그 목표물은 나중에 일부러 갔을 때 127회로
        #    확정됐다. (사용자가 GUI 에서 여러 번 짚었다: "분명히 카메라에서
        #    목표물을 보았음에도 패스함", "검출했으면 그 쪽으로 가는 게 맞지 않아?")
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
        if self._needs_new_goal(pose):
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
            #    죽은 코드였다. docs/무엇을-빼기로-했나.md 참고.
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
            if self.goal is not None and \
                    common.distance(*pose[:2], *self.goal) > config.FOLLOW_GOAL_TOLERANCE:
                self._fail_goal("경로는 끝났는데 목표에 못 닿았다")
            elif self.goal is not None and \
                    exploration.frontier_near(self.plan_grid, *self.goal):
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
           실측(comb1): 뱀길을 돌아 6 m 를 가야 하는 목표가 45초에 걸려 매번
           절반쯤에서 버려졌고, 블랙리스트가 리셋될 때마다 같은 일이 반복됐다
           (80·260·440초에 같은 서쪽 목표). 탐색률 51% 로 끝났다.
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
        if target is None and self._verify_tried:
            # 확인하러 온 경우다. 확정된 것이 없으면 그 후보를 목표로 삼는다.
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
        #    게다가 nearest_free 가 로봇 반대편 자리를 골라 우회가 생겼다
        #    (16월드: 484.5 -> 541.2 m, 3570 -> 4118초, 47/48 -> 46/48).
        #    제대로 하려면 겨냥이 아니라 **DWA 장애물 집합에서 그 목표물을 빼야**
        #    한다 (docs/무엇을-빼기로-했나.md).
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
        # ⚠️ 여기에 "카메라가 안 본 곳으로 간다" 는 마지막 수단이 있었다 (뺐다).
        #    rand2 하나를 고쳤지만 지금 벤치 16월드에는 그 근거가 없고, 층을
        #    하나 더 쌓는 값이었다. 필요하면 숫자를 붙여 다시 올린다.
        #    (제거 근거: docs/무엇을-빼기로-했나.md)

        # 확정은 못 했지만 "뭔가 봤다" 는 후보가 있으면, 포기하기 전에 가서 본다.
        # ⚠️ 확정 문턱(25회)에 못 미친 후보는 nearest_unvisited 에 안 잡혀서
        #    로봇이 그 정보를 쥐고도 복귀했다 (corridor: 후보 3, 확정 2, DONE).
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
        #    실측(comb0): 오른쪽 아래 구석 목표물을 보고도 기록이 없어 554초에
        #    2/3 으로 복귀했다 — 그 구석에 1.87 m 보다 가까이 간 적이 없다.
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
        # ⚠️ 목표물을 못 찾았어도 **돌아온다.** 과제는 "찾아가서 시작점으로 돌아오기" 다.
        #    예전에는 그 자리에서 DONE 이었다 — apartment 에서 61초에 탐색을 포기하고
        #    시작점에서 4.55 m 떨어진 곳에 선 채 끝났다 (9/30, 정밀 매칭 실행).
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
        self.state = SWEEP
        self.goal = spot
        self._goal_age = 0.0
        self._sweep_turned = 0.0
        self._sweep_theta = None
        # ⚠️ **여기서** 기록한다 (도착한 뒤가 아니다). 못 가는 자리를 기록하지
        #    않으면 다음 선택에서 또 같은 곳이 뽑혀 영원히 반복한다.
        self._sweep_points.append(spot)
        _tally("둘러보기 시작")
        self._replan(pose)
        self.status = f"둘러보러 간다 ({spot[0]:+.2f}, {spot[1]:+.2f})"
        return True

    def _pick_sweep_point(self, pose):
        """**카메라가 아직 안 본** 칸에 가장 가까이 설 수 있는 자리를 고른다.

        ⚠️ 예전 규칙은 "이미 둘러본 자리에서 **가장 먼** 빈 칸" 이었다. 그런데
           SWEEP 의 존재 이유는 "LiDAR 는 360°인데 카메라는 57°뿐이라, 지도를 다
           그려도 목표물 쪽을 한 번도 안 봤을 수 있다" 다. 그러면 골라야 할 곳은
           **카메라가 안 본 곳** 인데 옛 규칙은 거리만 최대화했다.
           그 결과 구조적으로 맵을 가로지르며 왕복했다 — 실측(comb0): 280~420초
           동안 (+2.68,-2.02) 과 (-2.52,-2.62) 사이를 오가며 탐색 면적이
           13,187 -> 13,283 칸(180초에 96칸)밖에 안 늘었다. 사용자가 GUI 에서
           "왔던 곳을 재탐색한다" 고 짚은 것이 이것이다.
           그리고 최소 버전이 open0 에서 목표물을 놓친 이유도 같다 —
           **갈 수 있는 카메라 미관측 칸 225개** 를 남기고 끝냈다.
        """
        todo = (mapping.is_free(self.plan_grid) & ~self.camera_seen
                & ~planner.inflate(self.plan_grid))
        rows, cols = np.nonzero(todo)
        if len(rows) == 0:
            return None
        xs, ys = common.to_world(rows, cols)

        # ⚠️ 이미 시도한 자리는 뺀다. 안 빼면 못 간 자리를 영원히 다시 고른다
        #    (옛 코드는 **360°를 다 돈 뒤에만** 기록해서, 못 간 자리가 영원히
        #     "가장 먼 곳" 으로 남았다 — comb0 이 같은 지점을 두 번 목표로 잡았다).
        #
        # ⚠️ 배제 반경은 **작아야** 한다. 이 목록의 유일한 용도는 "못 간 자리를 또
        #    고르지 않기" 다 — 실제로 도착해 한 바퀴 돌면 camera_seen 이 채워져
        #    그 구역은 todo 에서 저절로 빠지므로 넓게 금지할 이유가 없다.
        #    실측: 여기에 MISSION_SWEEP_SPACING(1.5 m)을 썼더니 몇 번 실패한 뒤
        #    남은 미관측 칸이 전부 금지 구역에 들어가 둘러보기가 조용히 끝났다
        #    (corridor3: **갈 수 있는 미관측 칸 238개** 를 남기고 종료, 3/3 -> 2/3).
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
                    self._clear_goal()
                    self._decide_next(pose, exploring_possible=False)
                    return 0.0, 0.0
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
        self._clear_goal()
        self._decide_next(pose, exploring_possible=False)
        return 0.0, 0.0

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
        self._retrace = []          # 지도 경로가 다시 생겼다. 되짚기는 버린다

        speed, turn, status, self.path_index = follower.step(
            pose, self.path, ranges, self.path_index,
            current_speed=self.last_speed, current_turn=self.last_turn, dt=dt,
            allow_idle=self._person_is_close(pose), people=self._people + self._low_ghosts())
        self.status = f"RETURN — {status} ({gap:.2f} m 남음)"
        return speed, turn

    # ------------------------------------------------------------------

    def _check_slip(self, pose, wheel_turn, dt):
        """바퀴 회전과 나침반 회전이 어긋나면 미끄러지는 중이다 (config 의 설명 참고)."""
        theta = pose[2]
        if self._slip_theta is None or wheel_turn is None or dt <= 0.0:
            self._slip_theta = theta
            return
        compass_turn = common.angle_diff(theta, self._slip_theta) / dt
        self._slip_theta = theta
        if not config.SLIP_ENABLED or self.state in (SCAN, DONE):
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
        if not config.LOW_OBSTACLES_ENABLED:
            return []
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

    def _why_no_goal(self, pose):
        """고를 후보가 없을 때, **그 순간** 의 사유를 찍는다 (한 번만).

        ⚠️ 끝난 뒤의 지도를 보고 원인을 추론하다 두 번 틀렸다. 로봇이 포기하는
           시점과 실행이 끝나는 시점의 지도가 다르기 때문이다 (시드 222 는 시작
           스캔 직후 16.8초에 포기하는데, 그때 지도는 3.5 m² 뿐이다).
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
            #    가로채고 방을 한 번 더 훑었다. 이득 없이 시간만 먹는다:
            #      open0  101 -> 354초,  open3  102 -> 332초,
            #      comb0  284초 완주 -> 820초에도 탐색 중 (상한에 걸림)
            #    정작 rand2 를 고친 것은 _decide_next 쪽(포기 직전) 검사다.
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
        if config.SLIP_CLEAR_BEFORE_GIVEUP and self.slip_spots:
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
           실측: corridor 지도에서 좌상단 주머니를 아예 안 가 보고 DONE 이 됐다
           (LiDAR 90%, 목표물 2/3, 못 찾은 것은 (-2.5,+2.5)).

           그래서 목록을 따로 계산하지 않고 **choose() 에게 직접 묻는다.**
           실제로 목표를 고르는 논리와 같은 답이 나와야 하기 때문이다.
        """
        return exploration.choose(self.plan_grid, pose[:2],
                                  blacklist=None) is not None

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
        #    실측(같은 시작점): maze 29.1 -> 45.5 m, open 21.0 -> 39.2 m.
        #    "못 가는 목표" 문제는 목표를 **고르는 쪽**(exploration.choose)에서
        #    exact=True 로 이미 막았다. 길을 짤 때까지 엄격할 이유가 없다.
        # ⚠️ "평소 여유로 안 되면 좁은 여유로 한 번 더". 연습 16월드로 기각했다가 대회형
        #    apartment 에서 되살렸다 (exploration.candidate_list 의 주석 참고) —
        #    후보 선택과 **같은 규칙** 이어야 한다.
        found = planner.plan(self.plan_grid, pose[:2], self.goal,
                             people=self._people_xy)
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

    # ------------------------------------------------------------------

    def _return_deadline(self, pose):
        """이 시각을 넘기면 복귀를 시작해야 한다 [s].

        ⚠️ 처음엔 "제한 시간의 75%" 라는 고정 비율이었다. 근거가 없었고,
           maze3 이 그 선에 걸려 목표물 3/3 -> 1/3 이 됐다.
           실제로 필요한 것은 **시작점까지 돌아갈 시간** 이므로 거기서 거꾸로 센다.
           시작점 근처면 거의 끝까지 탐색하고, 멀리 있으면 일찍 돌아선다.
        """
        if not config.MISSION_TIME_LIMIT:
            return math.inf
        home = (self.start_pose[:2] if self.start_pose
                else (config.START_X, config.START_Y))

        # ⚠️ 처음엔 **직선거리 / 최대속도** 로 쟀다. 두 가정이 다 틀렸다:
        #    ① 실제 경로는 직선보다 훨씬 길다 (빗살·복도를 돌아나가야 한다)
        #    ② 실제 평균 속도는 최대속도의 절반도 안 된다 (comb0: 느린 주행 78.5%)
        #    그래서 comb0 이 복귀를 **5.88 m 남기고** 900초에 끝났다.
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

    def _person_is_close(self, pose):
        """가까이에 사람이 있는가 — 그때만 "가만히 있어도 된다" 를 허용한다.

        ⚠️ 예전에는 `bool(self._people)` 였다. **유령 하나만 있어도 정지 벌점이
           통째로 꺼진다.** 검출은 완벽할 수 없는데 그 위에 중요한 안전장치를
           얹은 것이 잘못이었다.
           실측(comb0, 사람이 **없는** 월드): 유령 5,670회(0.10/틱)가 잡혔고,
           그 때문에 로봇이 640초부터 끝까지 한자리에 붙박여 복귀를 6.16 m
           남기고 실패했다.

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
