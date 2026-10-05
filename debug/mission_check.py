"""자율 탐색을 돌리면서 "잘 하고 있는가" 를 숫자로 재는 검증 스크립트. (Phase 4 검증)

받는 것: practice.wbt (컨트롤러를 mission_check 로 바꿔서 실행).
내놓는 것: 주기적인 진행 표, 끝나고 요약, 그리고 debug/out/mission.png
핵심 아이디어: sar_controller 와 똑같이 돌리되, Supervisor 로 진짜 위치를 곁눈질해서
              오도메트리가 얼마나 틀어졌는지와 벽에 얼마나 가까웠는지를 같이 잰다.
⚠️ 여기는 debug/ 다. 판단에는 정답 위치를 쓰지 않는다 — 재기만 한다 (CLAUDE.md 규칙 1).
"""

import math
import os
import sys

import matplotlib
# ⚠️ 여기서 무조건 "Agg" 로 못박으면 GUI 로 돌려도 지도 창이 안 뜬다 (Agg 는 창을 못 띄운다).
#    화면을 쓸 때는 기본 백엔드(macOS 면 macosx)를 그대로 두고,
#    헤드리스일 때만 Agg 로 내린다. 실제로 이것 때문에 창이 안 떴다.
if os.environ.get("SAR_VIZ", "1") == "0":
    matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from sar import common
from sar import config
from sar import localization
from sar import mapping
from sar import planner
from sar import follower
from sar import mission as mission_mod
from sar import people as people_mod
from sar import sensors as sensors_mod
from sar import viz as viz_mod
from debug import truth as truth_mod

# 사람의 몸 반경 [m] — Pedestrian.proto (R2025a) 에서 읽었다. 추측 아님.
#   다리 boundingObject: Capsule radius 0.075, 중심에서 y = ±0.116
#   (PROTO 400~406행 / 499~505행)  → 0.116 + 0.075 = 0.191
# LiDAR 가 있는 높이(z≈0.17 m)에서 실제로 부딪히는 것은 다리다.
# 카메라 프레임을 몇 틱마다 남길 것인가 (GIF·시각화용, 판단과 무관). SAR_FRAME_EVERY=0 이면 안 남긴다.
# ⚠️ 프레임은 실행 내내 **메모리** 에 쌓인다 (8틱마다면 한 번에 약 1.2 GB, 저장할 때 한 번 더 복사).
#    2026-10-05 에 셋을 동시에 돌렸다가 메모리·스왑·디스크가 차서 세션이 죽었다.
FRAME_EVERY = int(os.environ.get("SAR_FRAME_EVERY", "8"))
PERSON_RADIUS = 0.191
# 여러 실행을 동시에 돌릴 때 결과가 서로 덮어쓰지 않게 SAR_OUT 으로 바꿀 수 있다.
OUT_DIR = os.environ.get("SAR_OUT") or os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "out")
# ⚠️ 실제/이론 시간이 이 배수를 넘으면 "무언가 이상하다" 로 본다.
#    값의 근거는 아래 측정으로 정한다 (지금은 관찰용이다).
TIME_RATIO_SUSPECT = float(os.environ.get("SAR_TIME_RATIO", "3.0"))
REPORT_EVERY = float(os.environ.get("SAR_REPORT_EVERY", "20.0"))
#                       ^ [s] 이 주기로 한 줄 찍는다 (원인 추적 때 촘촘하게 본다)
TIME_LIMIT = 900.0      # [s] 이만큼 지나면 포기하고 요약한다
# (로봇의 MISSION_TIME_LIMIT 을 늘려 재는 경우에는 그보다 조금 더 기다린다 — main() 에서 맞춘다)
ARENA_AREA = 36.0       # [m²] RectangleArena floorSize 6 6
# "얼마나 빨리 탐색하나" 를 재려면 끝의 숫자 하나로는 모자란다. 진도가 필요하다.
MILESTONES = (0.50, 0.70, 0.80, 0.90, 0.95)
CAMERA_RAYS = 15        # 카메라 시야를 몇 갈래로 쪼개 볼 것인가 (계측용)
DETECT_MATCH_RADIUS = 0.5   # [m] 검출 위치가 이 안이면 그 물체를 맞힌 것으로 본다


def best_tour_on_map(grid, trail, targets):
    """로봇이 끝낸 지도 위에서 "시작 -> 목표물 셋 -> 시작" 최단 순회 길이 [m].

    왜 이 값이 필요한가: "달린 거리 / 최고속도" 를 기준선으로 쓰면 **헛걸음이
    기준선에도 같이 들어가** 이상이 숨는다. diag 는 탐색을 120초에 95% 끝내고도
    총 667초를 썼는데, 달린 거리로 보면 1.27배로 "조금 느린" 정도로 보였다.
    지도와 목표물 위치로만 정한 기준선이라야 헛걸음이 드러난다 (6.64배).

    ⚠️ 왜 **끝난 지도** 위에서 푸는가: 벽 기하를 모르는 월드(사람 시나리오)에서도
       재야 한다. 벽을 아는 월드에서 teacher.best_target_tour 와 대조하면 격자가
       벽을 부풀려 잡는 만큼 7% 큰 쪽으로 나온다 (maze0: 24.4 vs 22.79 m).
       즉 하한이 살짝 느슨해 시간 배수를 **작게** 만든다 — 이상을 과하게 신고하지
       않는 방향이라 하한으로 쓰기에 안전하다.

    ⚠️ 목표물 칸은 점유 상태이므로 그 자리에서 출발할 수 없고, plan() 은 막힌
       **목표** 만 근처로 바꿔 주고 출발 칸은 안 바꾼다. 그래서 목표물마다
       "설 자리" 를 먼저 구해야 한다. 그런데 그 자리는 **갈 수 있는 자리** 여야
       한다 — nearest_free 는 유클리드 거리로만 찾으므로 벽 반대편을 고른다.
       실측으로 세 번 틀렸다:
         ① 목표물 칸에서 바로 풀었다 → 16월드 중 6개가 값 없음
         ② 막힘 정의에 미지 영역을 빼먹었다 → 설 자리를 미지 공간에 잡았다
         ③ nearest_free 로 찾았다 → comb1·comb2 에서 벽 반대편을 골라 길이 없었다
       그래서 **시작점에서 물을 부어(flood fill) 갈 수 있는 칸을 먼저 구하고**,
       그 안에서 목표물에 가장 가까운 칸을 고른다.
    """
    import itertools
    from collections import deque

    # 막힘의 정의는 plan() 과 같아야 한다 (planner.py:280-282): 팽창 + 미지.
    blocked = planner.inflate(grid, None) | mapping.is_unknown(grid)
    height, width = blocked.shape

    # ⚠️ 시작 칸 자체가 막혀 있을 수 있다 — **로봇 몸 아래 칸은 LiDAR 가 한 번도
    #    보지 못하므로 미지로 남는다.** 그래서 시작점 주변의 트인 칸들로 씨를 뿌린다.
    start_cell = common.to_cell(*trail[0])
    seeds = []
    span = common.to_cells(0.5)
    for dr in range(-span, span + 1):
        for dc in range(-span, span + 1):
            r, c = start_cell[0] + dr, start_cell[1] + dc
            if common.in_bounds(r, c) and not blocked[r, c]:
                seeds.append((r, c))
    if not seeds:
        return None, ["시작점 주변에 갈 수 있는 칸이 없다"]

    # 갈 수 있는 칸 = 씨에서 이어진 칸 (A* 와 같이 8방향으로 본다)
    reach = np.zeros_like(blocked, dtype=bool)
    queue = deque(seeds)
    for cell in seeds:
        reach[cell] = True
    while queue:
        r, c = queue.popleft()
        for dr in (-1, 0, 1):
            for dc in (-1, 0, 1):
                nr, nc = r + dr, c + dc
                if (0 <= nr < height and 0 <= nc < width
                        and not reach[nr, nc] and not blocked[nr, nc]):
                    reach[nr, nc] = True
                    queue.append((nr, nc))

    rows, cols = np.nonzero(reach)
    xs, ys = common.to_world(rows, cols)

    def closest_reachable(xy):
        """갈 수 있는 칸 중 그 지점에 가장 가까운 자리와 그 거리."""
        gaps = np.hypot(xs - xy[0], ys - xy[1])
        k = int(np.argmin(gaps))
        return (float(xs[k]), float(ys[k])), float(gaps[k])

    spots = [closest_reachable(trail[0])[0]]
    names = ["시작"] + [f"목표물{i + 1}" for i in range(len(targets))]
    for t in targets:
        spot, gap = closest_reachable(t)
        # 목표물에 이만큼도 못 붙는다면 지도가 그 방을 통째로 못 본 것이다.
        if gap > config.APPROACH_DISTANCE * 2.0:
            print(f"  (참고) 목표물 ({t[0]:+.2f},{t[1]:+.2f}) 에 갈 수 있는 가장"
                  f" 가까운 자리가 {gap:.2f} m 나 떨어져 있다 — 하한이 느슨해진다")
        spots.append(spot)

    gap_of, broken = {}, []
    for i in range(len(spots)):
        for j in range(i + 1, len(spots)):
            path = planner.plan(grid, spots[i], spots[j])
            if not path:
                broken.append(f"{names[i]}~{names[j]}")
                gap_of[(i, j)] = gap_of[(j, i)] = math.inf
                continue
            d = sum(common.distance(*a, *b) for a, b in zip(path, path[1:]))
            gap_of[(i, j)] = gap_of[(j, i)] = d

    best = math.inf
    for order in itertools.permutations(range(1, len(spots))):
        route = (0,) + order + (0,)
        best = min(best, sum(gap_of[(route[k], route[k + 1])]
                             for k in range(len(route) - 1)))
    if math.isinf(best):
        return None, broken
    return best, broken


class _Tee:
    """print 를 화면과 파일에 동시에 쓴다 (요약이 Webots 종료에 날아가지 않게)."""

    def __init__(self, *streams):
        self.streams = streams

    def write(self, text):
        for stream in self.streams:
            stream.write(text)

    def flush(self):
        for stream in self.streams:
            stream.flush()


def save_tape(tape, compressed=True):
    """LiDAR 녹화를 남긴다. 중간 저장은 빨리 끝나게 압축하지 않는다."""
    if not tape["t"]:
        return
    saver = np.savez_compressed if compressed else np.savez
    saver(os.path.join(OUT_DIR, "tape.npz"),
          t=np.array(tape["t"], dtype=np.float32),
          pose=np.array(tape["pose"], dtype=np.float32),
          ranges=np.array(tape["ranges"], dtype=np.float32),
          person=np.array(tape["person"], dtype=np.float32),
          truth=np.array(tape["truth"], dtype=np.float32),
          cmd=np.array(tape["cmd"], dtype=np.float32))


def write_progress(brain, truth, pose, worst_drift, movers, low_objects, tick_seconds, note=""):
    """지금까지의 핵심 숫자를 progress.txt 에 **덮어쓴다** (중간에 꺼도 남게)."""
    start = brain.start_pose or (config.START_X, config.START_Y, 0.0)
    lines = [
        f"시각 {brain.elapsed:.1f} s  상태 {brain.state}  {brain.status}  {note}",
        f"위치 오차 지금 {common.distance(*truth, *pose[:2]) * 100:.1f} cm, 최대 {worst_drift * 100:.1f} cm",
        f"시작점까지 실제 거리 {common.distance(*truth, start[0], start[1]):.2f} m",
        f"확정 목표물 {len(brain.targets.confirmed)} 개, 방문 "
        f"{sum(1 for t in brain.targets.confirmed if t.visited)} 개: "
        + ", ".join(f"({t.x:+.2f},{t.y:+.2f}){' 방문' if t.visited else ''}"
                    for t in brain.targets.confirmed),
        f"밀린 낮은 물체 {sum(1 for o in low_objects if o['moved_at'] is not None)} / {len(low_objects)}",
        f"미끄러짐 감지 {getattr(brain, 'slip_count', 0)} 회",
        "사람과 닿은 시간 " + (", ".join(f"{m['name']} {m['touch'] * tick_seconds:.1f}초" for m in movers)
                             or "(움직이는 것 없음)"),
    ]
    with open(os.path.join(OUT_DIR, "progress.txt"), "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")


def main():
    # ⚠️ Supervisor 는 debug 전용이다 (CLAUDE.md 규칙 1). 여기서는 사람과의
    #    "진짜" 거리를 재는 데만 쓴다 — LiDAR 최근접은 LIDAR_MIN_RANGE(0.12 m)에
    #    붙어 버려서, 어떤 설정을 비교해도 늘 0.120 m 로 똑같이 나왔다.
    #    월드에 supervisor TRUE 가 없으면(실전 월드) 그냥 Robot 으로 돈다.
    from controller import Supervisor

    robot = Supervisor()
    # ⚠️ 처음엔 DEF 이름 하나("PEDESTRIAN")만 찾았다. stress_check.wbt 는 DEF 가
    #    PEDESTRIAN1 이라 노드를 못 찾았는데도 실행이 멀쩡히 끝나서, 아무것도 안 잰
    #    요약을 "충돌 0" 으로 읽을 뻔했다. 그 다음엔 DEF 이름 목록을 손으로 들고
    #    있었는데, 이번엔 **사람이 없는 것이 정상인 월드**(maze_check)에서도 같은
    #    ⚠️ 가 떠서 경고가 늑대 소년이 됐다. 이름 목록 관리가 원인이다.
    #
    #    그래서 이름을 버리고 장면 트리를 직접 훑는다. API 근거:
    #      Supervisor.getRoot/getSelf : lib/controller/python/controller/supervisor.py:26,29
    #      Field.getMFNode            : lib/controller/python/controller/node.py:352
    #      Node.getTypeName/getDef    : lib/controller/python/controller/node.py:110,86
    #    이제 "없는 것이 정상" 과 "있는데 못 찾았다" 가 다른 문구로 나온다.
    RADIUS_BY_DEF = {"SMALLMOVER": 0.07, "BIGMOVER": 0.15}
    DEFAULT_MOVER_RADIUS = 0.15     # 모르는 물체는 보수적으로 이 값으로 잰다

    movers = []
    if robot.getSupervisor():
        me = robot.getSelf()
        children = robot.getRoot().getField("children")
        for i in range(children.getCount()):
            node = children.getMFNode(i)
            if node is None or (me is not None and node.getId() == me.getId()):
                continue
            type_name = node.getTypeName()
            def_name = node.getDef() or type_name
            if type_name == "Pedestrian":
                radius = PERSON_RADIUS
            elif node.getBaseTypeName() == "Robot":
                # ⚠️ 얇은 사람 시나리오(debug/make_people_world.py)는 Pedestrian
                #    PROTO 대신 원기둥을 쓴다. 그때 반경을 여기서 알려 주지 않으면
                #    기본값 0.15 로 재어, 닿지도 않았는데 닿았다고 나온다.
                override = os.environ.get("SAR_PERSON_RADIUS")
                if override and def_name.startswith("PEDESTRIAN"):
                    radius = float(override)
                else:
                    radius = RADIUS_BY_DEF.get(def_name, DEFAULT_MOVER_RADIUS)
            else:
                continue                # 벽·바닥·목표물은 안 움직인다
            movers.append({"name": def_name, "node": node, "radius": radius,
                           "closest": math.inf, "when": 0.0,
                           "doing": "", "touch": 0})

    # ⚠️ 목표물의 **진짜 위치** 도 모은다. 이론 최소 시간을 재려면 필요하고,
    #    벽 기하를 모르는 월드(사람 시나리오)에서도 이 방법은 된다.
    targets_true = []
    if robot.getSupervisor():
        children = robot.getRoot().getField("children")
        for i in range(children.getCount()):
            node = children.getMFNode(i)
            if node is None:
                continue
            if (node.getDef() or "").startswith("TARGET"):
                tx, ty, _ = node.getPosition()
                targets_true.append((tx, ty))

    # ⚠️ 바닥의 **낮은 물체**(과일·캔)는 LiDAR 평면(약 17 cm)보다 낮아 로봇이 못 본다.
    #    부딪혀 밀렸는지 재려고 시작 위치를 적어 둔다 (채점 전용).
    LOW_TYPES = ("Apple", "RedApple", "GreenApple", "PurpleApple", "OrangeApple",
                 "Orange", "Can")
    low_objects = []
    if robot.getSupervisor():
        children = robot.getRoot().getField("children")
        for i in range(children.getCount()):
            node = children.getMFNode(i)
            if node is None or node.getTypeName() not in LOW_TYPES:
                continue
            ox, oy, oz = node.getPosition()
            if oz > 0.2:
                continue                # 식탁 위 과일은 로봇이 못 친다
            low_objects.append({"name": node.getDef() or node.getTypeName(), "node": node,
                                "start": (ox, oy), "moved_at": None, "robot": None})

    # tape 에는 사람 하나의 위치만 남긴다 (people.py 검증용). 사람이 없으면 첫 물체.
    pedestrian = movers[0]["node"] if movers else None
    if not robot.getSupervisor():
        print("  (supervisor 가 아니라 물체와의 진짜 거리는 못 잰다)")
    elif movers:
        print("  움직이는 물체를 잰다: "
              + ", ".join(f"{m['name']}(r={m['radius']:.2f})" for m in movers))
    else:
        print("  이 월드엔 움직이는 물체가 없다 — 충돌 측정은 정적 장애물만 본다.")

    # ⚠️ 컨트롤러 주기는 config.TIME_STEP 이다. basicTimeStep 을 그대로 쓰면
    #    월드의 물리 해상도가 바뀔 때 제어 주기까지 따라 바뀌어, 두 가지가
    #    한꺼번에 달라진다 (실험을 오염시킨다). 물리와 제어는 따로 두어야 한다.
    #    Webots 규약상 주기는 basicTimeStep 의 배수여야 한다.
    basic = int(robot.getBasicTimeStep())
    timestep = max(basic, int(round(config.TIME_STEP / basic)) * basic)
    # ⚠️ 실제 주기를 config 에도 적는다. 컨트롤러는 월드 주기에 맞춰 반올림하는데
    #    (apartment 는 basicTimeStep 64 라 16 -> 64), 다른 모듈은 config.TIME_STEP 으로
    #    틱을 초로 바꾼다 — people.py 가 사람 속도를 그렇게 계산하므로 안 맞추면
    #    **속도가 4배로 부풀려진다** (follower 의 dt 기본값도 같다).
    config.apply_timestep(timestep)   # 틱 단위 상수도 같이 맞춘다 (config 참고)
    dt = timestep / 1000.0

    sensors = sensors_mod.Sensors(robot)
    truth_src = truth_mod.Truth(robot)   # 정답 위치 — 채점 전용 (debug/truth.py)
    odometry = localization.Odometry()
    brain = mission_mod.Mission(odometry.pose)
    from sar import yolo_check
    brain.classify = yolo_check.load()   # 모델이 없으면 여기서 크게 알리고 멈춘다
    # 검증 중에도 지도를 눈으로 볼 수 있게 한다. SAR_VIZ=0 이면 알아서 꺼진다.
    display = viz_mod.Viz("mission_check — 검증하며 지도 보기")
    camera_fov = sensors.camera.getFov()

    print("=" * 74)
    print("[mission_check] Phase 4 — 자율 탐색 검증")
    print("=" * 74)
    print("  시간   상태      추정위치        진짜위치(정답)   오차   최근접장애물  탐색면적")
    sys.stdout.flush()

    # 매 틱을 CSV 로 남긴다. 나중에 "어느 구간에서 왜 헤맸나" 를 뒤져 보기 위해서다.
    os.makedirs(OUT_DIR, exist_ok=True)
    trace = open(os.path.join(OUT_DIR, "trace.csv"), "w", encoding="utf-8")
    trace.write("t,x,y,theta,gx,gy,v,w,state,goal_x,goal_y,path_len,"
                "near_d,near_a,squeeze,path_clear,dwa_fwd,dwa_safe_fwd,dwa_squeeze,status\n")

    trail_odom = []
    trail_true = []
    closest_ever = math.inf
    closest_where = None
    closest_when = 0.0
    closest_doing = ""

    # 사람 검출·추적을 나중에 오프라인에서 마음껏 실험하려고, 원시 LiDAR 와
    # 사람의 진짜 위치를 통째로 남긴다 (Webots 를 매번 돌리지 않으려고).
    tape = {"t": [], "pose": [], "ranges": [], "person": [], "truth": [],
            "cmd": []}
    # 카메라는 무겁다 (128x96x3). 사람 검출 실험용으로 가끔만 남긴다.
    frames = {"t": [], "image": [], "pose": [], "person": []}
    worst_drift = 0.0
    reached = {}               # 탐색률 이정표 -> (시각, 달린거리)
    # 사람 검출이 얼마나 맞는가. 움직이는 물체의 **진짜 위치** 를 알고 있으므로
    # brain.people_points() 와 대조하면 된다.
    # ⚠️ 이 숫자가 나와야 "카메라 확인을 붙일 가치가 있는가" 를 말할 수 있다.
    #    유령이 이미 0 에 가까우면 카메라는 복잡도만 늘린다.
    people_score = {"맞음": 0, "헛것": 0, "놓침": 0, "가려짐": 0, "틱": 0}
    # "이동 자체가 최적인가" 를 보려면 시간이 어디로 갔는지 쪼개야 한다.
    # 거리가 최적에 가까워도 시간이 길면, 범인은 경로가 아니라 멈춤·회전이다.
    spent = {"최대속도 주행": 0.0, "느린 주행": 0.0,
             "제자리 회전": 0.0, "완전 정지": 0.0}
    # ⚠️ "탐색률 94%" 는 **LiDAR** 가 그린 지도다. 목표물을 찾는 것은 카메라이고
    #    카메라는 57° 밖에 못 본다 (LiDAR 는 360°). 지도를 다 그려도 카메라가
    #    그쪽을 한 번도 안 봤으면 목표물은 영영 못 찾는다. 실제로 그래서
    #    "탐색률 94%, 목표물 1/3" 이 나왔다. 그러니 따로 재야 한다.
    camera_seen = np.zeros_like(brain.grid, dtype=bool)
    next_report = 0.0
    start_true = None
    tick = 0

    # ⚠️ Webots 를 닫거나 되돌리면 step() 이 -1 을 주고, **1초 뒤 강제 종료** 한다
    #    ("Forced termination … after 1 second"). 그때는 가벼운 기록만 먼저 남긴다.
    global TIME_LIMIT
    TIME_LIMIT = max(TIME_LIMIT, config.MISSION_TIME_LIMIT + 60.0)
    terminated = False
    next_tape_save = 60.0
    truth, pose = truth_src.xy(), odometry.pose   # 첫 틱 전에 닫혀도 progress 를 쓸 수 있게
    # 발표용 시연 영상. SAR_MOVIE=경로.mp4 일 때만 3D 화면을 녹화한다 (Supervisor 전용 기능).
    # 근거: Webots R2025a docs/reference/supervisor.md movieStartRecording — MP4 만, 화면 렌더링 필요
    #       (--no-rendering 으로 띄우면 녹화할 화면이 없다).
    movie = os.environ.get("SAR_MOVIE")
    if movie:
        robot.movieStartRecording(movie, 1280, 720, 0, 90, 1, False)
        print(f"  영상 녹화 시작: {movie}")
    # 재생 비교용 녹화 (debug/replay_check.py). SAR_RECORD=경로.pkl.gz 일 때만 — 판단에는 끼어들지 않는다.
    record_path = os.environ.get("SAR_RECORD")
    recorder = None
    if record_path:
        from debug import replay_check
        recorder = replay_check.Recorder(record_path, timestep, dt, camera_fov)
        brain.classify = recorder.wrap_classify(brain.classify)
    while True:
        if robot.step(timestep) == -1:
            terminated = True
            break
        tick += 1
        left, right = sensors.read_encoders()
        compass = sensors.read_compass()
        odometry.update(left, right, dt, compass_values=compass)
        pose = odometry.pose

        ranges = sensors.read_lidar()
        image = (sensors.read_camera_bgr()
                 if tick % config.DETECT_EVERY == 0 else None)
        taken = recorder.inputs(left, right, compass, ranges, image) if recorder else None
        speed, turn = brain.step(pose, ranges, dt,
                                 image=image, camera_fov=camera_fov,
                                 wheel_turn=odometry.wheel_turn_rate)
        if recorder:
            recorder.outputs(taken, brain, odometry, speed, turn, pose)
        sensors.drive(speed, turn)

        # 스캔 정합이 위치를 고쳐 줬으면 오도메트리에 되먹인다.
        # 이게 없으면 다음 틱에 또 틀어진 자리에서 누적을 이어간다.
        if brain.pose_fix is not None:
            odometry.correct(brain.pose_fix)

        # --- 여기서부터는 재기만 한다 (판단에 쓰지 않는다) ------------------
        truth = truth_src.xy()
        if tick == 0 and truth is not None:
            # ⚠️ config 의 시작 pose 와 월드의 로봇 배치가 어긋나면 로봇은 첫 틱부터
            #    엉뚱한 곳에 있다고 믿고 출발한다. 조용히 틀린 채 900초를 돌면 그
            #    측정은 통째로 쓰레기다 (실제로 5 m 어긋난 채 쟀다).
            #    ⚠️ 이 검사는 **시뮬레이션을 앞당기면 안 된다.** 본 루프 밖에서
            #       robot.step() 을 한 번 더 돌렸더니 궤적이 달라져 같은 설정인데
            #       maze0 이 38.4 -> 50.6 m 로 바뀌었다. 진단이 측정을 흔들면 안 된다.
            _gap = common.distance(truth[0], truth[1],
                                   config.START_X, config.START_Y)
            if _gap > 0.30:
                print("=" * 74)
                print(f"  ❌❌ 시작 pose 가 어긋났다: config ({config.START_X:+.2f},"
                      f" {config.START_Y:+.2f}) vs 월드 ({truth[0]:+.2f},"
                      f" {truth[1]:+.2f}) — {_gap:.2f} m")
                print("     이대로 재면 측정이 통째로 무효다. config.START_X/Y 를 맞춰라.")
                print("=" * 74)
                sys.stdout.flush()
        if start_true is None:
            start_true = truth
        trail_odom.append(pose[:2])
        trail_true.append(truth)

        # 물리 엔진이 터지면 로봇이 아레나 밖으로 날아간다. 그대로 두면
        # 이후 숫자가 전부 쓰레기가 되므로 여기서 알아채고 멈춘다.
        if max(abs(truth[0]), abs(truth[1])) > 20.0:
            print(f"\n  ❌ 로봇이 아레나 밖으로 날아갔다: "
                  f"({truth[0]:.1f}, {truth[1]:.1f}) — 시뮬레이션이 터진 것이다.")
            print(f"     시각 {brain.elapsed:.1f}s, 그때 하던 것: {brain.status}")
            sys.stdout.flush()
            break

        drift = common.distance(*truth, *pose[:2])
        worst_drift = max(worst_drift, drift)

        finite = ranges[np.isfinite(ranges) & (ranges >= config.LIDAR_MIN_RANGE)]
        if len(finite):
            nearest = float(finite.min())
            for obj in low_objects:
                if obj["moved_at"] is None and tick % 8 == 0:
                    ox, oy, _ = obj["node"].getPosition()
                    if math.hypot(ox - obj["start"][0], oy - obj["start"][1]) > 0.02:
                        obj["moved_at"] = brain.elapsed
                        obj["robot"] = tuple(truth)
            if pedestrian is not None:
                px, py, _ = pedestrian.getPosition()
                tape["t"].append(brain.elapsed)
                tape["pose"].append(tuple(pose))
                tape["ranges"].append(np.asarray(ranges, dtype=np.float32))
                tape["person"].append((px, py))
                tape["truth"].append(tuple(truth))
                tape["cmd"].append((speed, turn))
                if image is not None and FRAME_EVERY and tick % FRAME_EVERY == 0:
                    frames["t"].append(brain.elapsed)
                    frames["image"].append(image.copy())
                    frames["pose"].append(tuple(pose))
                    frames["person"].append((px, py))
            for m in movers:
                mx, my, _ = m["node"].getPosition()
                gap = math.hypot(truth[0] - mx, truth[1] - my)
                if gap < m["closest"]:
                    m["closest"] = gap
                    m["when"] = brain.elapsed
                    m["doing"] = brain.status
                if gap < config.ROBOT_RADIUS + m["radius"]:
                    m["touch"] += 1

            if nearest < closest_ever:
                closest_ever = nearest
                closest_where = truth
                closest_when = brain.elapsed
                closest_doing = brain.status

        near_d, near_a = follower.nearest_obstacle(ranges)
        goal = brain.goal if brain.goal else (float("nan"), float("nan"))
        trace.write(f"{brain.elapsed:.3f},{pose[0]:.4f},{pose[1]:.4f},{pose[2]:.4f},"
                    f"{truth[0]:.4f},{truth[1]:.4f},{speed:.4f},{turn:.4f},"
                    f"{brain.state},{goal[0]:.3f},{goal[1]:.3f},{len(brain.path)},"
                    f"{near_d:.3f},{math.degrees(near_a):.1f},"
                    f"{int(brain.squeezing)},{path_clearance(brain):.3f},"
                    f"{follower.LAST['fwd']},{follower.LAST['safe_fwd']},{follower.LAST['squeeze']},"
                    f"\"{brain.status}\"\n")

        if tick % config.VIZ_UPDATE_EVERY == 0:
            display.update(brain.grid, pose, path=brain.path,
                           frontiers=brain.frontier_points(),
                           targets=brain.target_points(),
                           status=f"{brain.state} {brain.elapsed:.0f}s",
                           people=brain.people_points(),
                           camera_seen=camera_seen,
                           visited=brain.visited_points())

        # --- 사람 검출이 맞았나 ------------------------------------
        people_score["틱"] += 1
        real = []
        for m in movers:
            mx, my, _ = m["node"].getPosition()
            real.append((mx, my))
        matched = set()
        for sx, sy in brain.people_points():
            hit = None
            for i, (mx, my) in enumerate(real):
                if math.hypot(sx - mx, sy - my) <= DETECT_MATCH_RADIUS:
                    hit = i
                    break
            if hit is None:
                people_score["헛것"] += 1
            else:
                people_score["맞음"] += 1
                matched.add(hit)
        # ⚠️ **가려진 것은 놓침이 아니다.** 예전에는 3.5 m 안의 진짜 물체가 매칭되지
        #    않으면 무조건 놓침으로 셌다. 벽 뒤에 있으면 어떤 검출기도 볼 수 없는데
        #    그것까지 세면 재현율이 허수가 된다 (실측: 그렇게 세면 29% 였다).
        #    그 방향의 LiDAR 가 물체보다 확실히 앞에서 멈췄으면 가려진 것이다.
        lidar_angles = common.lidar_angles()
        finite_r = np.asarray(ranges, dtype=np.float64)
        for i, (mx, my) in enumerate(real):
            if i in matched:
                continue
            gap = common.distance(*truth, mx, my)
            if gap > config.LIDAR_MAX_RANGE:
                continue
            bearing = common.wrap_angle(math.atan2(my - truth[1], mx - truth[0])
                                        - pose[2])
            near = np.abs(common.wrap_angle(lidar_angles - bearing)) < math.radians(4)
            seen_r = finite_r[near & np.isfinite(finite_r)]
            radius = movers[i]["radius"]
            if len(seen_r) and float(np.max(seen_r)) < gap - radius:
                people_score["가려짐"] += 1      # 볼 수 없었다 — 놓침이 아니다
                continue
            people_score["놓침"] += 1

        # --- 카메라가 어디를 봤나 ----------------------------------
        if tick % config.DETECT_EVERY == 0:
            half = camera_fov / 2.0
            reach = common.to_cells(config.DETECT_MAX_RANGE)
            occupied = mapping.is_occupied(brain.grid)
            here = common.to_cell(*truth)
            for k in range(CAMERA_RAYS):
                a = pose[2] - half + camera_fov * k / max(CAMERA_RAYS - 1, 1)
                dr, dc = math.sin(a), math.cos(a)
                for step in range(1, reach + 1):
                    r = int(round(here[0] + dr * step))
                    c = int(round(here[1] + dc * step))
                    if not common.in_bounds(r, c) or occupied[r, c]:
                        break
                    camera_seen[r, c] = True

        # --- 시간이 어디로 갔나 ------------------------------------
        if abs(speed) >= 0.95 * config.FOLLOW_MAX_SPEED:
            spent["최대속도 주행"] += dt
        elif abs(speed) > 0.01:
            spent["느린 주행"] += dt
        elif abs(turn) > 0.01:
            spent["제자리 회전"] += dt
        else:
            spent["완전 정지"] += dt

        # --- 탐색 진도 이정표 -------------------------------------
        ratio = int(mapping.is_free(brain.grid).sum()) * common.cell_area() / ARENA_AREA
        for mark in MILESTONES:
            if mark not in reached and ratio >= mark:
                so_far = sum(common.distance(*a, *b)
                             for a, b in zip(trail_true, trail_true[1:])) if len(trail_true) > 1 else 0.0
                reached[mark] = (brain.elapsed, so_far)

        if brain.elapsed >= next_report:
            trace.flush()                       # 중간에 꺼도 trace.csv 가 거의 다 남게
            write_progress(brain, truth, pose, worst_drift, movers, low_objects,
                           timestep / 1000.0)
        if brain.elapsed >= next_tape_save:
            next_tape_save += 60.0
            save_tape(tape, compressed=False)
        if brain.elapsed >= next_report:
            next_report += REPORT_EVERY
            near_d, near_a = follower.nearest_obstacle(ranges)
            explored = int(mapping.is_free(brain.grid).sum())
            print(f"  {brain.elapsed:6.1f} {brain.state:<8s}"
                  f" ({pose[0]:+5.2f},{pose[1]:+5.2f})"
                  f" ({truth[0]:+5.2f},{truth[1]:+5.2f})"
                  f" {drift * 100:5.1f}cm"
                  f" v={speed:+.3f} w={turn:+.2f}"
                  f" 최근접 {near_d:.2f}m@{math.degrees(near_a):+4.0f}°"
                  f" {explored:6d}칸"
                  # ⚠️ **현재 목표** 를 같이 찍는다. 상태줄만 보면 "EXPLORE — 주행"
                  #    이라 목표를 향해 잘 가는 것처럼 보이는데, 같은 자리를 맴돌면서
                  #    지도가 안 늘어나는 경우를 구별할 수 없다.
                  #    실측(comb1): 46초에 탐색률 50% 를 찍고 754초 동안 한 칸도
                  #    못 늘렸는데 내내 "EXPLORE — 주행" 이었다.
                  f" 목표{('(%+.2f,%+.2f)' % brain.goal) if brain.goal else '없음'}"
                  # ⚠️ 사람 몇 명을 보고 있나를 같이 찍는다. 유령 사람은 계획기에
                  #    사회적 비용을 얹어 경로를 왜곡하므로, "목표가 튄다" 를 볼 때
                  #    이 숫자가 같이 움직이는지 봐야 원인을 가른다.
                  #    (계획기용/전체 — 계획기는 확신이 선 것만 본다)
                  f" 사람{len(brain.people_points())}/{len(getattr(brain, '_people', []))}"
                  f" | {brain.status}")
            sys.stdout.flush()

        if brain.state == mission_mod.DONE or brain.elapsed > TIME_LIMIT:
            sensors.stop()
            break

    if recorder:
        recorder.close()
        print(f"  재생 비교용 녹화: {record_path} (틱 {recorder.ticks}개)")
        sys.stdout.flush()
    if movie and not terminated:
        robot.movieStopRecording()
        while not robot.movieIsReady():          # 인코딩이 끝날 때까지 기다린다
            if robot.step(timestep) == -1:
                break
        print("  영상:", "실패 (movieFailed)" if robot.movieFailed() else movie)
        sys.stdout.flush()

    trace.close()
    write_progress(brain, truth, pose, worst_drift, movers, low_objects, timestep / 1000.0,
                   note="(Webots 를 닫아서 멈춤)" if terminated else "(끝)")
    if terminated:
        save_tape(tape, compressed=False)     # 1초 안에 끝나야 한다 — 무거운 요약은 건너뛴다
        print("  Webots 가 닫혀 멈췄다 — progress.txt 와 tape.npz 만 남긴다", flush=True)
        return

    # --- 요약 ---------------------------------------------------------------
    # 화면과 summary.txt 에 동시에 쓴다 (화면 출력은 Webots 가 닫힐 때 날아갈 수 있다).
    summary_file = open(os.path.join(OUT_DIR, "summary.txt"), "w", encoding="utf-8")
    real_stdout = sys.stdout
    sys.stdout = _Tee(real_stdout, summary_file)
    explored = int(mapping.is_free(brain.grid).sum())
    area = explored * common.cell_area()
    travelled = sum(common.distance(*a, *b)
                    for a, b in zip(trail_true, trail_true[1:]))

    print("\n" + "=" * 74)
    print("요약")
    print("=" * 74)
    print(f"  상태            : {brain.state}  ({brain.status})")
    confirmed = brain.targets.confirmed
    from sar import detect as _detect
    print(f"  YOLO 판정        : {dict(_detect.YOLO_COUNT) or '한 번도 안 돌았다'}")
    print(f"  미끄러짐 감지     : {brain.slip_count} 회"
          + (f"  {[(round(x, 2), round(y, 2)) for x, y in brain.slip_spots]}" if brain.slip_spots else ""))
    hit = [o for o in low_objects if o["moved_at"] is not None]
    print(f"  밀린 낮은 물체    : {len(hit)} / {len(low_objects)} 개"
          + "".join(f"\n      {o['name']:12s} ({o['start'][0]:+.2f},{o['start'][1]:+.2f})"
                    f" {o['moved_at']:.0f}초에 처음 움직임"
                    f" → 지금 ({o['node'].getPosition()[0]:+.2f},{o['node'].getPosition()[1]:+.2f})"
                    for o in hit))
    print(f"  찾은 목표물      : {len(confirmed)} 개"
          f" (방문 {sum(1 for t in confirmed if t.visited)} 개,"
          f" 전체 후보 {len(brain.targets.targets)} 개)")
    for target in confirmed:
        print(f"      ({target.x:+.2f}, {target.y:+.2f})"
              f"  {target.sightings:4d} 회 봄"
              f"  추정이 움직인 총거리 {getattr(target, 'wobble', 0.0):.2f} m"
              f"  {'방문함' if target.visited else '못 감'}")
    print(f"  걸린 시간        : {brain.elapsed:.1f} s")
    print(f"  달린 거리        : {travelled:.1f} m")
    floor = travelled / config.FOLLOW_MAX_SPEED
    print(f"  실제 거리로 필요했던 시간: {floor:.1f} s"
          f"  (= {travelled:.1f} m / {config.FOLLOW_MAX_SPEED} m/s)"
          f"  ← **달린 거리**로 계산하므로 헛걸음을 잡지 못한다")

    # ⚠️ 위 값은 기준선이 될 수 없다. 로봇이 쓸데없이 100 m 를 달리면 기준선도
    #    같이 부풀어 "조금 느리다" 로 보인다 (diag: 667.7 / 525.6 = 1.27배).
    #    **이론 최소 시간** 은 달린 거리가 아니라 지도와 목표물 위치로 정한다.
    #    실측: diag 는 탐색을 120초에 95% 끝냈는데 총 667초를 썼다. 남은 500초는
    #    목표를 5초마다 갈아타며 제자리를 맴돈 시간이다 — 거리 기준으로는 안 보인다.
    ideal = None
    if targets_true:
        tour, broken = best_tour_on_map(brain.grid, trail_true, targets_true)
        if tour is None:
            print(f"  이론 최소 시간    : 못 구했다 — 끝낸 지도에서 길이 없는 구간"
                  f" {broken}  (지도가 덜 찼거나 문이 막혀 보인다)")
        else:
            if broken:
                print(f"  (참고) 길이 없던 구간 {broken} — 순회는 나머지로 풀었다")
            ideal = tour / config.FOLLOW_MAX_SPEED
            print(f"  이론 최소 시간    : {ideal:.1f} s"
                  f"  (최적 순회 {tour:.1f} m / {config.FOLLOW_MAX_SPEED} m/s"
                  f" — 회전·감속·탐색 비용을 0 으로 본 하한)")
            ratio = brain.elapsed / ideal
            verdict = ("⚠️ 이론보다 너무 길다 — 헛걸음을 의심한다"
                       if ratio > TIME_RATIO_SUSPECT else "정상 범위")
            print(f"  시간 배수         : {ratio:.2f}x  ({verdict},"
                  f" 기준 {TIME_RATIO_SUSPECT:.1f}x)")
    print("  시간이 어디로 갔나:")
    for label, seconds in spent.items():
        share = seconds / max(brain.elapsed, 1e-6) * 100
        print(f"      {label:12s} {seconds:6.1f} s  ({share:4.1f}%)")
    free_now = mapping.is_free(brain.grid)
    looked = int((camera_seen & free_now).sum())
    print(f"  ▶ 카메라가 훑은 빈 칸: {looked} / {explored}"
          f"  ({looked / max(explored, 1) * 100:.0f}%)"
          f"   ← 목표물을 찾을 수 있었던 범위 (LiDAR 탐색률과 다르다)")
    # 왜 후진을 고르는가 (follower.COUNT — 관찰용 계수기)
    if follower.COUNT.get("틱"):
        n = follower.COUNT["틱"]
        print("  DWA 후보 사정 (틱 기준):")
        for k in ("창에 전진 후보 없음", "전진 후보가 모두 위험",
                  "조준점이 뒤에 있음",
                  "들어온 속도: 후진", "들어온 속도: 거의0",
                  "들어온 속도: 느린전진", "들어온 속도: 최고속",
                  "후진을 골랐다", "후진 & 조준점 뒤", "후진 & 조준점 앞",
                  "후진 — 창의 최저(더 깊이)", "후진 — 창의 최고(회복 중)"):
            c = follower.COUNT.get(k, 0)
            print(f"      {k:24s} {c:7d} 틱  ({c / n * 100:5.1f}%)")
        for k in sorted(k for k in follower.COUNT if k.startswith("사람속도")):
            print(f"      {k:24s} {follower.COUNT[k]:7d}")
    # ⚠️ 못 찾은 목표물이 **진짜로 갈 수 없는 곳** 인지, 아니면 로봇 지도가 틀린
    #    것인지 가른다. 이걸 모르면 "탐색이 일찍 끝났다" 와 "원리적으로 못 찾는다"
    #    를 구별할 수 없다.
    if targets_true and len(confirmed) < len(targets_true):
        found_xy = [(t.x, t.y) for t in confirmed]
        for tx, ty in targets_true:
            if any(math.hypot(tx - fx, ty - fy) < 0.6 for fx, fy in found_xy):
                continue
            here = trail_true[0]
            mine = planner.plan(brain.grid, here, (tx, ty))
            cell = common.to_cell(tx, ty)
            state = "미지"
            if common.in_bounds(*cell):
                if mapping.is_free(brain.grid)[cell]:
                    state = "빈칸"
                elif mapping.is_occupied(brain.grid)[cell]:
                    state = "점유"
            # ⚠️ 그 목표물 근처에 **프론티어가 있기는 한가** 를 두 가지 마스크로
            #    본다. 엄격한 마스크(reachable_only=True)는 벽 근처 프론티어를
            #    지우므로, 둘의 차이가 "필터가 지웠다" 의 증거가 된다.
            from sar import exploration as _ex
            for label, strict in (("엄격", True), ("느슨", False)):
                m = _ex.frontier_mask(brain.grid, reachable_only=strict)
                frows, fcols = np.nonzero(m)
                if len(frows) == 0:
                    print(f"      프론티어({label}): 0칸")
                    continue
                fxs, fys = common.to_world(frows, fcols)
                gaps = np.hypot(fxs - tx, fys - ty)
                k = int(np.argmin(gaps))
                print(f"      프론티어({label}): {len(frows)}칸,"
                      f" 목표물에 가장 가까운 것 {gaps[k]:.2f} m"
                      f" @({fxs[k]:+.2f},{fys[k]:+.2f})")
            print(f"  ▶ 못 찾은 목표물 ({tx:+.2f},{ty:+.2f}):"
                  f" 로봇 지도에서 그 칸은 '{state}',"
                  f" 로봇 지도로 길 {'있음' if mine else '없음'}")

    # 목표가 얼마나 자주, 왜 바뀌나 (mission.COUNT — 관찰용 계수기)
    if mission_mod.COUNT.get("틱"):
        n = mission_mod.COUNT["틱"]
        picked = mission_mod.COUNT.get("새 목표를 세웠다", 0)
        print(f"  목표 교체 (틱 {n}, 새 목표 {picked}회"
              f" = {brain.elapsed / max(picked, 1):.1f}초마다):")
        for k in ("교체: 목표가 없었다", "교체: 시간 초과", "교체: 지도 밖",
                  "교체: 주변을 이미 다 봤다"):
            c = mission_mod.COUNT.get(k, 0)
            print(f"      {k:22s} {c:7d} 틱  ({c / n * 100:5.1f}%)")
        sec = timestep / 1000.0
        ap = mission_mod.COUNT.get("APPROACH 틱", 0)
        if ap:
            slow = mission_mod.COUNT.get("APPROACH 거의 멈춤", 0)
            near = mission_mod.COUNT.get("APPROACH 목표물 근처 틱", 0)
            nslow = mission_mod.COUNT.get("APPROACH 목표물 근처에서 거의 멈춤", 0)
            print(f"      {'APPROACH 시간':22s} {ap * sec:7.1f}초"
                  f"  (거의 멈춤 {slow * sec:.1f}초 = {slow / ap * 100:.0f}%)")
            print(f"      {'  그중 목표물 근처':22s} {near * sec:7.1f}초"
                  f"  (거의 멈춤 {nslow * sec:.1f}초"
                  f" = {nslow / max(near, 1) * 100:.0f}%)  ← 주저")
        wob = mission_mod.COUNT.get("APPROACH: 추정이 움직여 재계획", 0)
        print(f"      {'APPROACH 재계획(추정이 움직여)':22s} {wob:7d} 회")
        sweeps = mission_mod.COUNT.get("둘러보기 시작", 0)
        spin = mission_mod.COUNT.get("둘러보며 도는 틱", 0)
        # ⚠️ 여기서 spin/n 으로 비율을 찍었다가 뺐다. n 은 _needs_new_goal 호출
        #    수(EXPLORE 중에만 는다)라 SWEEP 중의 틱과 분모가 맞지 않는다.
        print(f"      {'둘러보기':22s} {sweeps:7d} 회"
              f"  (도는 데 {spin * timestep / 1000.0:.0f}초"
              f", 전체 {brain.elapsed:.0f}초 중)")

    # 사람 검출이 어디서 걸러지나 (people.COUNT — 관찰용 계수기)
    if getattr(people_mod, "COUNT", {}).get("틱"):
        n = people_mod.COUNT["틱"]
        print("  사람 검출 단계별 (틱 기준):")
        for k in ("후보: 지도차이", "후보: 다리모양", "후보가 0개인 틱",
                  "그룹", "기각: 스캔 수 부족", "기각: 안 움직였다"):
            c = people_mod.COUNT.get(k, 0)
            print(f"      {k:20s} {c:8d}  ({c / n:6.2f} /틱)")

    if people_score["틱"]:
        t = people_score["틱"]
        shown = people_score["맞음"] + people_score["헛것"]
        print("  사람 검출 성적 (진짜 위치와 대조):")
        for k in ("맞음", "헛것", "놓침", "가려짐"):
            print(f"      {k} {people_score[k]:8d} 회  ({people_score[k] / t:5.2f} /틱)")
        # ⚠️ 재현율은 **볼 수 있었던 것** 으로만 계산한다 (가려짐 제외).
        can_see = people_score["맞음"] + people_score["놓침"]
        if can_see:
            print(f"      → 볼 수 있었을 때의 재현율 "
                  f"{people_score['맞음'] / can_see * 100:.0f}%"
                  f"  (가려진 {people_score['가려짐']}회는 뺀다)")
        if shown:
            print(f"      → 검출한 것 중 헛것 비율 {people_score['헛것'] / shown * 100:.0f}%")
        else:
            print("      → 한 번도 검출하지 않았다")
    # 로봇이 **스스로** 들고 있는 카메라 격자 (위 숫자는 검증 스크립트가 따로 센 것)
    own = getattr(brain, "camera_seen", None)
    if own is not None:
        own_seen = int((own & free_now).sum())
        todo = free_now & ~own & ~planner.inflate(brain.grid,
                                                  config.PLANNER_INFLATION_MARGIN)
        print(f"  ▶ 로봇이 아는 카메라 커버리지: {own_seen} / {explored}"
              f"  ({own_seen / max(explored, 1) * 100:.0f}%)"
              f" — 안 본 곳 중 갈 수 있는 칸 {int(todo.sum())}")
    # ⚠️ 여기서 choose_unseen / camera_gain 을 불러 보고 있었다 (둘 다 없앴다).
    #    카메라 커버리지 자체는 위에 그대로 남아 있다 — 없앤 것은 그 값을 **목표
    #    점수에 쓰던 것** 이고, 얼마나 훑었는지 보는 계측은 여전히 필요하다.
    from sar import exploration as _exp
    _cl = _exp.candidate_list(brain.grid, truth)
    if _cl:
        print(f"  ▶ 후보들의 경로 길이: {[round(v[1], 2) for v in _cl]}  (m)"
              f"   ← 점수는 이것 하나다 (가중치 없음)")
    cands = len(_cl)
    lumps = len(_exp.cluster(_exp.frontier_mask(brain.grid)))
    # ⚠️ 덩어리는 남았는데 후보가 0개면 **어느 필터가 거부했는지** 를 말한다.
    #    그걸 모르면 "프론티어가 바닥났다" 와 "필터가 다 버렸다" 를 구별할 수 없고,
    #    실제로 그 둘을 헷갈려 엉뚱한 곳을 고친 적이 있다.
    if lumps and not cands:
        _blocked = planner.inflate(brain.grid, config.PLANNER_INFLATION_MARGIN)
        _why = {}
        for _r, _c, _n in _exp.cluster(_exp.frontier_mask(brain.grid)):
            _cell = (_r, _c)
            _spot = common.to_world(_r, _c)
            if _blocked[_cell]:
                _near = planner.nearest_free(_blocked, _cell)
                if _near is None:
                    _why["설 자리가 없다"] = _why.get("설 자리가 없다", 0) + 1
                    continue
                _cell = _near
            _x, _y = common.to_world(*_cell)
            if common.distance(_x, _y, *_spot) > config.FRONTIER_STALE_RADIUS:
                _why["설 자리가 프론티어에서 멀다"] = (
                    _why.get("설 자리가 프론티어에서 멀다", 0) + 1)
                continue
            if common.distance(_x, _y, *truth) < _exp._worth_driving_to():
                _why["너무 가깝다"] = _why.get("너무 가깝다", 0) + 1
                continue
            if not planner.plan(brain.grid, truth, (_x, _y), exact=True):
                _why["길이 없다"] = _why.get("길이 없다", 0) + 1
                continue
            _why["통과(블랙리스트였다)"] = _why.get("통과(블랙리스트였다)", 0) + 1
        print(f"      → 덩어리 {lumps}개를 왜 버렸나: {_why}")
    print(f"  ▶ 끝난 시점의 프론티어: 고를 수 있는 후보 {cands}개 / 덩어리 {lumps}개"
          f"   ← 1개면 '무엇을 고를까' 가 무의미하다")
    print("  탐색 진도 (사람 없을 때의 효율 지표):")
    for mark in MILESTONES:
        if mark in reached:
            when, how_far = reached[mark]
            print(f"      {mark * 100:3.0f}% 까지  {when:6.1f}s  {how_far:6.1f}m"
                  f"   (m 당 {mark * ARENA_AREA / max(how_far, 1e-6):.2f} m²)")
        else:
            print(f"      {mark * 100:3.0f}% 까지  — 도달 못 함")
    print(f"  탐색 면적        : {explored} 칸 = {area:.2f} m²"
          f"  (아레나 36 m² 중 {area / 36 * 100:.0f}%)")
    print(f"  오도메트리 최대오차: {worst_drift * 100:.1f} cm")
    print(f"  시작점 복귀오차    : "
          f"{common.distance(*start_true, *trail_true[-1]) * 100:.1f} cm"
          f"  (Phase 4 에서는 복귀를 안 하므로 참고만)")
    print(f"  LiDAR 최솟값(참고): {closest_ever:.3f} m"
          f"  — {config.LIDAR_MIN_RANGE} m 에서 포화하고 원점보다 3 cm 뒤에서"
          f" 재므로 **충돌 판정에 쓸 수 없다**")
    if closest_where:
        print(f"      그때 위치     : ({closest_where[0]:+.2f}, {closest_where[1]:+.2f})"
              f"  시각 {closest_when:.1f}s")
        print(f"      그때 하던 것  : {closest_doing}")

    for m in movers:
        print(f"  {m['name']} 과 가장 가까웠던 거리: {m['closest']:.3f} m"
              f"  (로봇 {config.ROBOT_RADIUS:.2f} + 물체 {m['radius']:.2f}"
              f" = {config.ROBOT_RADIUS + m['radius']:.2f} m 아래면 닿은 것)")
        print(f"      그때 시각 {m['when']:.1f}s / 하던 것: {m['doing']}")
        if m["touch"]:
            print(f"      ❌ 닿아 있던 시간: {m['touch'] * (timestep / 1000.0):.1f}초")
        else:
            print(f"      ✅ 한 번도 닿지 않았다")

    print("=" * 74)
    sys.stdout.flush()
    sys.stdout = real_stdout
    summary_file.close()

    if tape["t"]:
        save_tape(tape)
        if frames["t"]:
            np.savez_compressed(os.path.join(OUT_DIR, "frames.npz"),
                                t=np.array(frames["t"], dtype=np.float32),
                                image=np.array(frames["image"], dtype=np.uint8),
                                pose=np.array(frames["pose"], dtype=np.float32),
                                person=np.array(frames["person"], dtype=np.float32))
            print(f"  카메라 프레임을 남겼다: {len(frames['t'])} 장")
        print(f"  원시 기록을 남겼다: {len(tape['t'])} 틱 "
              f"(LiDAR + 로봇 추정위치 + 사람 진짜위치)")

    save_picture(brain, trail_odom, trail_true)

    print("[[DONE]]")          # run_headless.sh 가 이걸 보고 끝낸다
    sys.stdout.flush()

    while robot.step(timestep) != -1:
        pass


def path_clearance(brain):
    """지금 계획된 경로가 벽에서 최소 얼마나 떨어져 있는가 [m].

    "계획이 애초에 벽에 붙어 있었나" 와 "주행이 벗어난 것인가" 를 가르기 위해 잰다.
    """
    if not brain.path:
        return float("nan")
    occupied = mapping.is_occupied(brain.grid)
    rows, cols = np.nonzero(occupied)
    if len(rows) == 0:
        return float("inf")
    xs, ys = common.to_world(rows, cols)
    # ⚠️ 첫 점은 로봇 자신의 위치다. 로봇이 벽에 붙어 있으면 그 점이 당연히 가까워서,
    #    "계획이 나쁘다" 는 잘못된 결론이 나온다. 출발점 근처는 빼고 잰다.
    start = brain.path[0]
    best = float("inf")
    for px, py in brain.path[1:]:
        if common.distance(px, py, *start) < 0.5:
            continue
        best = min(best, float(np.min(np.hypot(xs - px, ys - py))))
    return best


def save_picture(brain, trail_odom, trail_true):
    os.makedirs(OUT_DIR, exist_ok=True)
    figure, axis = plt.subplots(figsize=(8, 8))
    axis.imshow(viz_mod.grid_to_image(brain.grid), cmap="gray", vmin=0.0, vmax=1.0,
                origin="lower", extent=viz_mod.map_extent(), interpolation="nearest")
    # ⚠️ **카메라가 본 범위** 를 겹쳐 그린다. 목표물은 카메라로만 찾을 수 있으므로
    #    (LiDAR 는 색을 모른다) "어디를 아직 안 봤나" 가 LiDAR 탐색률보다 중요하다.
    #    실측(maze2): LiDAR 로는 다 그렸는데 카메라로 안 본 방이 남아 끝내지 못했다.
    #    빨강 = 빈 칸인데 카메라로 아직 못 본 곳 = 목표물이 숨어 있을 수 있는 곳.
    free_now = mapping.is_free(brain.grid)
    unseen = free_now & ~brain.camera_seen
    if unseen.any():
        overlay = np.zeros(unseen.shape + (4,), dtype=np.float32)
        overlay[unseen] = (1.0, 0.25, 0.25, 0.55)
        axis.imshow(overlay, origin="lower", extent=viz_mod.map_extent(),
                    interpolation="nearest")
    seen = free_now & brain.camera_seen
    if seen.any():
        overlay2 = np.zeros(seen.shape + (4,), dtype=np.float32)
        overlay2[seen] = (0.3, 0.7, 1.0, 0.22)
        axis.imshow(overlay2, origin="lower", extent=viz_mod.map_extent(),
                    interpolation="nearest")

    axis.plot([p[0] for p in trail_true], [p[1] for p in trail_true], "-",
              color="tab:green", linewidth=1.0, label="진짜 경로 (정답)")
    axis.plot([p[0] for p in trail_odom], [p[1] for p in trail_odom], "-",
              color="tab:blue", linewidth=1.0, label="추정 경로 (오도메트리)")
    axis.set_xlim(-3.4, 3.4)
    axis.set_ylim(-3.4, 3.4)
    axis.set_aspect("equal")
    axis.grid(alpha=0.15)
    axis.legend(loc="upper right", fontsize=8)
    axis.set_xlabel("x [m]")
    axis.set_ylabel("y [m]")
    looked = int((brain.camera_seen & free_now).sum())
    total_free = int(free_now.sum())
    axis.set_title(f"{brain.state} — {brain.elapsed:.0f}s   "
                   f"카메라가 본 빈 칸 {looked}/{total_free}"
                   f" ({looked / max(total_free, 1) * 100:.0f}%)"
                   f"   빨강 = 아직 안 본 곳")
    # 격자를 그대로 남긴다 — "경로가 왜 벽을 뚫었나" 를 나중에 뜯어보려면
    # 그림이 아니라 숫자가 필요하다.
    np.savez(os.path.join(OUT_DIR, "grid.npz"), grid=brain.grid)
    path = os.path.join(OUT_DIR, "mission.png")
    figure.savefig(path, dpi=110, bbox_inches="tight")
    print(f"  그림을 저장했다: {path}")
    sys.stdout.flush()
