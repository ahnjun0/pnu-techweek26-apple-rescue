"""사람 없는 '복잡한 맵' 월드를 만들고, 만들기 전에 풀 수 있는지 검사한다.

1단계(사람 없을 때) 탐색 효율만 재기 위한 월드다. 보행자도 mover 도 없다.

⚠️ 월드 파일과 검증이 따로 놀면 안 된다. 벽 목록 WALLS 하나에서 .wbt 도 만들고
   연결성 검사도 하므로, "검사는 통과했는데 월드는 다르다" 가 생길 수 없다.

검사 없이 월드를 돌리면 안 되는 이유: 문이 로봇보다 좁으면 900초를 버린 뒤에야
그 사실을 알게 된다. 여기서는 planner 와 똑같은 팽창을 써서 미리 확인한다.
"""
import math
import os
import random
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import common
import config
import mapping
import planner

ARENA = 6.0
THICK = 0.1
HEIGHT = 0.5
# ⚠️ 지도마다 1회씩만 재면 개선과 운을 구별할 수 없다. 설정이 조금만 바뀌어도
#    궤적이 통째로 달라져 목표물을 우연히 보거나 놓친다 (회전 비용 0.0→3/3,
#    0.5→1/3, 1.0→3/3 처럼 널뛰었다). 그래서 시작 위치를 바꿔 가며 여러 번 잰다.
# ⚠️ 시작점은 목표물과 겹치면 안 된다. 처음에 네 귀퉁이를 그대로 썼다가, maze 의
#    목표물 (-2.5,2.5) / (2.5,-2.5) 와 같은 자리가 되어 로봇이 목표물에 끼었다
#    (maze2: 900초 중 684초가 완전 정지, 5.8 m, 0/3). 아래 check() 가 검사한다.
STARTS = [(-2.5, -2.5), (0.0, -2.5), (-2.5, 0.0), (2.5, 0.0)]
START = STARTS[int(os.environ.get("SAR_START", "0"))]

# (중심x, 중심y, 가로, 세로) — 세로벽은 가로=THICK, 가로벽은 세로=THICK
# 벽은 '문' 을 남기려고 토막으로 끊어 둔다. 문 폭을 주석에 적는다:
# ⚠️ 측정: 팽창 반경 = ROBOT_RADIUS + PLANNER_INFLATION_MARGIN = 0.13 + 0.22 = 0.35 m.
#    따라서 문이 0.70 m 이하면 계획기가 통과 자체를 못 한다 (0.8 m 문으로 실패했다).
#    여기서는 1.0 m 로 둔다. 좁은 문은 PLANNER_SQUEEZE_MARGIN 쪽 이야기다.
# ⚠️ 지도 한 장으로 "탐색이 효율적이다" 를 말할 수 없다. 전략의 이득은 지도 모양에
#    크게 좌우된다 (방이 많으면 들락날락이 심해진다). 그래서 성격이 다른 지도를
#    여러 장 두고 같은 잣대로 잰다.
#    문은 0.70 m 보다 넓어야 통과한다 (팽창 반경 0.35 m × 2). 여유를 두어 1.0 m.
LAYOUTS = {
    # 방 + 복도가 섞인 기본형
    "maze": {
        "walls": [
            (-1.0, -2.0, THICK, 2.0),
            (-1.0, 1.5,  THICK, 3.0),
            (-0.1, 0.6,  1.8, THICK),
            (2.4,  0.6,  1.2, THICK),
            (1.0,  -1.9, THICK, 2.2),
            (-2.5, 1.0,  1.0, THICK),
        ],
        "targets": [(-2.5, 2.5), (2.5, 2.5), (2.5, -2.5)],
    },
    # 거의 트인 방 — 프론티어 순서가 거의 영향을 못 준다
    "open": {
        "walls": [
            (0.0, 0.0, 1.2, THICK),
            (1.5, 1.5, THICK, 1.2),
        ],
        "targets": [(-2.5, 2.5), (2.5, 2.5), (2.5, -2.5)],
    },
    # 빗살 — 작은 방이 다섯. SEG 가 가장 유리해야 할 모양이다
    "comb": {
        "walls": [
            (-1.8, 1.0,  THICK, 4.0),   # x=-1.8, y  -1.0..3.0
            (-0.6, -1.0, THICK, 4.0),   # x=-0.6, y  -3.0..1.0
            (0.6,  1.0,  THICK, 4.0),
            (1.8,  -1.0, THICK, 4.0),
        ],
        "targets": [(-2.4, 2.5), (0.0, 2.5), (2.4, -2.5)],
    },
    # 긴 복도 + 막다른 가지 — 되짚기가 강제된다
    "corridor": {
        "walls": [
            (-0.5, 0.6,  5.0, THICK),   # y=0.6  가로 긴 벽
            (-0.5, -0.6, 5.0, THICK),   # y=-0.6 가로 긴 벽
            (-1.5, 2.3,  THICK, 1.4),   # y 1.6..3.0 — 위 통로 0.95 m 남긴다
            (0.5,  2.3,  THICK, 1.4),
            (-1.5, -2.3, THICK, 1.4),   # y -3.0..-1.6
            (0.5,  -2.3, THICK, 1.4),
        ],
        "targets": [(-2.5, 2.5), (2.5, 2.5), (2.5, -2.5)],
    },
}

def random_layout(seed):
    """무작위 벽·목표물 배치. **우리가 안 본 지도** 로 검증하려고 만든다.

    ⚠️ 왜 필요한가: 지금까지 고른 상수들(FRONTIER_TURN_COST, SCANMATCH_MIN_GAIN,
       MISSION_SCAN_TURNS 등)은 전부 **내가 설계한 네 모양** (미로·빈방·빗살·복도)
       에서 비교해 정했다. 대회 아레나가 그중 하나와 닮으리란 보장이 없다.
       설정을 전혀 손대지 않고 여기서 돌려 봐야 "우리 월드에만 맞춘 것" 인지 안다.

    벽은 가로/세로 토막을 무작위로 놓되, 문이 막히지 않게 길이를 제한한다.
    풀 수 있는지는 어차피 check() 가 쓰기 전에 확인하므로, 안 되면 다시 뽑으면 된다.
    """
    rng = random.Random(seed)
    walls = []
    for _ in range(rng.randint(4, 7)):
        vertical = rng.random() < 0.5
        length = rng.uniform(1.0, 3.0)
        cx = rng.uniform(-2.0, 2.0)
        cy = rng.uniform(-2.0, 2.0)
        walls.append((cx, cy, THICK, length) if vertical
                     else (cx, cy, length, THICK))
    targets = []
    while len(targets) < 3:
        t = (round(rng.uniform(-2.6, 2.6), 2), round(rng.uniform(-2.6, 2.6), 2))
        if all(math.hypot(t[0] - o[0], t[1] - o[1]) > 1.5 for o in targets):
            targets.append(t)
    return {"walls": walls, "targets": targets}


LAYOUT = os.environ.get("SAR_LAYOUT", "maze")
if LAYOUT.startswith("rand"):
    LAYOUTS[LAYOUT] = random_layout(int(LAYOUT[4:] or 0))
if LAYOUT not in LAYOUTS:
    raise SystemExit(f"모르는 지도: {LAYOUT} (가능: {', '.join(LAYOUTS)})")
WALLS = LAYOUTS[LAYOUT]["walls"]
TARGETS = LAYOUTS[LAYOUT]["targets"]


def build_grid():
    """WALLS 와 아레나 외벽을 점유 격자로 굽는다 (planner 가 보는 것과 같은 형식)."""
    grid = mapping.new_map()

    def fill(x0, y0, x1, y1):
        r0, c0 = common.to_cell(x0, y0)
        r1, c1 = common.to_cell(x1, y1)
        grid[min(r0, r1):max(r0, r1) + 1, min(c0, c1):max(c0, c1) + 1] = 10.0

    half = ARENA / 2.0
    # 바닥을 '빈 곳' 으로 칠한다 (로그오즈 음수 = 비어 있음)
    r0, c0 = common.to_cell(-half, -half)
    r1, c1 = common.to_cell(half, half)
    grid[r0:r1 + 1, c0:c1 + 1] = -10.0

    # 외벽
    fill(-half, -half, half, -half + THICK)
    fill(-half, half - THICK, half, half)
    fill(-half, -half, -half + THICK, half)
    fill(half - THICK, -half, half, half)
    for cx, cy, w, h in WALLS:
        fill(cx - w / 2, cy - h / 2, cx + w / 2, cy + h / 2)
    return grid


def wall_rects():
    """벽을 사각형 목록 [(x0, y0, x1, y1)] 로. 외벽 + LAYOUTS 의 내벽.

    ⚠️ build_grid() 가 아니라 이것을 쓰는 이유: 격자는 0.05 m 라 1 cm 여유를
       판정할 수 없다 (양자화 오차가 최대 3.5 cm). 충돌 여부는 해석적으로 재야 한다.
    """
    half = ARENA / 2.0
    rects = [
        (-half, -half, half, -half + THICK),
        (-half, half - THICK, half, half),
        (-half, -half, -half + THICK, half),
        (half - THICK, -half, half, half),
    ]
    for cx, cy, w, h in WALLS:
        rects.append((cx - w / 2, cy - h / 2, cx + w / 2, cy + h / 2))
    return rects


TARGET_SIZE = 0.2            # 목표물 상자 한 변 [m] — 아래 월드 작성부와 같은 값


def target_rects():
    """목표물 상자를 사각형 목록으로. 가까이 가는 것은 의도, 닿는 것은 아니다."""
    h = TARGET_SIZE / 2.0
    return [(tx - h, ty - h, tx + h, ty + h) for tx, ty in TARGETS]


def _poly_gap(a, b):
    """두 볼록 다각형 사이 거리의 **하한** (분리축 정리). 겹치면 0.

    ⚠️ 꼭짓점-꼭짓점 이 최근접인 경우 실제 거리보다 작게 나온다 — 안전한 방향
       (여유를 과소평가) 이므로 충돌 판정에 쓰기에 알맞다.
    """
    best = 0.0
    for poly in (a, b):
        n = len(poly)
        for i in range(n):
            x0, y0 = poly[i]
            x1, y1 = poly[(i + 1) % n]
            ax, ay = -(y1 - y0), (x1 - x0)          # 변의 법선
            length = math.hypot(ax, ay)
            if length < 1e-12:
                continue
            ax, ay = ax / length, ay / length
            pa = [px * ax + py * ay for px, py in a]
            pb = [px * ax + py * ay for px, py in b]
            gap = max(min(pb) - max(pa), min(pa) - max(pb))
            if gap > best:
                best = gap
    return best


def body_polygons(x, y, theta):
    """(x, y, theta) 에 있는 로봇 몸체 = 상자 두 개의 꼭짓점 목록.

    출처: TurtleBot3Burger.proto:186-201 (config._BODY_BOXES 와 같은 값).
    """
    c, s = math.cos(theta), math.sin(theta)
    out = []
    for w, h in config._BODY_BOXES:
        corners = []
        for sx in (-1, 1):
            for sy in (-1, 1):
                lx = config._BODY_CENTRE_X + sx * w / 2.0
                ly = sy * h / 2.0
                corners.append((x + lx * c - ly * s, y + lx * s + ly * c))
        # 사각형 순서로 (SAT 는 순서를 쓰므로)
        corners = [corners[0], corners[1], corners[3], corners[2]]
        out.append(corners)
    return out


def body_clearance(x, y, theta, rects=None):
    """몸체(상자 두 개)에서 가장 가까운 벽면까지의 거리 [m]. 겹치면 0.

    외접원(ROBOT_BODY_RADIUS) 대신 실제 방향을 쓰므로 정확하다.
    """
    rects = wall_rects() if rects is None else rects
    polys = body_polygons(x, y, theta)
    best = float("inf")
    for x0, y0, x1, y1 in rects:
        box = [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]
        for poly in polys:
            best = min(best, _poly_gap(poly, box))
    return best


def clearance_to_walls(x, y, rects=None):
    """(x, y) 에서 가장 가까운 벽면까지의 거리 [m]. 벽 안이면 0."""
    rects = wall_rects() if rects is None else rects
    best = float("inf")
    for x0, y0, x1, y1 in rects:
        dx = max(x0 - x, 0.0, x - x1)
        dy = max(y0 - y, 0.0, y - y1)
        best = min(best, math.hypot(dx, dy))
    return best


def check(grid):
    """시작점에서 목표물 하나하나까지 planner 가 실제로 길을 찾는지 본다."""
    ok = True
    # 시작점이 서 있을 수 있는 자리인가 (벽·목표물과 겹치지 않는가)
    blocked = planner.inflate(grid, config.PLANNER_INFLATION_MARGIN)
    cell = common.to_cell(*START)
    if not common.in_bounds(*cell) or blocked[cell]:
        print(f"  시작점 {START} : ❌ 벽에 너무 가깝다")
        ok = False
    for i, t in enumerate(TARGETS, 1):
        gap = common.distance(*START, *t)
        if gap < config.APPROACH_DISTANCE:
            print(f"  시작점 {START} : ❌ 목표물 {i} {t} 와 {gap:.2f} m —"
                  f" {config.APPROACH_DISTANCE:.2f} m 는 떨어져야 한다")
            ok = False
    free = int(mapping.is_free(grid).sum())
    print(f"  빈 칸 {free} = {free * common.cell_area():.2f} m²  (아레나 {ARENA**2:.0f} m²)")
    for i, (tx, ty) in enumerate(TARGETS, 1):
        # 목표물은 그 자체가 장애물이라 조금 앞까지만 간다
        path = planner.plan(grid, START, (tx, ty))
        if path:
            length = sum(common.distance(*a, *b) for a, b in zip(path, path[1:]))
            print(f"  목표물 {i} ({tx:+.1f},{ty:+.1f}) : 길 있음, {length:.2f} m")
        else:
            print(f"  목표물 {i} ({tx:+.1f},{ty:+.1f}) : ❌ 길이 없다 — 문이 너무 좁다")
            ok = False
    return ok


def best_tour():
    """지도를 이미 다 안다고 쳤을 때의 최적 순회 거리 [m].

    시작 → 목표물 셋을 모두 방문 → 시작. 이것이 '달린 거리' 의 **하한** 이다.
    탐색은 지도를 모르는 채 하므로 절대 이보다 짧을 수 없다. 얼마나 더 썼는지가
    경로 효율이다 (실제 / 하한).
    """
    import itertools
    grid = build_grid()
    spots = [START] + TARGETS

    def hop(a, b):
        path = planner.plan(grid, a, b)
        if not path:
            return math.inf
        return sum(common.distance(*p, *q) for p, q in zip(path, path[1:]))

    d = {(i, j): hop(spots[i], spots[j])
         for i in range(len(spots)) for j in range(len(spots)) if i != j}
    best = math.inf
    for order in itertools.permutations(range(1, len(spots))):
        route = (0,) + order + (0,)
        best = min(best, sum(d[(route[k], route[k + 1])] for k in range(len(route) - 1)))
    return best


def wbt(path):
    solids = []
    for i, (cx, cy, w, h) in enumerate(WALLS, 1):
        solids.append(f"""DEF WALL_{i} Solid {{
  translation {cx} {cy} {HEIGHT / 2}
  name "wall_{i}"
  children [
    Shape {{
      appearance PBRAppearance {{ baseColor 0.6 0.6 0.6 roughness 1 metalness 0 }}
      geometry Box {{ size {w} {h} {HEIGHT} }}
    }}
  ]
  boundingObject Box {{ size {w} {h} {HEIGHT} }}
}}""")
    for i, (tx, ty) in enumerate(TARGETS, 1):
        solids.append(f"""DEF TARGET_{i} Solid {{
  translation {tx} {ty} 0.2
  children [
    Shape {{
      appearance PBRAppearance {{ baseColor 1 0 0 roughness 0.4 metalness 0 }}
      geometry Box {{ size 0.2 0.2 0.4 }}
    }}
  ]
  name "target_{i}"
  boundingObject Box {{ size 0.2 0.2 0.4 }}
}}""")
    body = "\n\n".join(solids)
    # ⚠️ 로봇 노드는 worlds/stress_check.wbt 에서 그대로 베꼈다. 추측하지 않는다.
    #    supervisor TRUE 는 debug 검증(정답 위치, debug/truth.py) 전용이다 (CLAUDE.md 규칙 1).
    #    빠뜨리면 mission_check 가 시작하자마자 멈춘다.
    open(path, "w", encoding="utf-8").write(f"""#VRML_SIM R2025a utf8

EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects/objects/backgrounds/protos/TexturedBackground.proto"
EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects/objects/backgrounds/protos/TexturedBackgroundLight.proto"
EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects/objects/floors/protos/RectangleArena.proto"
EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects/robots/robotis/turtlebot/protos/TurtleBot3Burger.proto"
EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects/devices/robotis/protos/RobotisLds01.proto"

WorldInfo {{
  info [ "1단계 — 사람 없는 복잡한 맵. 탐색 속도와 경로 효율만 잰다." ]
  basicTimeStep 16
}}
Viewpoint {{
  orientation 0 0 -1 3.14159
  position 0 0 12
}}
TexturedBackground {{
}}
TexturedBackgroundLight {{
}}
RectangleArena {{
  floorSize {ARENA} {ARENA}
  wallHeight {HEIGHT}
}}

{body}

TurtleBot3Burger {{
  translation {START[0]} {START[1]} 0
  rotation 0 0 1 0
  controller "mission_check"
  supervisor TRUE
  extensionSlot [
    RobotisLds01 {{
    }}
    Camera {{
      translation 0.02 0 0.06
      name "camera"
      fieldOfView 1
      width 128
      height 96
    }}
  ]
}}
""")


if __name__ == "__main__":
    grid = build_grid()
    print("[make_maze_world] 월드를 쓰기 전에 풀 수 있는지 검사한다")
    if not check(grid):
        print("\n❌ 못 푸는 월드다. WALLS 를 고쳐라. 파일은 쓰지 않았다.")
        sys.exit(1)
    floor = best_tour()
    print(f"\n  최적 순회(지도를 다 안다고 칠 때) : {floor:.2f} m"
          f"  ← 달린 거리는 절대 이보다 짧을 수 없다")
    out = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                       "worlds", f"{LAYOUT}{os.environ.get('SAR_START', '0')}_check.wbt")
    wbt(out)
    print(f"\n✅ 전부 통과. 썼다: {out}")
