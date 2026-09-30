"""사람 시나리오 변형 월드를 만든다 (속도·경로·크기를 바꿔 가며).

⚠️ 왜 필요한가: 2단계를 **보행자 궤적 하나, 속도 하나** 로만 쟀다. 그 월드는
   결정적이라 3회 반복해도 같은 값이 나오므로, "접촉 0초" 는 **그 한 경우에서**
   안 닿았다는 뜻일 뿐 회피가 튼튼하다는 증거가 아니다.
   (사용자 지적: "사람과 부딪히지 않은 것은 우연일 확률이 더 높아 보인다.
    '작은 사람' 은 왜 테스트해보지 않았나?")

쓰는 법:  python3 debug/make_people_world.py            # 전부 만든다
          SAR_PEOPLE=cross python3 debug/make_people_world.py

worlds/stress_check.wbt 를 본으로 삼아 worlds/people_<이름>_check.wbt 를 쓴다.
"""

import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
TEMPLATE = os.path.join(ROOT, "worlds", "stress_check.wbt")
sys.path.insert(0, ROOT)            # config 의 ROBOT_RADIUS 를 쓴다

# 사람이 **작을** 때를 반드시 넣는다. LiDAR 는 z = 0.153 m 를 보므로 그 높이의
# 단면이 곧 검출 가능성이다 — 성인 보행자는 정강이(반경 0.191 m 로 실측),
# 아이나 마른 사람은 그보다 얇다.
VARIANTS = {
    # 이름:      (보행자 속도, 궤적, 작은사람으로 바꿀까, 작은사람 반경)
    "base":      (0.5, "-2.6 -1.4, 2.6 -1.4", False, None),
    "fast":      (0.9, "-2.6 -1.4, 2.6 -1.4", False, None),
    "slow":      (0.25, "-2.6 -1.4, 2.6 -1.4", False, None),
    "cross":     (0.5, "0.0 -2.6, 0.0 2.6", False, None),
    # ⚠️ 한때 "-2.6 -2.6, 2.6 2.6" 이었다. 그 궤적은 **로봇 시작점을 관통한다**
    #    (로봇 -2.5 -2.5 에서 14 cm 에서 출발해 정면으로 지나간다). 로봇이 아직
    #    시작 스캔(제자리 회전) 중인 0.3초에 닿으므로 회피로 막을 수 없었다 —
    #    로봇이 가만히 있다고만 가정해도 최근접 0.003 m / 닿은 시간 0.9초가
    #    그대로 재현된다. 네 가지 설정에서 0.9초·2.2 cm 가 똑같이 나온 이유다.
    #    반대 대각선은 궤적이 시작점에서 3.54 m 떨어져 있고 맵 중앙을 지난다.
    "diag":      (0.5, "-2.6 2.6, 2.6 -2.6", False, None),
    # 얇은 사람 둘 — 원기둥으로 바꾼다 (Pedestrian PROTO 는 크기를 못 바꾼다)
    "thin":      (0.5, "-2.6 -1.4, 2.6 -1.4", True, 0.09),
    "child":     (0.5, "-2.6 -1.4, 2.6 -1.4", True, 0.05),
}


def small_person(traj, speed, radius):
    """보행자를 **얇은 원기둥** 으로 바꾼 노드 글.

    ⚠️ DEF 를 PEDESTRIAN1 로 유지한다 — mission_check 가 그 이름으로 반경
       PERSON_RADIUS 를 붙이므로, 이름을 바꾸면 충돌 판정이 조용히 달라진다.
       대신 아래에서 반경을 따로 알려 준다 (SAR_PERSON_RADIUS).
    """
    return f'''DEF PEDESTRIAN1 Robot {{
  translation {traj.split(',')[0].strip()} 0.25
  name "pedestrian1"
  controller "mover"
  controllerArgs [
    "--trajectory={traj}"
    "--speed={speed}"
  ]
  supervisor TRUE
  children [
    Shape {{
      appearance PBRAppearance {{ baseColor 0.9 0.2 0.2 roughness 0.5 metalness 0 }}
      geometry Cylinder {{ radius {radius} height 0.5 }}
    }}
  ]
  boundingObject Cylinder {{ radius {radius} height 0.5 }}
}}'''


# 본 월드의 로봇 시작점 (worlds/stress_check.wbt:194 TurtleBot3Burger.translation)
ROBOT_START = (-2.5, -2.5)


def check_trajectory(name, traj, radius):
    """궤적이 **로봇 시작점을 관통하지 않는지** 본다.

    ⚠️ 이 검사가 없어서 diag 시나리오가 로봇 시작점을 관통하고 있었고, 그
       접촉 0.9초를 "회피 결함" 으로 읽어 엉뚱한 곳을 네 번 고쳤다.
       로봇은 시작 스캔 중이라 피할 수 없다 — 잴 수 없는 것을 재고 있었다.
    """
    import math
    import config

    need = config.ROBOT_RADIUS + radius
    pts = [tuple(float(v) for v in part.split()) for part in traj.split(",")]
    rx, ry = ROBOT_START
    worst = min(
        math.hypot(ax + t * (bx - ax) - rx, ay + t * (by - ay) - ry)
        for (ax, ay), (bx, by) in zip(pts, pts[1:])
        for t in [max(0.0, min(1.0,
                  (((rx - ax) * (bx - ax) + (ry - ay) * (by - ay))
                   / ((bx - ax) ** 2 + (by - ay) ** 2))))]
    )
    if worst < need:
        raise SystemExit(
            f"{name}: 궤적이 로봇 시작점({rx}, {ry})을 관통한다 — 최소 {worst:.2f} m "
            f"< 요구 {need:.2f} m. 로봇은 시작 스캔 중이라 피할 수 없으므로 "
            f"이 시나리오는 회피를 재지 못한다.")
    return worst


def build(name):
    speed, traj, thin, radius = VARIANTS[name]
    gap = check_trajectory(name, traj, radius if thin else 0.191)
    text = open(TEMPLATE).read()

    block = re.search(r"DEF PEDESTRIAN1 Pedestrian \{.*?\n\}", text, re.S)
    if block is None:
        raise SystemExit("본 월드에서 DEF PEDESTRIAN1 을 못 찾았다")

    if thin:
        new = small_person(traj, speed, radius)
    else:
        new = (block.group(0)
               .replace(re.search(r'--trajectory=[^"]*', block.group(0)).group(0),
                        f"--trajectory={traj}")
               .replace(re.search(r'--speed=[^"]*', block.group(0)).group(0),
                        f"--speed={speed}"))
        start = traj.split(",")[0].strip()
        new = re.sub(r"translation [-\d. ]+\n", f"translation {start} 1.27\n", new, count=1)

    out = text[:block.start()] + new + text[block.end():]
    path = os.path.join(ROOT, "worlds", f"people_{name}_check.wbt")
    open(path, "w").write(out)
    r = radius if thin else 0.191
    print(f"  {name:8s} 속도 {speed:4}  반경 {r:5}  시작점과 {gap:4.2f} m  경로 {traj}")
    return path


if __name__ == "__main__":
    want = os.environ.get("SAR_PEOPLE")
    names = [want] if want else list(VARIANTS)
    print("사람 시나리오 월드를 만든다 (본: stress_check.wbt)")
    for n in names:
        build(n)
