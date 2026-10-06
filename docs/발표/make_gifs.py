"""주행 기록(tape.npz, trace.csv, frames.npz)으로 발표용 GIF 를 만든다. Webots 없이.

위에서 내려다본 지도: 로봇이 그 순간까지 그린 점유 격자(기록을 재생해 다시 그린다)
+ 실제 경로(정답) + 로봇 추정 경로 + 보행자 + 빨간 사과 + 카펫 영역(표시용).

쓰는 법:  python docs/발표/make_gifs.py --run-best <채점 실행 폴더> --run-carpet <채점 실행 폴더> [full carpet apple ped retrace]
          python docs/발표/make_gifs.py --run-best <채점 실행 폴더> best1006 ped1006
  실행 폴더 = debug/mission_check.py 가 SAR_OUT 에 남긴 tape.npz·trace.csv·frames.npz 가 있는 곳.
  발표 GIF 는 9/30 대회 설정 실행 두 개(포기 전 풀기, 바퀴 반지름 보정 전)로 만들었다.
  best1006·ped1006 은 2026-10-06 대회 조건 실행(detour-hold f117ecb, 2/2, 509.2초)용이다.

⚠️ 보행자 위치는 월드 파일의 Pedestrian 궤적·속도로 **계산** 한다 (pedestrian_clock).
   tape.npz 의 person 은 "첫 번째 움직이는 물체" 인데 apartment 에서는 탁자 위 노트북이었다 —
   그래서 9/30 GIF 의 "보행자" 는 노트북 자리에 서 있었다 (2026-10-06 확인).
"""
import argparse
import csv
import math
import os
import sys

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, REPO)

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager
import numpy as np
from PIL import Image

from sar import common
from sar import config
from sar import mapping
from sar import detect

config.apply_timestep(64)
OUT = os.path.join(REPO, "docs", "발표", "assets", "gif")   # --out 으로 바꾼다
PED, DELAY = None, 0.0     # 보행자 시계와 출발 지연 (--world, --delay 로 정한다)

KFONT = "Apple SD Gothic Neo"
if KFONT not in {f.name for f in font_manager.fontManager.ttflist}:
    KFONT = "AppleGothic"
plt.rcParams["font.family"] = KFONT
plt.rcParams["axes.unicode_minus"] = False

APPLES = [(-12.02, -3.02), (-5.34, -10.54)]
CARPET = (-6.6, -3.76, 1.6, 2.4)          # x, y, w, h (표시용, 월드 파일 값)
C_TRUE, C_EST, C_PED, C_RED = "#117A65", "#6C3483", "#E67E22", "#C0392B"


PERSON_RADIUS = 0.191                      # debug/mission_check.py 와 같은 값 (접촉 판정용)
TOUCH = 0.13 + PERSON_RADIUS               # 로봇 0.13 + 보행자 — 중심이 이보다 가까우면 "닿음"


def pedestrian_clock(world):
    """월드 파일의 Pedestrian 궤적·속도로 "시각 → (x, y)" 함수를 만든다. 보행자가 없으면 None.

    Webots projects/humans/pedestrian/controllers/pedestrian/pedestrian.py 91~136줄과 같은 식이다
    (닫힌 꺾은선을 일정한 속도로 돈다). 보행자 컨트롤러는 우리보다 한 틱 늦게 움직인다 — 한 틱
    당기면 실행 7개의 "가장 가까웠던 거리" 와 3 mm 안으로 맞았다 (그래서 호출하는 쪽이 한 틱 뺀다).
    """
    import re
    text = open(world, encoding="utf-8").read()
    block = re.search(r"\nPedestrian \{(.*?)\n\}", text, re.S)
    if block is None:
        return None
    speed = float(re.search(r'"--speed=([0-9.]+)"', block.group(1)).group(1))
    route = re.search(r'"--trajectory=([^"]+)"', block.group(1)).group(1)
    points = [tuple(map(float, p.split())) for p in route.split(",")]
    n = len(points)
    reach = np.cumsum([math.dist(points[i], points[(i + 1) % n]) for i in range(n)])

    def at(t):
        rel = (t * speed) % reach[-1]
        i = int(np.searchsorted(reach, rel, side="right"))
        start = reach[i - 1] if i else 0.0
        ratio = (rel - start) / (reach[i] - start)
        (ax_, ay_), (bx_, by_) = points[i], points[(i + 1) % n]
        return ax_ + (bx_ - ax_) * ratio, ay_ + (by_ - ay_) * ratio
    return at


def load(run):
    tape = np.load(run + "/tape.npz")
    rows = list(csv.DictReader(open(run + "/trace.csv")))
    status_t = np.array([float(r["t"]) for r in rows])
    status = [r["status"] for r in rows]
    return tape, status_t, status


def status_at(status_t, status, t):
    i = int(np.clip(np.searchsorted(status_t, t), 0, len(status) - 1))
    s = status[i]
    for a, b in (("EXPLORE — ", ""), ("RETURN — ", "복귀 "), ("APPROACH — ", "접근 "), (" — ", " - ")):
        s = s.replace(a, b)
    return s[:38]


def map_rgb(grid):
    img = np.full(grid.shape + (3,), 0.86)
    img[mapping.is_free(grid)] = 1.0
    img[mapping.is_occupied(grid)] = 0.15
    return img


def extent():
    return list(common.map_bounds())


def scan_points(pose, ranges):
    x, y, th = pose
    a = np.asarray(common.lidar_angles())
    r = np.asarray(ranges)
    ok = np.isfinite(r) & (r >= config.LIDAR_MIN_RANGE) & (r <= 3.0)
    return x + r[ok] * np.cos(th + a[ok]), y + r[ok] * np.sin(th + a[ok])


def topdown(run, t0, t1, step, name, view, title, show_est=False, fps=12, size=(7.2, 5.4), trail_from=None, faint_from=None,
            hold=0):
    tape, st_t, st = load(run)
    T, P, R, TR = tape["t"], tape["pose"], tape["ranges"], tape["truth"]
    t1 = T[-1] if t1 is None else t1
    lag = config.TIME_STEP / 1000.0            # 보행자는 한 틱 늦다 (pedestrian_clock 설명)
    PE = np.array([PED(t + DELAY - lag) for t in T]) if PED else None
    grid = mapping.new_map()
    frames, next_t = [], t0
    trail_from = t0 if trail_from is None else trail_from
    for k in range(len(T)):
        mapping.update(grid, tuple(P[k]), R[k])
        if T[k] > t1:
            break
        final = k == len(T) - 1 or T[k + 1] > t1      # 마지막 장면은 간격과 상관없이 넣는다
        if T[k] < next_t and not final:
            continue
        next_t = T[k] + step
        dx, dy = view[1] - view[0], view[3] - view[2]
        fig, ax = plt.subplots(figsize=(size[1] * dx / dy + 0.3, size[1] + 0.4), dpi=100)
        ax.imshow(map_rgb(grid), origin="lower", extent=extent(), interpolation="nearest")
        cx, cy, cw, ch = CARPET
        ax.add_patch(plt.Rectangle((cx, cy), cw, ch, fill=True, color="#A0522D", alpha=0.18, lw=0))
        if view[0] <= cx + 0.1 <= view[1] and view[2] <= cy + ch - 0.3 <= view[3]:
            ax.text(cx + 0.08, cy + ch - 0.28, "카펫", fontsize=10, color="#A0522D", zorder=3)
        for ax_, ay_ in APPLES:
            ax.add_patch(plt.Circle((ax_, ay_), 0.12, color=C_RED, zorder=5))
        if faint_from is not None:
            old_sel = (T >= faint_from) & (T <= min(T[k], trail_from))
            ax.plot(TR[old_sel, 0], TR[old_sel, 1], color=C_TRUE, lw=1.0, alpha=0.3)
        sel = (T >= trail_from) & (T <= T[k])
        ax.plot(TR[sel, 0], TR[sel, 1], color=C_TRUE, lw=2.2, label="실제 경로")
        if show_est:
            ax.plot(P[sel, 0], P[sel, 1], color=C_EST, lw=1.6, ls="--", label="로봇이 믿는 경로")
        sx, sy = scan_points(P[k], R[k])
        ax.scatter(sx, sy, s=2, color="#2E86C1", alpha=0.6, zorder=4)
        rx, ry = TR[k]
        if PE is not None:
            px, py = PE[k]
            past = (T >= T[k] - 10.0) & (T <= T[k])
            ax.plot(PE[past, 0], PE[past, 1], color=C_PED, lw=1.6, alpha=0.45, zorder=5)
            near = math.hypot(px - rx, py - ry) < TOUCH
            ax.add_patch(plt.Circle((px, py), PERSON_RADIUS, color=C_PED, alpha=0.9, zorder=6,
                                    ec=C_RED if near else "none", lw=3 if near else 0))
            j = max(0, k - 16)                      # 1초쯤 전 자리 → 진행 방향
            hx, hy = px - PE[j][0], py - PE[j][1]
            if math.hypot(hx, hy) > 1e-6:
                u = 0.35 / math.hypot(hx, hy)
                ax.annotate("", xy=(px + hx * u, py + hy * u), xytext=(px, py),
                            arrowprops=dict(arrowstyle="->", color=C_PED, lw=2), zorder=6)
            if view[0] <= px <= view[1] and view[2] <= py <= view[3]:
                ax.text(px + 0.3, py + 0.1, "보행자 (스침)" if near else "보행자", fontsize=10,
                        color=C_RED if near else C_PED, weight="bold", zorder=9)
        th = P[k][2]
        ax.add_patch(plt.Circle((rx, ry), 0.11, color=C_TRUE, zorder=7))
        ax.plot([rx, rx + 0.3 * math.cos(th)], [ry, ry + 0.3 * math.sin(th)], color="white", lw=2, zorder=8)
        ax.set_xlim(view[0], view[1]); ax.set_ylim(view[2], view[3])
        ax.set_aspect("equal"); ax.set_xticks([]); ax.set_yticks([])
        ax.set_title(f"{title}    t = {T[k]:5.1f} s    {status_at(st_t, st, T[k])}", fontsize=11, loc="left")
        if show_est:
            err = math.hypot(P[k][0] - rx, P[k][1] - ry) * 100
            ax.text(0.02, 0.03, f"위치 오차 {err:4.0f} cm", transform=ax.transAxes, fontsize=13,
                    color=C_EST, weight="bold", bbox=dict(facecolor="white", alpha=0.85, lw=0))
            ax.legend(loc="upper right", fontsize=9, framealpha=0.9)
        fig.tight_layout()
        fig.canvas.draw()
        frames.append(Image.fromarray(np.asarray(fig.canvas.buffer_rgba())[..., :3]))
        plt.close(fig)
    frames += frames[-1:] * (hold * fps)            # 끝 장면을 hold 초 붙잡는다
    save(frames, name, fps)


def camera(run, t0, t1, name, fps=6):
    f = np.load(run + "/frames.npz")
    T, IM = f["t"], f["image"]
    frames = []
    for i in np.flatnonzero((T >= t0) & (T <= t1)):
        img = IM[i][:, :, ::-1].copy()          # BGR -> RGB
        fig, ax = plt.subplots(figsize=(6.4, 4.8), dpi=100)
        ax.imshow(img); ax.set_xticks([]); ax.set_yticks([])
        for b in detect.blob_boxes(IM[i]):
            if b["edge"]:
                continue
            m = detect.camera_range(b, 640, 480, 1.0472)
            ok = (m is not None and abs(m[0] - m[1]) <= config.DETECT_RANGE_AGREEMENT * m[1]
                  and config.DETECT_MIN_ASPECT * b["w"] <= b["h"] <= config.DETECT_MAX_ASPECT * b["w"])
            col = "#2ECC71" if ok else "#E74C3C"
            ax.add_patch(plt.Rectangle((b["left"], b["top"]), b["w"], b["h"], fill=False, color=col, lw=2.5))
            if m is not None:
                ax.text(b["left"], b["top"] - 6, f"크기 {m[0]:.2f} m / 바닥 {m[1]:.2f} m",
                        color="white", fontsize=10, bbox=dict(facecolor=col, alpha=0.85, lw=0))
        ax.set_title(f"로봇 카메라    t = {T[i]:5.1f} s", fontsize=11, loc="left")
        fig.tight_layout(); fig.canvas.draw()
        frames.append(Image.fromarray(np.asarray(fig.canvas.buffer_rgba())[..., :3]))
        plt.close(fig)
    save(frames, name, fps)


def save(frames, name, fps):
    if not frames:
        print("프레임 없음:", name); return
    small = [fr.convert("P", palette=Image.ADAPTIVE, colors=128) for fr in frames]
    path = os.path.join(OUT, name)
    small[0].save(path, save_all=True, append_images=small[1:], duration=int(1000 / fps), loop=0, optimize=True)
    print(f"{name}: {len(frames)}장, {os.path.getsize(path) / 1e6:.1f} MB")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="채점 실행 기록으로 발표용 GIF 를 만든다")
    parser.add_argument("which", nargs="*", default=["full", "carpet", "apple", "ped", "retrace"])
    parser.add_argument("--run-best", required=True, help="전체 주행·사과·보행자·되짚기 장면 실행 폴더")
    parser.add_argument("--run-carpet", help="카펫 미끄러짐 장면 실행 폴더 (carpet 에만 필요)")
    parser.add_argument("--out", default=OUT, help="GIF 를 쓸 폴더")
    parser.add_argument("--world", default=os.path.join(REPO, "worlds", "apartment_competition_check.wbt"),
                        help="보행자 궤적을 읽을 월드 파일")
    parser.add_argument("--delay", type=float, default=0.0, help="그 실행의 SAR_START_DELAY [s]")
    args = parser.parse_args()
    which, RUN_BEST, RUN_CARPET, OUT = args.which, args.run_best, args.run_carpet, args.out
    if "carpet" in which and not RUN_CARPET:
        parser.error("carpet 에는 --run-carpet 이 필요하다")
    PED, DELAY = pedestrian_clock(args.world), args.delay
    os.makedirs(OUT, exist_ok=True)
    WHOLE = (-13.2, 1.2, -13.8, 0.6)
    if "full" in which:
        topdown(RUN_BEST, 0, 483, 2.0, "gif1_full.gif", WHOLE, "전체 주행", fps=15, trail_from=0, hold=2)
    if "carpet" in which:
        topdown(RUN_CARPET, 296, 312, 0.25, "gif2_carpet.gif", (-8.0, -4.4, -4.6, -0.6),
                "카펫 턱 미끄러짐", show_est=True, fps=10, trail_from=280, hold=1)
    if "apple" in which:
        camera(RUN_BEST, 358, 375, "gif3_apple_camera.gif")
    if "ped" in which:
        # ⚠️ 예전 장면(326~345초)에는 진짜 보행자가 없었다 — 노트북을 보행자로 그렸다 (머리말 참고).
        #    이 실행에서 보행자를 실제로 만난 것은 415~437초다 (최소 0.39 m, 복귀 중).
        topdown(RUN_BEST, 408, 440, 0.25, "gif4_pedestrian.gif", (-7.0, -3.4, -4.6, -1.0),
                "보행자와 엇갈림 (복귀 중)", fps=10, trail_from=395, hold=1)
    if "best1006" in which:
        topdown(RUN_BEST, 0, None, 2.0, "gif6_full_1006.gif", WHOLE, "전체 주행 (10/6)", fps=15,
                trail_from=0, hold=2)
    if "ped1006" in which:
        topdown(RUN_BEST, 476, 492, 0.25, "gif7_pedestrian_1006.gif", (-6.8, -3.4, -9.0, -6.0),
                "보행자와 스침 (10/6)", fps=10, trail_from=440, hold=1)
    if "retrace" in which:
        topdown(RUN_BEST, 414, 452, 0.4, "gif5_retrace.gif", (-7.4, -2.9, -6.4, -0.8),
                "지나온 길 되짚기", fps=10, trail_from=414, faint_from=0, hold=1)
