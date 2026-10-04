"""주행 기록(tape.npz, trace.csv, frames.npz)으로 발표용 GIF 를 만든다. Webots 없이.

위에서 내려다본 지도: 로봇이 그 순간까지 그린 점유 격자(기록을 재생해 다시 그린다)
+ 실제 경로(정답) + 로봇 추정 경로 + 보행자 + 빨간 사과 + 카펫 영역(표시용).

쓰는 법:  python docs/발표/make_gifs.py --run-best <채점 실행 폴더> --run-carpet <채점 실행 폴더> [full carpet apple ped retrace]
  실행 폴더 = debug/mission_check.py 가 SAR_OUT 에 남긴 tape.npz·trace.csv·frames.npz 가 있는 곳.
  발표 GIF 는 9/30 대회 설정 실행 두 개(포기 전 풀기, 바퀴 반지름 보정 전)로 만들었다.
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

KFONT = "Apple SD Gothic Neo"
if KFONT not in {f.name for f in font_manager.fontManager.ttflist}:
    KFONT = "AppleGothic"
plt.rcParams["font.family"] = KFONT
plt.rcParams["axes.unicode_minus"] = False

APPLES = [(-12.02, -3.02), (-5.34, -10.54)]
CARPET = (-6.6, -3.76, 1.6, 2.4)          # x, y, w, h (표시용, 월드 파일 값)
C_TRUE, C_EST, C_PED, C_RED = "#117A65", "#6C3483", "#E67E22", "#C0392B"


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


def topdown(run, t0, t1, step, name, view, title, show_est=False, fps=12, size=(7.2, 5.4), trail_from=None, faint_from=None):
    tape, st_t, st = load(run)
    T, P, R, TR, PE = tape["t"], tape["pose"], tape["ranges"], tape["truth"], tape["person"]
    grid = mapping.new_map()
    frames, next_t = [], t0
    trail_from = t0 if trail_from is None else trail_from
    for k in range(len(T)):
        mapping.update(grid, tuple(P[k]), R[k])
        if T[k] < next_t:
            continue
        if T[k] > t1:
            break
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
        ax.add_patch(plt.Circle(tuple(PE[k]), 0.25, color=C_PED, alpha=0.9, zorder=6))
        if view[0] <= PE[k][0] <= view[1] and view[2] <= PE[k][1] <= view[3]:
            ax.text(PE[k][0] + 0.3, PE[k][1] + 0.1, "보행자", fontsize=10, color=C_PED, weight="bold", zorder=9)
        rx, ry = TR[k]
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
    parser.add_argument("--run-carpet", required=True, help="카펫 미끄러짐 장면 실행 폴더")
    parser.add_argument("--out", default=OUT, help="GIF 를 쓸 폴더")
    args = parser.parse_args()
    which, RUN_BEST, RUN_CARPET, OUT = args.which, args.run_best, args.run_carpet, args.out
    os.makedirs(OUT, exist_ok=True)
    WHOLE = (-13.2, 1.2, -13.8, 0.6)
    if "full" in which:
        topdown(RUN_BEST, 0, 483, 2.0, "gif1_full.gif", WHOLE, "전체 주행", fps=15, trail_from=0)
    if "carpet" in which:
        topdown(RUN_CARPET, 296, 312, 0.25, "gif2_carpet.gif", (-8.0, -4.4, -4.6, -0.6),
                "카펫 턱 미끄러짐", show_est=True, fps=10, trail_from=280)
    if "apple" in which:
        camera(RUN_BEST, 358, 375, "gif3_apple_camera.gif")
    if "ped" in which:
        topdown(RUN_BEST, 326, 345, 0.25, "gif4_pedestrian.gif", (-8.0, -3.0, -6.0, -1.0),
                "보행자와 교차", fps=10, trail_from=310)
    if "retrace" in which:
        topdown(RUN_BEST, 414, 452, 0.4, "gif5_retrace.gif", (-7.4, -2.9, -6.4, -0.8),
                "지나온 길 되짚기", fps=10, trail_from=414, faint_from=0)
