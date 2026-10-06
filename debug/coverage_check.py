"""카메라 커버리지 — 집의 몇 % 를 언제 카메라로 훑었나. 사과가 어디 있든 상관없이 탐색 실력을 잰다.

채점 실행 폴더(debug/mission_check.py 가 SAR_OUT 에 남긴 곳)의 tape.npz 와 trace.csv 를 다시 돌린다.
  1. 틱마다 추정 위치와 LiDAR 로 로봇이 그때까지 그린 지도를 다시 그린다.
  2. 그 지도 위에 카메라 시야를 찍어 칸마다 처음 보인 시각을 적는다.
  3. 공통 기준 지도의 빈 칸 중 얼마를 언제 봤는지 센다.
Webots 없이 돈다.

쓰는 법:
  uv run python debug/coverage_check.py <실행 폴더> [<실행 폴더> ...]
  uv run python debug/coverage_check.py --make-reference <저장할 .npz> <실행 폴더> ...

⚠️ 실행마다 지도가 달라 각자의 지도로 세면 서로 비교가 안 된다. 그래서 기준 지도 하나로 센다.
   docs/data/coverage_reference.npz 는 네 실행의 마지막 지도에서 둘 이상이 빈 칸이라고 한 칸이다
   (106 m²). 네 실행은 9/30 발표본 기록(docs/data/2026-09-30_발표본기록), main 대회 조건,
   detour-hold 의 3091f2d, detour-hold 의 f117ecb 다.
⚠️ 로봇이 끝낼 때까지 실제로 훑은 비율과는 다르다. 로봇은 사과를 다 찾으면 바로 돌아가므로,
   "다 찾은 순간 훑은 비율" 은 사과를 만난 순서에 크게 좌우된다. 실력 비교는 50%·70% 도달 시각으로 한다.
"""
import argparse
import csv
import math
import os
import sys

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, REPO)

import numpy as np

from sar import common
from sar import config
from sar import mapping

REFERENCE = os.path.join(REPO, "docs", "data", "coverage_reference.npz")
# 대회 월드 카메라의 fieldOfView (worlds/apartment_competition_check.wbt, 로봇은 camera.getFov() 로 같은 값을 받는다)
CAMERA_FOV = 1.0472
# 카메라 시야는 4틱(0.256초)마다 찍는다 — 성능 때문이다. 로봇은 64 ms 마다 찍는다.
# README 의 숫자는 이 값으로 쟀다.
CAMERA_EVERY = 4
LEVELS = (0.5, 0.7, 0.8, 0.9)
APPROACH_MERGE = 10.0     # [s] 이 안에 다시 들어간 APPROACH 는 같은 사과로 본다 (들락날락)


def replay(run):
    """(마지막 지도, 칸마다 카메라에 처음 보인 시각, 마지막 시각) — 로봇이 그린 지도를 다시 그린다."""
    tape = np.load(os.path.join(run, "tape.npz"))
    times, poses, ranges = tape["t"], tape["pose"], tape["ranges"]
    grid = mapping.new_map()
    first = np.full(grid.shape, np.inf)
    reach = common.to_cells(config.DETECT_MAX_RANGE)
    for k in range(len(times)):
        mapping.update(grid, tuple(poses[k]), ranges[k])
        if k % CAMERA_EVERY == 0:
            seen = np.zeros(grid.shape, dtype=bool)
            mapping.mark_camera_seen(seen, grid, tuple(poses[k]), CAMERA_FOV, reach)
            first[seen & np.isinf(first)] = times[k]
    return grid, first, float(times[-1])


def events(run):
    """trace.csv 에서 사과로 다가가기 시작한 시각들과, 사과를 다 찾고 복귀를 시작한 시각."""
    approach, done, before, last = [], None, None, -math.inf
    with open(os.path.join(run, "trace.csv"), encoding="utf-8") as f:
        for row in csv.DictReader(f):
            t = float(row["t"])
            if row["state"] == "APPROACH" and before != "APPROACH":
                if t - last > APPROACH_MERGE:
                    approach.append(t)
                last = t
            if done is None and row["state"] == "RETURN" and "다 찾았다" in row["status"]:
                done = t
            before = row["state"]
    return approach, done


def report(run, reference):
    _, first, end = replay(run)
    seen_at = np.sort(first[reference])
    cover = lambda t: float(np.mean(seen_at <= t))

    def reach_time(level):
        when = seen_at[int(math.ceil(level * len(seen_at))) - 1]
        return f"{when:5.0f}초" if np.isfinite(when) else "    -  "
    approach, done = events(run)
    print(f"{os.path.basename(os.path.normpath(run))}")
    print(f"  끝 {end:6.1f}초, 그때까지 훑은 비율 {cover(end):.0%}")
    print("  " + "  ".join(f"{level:.0%} 도달 {reach_time(level)}" for level in LEVELS))
    print(f"  사과로 다가가기 시작 {[round(t) for t in approach]}"
          + (f", 다 찾고 복귀 {done:.1f}초 (그때 훑은 비율 {cover(done):.0%})" if done else ", 다 찾지 못함"))


def make_reference(runs, out):
    grids = [replay(run)[0] for run in runs]
    free = sum(mapping.is_free(g).astype(int) for g in grids) >= math.ceil(len(grids) / 2)
    np.savez_compressed(out, free=free, runs=np.array([os.path.basename(os.path.normpath(r)) for r in runs]))
    print(f"기준 지도 {out}: 빈 칸 {int(free.sum())} ({free.sum() * common.cell_area():.0f} m²)")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="채점 실행 기록으로 카메라 커버리지를 잰다")
    parser.add_argument("runs", nargs="+", help="tape.npz 와 trace.csv 가 있는 실행 폴더")
    parser.add_argument("--reference", default=REFERENCE, help="공통 기준 지도 (.npz)")
    parser.add_argument("--make-reference", metavar="OUT", help="주어진 실행들로 기준 지도를 만들어 OUT 에 쓴다")
    args = parser.parse_args()
    config.apply_timestep(64)       # 대회 월드 주기 (debug/mission_check.py 와 같다)
    if args.make_reference:
        make_reference(args.runs, args.make_reference)
        sys.exit(0)
    reference = np.load(args.reference)["free"]
    print(f"기준 지도: 빈 칸 {int(reference.sum())} ({reference.sum() * common.cell_area():.0f} m²)")
    for run in args.runs:
        report(run, reference)
