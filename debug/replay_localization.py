"""녹화(tape.npz)를 재생하며 위치 보정 방법을 Webots 없이 비교한다.

쓰는 법:  SAR_PROFILE=apartment .venv/bin/python3 debug/replay_localization.py debug/out/tape.npz [방법...]
방법: odom (보정 없음) / grid (지금 scanmatch.match) / 그 밖에 아래 METHODS

재생 방식: 녹화된 추정 위치의 **틱 사이 변화량**(오도메트리가 낸 이동)을 그대로 쓰고,
방향은 녹화값(나침반)을 쓴다. 보정기는 그 위에 위치만 고친다. 지도는 **고친 위치로**
매 틱 새로 그린다 — 실제 로봇과 같은 닫힌 고리다 (다만 주행 경로는 녹화 그대로).
정답은 녹화된 truth(Supervisor). 판단에는 쓰지 않고 오차만 잰다.
"""
import math
import sys
import time

import numpy as np

sys.path.insert(0, ".")
import config          # noqa: E402
import mapping         # noqa: E402
import scanmatch       # noqa: E402

config.apply_timestep(64)


def replay(tape, method, every=1):
    t, est, ranges, truth = tape["t"], tape["pose"], tape["ranges"], tape["truth"]
    grid = mapping.new_map()
    x, y = float(est[0][0]), float(est[0][1])
    errors = []
    fixes = 0
    for k in range(len(t)):
        if k:
            x += float(est[k][0] - est[k - 1][0])
            y += float(est[k][1] - est[k - 1][1])
        theta = float(est[k][2])
        if k > 20 and k % every == 0 and method != "odom":
            (nx, ny, _), _ = METHODS[method](grid, (x, y, theta), ranges[k])
            if (nx, ny) != (x, y):
                fixes += 1
            x, y = nx, ny
        mapping.update(grid, (x, y, theta), ranges[k])
        errors.append(math.hypot(x - truth[k][0], y - truth[k][1]))
    return np.array(errors), fixes


def grid_match(grid, pose, ranges):
    field = scanmatch.likelihood_field(grid)
    return scanmatch.match(pose, ranges, field, known=scanmatch.known_cells(grid))


METHODS = {"odom": None, "grid": grid_match}


def main():
    tape = np.load(sys.argv[1])
    methods = sys.argv[2:] or ["odom", "grid"]
    t = tape["t"]
    end = float(t[-1])
    marks = [round(end * f) for f in (0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.85)] + [end]
    print("방법      " + "".join(f"{m:>7.0f}s" for m in marks) + "    최대   보정횟수  걸린시간")
    for m in methods:
        start = time.time()
        err, fixes = replay(tape, m, every=config.SCANMATCH_EVERY)
        at = [err[min(len(err) - 1, int(np.searchsorted(t, s)))] * 100 for s in marks]
        print(f"{m:8s}" + "".join(f"{e:7.1f}cm" for e in at)
              + f"  {err.max() * 100:6.1f}cm  {fixes:6d}  {time.time() - start:5.1f}s")



def _with(overrides, fn):
    def run(grid, pose, ranges):
        saved = {k: getattr(config, k) for k in overrides}
        for k, v in overrides.items():
            setattr(config, k, v)
        try:
            return fn(grid, pose, ranges)
        finally:
            for k, v in saved.items():
                setattr(config, k, v)
    return run


def fine_match(grid, pose, ranges):
    field = scanmatch.likelihood_field(grid)
    return scanmatch.match_fine(pose, ranges, field, known=scanmatch.known_cells(grid))


METHODS["fine"] = fine_match
METHODS["fine_half"] = _with({"SCANMATCH_FINE_BLEND": 0.5}, fine_match)
METHODS["grid_noturn"] = _with({"SCANMATCH_TURN": 0.0, "SCANMATCH_TURN_STEPS": 0}, grid_match)


if __name__ == "__main__":
    main()
