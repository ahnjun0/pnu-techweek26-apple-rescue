"""기록(trace.csv + tape.npz)에서 '좁은 곳 앞에서 버벅이는' 구간을 찾아 원인을 보여 준다.

쓰는 법:  .venv/bin/python3 debug/doorway_report.py debug/out/ab_XXXX_이름

버벅임 = 2초 동안 평균 전진 속도 < 0.05 m/s 이고 회전 방향이 6번 이상 뒤바뀐 구간.
그 자리에서:
  통로 폭  — LiDAR 로 본 왼쪽(60~120°)·오른쪽(-120~-60°) 최근접 거리의 합
  DWA     — 전진 후보 중 평소 여유(로봇 반경+0.15 m)로 안전한 것의 비율, 좁은 여유 비상구를 쓴 비율
"""
import csv
import math
import sys

import numpy as np

sys.path.insert(0, ".")
import common  # noqa: E402


def main():
    folder = sys.argv[1]
    rows = list(csv.DictReader(open(folder + "/trace.csv", encoding="utf-8")))
    if "dwa_fwd" not in rows[0]:
        sys.exit("이 기록에는 DWA 칸이 없다 — 새 mission_check 로 다시 돌린다")
    tape = np.load(folder + "/tape.npz")
    angles = np.degrees(common.lidar_angles())
    left = (angles > 60) & (angles < 120)
    right = (angles < -60) & (angles > -120)
    t = np.array([float(r["t"]) for r in rows])
    v = np.array([float(r["v"]) for r in rows])
    w = np.array([float(r["w"]) for r in rows])
    episodes = []
    k = 0
    while k < len(rows):
        j = np.searchsorted(t, t[k] + 2.0)
        if j >= len(rows):
            break
        flips = int(np.sum(np.sign(w[k:j - 1]) * np.sign(w[k + 1:j]) < 0))
        if v[k:j].mean() < 0.05 and flips >= 6 and rows[k]["state"] != "SCAN":
            episodes.append((k, j, flips))
            k = j
        else:
            k += 1
    print(f"버벅임 {len(episodes)} 구간")
    for k, j, flips in episodes:
        seg = rows[k:j]
        ti = np.searchsorted(tape["t"], t[k])
        ranges = tape["ranges"][min(ti, len(tape["t"]) - 1)]
        ok = np.isfinite(ranges)
        width = float(np.min(ranges[left & ok], initial=9)) + float(np.min(ranges[right & ok], initial=9))
        fwd = sum(int(r["dwa_fwd"]) for r in seg)
        safe = sum(int(r["dwa_safe_fwd"]) for r in seg)
        sq = sum(int(r["dwa_squeeze"]) for r in seg) / len(seg)
        near = min(float(r["near_d"]) for r in seg)
        print(f"  {t[k]:6.1f}~{t[j]:6.1f}s 정답({float(seg[0]['gx']):+.2f},{float(seg[0]['gy']):+.2f})"
              f" 통로폭 {width:.2f} m, 최근접 {near:.2f} m, 회전 뒤바뀜 {flips}회,"
              f" 안전한 전진 후보 {safe}/{fwd} ({safe / max(fwd, 1):.0%}), 비상구 {sq:.0%}"
              f", 경로 좁은여유 {seg[0]['squeeze']} | {seg[0]['status'][:30]}")


if __name__ == "__main__":
    main()
