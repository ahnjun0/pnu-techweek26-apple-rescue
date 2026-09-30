"""지도·로봇·경로·프론티어·목표물을 matplotlib 창에 실시간으로 보여 준다.

받는 것: 지도 격자, pose, (선택) 경로·프론티어·목표물 목록.
내놓는 것: 화면. 값을 돌려주지 않는다.
핵심 아이디어: 창을 매번 새로 그리면 느리다. 처음 한 번만 만들고 데이터만 갈아 끼운다.
⚠️ imshow 는 반드시 origin="lower" — 안 그러면 지도가 위아래로 뒤집혀 보인다
   (row 가 y 를 따라가기 때문. common.py 좌표 규약 참고).
"""

import math
import os

import matplotlib
import matplotlib.pyplot as plt
import numpy as np

import common
import config
import mapping

# --- 한글 라벨이 네모로 깨지지 않게 폰트를 고른다 --------------------------
# 설치된 것 중 제일 먼저 찾은 것을 쓰고, 하나도 없으면 라벨을 영어로 바꾼다.
_KOREAN_FONTS = ["AppleGothic", "Apple SD Gothic Neo", "Nanum Gothic",
                 "Malgun Gothic", "Noto Sans CJK KR"]


def _setup_font():
    """쓸 수 있는 한글 폰트 이름을 돌려준다. 없으면 None."""
    import matplotlib.font_manager as font_manager
    installed = {f.name for f in font_manager.fontManager.ttflist}
    for name in _KOREAN_FONTS:
        if name in installed:
            matplotlib.rcParams["font.family"] = name
            matplotlib.rcParams["axes.unicode_minus"] = False  # 마이너스가 깨진다
            return name
    return None


HAS_KOREAN_FONT = _setup_font() is not None

# 폰트가 없으면 영어로 쓴다. 뜻은 같다.
_L = {
    "trail": "지나온 길" if HAS_KOREAN_FONT else "trail",
    "path": "계획 경로" if HAS_KOREAN_FONT else "planned path",
    "frontier": "프론티어" if HAS_KOREAN_FONT else "frontier",
    "people": "움직이는 것" if HAS_KOREAN_FONT else "moving",
    "target": "목표물" if HAS_KOREAN_FONT else "target",
    "xlabel": "x [m]  (동쪽 →)" if HAS_KOREAN_FONT else "x [m]  (east ->)",
    "ylabel": "y [m]  (북쪽 ↑)" if HAS_KOREAN_FONT else "y [m]  (north ^)",
}

# 격자를 그림으로 바꿀 때 쓰는 회색조 값
_SHADE_UNKNOWN = 0.6
_SHADE_FREE = 1.0
_SHADE_OCCUPIED = 0.0


def grid_to_image(grid):
    """log-odds 격자 → 회색조 이미지. 막힘=검정, 빈칸=흰색, 모름=회색."""
    image = np.full(grid.shape, _SHADE_UNKNOWN, dtype=np.float32)
    image[mapping.is_free(grid)] = _SHADE_FREE
    image[mapping.is_occupied(grid)] = _SHADE_OCCUPIED
    return image


def map_extent():
    """imshow 의 extent — 축 눈금을 셀 번호가 아니라 미터로 읽게 해 준다."""
    return list(common.map_bounds())


class Viz:
    """창 하나를 들고 있는 상자. 창을 못 띄우는 환경이면 조용히 아무것도 안 한다."""

    def __init__(self, title="SAR", enabled=None):
        self.ok = False
        self.trail_x = []
        self.trail_y = []

        if enabled is None:
            enabled = config.VIZ_ENABLED and os.environ.get("SAR_VIZ", "1") != "0"
        if not enabled:
            print("[viz] 화면 표시를 껐다 (SAR_VIZ=0 또는 config.VIZ_ENABLED=False).")
            return

        try:
            plt.ion()
            self.fig, self.ax = plt.subplots(figsize=(7, 7))
            self.fig.canvas.manager.set_window_title(title)
        except Exception as error:      # 헤드리스 등 — 여기서 죽으면 안 된다
            print(f"[viz] 창을 못 열었다 ({error}). 화면 표시 없이 계속한다.")
            return

        extent = map_extent()
        blank = np.full((config.MAP_HEIGHT_CELLS, config.MAP_WIDTH_CELLS),
                        _SHADE_UNKNOWN, dtype=np.float32)
        # ⚠️ origin="lower" 가 핵심이다.
        # ⚠️ **카메라가 본 범위** 를 겹쳐 그린다. 목표물은 카메라로만 찾을 수 있고
        #    (LiDAR 는 색을 모른다) "어디를 아직 안 봤나" 가 LiDAR 탐색률보다
        #    중요하다. 끝나고 저장하는 PNG 에만 넣었다가, 정작 보는 실시간 창에
        #    없어서 사용자가 "띄우기로 했잖아" 하고 짚어 줬다.
        self.camera_image = None
        self.image = self.ax.imshow(blank, cmap="gray", vmin=0.0, vmax=1.0,
                                    origin="lower", extent=extent,
                                    interpolation="nearest")

        # 한 번만 만들어 두고 나중에 데이터만 갈아 끼운다.
        (self.trail_line,) = self.ax.plot([], [], "-", color="tab:blue",
                                          linewidth=1.0, label=_L["trail"])
        (self.path_line,) = self.ax.plot([], [], "-", color="tab:orange",
                                         linewidth=2.0, label=_L["path"])
        (self.frontier_dots,) = self.ax.plot([], [], ".", color="tab:green",
                                             markersize=4, label=_L["frontier"])
        (self.people_dots,) = self.ax.plot([], [], "X", color="tab:purple",
                                           markersize=12, label=_L["people"])
        # ⚠️ **발견** 과 **도달** 은 다른 것이다. 대회 기준은 "식별하여 각 객체
        #    위치까지 **이동**" 이므로 점수는 도달이다. 화면에서 둘이 같아 보이면
        #    "3개 찾았는데 2개만 갔다" 를 눈으로 알 수 없다.
        (self.target_dots,) = self.ax.plot([], [], "*", color="red",
                                           markersize=14, label="발견 (아직 안 감)")
        (self.visited_dots,) = self.ax.plot([], [], "*", color="lime",
                                            markeredgecolor="darkgreen",
                                            markeredgewidth=1.2,
                                            markersize=18, label="도달 완료")
        (self.robot_dot,) = self.ax.plot([], [], "o", color="tab:blue",
                                         markersize=8)
        self.heading = self.ax.quiver([0], [0], [0], [0], color="tab:blue",
                                      scale=8, width=0.006)

        self.ax.set_xlabel(_L["xlabel"])
        self.ax.set_ylabel(_L["ylabel"])
        self.ax.set_aspect("equal")
        self.ax.grid(alpha=0.15)
        self.ax.legend(loc="upper right", fontsize=8, framealpha=0.85)
        self.title = self.ax.set_title(title)
        self.ok = True

    def update(self, grid, pose, path=None, frontiers=None, targets=None,
               status="", people=None, camera_seen=None, visited=None):
        """화면을 갱신한다. 인자는 전부 월드 좌표 [m] 다 (격자 좌표 아님).

        path      : [(x, y), ...] 계획 경로
        frontiers : [(x, y), ...] 가 볼 만한 곳들
        targets   : [(x, y), ...] 찾은 목표물
        """
        if not self.ok:
            return

        x, y, theta = pose

        self.trail_x.append(x)
        self.trail_y.append(y)
        if len(self.trail_x) > config.VIZ_TRAIL_MAX:
            del self.trail_x[0]
            del self.trail_y[0]

        self.image.set_data(grid_to_image(grid))
        # ⚠️ **카메라가 본 범위**. 목표물은 카메라로만 찾을 수 있으므로 (LiDAR 는
        #    색을 모른다) "어디를 아직 안 봤나" 가 LiDAR 탐색률보다 중요하다.
        #    빨강 = 빈 칸인데 아직 못 본 곳 = 목표물이 숨어 있을 수 있는 곳.
        if camera_seen is not None:
            free_now = mapping.is_free(grid)
            rgba = np.zeros(grid.shape + (4,), dtype=np.float32)
            rgba[free_now & ~camera_seen] = (1.0, 0.25, 0.25, 0.50)
            rgba[free_now & camera_seen] = (0.30, 0.70, 1.0, 0.18)
            if self.camera_image is None:
                self.camera_image = self.ax.imshow(
                    rgba, origin="lower", extent=map_extent(),
                    interpolation="nearest", zorder=2)
            else:
                self.camera_image.set_data(rgba)
        self.trail_line.set_data(self.trail_x, self.trail_y)
        self.robot_dot.set_data([x], [y])
        self.heading.set_offsets([[x, y]])
        self.heading.set_UVC([math.cos(theta)], [math.sin(theta)])

        self._set_points(self.visited_dots, visited)
        self._set_points(self.path_line, path)
        self._set_points(self.frontier_dots, frontiers)
        self._set_points(self.target_dots, targets)
        self._set_points(self.people_dots, people)

        self.title.set_text(f"{status}   pose=({x:+.2f}, {y:+.2f}) "
                            f"{math.degrees(theta):+.0f}°")
        try:
            # plt.pause 하나면 다시 그리기와 이벤트 처리를 다 한다.
            # draw_idle + flush_events 를 같이 부르면 두 번 그리게 된다.
            plt.pause(0.001)
        except Exception:
            self.ok = False     # 창을 닫았다면 조용히 포기한다

    @staticmethod
    def _set_points(artist, points):
        if points:
            artist.set_data([p[0] for p in points], [p[1] for p in points])
        else:
            artist.set_data([], [])

    def save(self, path):
        """지금 화면을 PNG 로 저장한다."""
        if not self.ok:
            return None
        self.fig.savefig(path, dpi=110, bbox_inches="tight")
        return path
