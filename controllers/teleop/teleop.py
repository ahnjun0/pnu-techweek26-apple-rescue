"""키보드로 직접 몰면서 지도가 제대로 그려지는지 눈으로 보는 도구. (Phase 2 검증)

받는 것: 화살표 키. 내놓는 것: matplotlib 창에 실시간 지도.
핵심 아이디어: 자동 주행을 붙이기 전에 "센서 → 오도메트리 → 지도" 사슬만 따로 확인한다.
여기서 지도가 뒤집히거나 벽이 두 겹으로 번지면, 그건 매핑 문제지 탐색 문제가 아니다.
조작: ↑↓ 전후, ←→ 회전, 스페이스 정지, S 저장, R 지도 리셋
"""

import os
import sys
import time

_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

import boot  # noqa: E402  (config 보다 먼저 — 대회 설정·라이브러리 확인)
boot.start("teleop")

from controller import Keyboard, Robot

import config
import localization
import mapping
import sensors as sensors_mod
import viz as viz_mod

HELP = """
------------------------------------------------------------------
  조작법   (반드시 Webots 3D 화면을 한 번 클릭해서 포커스를 줄 것)
------------------------------------------------------------------
    ↑ / ↓      앞으로 / 뒤로
    ← / →      왼쪽 / 오른쪽 회전  (↑ 와 같이 누르면 곡선 주행)
    스페이스    정지
    S          지금 지도를 PNG 로 저장
    R          지도 지우고 처음부터 다시
------------------------------------------------------------------
  볼 것: 지도가 월드와 같은 모양인가 / 위아래나 좌우로 뒤집히지 않았는가
         제자리에서 한 바퀴 돌았을 때 벽이 두 겹으로 번지지 않는가
------------------------------------------------------------------
"""


def read_keys(keyboard):
    """지금 눌려 있는 키들을 집합으로 돌려준다.

    getKey 는 -1 이 나올 때까지 여러 번 불러야 동시에 눌린 키를 다 준다
    (keyboard.md — 최대 7개).
    """
    keys = set()
    while True:
        key = keyboard.getKey()
        if key == -1:
            break
        keys.add(key & Keyboard.KEY)   # 시프트 등 수식키 비트를 떼어 낸다
    return keys


def main():
    robot = Robot()
    timestep = int(robot.getBasicTimeStep())
    dt = timestep / 1000.0

    sensors = sensors_mod.Sensors(robot)          # GPS 는 쓰지 않는다
    keyboard = Keyboard()
    keyboard.enable(timestep)

    odometry = localization.Odometry()
    grid = mapping.new_map()
    display = viz_mod.Viz("teleop — 수동 조종 지도 작성")

    # 당일 로봇이 바뀌었는데 config 를 안 고쳤으면 여기서 바로 드러난다.
    sensors.verify_against_config()
    print(HELP)
    sys.stdout.flush()

    tick = 0
    while robot.step(timestep) != -1:
        tick += 1

        # --- 1. 센서 읽고 내 위치 갱신 ---------------------------------
        left, right = sensors.read_encoders()
        odometry.update(left, right, dt,
                        compass_values=sensors.read_compass())
        pose = odometry.pose

        # --- 2. 키 처리 -------------------------------------------------
        keys = read_keys(keyboard)
        speed = 0.0
        turn = 0.0
        if Keyboard.UP in keys:
            speed += config.TELEOP_SPEED
        if Keyboard.DOWN in keys:
            speed -= config.TELEOP_SPEED
        if Keyboard.LEFT in keys:
            turn += config.TELEOP_TURN
        if Keyboard.RIGHT in keys:
            turn -= config.TELEOP_TURN
        if ord(' ') in keys:
            speed = turn = 0.0

        if ord('S') in keys:
            name = f"map_{time.strftime('%H%M%S')}.png"
            saved = display.save(name)
            print(f"[teleop] 지도를 저장했다: {saved}")
            sys.stdout.flush()
        if ord('R') in keys:
            grid = mapping.new_map()
            print("[teleop] 지도를 지웠다.")
            sys.stdout.flush()

        sensors.drive(speed, turn)

        # --- 3. 지도 갱신 (매 틱이 아니라 가끔) --------------------------
        if tick % config.MAP_UPDATE_EVERY == 0:
            mapping.update(grid, pose, sensors.read_lidar())

        # --- 4. 화면 갱신 (더 가끔) --------------------------------------
        if tick % config.VIZ_UPDATE_EVERY == 0:
            display.update(grid, pose, status="teleop")


if __name__ == "__main__":
    main()
