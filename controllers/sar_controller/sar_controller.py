"""메인 컨트롤러. 다른 모듈을 조립만 한다 — 여기에 로직을 쓰지 말 것.

받는 것: Webots 로봇.
내놓는 것: 바퀴 명령. 그리고 matplotlib 창에 지도·경로·프론티어.
핵심 아이디어: 센서 읽기(sensors) → 내 위치(localization) → 판단(mission) → 바퀴.
판단은 전부 mission.py 에 있고 Webots 를 모른다. 그래서 pytest 로 따로 검증할 수 있다.
⚠️ GPS 는 여기서 절대 쓰지 않는다 (CLAUDE.md 규칙 1).
"""

import os
import sys

_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

import boot  # noqa: E402  (config 보다 먼저 — 대회 설정·라이브러리 확인)
boot.start("sar_controller")

from controller import Robot

import config
import localization
import mission as mission_mod
import sensors as sensors_mod
import viz as viz_mod


def main():
    robot = Robot()
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

    sensors = sensors_mod.Sensors(robot)          # GPS 는 켜지 않는다
    odometry = localization.Odometry()
    brain = mission_mod.Mission(odometry.pose)
    if config.DETECT_YOLO:
        import yolo_check
        brain.classify = yolo_check.load()   # 모델이 없으면 여기서 크게 알리고 멈춘다
    display = viz_mod.Viz("sar — 자율 탐색")
    camera_fov = sensors.camera.getFov()

    print("=" * 62)
    print("[sar_controller] 자율 탐색 시작")
    print("=" * 62)
    sensors.verify_against_config()
    print(f"\n  시작 pose: ({config.START_X:+.2f}, {config.START_Y:+.2f}, "
          f"{config.START_THETA:+.2f} rad)")
    print("  상태가 바뀔 때마다 한 줄씩 찍는다.\n")
    sys.stdout.flush()

    tick = 0
    last_status = None
    announced_done = False
    while robot.step(timestep) != -1:
        tick += 1

        # 1. 내가 어디 있는가
        left, right = sensors.read_encoders()
        odometry.update(left, right, dt, compass_values=sensors.read_compass())
        pose = odometry.pose

        # 2. 무엇을 할 것인가 (Webots 를 모르는 순수 파이썬)
        #    카메라는 매 틱 볼 필요가 없다 — 목표물은 갑자기 나타나지 않는다.
        image = (sensors.read_camera_bgr()
                 if tick % config.DETECT_EVERY == 0 else None)
        speed, turn = brain.step(pose, sensors.read_lidar(), dt,
                                 image=image, camera_fov=camera_fov)

        # 3. 바퀴에 넣는다
        sensors.drive(speed, turn)

        # 스캔 정합이 위치를 고쳐 줬으면 오도메트리에 되먹인다.
        # 이게 없으면 다음 틱에 또 틀어진 자리에서 누적을 이어간다.
        if brain.pose_fix is not None:
            odometry.correct(brain.pose_fix)

        # 4. 사람이 보는 것들
        if brain.status != last_status:
            print(f"  [{brain.elapsed:6.1f}s] {brain.status}")
            sys.stdout.flush()
            last_status = brain.status

        if tick % config.VIZ_UPDATE_EVERY == 0:
            display.update(brain.grid, pose,
                           path=brain.path,
                           frontiers=brain.frontier_points(),
                           targets=brain.target_points(),
                           status=brain.state,
                           people=brain.people_points(),
                           camera_seen=brain.camera_seen,
                           visited=brain.visited_points())

        if brain.state == mission_mod.DONE:
            sensors.stop()
            if not announced_done:
                announced_done = True
                print("[[DONE]]")      # run_headless.sh 가 이걸 보고 끝낸다
                sys.stdout.flush()


if __name__ == "__main__":
    main()
