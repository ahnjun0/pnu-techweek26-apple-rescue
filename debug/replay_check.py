"""녹화한 입력을 판단부에 다시 넣어, 출력이 녹화 때와 틱마다 같은지 본다.

받는 것: 채점 실행(controllers/mission_check, SAR_RECORD=경로.pkl.gz)이 남긴 녹화.
내놓는 것: 같으면 "✅ 틱 N개 모두 같다" (exit 0), 다르면 처음 어긋난 틱과 항목 (exit 1).
핵심 아이디어: 위치 추정(localization.Odometry)과 판단(mission.Mission)은 Webots 를 모르고
난수·시계를 쓰지 않는다. 같은 입력이면 같은 출력이어야 한다. 그래서 코드를 다듬은 뒤
이 비교가 통과하면 실제 주행도 같다.
⚠️ sensors.py 의 변환(바퀴 속도 환산, LiDAR 배열 정렬)은 재생 밖이다 — 그 파일은 다듬지 않는다.
⚠️ 녹화마다 새 프로세스로 돌린다. 같은 프로세스라도 모듈 수준 계수기는 시작할 때 비운다.
쓰는 법:  python debug/replay_check.py 녹화.pkl.gz
"""
import gzip
import os
import pickle
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
try:
    from sar import config, detect, follower, localization, mission, people  # noqa: E402
except ImportError:          # 패키지로 옮기기 전 구조 (모듈이 저장소 루트에 있다)
    import config  # noqa: E402
    import detect  # noqa: E402
    import follower  # noqa: E402
    import localization  # noqa: E402
    import mission  # noqa: E402
    import people  # noqa: E402

FORMAT = 1


def _floats(values):
    return None if values is None else tuple(float(v) for v in values)


def snapshot(brain, odometry, speed, turn, pose):
    """한 틱의 출력. 재생과 녹화가 같은 함수로 만든다."""
    return {"speed": float(speed), "turn": float(turn), "pose": _floats(pose),
            "wheel_turn": float(odometry.wheel_turn_rate),
            "pose_fix": _floats(brain.pose_fix), "state": brain.state,
            "status": brain.status, "elapsed": float(brain.elapsed)}


class Recorder:
    """채점 컨트롤러가 틱마다 부른다. 판단에는 끼어들지 않는다."""

    def __init__(self, timestep, dt, camera_fov):
        self.data = {"format": FORMAT, "timestep": int(timestep), "dt": float(dt),
                     "camera_fov": float(camera_fov), "yolo_enabled": False,
                     "yolo": [], "ticks": []}

    def wrap_classify(self, classify):
        if classify is None:
            return None
        self.data["yolo_enabled"] = True

        def recorded(image):
            answer = classify(image)
            self.data["yolo"].append(answer)
            return answer
        return recorded

    def inputs(self, left, right, compass, ranges, image):
        """판단부에 넣기 **전에** 복사한다 (판단부가 배열을 고쳐도 녹화는 그대로)."""
        return {"left": float(left), "right": float(right), "compass": _floats(compass),
                "ranges": np.array(ranges, dtype=np.float32, copy=True),
                "image": None if image is None else np.array(image, copy=True)}

    def outputs(self, taken, brain, odometry, speed, turn, pose):
        self.data["ticks"].append({**taken, "out": snapshot(brain, odometry, speed, turn, pose)})

    def save(self, path):
        with gzip.open(path, "wb", compresslevel=3) as f:
            pickle.dump(self.data, f, protocol=pickle.HIGHEST_PROTOCOL)


def _clear_counters():
    for module in (mission, follower, people, detect):
        for name, value in vars(module).items():
            if name.endswith("COUNT") and isinstance(value, dict):
                value.clear()


def replay(data):
    """녹화 입력을 controllers 와 같은 순서로 넣고, 틱마다 출력을 내놓는다."""
    assert data["format"] == FORMAT, f"녹화 형식 {data['format']} — 이 장치는 {FORMAT}"
    _clear_counters()
    config.apply_timestep(data["timestep"])
    odometry = localization.Odometry()
    brain = mission.Mission(odometry.pose)
    if data["yolo_enabled"]:
        answers = iter(data["yolo"])
        brain.classify = lambda image: next(answers)
    dt, fov = data["dt"], data["camera_fov"]
    for t in data["ticks"]:
        odometry.update(t["left"], t["right"], dt, compass_values=t["compass"])
        pose = odometry.pose
        speed, turn = brain.step(pose, t["ranges"], dt, image=t["image"],
                                 camera_fov=fov, wheel_turn=odometry.wheel_turn_rate)
        yield snapshot(brain, odometry, speed, turn, pose)
        if brain.pose_fix is not None:
            odometry.correct(brain.pose_fix)


def check(path):
    with gzip.open(path, "rb") as f:
        data = pickle.load(f)
    count = 0
    for k, (got, want) in enumerate(zip(replay(data), (t["out"] for t in data["ticks"]))):
        if repr(got) != repr(want):          # repr: nan 도 같게 본다, 부동소수점은 비트 그대로
            print(f"❌ 틱 {k} (t={want['elapsed']:.3f}s, {want['state']}) 에서 어긋난다: {path}")
            for key in want:
                if repr(got[key]) != repr(want[key]):
                    print(f"   {key}: 녹화 {want[key]!r}  /  재생 {got[key]!r}")
            return 1
        count += 1
    if count != len(data["ticks"]):
        print(f"❌ 재생이 {count}틱에서 멈췄다 (녹화 {len(data['ticks'])}틱): {path}")
        return 1
    print(f"✅ 틱 {count}개 모두 같다: {path}")
    return 0


if __name__ == "__main__":
    sys.exit(check(sys.argv[1]))
