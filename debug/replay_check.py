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

import cv2
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

FORMAT = 2
# 녹화 파일 = gzip 안에 pickle 을 차례로: 머리 정보 dict 하나, 그 뒤 틱 dict 들.
# ⚠️ 틱마다 바로 쓴다 — 640x480 영상이 두 틱에 한 장이라 메모리에 모으면 수 GB 가 된다
#    (2026-10-04 첫 녹화가 그래서 디스크를 채우고 죽었다). 영상은 PNG(무손실)로 줄인다.


def encode_image(image):
    ok, png = cv2.imencode(".png", np.ascontiguousarray(image))
    assert ok, "PNG 인코딩 실패"
    return png.tobytes()


def decode_image(data):
    return None if data is None else cv2.imdecode(np.frombuffer(data, np.uint8),
                                                  cv2.IMREAD_UNCHANGED)


def _floats(values):
    return None if values is None else tuple(float(v) for v in values)


def snapshot(brain, odometry, speed, turn, pose):
    """한 틱의 출력. 재생과 녹화가 같은 함수로 만든다."""
    return {"speed": float(speed), "turn": float(turn), "pose": _floats(pose),
            "wheel_turn": float(odometry.wheel_turn_rate),
            "pose_fix": _floats(brain.pose_fix), "state": brain.state,
            "status": brain.status, "elapsed": float(brain.elapsed)}


class Recorder:
    """채점 컨트롤러가 틱마다 부른다. 판단에는 끼어들지 않는다. 끝나면 close()."""

    def __init__(self, path, timestep, dt, camera_fov):
        self._file = gzip.open(path, "wb", compresslevel=3)
        self._yolo = []             # 이번 틱에 YOLO 가 낸 답 (부른 순서대로)
        self.ticks = 0
        self._dump({"format": FORMAT, "timestep": int(timestep), "dt": float(dt),
                    "camera_fov": float(camera_fov)})

    def _dump(self, obj):
        pickle.dump(obj, self._file, protocol=pickle.HIGHEST_PROTOCOL)

    def wrap_classify(self, classify):
        if classify is None:
            return None

        def recorded(image):
            answer = classify(image)
            self._yolo.append(answer)
            return answer
        return recorded

    def inputs(self, left, right, compass, ranges, image):
        """판단부에 넣기 **전에** 복사한다 (판단부가 배열을 고쳐도 녹화는 그대로)."""
        self._yolo = []
        return {"left": float(left), "right": float(right), "compass": _floats(compass),
                "ranges": np.array(ranges, dtype=np.float32, copy=True),
                "image": None if image is None else encode_image(image)}

    def outputs(self, taken, brain, odometry, speed, turn, pose):
        self._dump({**taken, "yolo": self._yolo,
                    "out": snapshot(brain, odometry, speed, turn, pose)})
        self.ticks += 1

    def close(self):
        self._file.close()


def read(path):
    """머리 정보, 그다음 틱들을 차례로 내놓는다."""
    with gzip.open(path, "rb") as f:
        while True:
            try:
                yield pickle.load(f)
            except EOFError:
                return


def write(path, records):
    with gzip.open(path, "wb", compresslevel=3) as f:
        for obj in records:
            pickle.dump(obj, f, protocol=pickle.HIGHEST_PROTOCOL)


def _clear_counters():
    for module in (mission, follower, people, detect):
        for name, value in vars(module).items():
            if name.endswith("COUNT") and isinstance(value, dict):
                value.clear()


class YoloMismatch(Exception):
    pass


def replay(head, ticks):
    """녹화 입력을 controllers 와 같은 순서로 넣고, 틱마다 (출력, 녹화된 출력) 을 내놓는다."""
    assert head["format"] == FORMAT, f"녹화 형식 {head['format']} — 이 장치는 {FORMAT}"
    _clear_counters()
    config.apply_timestep(head["timestep"])
    odometry = localization.Odometry()
    brain = mission.Mission(odometry.pose)
    answers = []

    def classify(image):
        if not answers:
            raise YoloMismatch("YOLO 를 녹화 때보다 많이 불렀다")
        return answers.pop(0)
    brain.classify = classify
    dt, fov = head["dt"], head["camera_fov"]
    for t in ticks:
        answers[:] = list(t["yolo"])
        odometry.update(t["left"], t["right"], dt, compass_values=t["compass"])
        pose = odometry.pose
        speed, turn = brain.step(pose, t["ranges"], dt, image=decode_image(t["image"]),
                                 camera_fov=fov, wheel_turn=odometry.wheel_turn_rate)
        if answers:
            raise YoloMismatch(f"YOLO 를 녹화 때보다 {len(answers)}번 덜 불렀다")
        yield snapshot(brain, odometry, speed, turn, pose), t["out"]
        if brain.pose_fix is not None:
            odometry.correct(brain.pose_fix)


def check(path):
    records = read(path)
    head = next(records)
    count = 0
    try:
        for k, (got, want) in enumerate(replay(head, records)):
            if repr(got) != repr(want):      # repr: nan 도 같게 본다, 부동소수점은 비트 그대로
                print(f"❌ 틱 {k} (t={want['elapsed']:.3f}s, {want['state']}) 에서 어긋난다: {path}")
                for key in want:
                    if repr(got[key]) != repr(want[key]):
                        print(f"   {key}: 녹화 {want[key]!r}  /  재생 {got[key]!r}")
                return 1
            count += 1
    except YoloMismatch as exc:
        print(f"❌ 틱 {count} 에서 어긋난다: {exc}: {path}")
        return 1
    print(f"✅ 틱 {count}개 모두 같다: {path}")
    return 0


if __name__ == "__main__":
    sys.exit(check(sys.argv[1]))
