"""재생 비교 장치가 같은 것은 같다고, 다른 것은 다르다고 하는가."""
import numpy as np

import config
import localization
import mission
from debug import replay_check

DT = 0.016


def _record(tmp_path, ticks=40):
    """컨트롤러 고리를 흉내 내 짧은 녹화를 만든다 (빈 세상, 제자리에서 돈다)."""
    path = tmp_path / "rec.pkl.gz"
    odometry = localization.Odometry()
    brain = mission.Mission(odometry.pose)
    rec = replay_check.Recorder(str(path), 16, DT, 1.0)
    left = right = 0.0
    for _ in range(ticks):
        left, right = left + 0.05, right - 0.05
        compass = (0.0, 1.0, 0.0)
        ranges = np.full(config.LIDAR_RESOLUTION, 2.0, dtype=np.float32)
        odometry.update(left, right, DT, compass_values=compass)
        pose = odometry.pose
        taken = rec.inputs(left, right, compass, ranges, None)
        speed, turn = brain.step(pose, ranges, DT, image=None, camera_fov=1.0,
                                 wheel_turn=odometry.wheel_turn_rate)
        rec.outputs(taken, brain, odometry, speed, turn, pose)
        if brain.pose_fix is not None:
            odometry.correct(brain.pose_fix)
    rec.close()
    return path


def _rewrite(path, change):
    records = list(replay_check.read(str(path)))
    change(records[1:])                       # [0] 은 머리 정보, 나머지가 틱
    replay_check.write(str(path), records)


def test_an_untouched_recording_replays_identically(tmp_path):
    assert replay_check.check(str(_record(tmp_path))) == 0


def test_a_changed_output_is_reported_with_its_tick(tmp_path, capsys):
    path = _record(tmp_path)

    def nudge(ticks):
        ticks[10]["out"]["turn"] += 1e-9
    _rewrite(path, nudge)
    assert replay_check.check(str(path)) == 1
    assert "틱 10" in capsys.readouterr().out


def test_counters_left_over_in_the_process_do_not_matter(tmp_path):
    """녹화와 재생을 같은 프로세스에서 두 번 해도 같다 (모듈 수준 계수기를 비운다)."""
    _record(tmp_path)
    assert replay_check.check(str(_record(tmp_path))) == 0


def test_a_different_number_of_yolo_calls_is_reported(tmp_path, capsys):
    """다듬은 코드가 YOLO 를 녹화 때와 다른 횟수로 부르면 어긋남이다."""
    path = _record(tmp_path)

    def extra_answer(ticks):
        ticks[5]["yolo"].append([])
    _rewrite(path, extra_answer)
    assert replay_check.check(str(path)) == 1
    assert "YOLO" in capsys.readouterr().out


def test_camera_images_come_back_bit_for_bit(tmp_path):
    """영상은 PNG 로 줄여 쓰지만 무손실이다 — 카메라가 주는 BGRA 의 앞 세 채널(연속 아님)도."""
    rng = np.random.default_rng(7)
    bgra = rng.integers(0, 256, size=(48, 64, 4), dtype=np.uint8)
    image = bgra[:, :, :3]
    rec = replay_check.Recorder(str(tmp_path / "img.pkl.gz"), 16, DT, 1.0)
    taken = rec.inputs(0.0, 0.0, (0.0, 1.0, 0.0), np.zeros(4, dtype=np.float32), image)
    assert np.array_equal(replay_check.decode_image(taken["image"]), image)
