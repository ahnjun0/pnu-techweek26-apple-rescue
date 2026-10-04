"""재생 비교 장치가 같은 것은 같다고, 다른 것은 다르다고 하는가."""
import gzip
import pickle

import numpy as np

import config
import localization
import mission
from debug import replay_check

DT = 0.016


def _record(tmp_path, ticks=40):
    """컨트롤러 고리를 흉내 내 짧은 녹화를 만든다 (빈 세상, 제자리에서 돈다)."""
    odometry = localization.Odometry()
    brain = mission.Mission(odometry.pose)
    rec = replay_check.Recorder(16, DT, 1.0)
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
    path = tmp_path / "rec.pkl.gz"
    rec.save(str(path))
    return path


def test_an_untouched_recording_replays_identically(tmp_path):
    assert replay_check.check(str(_record(tmp_path))) == 0


def test_a_changed_output_is_reported_with_its_tick(tmp_path, capsys):
    path = _record(tmp_path)
    with gzip.open(path, "rb") as f:
        data = pickle.load(f)
    data["ticks"][10]["out"]["turn"] += 1e-9
    with gzip.open(path, "wb") as f:
        pickle.dump(data, f)
    assert replay_check.check(str(path)) == 1
    assert "틱 10" in capsys.readouterr().out


def test_counters_left_over_in_the_process_do_not_matter(tmp_path):
    """녹화와 재생을 같은 프로세스에서 두 번 해도 같다 (모듈 수준 계수기를 비운다)."""
    _record(tmp_path)
    assert replay_check.check(str(_record(tmp_path))) == 0
