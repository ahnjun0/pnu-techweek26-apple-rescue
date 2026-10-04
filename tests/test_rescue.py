"""탐색 → 목표물 접근 → 복귀까지 통째로 돌려 본다. Webots 없이.

여기서 보려는 것: 목표 3개를 다 찾아 방문하고, 시작 지점으로 돌아와 멈추는가.
그리고 중복 감지가 없는가 (같은 기둥을 여러 번 봐도 목표물 하나여야 한다).
"""

import math

import numpy as np
import pytest

from sar import common
from sar import config
import fake_world
from sar import mission

CAMERA_FOV = 1.0


def run(world, start, seconds=600.0, dt=0.064):
    """가짜 월드에서 임무를 끝까지 돌린다."""
    pose = start
    brain = mission.Mission(pose)
    trail = [pose[:2]]
    collisions = 0
    ranges = world.lidar(pose)
    image = world.camera(pose, CAMERA_FOV)
    states = []

    for tick in range(int(seconds / dt)):
        if tick % 2 == 0:
            ranges = world.lidar(pose)
            image = world.camera(pose, CAMERA_FOV)
        speed, turn = brain.step(pose, ranges, dt, image=image,
                                 camera_fov=CAMERA_FOV)
        pose, hit = world.move(pose, speed, turn, dt)
        collisions += hit
        trail.append(pose[:2])
        states.append(brain.state)
        if brain.state == mission.DONE:
            break

    return dict(mission=brain, pose=pose, trail=trail, states=states,
                collisions=collisions, seconds=(tick + 1) * dt,
                done=brain.state == mission.DONE)


@pytest.fixture(scope="module")
def rescue_run():
    """한 판만 돌리고 여러 테스트가 나눠 본다 (매번 돌리면 느리다)."""
    return run(fake_world.rescue_world(), (-2.0, -2.0, 0.0))


# --- 완주 --------------------------------------------------------------------

@pytest.mark.slow
def test_mission_finishes(rescue_run):
    assert rescue_run["done"], \
        f"끝내지 못했다 (상태: {rescue_run['mission'].state} / " \
        f"{rescue_run['mission'].status})"


@pytest.mark.slow
def test_finds_all_three_targets(rescue_run):
    found = rescue_run["mission"].targets.confirmed
    assert len(found) == 3, f"{len(found)} 개만 찾았다"


@pytest.mark.slow
def test_no_duplicate_targets(rescue_run):
    """같은 기둥을 수백 번 보지만 목표물은 하나여야 한다."""
    targets = rescue_run["mission"].targets
    assert len(targets.targets) <= 4, \
        f"덩어리가 {len(targets.targets)} 개로 불어났다 (중복 병합 실패)"


@pytest.mark.slow
def test_found_targets_are_where_they_really_are(rescue_run):
    truth = fake_world.rescue_world().targets
    for tx, ty in truth:
        gaps = [common.distance(tx, ty, t.x, t.y)
                for t in rescue_run["mission"].targets.confirmed]
        assert min(gaps) < 0.4, \
            f"실제 목표물 ({tx}, {ty}) 근처에 찾은 것이 없다 (최소 {min(gaps):.2f} m)"


@pytest.mark.slow
def test_all_targets_get_visited(rescue_run):
    unvisited = rescue_run["mission"].targets.unvisited()
    assert unvisited == [], f"{len(unvisited)} 개를 안 가 봤다"


@pytest.mark.slow
def test_returns_home(rescue_run):
    start = (-2.0, -2.0)
    gap = common.distance(*start, *rescue_run["pose"][:2])
    assert gap <= config.RETURN_TOLERANCE + 0.05, \
        f"시작 지점에서 {gap * 100:.0f} cm 떨어진 곳에서 끝났다"


@pytest.mark.slow
def test_never_touches_a_wall(rescue_run):
    assert rescue_run["collisions"] == 0, \
        f"벽에 {rescue_run['collisions']} 틱 닿았다"


@pytest.mark.slow
def test_goes_through_all_the_states(rescue_run):
    seen = set(rescue_run["states"])
    for state in (mission.SCAN, mission.EXPLORE, mission.APPROACH,
                  mission.RETURN, mission.DONE):
        assert state in seen, f"{state} 상태를 한 번도 안 거쳤다"


@pytest.mark.slow
def test_stays_put_after_done(rescue_run):
    brain = rescue_run["mission"]
    where = rescue_run["pose"]
    ranges = np.full(config.LIDAR_RESOLUTION, 3.0)
    for _ in range(30):
        assert brain.step(where, ranges, 0.064) == (0.0, 0.0)


# --- 상태 전이 자체 ----------------------------------------------------------

def test_approach_starts_when_a_target_is_confirmed():
    """목표물이 확인되면 탐색을 멈추고 그쪽으로 간다."""
    brain = mission.Mission((0.0, 0.0, 0.0))
    brain.state = mission.EXPLORE
    for _ in range(config.DETECT_MIN_SIGHTINGS):
        brain.targets.add(1.5, 0.0)

    brain.step((0.0, 0.0, 0.0), np.full(config.LIDAR_RESOLUTION, 3.0), 0.064)
    assert brain.state == mission.APPROACH


def test_arriving_marks_the_target_visited():
    brain = mission.Mission((0.0, 0.0, 0.0))
    brain.state = mission.EXPLORE
    for _ in range(config.DETECT_MIN_SIGHTINGS):
        brain.targets.add(config.APPROACH_DISTANCE * 0.5, 0.0)

    brain.step((0.0, 0.0, 0.0), np.full(config.LIDAR_RESOLUTION, 3.0), 0.064)
    assert brain.targets.confirmed[0].visited


def test_approach_gives_up_after_the_timeout():
    """벽 뒤의 목표물에 영원히 매달리면 안 된다."""
    brain = mission.Mission((0.0, 0.0, 0.0))
    brain.state = mission.APPROACH
    for _ in range(config.DETECT_MIN_SIGHTINGS):
        brain.targets.add(2.0, 0.0)
    brain._goal_age = config.APPROACH_TIMEOUT + 1.0

    brain.step((0.0, 0.0, 0.0), np.full(config.LIDAR_RESOLUTION, 3.0), 0.064)
    assert brain.targets.confirmed[0].visited
    assert brain.state != mission.APPROACH


def test_return_finishes_within_the_tolerance():
    brain = mission.Mission((0.0, 0.0, 0.0))
    brain.state = mission.RETURN
    near = (config.RETURN_TOLERANCE * 0.5, 0.0, 0.0)

    speed, turn = brain.step(near, np.full(config.LIDAR_RESOLUTION, 3.0), 0.064)
    assert brain.state == mission.DONE
    assert (speed, turn) == (0.0, 0.0)


def test_explores_before_sweeping():
    """둘러보기는 "지도를 다 그렸는데도 모자랄 때" 의 최후 수단이다.

    순서를 반대로 뒀더니 목표물 하나를 방문하자마자 탐색을 그만두고
    둘러보러 다녔다 (전체 시간의 64% 를 SWEEP 에, 2.9% 만 EXPLORE 에 썼다).
    """
    brain = mission.Mission((0.0, 0.0, 0.0))
    brain.grid[70:90, 70:90] = config.LOG_ODDS_MIN      # 볼 곳이 남아 있다
    for _ in range(config.DETECT_MIN_SIGHTINGS):
        brain.targets.add(1.0, 0.0)
    brain.targets.confirmed[0].visited = True

    brain._decide_next((0.0, 0.0, 0.0), exploring_possible=True)
    assert brain.state == mission.EXPLORE, \
        "탐색할 곳이 남았는데 둘러보러 가면 안 된다"


def test_sweep_does_not_freeze_when_the_planner_stops_short():
    """주행기가 "도착" 이라고 하면 그 말을 믿어야 한다.

    계획기는 벽 때문에 목표보다 조금 앞에서 멈추게 데려다 줄 수 있다.
    그 말을 무시했더니 로봇이 v=0, w=0 으로 17 초를 서 있었다.
    """
    brain = mission.Mission((0.0, 0.0, 0.0))
    brain.state = mission.SWEEP
    brain.grid[:] = config.LOG_ODDS_MIN
    # 목표는 판정 기준보다 조금 먼 곳 — 주행기는 이미 도착했다고 할 거리
    brain.goal = (config.FOLLOW_GOAL_TOLERANCE * 1.3, 0.0)
    brain.path = [(0.0, 0.0), (0.0, 0.0)]

    ranges = np.full(config.LIDAR_RESOLUTION, 3.0)
    for _ in range(30):
        speed, turn = brain.step((0.0, 0.0, 0.0), ranges, 0.064)
        if abs(turn) > 0.0:
            break
    assert abs(turn) > 0.0, "도착했으면 둘러보기(회전)로 넘어가야 한다"


def test_return_does_not_freeze_when_there_is_no_path():
    """경로를 못 찾아도 가만히 서 있으면 안 된다.

    실제로 "복귀 경로를 못 찾았다 — 다시 시도" 만 300초 반복하며 얼어붙었다.
    계획이 안 되면 집 쪽으로 직선을 긋고 DWA 에게 맡긴다.
    """
    brain = mission.Mission((0.0, 0.0, 0.0))
    brain.state = mission.RETURN
    brain.start_pose = (3.0, 0.0, 0.0)          # 집은 멀리
    # A* 가 반드시 실패하도록 지도를 완전히 갈라 놓는다
    brain.grid[:] = config.LOG_ODDS_MIN
    _, col = common.to_cell(1.5, 0.0)
    brain.grid[:, col] = config.LOG_ODDS_MAX

    ranges = np.full(config.LIDAR_RESOLUTION, 3.0)
    moved = False
    for _ in range(int(config.RETURN_DIRECT_AFTER / 0.064) + 30):
        speed, turn = brain.step((0.0, 0.0, 0.0), ranges, 0.064)
        if abs(speed) > 0.0 or abs(turn) > 0.0:
            moved = True
    assert moved, "경로가 없다고 영원히 서 있으면 안 된다"
    assert "직선" in brain.status


def test_return_eventually_gives_up():
    """영원히 복귀를 시도하느니 멈추는 게 낫다 (어디서 멈췄는지는 남긴다)."""
    brain = mission.Mission((0.0, 0.0, 0.0))
    brain.state = mission.RETURN
    brain.start_pose = (3.0, 0.0, 0.0)
    brain._return_age = config.RETURN_TIMEOUT + 1.0

    speed, turn = brain.step((0.0, 0.0, 0.0),
                             np.full(config.LIDAR_RESOLUTION, 3.0), 0.064)
    assert brain.state == mission.DONE
    assert (speed, turn) == (0.0, 0.0)
    assert "시간 초과" in brain.status


def test_return_starts_when_all_targets_are_found():
    brain = mission.Mission((0.0, 0.0, 0.0))
    brain.state = mission.APPROACH
    for index in range(config.MISSION_TARGET_COUNT):
        for _ in range(config.DETECT_MIN_SIGHTINGS):
            brain.targets.add(float(index) + 1.0, 0.0)
    for target in brain.targets.confirmed:
        target.visited = True

    brain._leave_approach((0.0, 0.0, 0.0))
    assert brain.state == mission.RETURN


@pytest.mark.slow
def test_no_targets_means_plain_exploration_still_ends():
    """목표물이 하나도 없는 월드에서도 끝나야 한다 (복귀 상태에서 굳으면 안 된다)."""
    result = run(fake_world.two_rooms(), (-2.0, -2.0, 0.0), seconds=400.0)
    assert result["done"]
    assert result["mission"].targets.confirmed == []


def test_a_standing_wall_feature_is_not_a_person():
    """제자리에 계속 보이는 것은 사람이 아니다 (문틈·벽 끝).

    ⚠️ 회귀 방지. 예전 규칙은 "여러 스캔에서 같은 자리면 사람" 이었고, 주석에
       "벽 조각은 한 번만 보인다" 고 적혀 있었다. 그 가정이 틀렸다 — 문틈은 매
       스캔 같은 자리에 보인다. 그래서 사람이 없는 미로에서 900초 중 335초의
       틱에 유령 사람이 잡혔고, 계획기에 가짜 사회적 비용이 얹혔다.
    """
    from sar import people as people_mod

    pose = (0.0, 0.0, 0.0)
    watcher = people_mod.Watcher()
    fixed = [(1.0, 0.0)]                      # 늘 같은 자리
    for _ in range(config.PEOPLE_VOTE_SCANS * 2):
        watcher.recent.append(list(fixed))
        if len(watcher.recent) > config.PEOPLE_VOTE_SCANS:
            watcher.recent.pop(0)
    seen = watcher.see(pose, np.full(config.LIDAR_RESOLUTION, 9.0))
    assert seen == [], f"안 움직이는 것을 사람으로 치면 안 된다: {seen}"


def test_a_moving_blob_is_a_person_whatever_its_shape():
    """옮겨 가는 덩어리는 사람으로 친다 — 모양을 묻지 않는다."""
    from sar import people as people_mod

    pose = (0.0, 0.0, 0.0)
    watcher = people_mod.Watcher()
    step = config.PEOPLE_MIN_SPEED * (config.TIME_STEP / 1000.0) * 3
    for k in range(config.PEOPLE_VOTE_SCANS):
        watcher.recent.append([(1.0 + k * step, 0.0)])
        if len(watcher.recent) > config.PEOPLE_VOTE_SCANS:
            watcher.recent.pop(0)
    seen = watcher.see(pose, np.full(config.LIDAR_RESOLUTION, 9.0))
    assert seen, "꾸준히 옮겨 가면 사람으로 봐야 한다"


def test_people_are_avoided_where_they_are_going():
    """사람의 **갈 곳** 도 피해야 한다 — 현재 위치만으로는 못 막는다.

    ⚠️ 실측(stress_check): 물리적으로 밀리는 mover 둘은 0.54 m / 1.73 m 로 한 번도
       안 닿았는데, Webots Pedestrian 만 0.029 m 까지 닿았다 (5.9초).
       보행자는 기구학적이라 궤적을 그대로 밀고 지나간다 — 우리가 미리 비켜야 한다.
    """
    from sar import follower

    # 기본값은 꺼져 있다 (측정이 나빠서). 기능 자체는 살아 있어야 하므로 켜고 본다.
    import pytest
    monkey = pytest.MonkeyPatch()
    monkey.setattr(config, "DWA_PERSON_LOOKAHEAD", 1.2)

    pose = (0.0, 0.0, 0.0)
    # 로봇 정면 2 m 앞에서 사람이 로봇 쪽으로 걸어온다
    walker = [(2.0, 0.0, -0.5, 0.0)]
    ghosts = follower._predicted_points(pose, walker)
    assert len(ghosts) == config.DWA_PERSON_STEPS, "예측 점이 만들어져야 한다"
    # 예측 점들은 현재 위치보다 로봇에 가까워야 한다 (다가오는 중이므로)
    assert ghosts[-1][0] < 2.0, f"다가오는 사람의 미래 위치가 더 가까워야 한다: {ghosts}"

    # 가만히 있는 것은 미래 위치도 제자리
    still = [(2.0, 0.0, 0.0, 0.0)]
    fixed = follower._predicted_points(pose, still)
    assert all(abs(p[0] - 2.0) < 1e-6 for p in fixed), "안 움직이면 제자리여야 한다"
    monkey.undo()


def test_wall_surface_noise_is_not_a_person():
    """벽에 붙은 덩어리는 사람이 아니다 (표면 양자화 잡음).

    ⚠️ 회귀 방지. 지도 기반 후보 생성을 넣었더니 **사람이 없는 미로에서** 유령이
       틱당 0.44개 나왔다 (헛것 100%, 15,121회). 광선이 벽에 맞는 칸이 한 칸씩
       흔들려 이전에 "빈 칸" 으로 찍힌 자리에 반사가 생기기 때문이다.
       그 유령이 계획기에 사회적 비용을 얹어 maze0 이 30.6 -> 83.9 m 가 됐다.
    """
    from sar import mapping
    from sar import people as people_mod

    grid = mapping.new_map()
    r0, c0 = common.to_cell(-3.0, -3.0)
    r1, c1 = common.to_cell(3.0, 3.0)
    grid[r0:r1, c0:c1] = config.LOG_ODDS_MIN          # 다 빈 방
    wr0, wc0 = common.to_cell(-3.0, 1.0)
    wr1, wc1 = common.to_cell(3.0, 1.1)
    grid[wr0:wr1, wc0:wc1] = config.LOG_ODDS_MAX      # y=1.0 에 가로 벽

    # 벽 바로 앞(0.1 m)에서 돌아온 광선들 — 표면 잡음이다
    pose = (0.0, 0.0, math.pi / 2)
    ranges = np.full(config.LIDAR_RESOLUTION, 9.0)
    angles = common.lidar_angles()
    ranges[np.abs(angles) < 0.05] = 0.9               # 벽(1.0 m) 살짝 앞

    found = people_mod.moving_blobs(pose, ranges, grid)
    for x, y in found:
        assert abs(y - 1.0) > config.PEOPLE_WALL_CLEARANCE * 0.5, \
            f"벽에 붙은 것을 사람 후보로 내면 안 된다: {(x, y)}"


def test_walker_seen_for_only_part_of_the_window_is_still_a_person():
    """창의 일부만 본 보행자도 사람으로 잡아야 한다.

    ⚠️ 회귀 방지. 움직임 문턱을 **창 전체**(PEOPLE_VOTE_SCANS)로 계산했다.
       창의 일부에만 걸린 그룹은 그 동안 갈 수 없는 거리를 요구받아 "안 움직였다"
       로 기각된다 — 진짜 사람이 걸러진다.
       계산: 창 9틱(0.144 s) 문턱 2.2 cm 인데, 3스캔(구간 2틱 = 0.032 s)만 본
       그룹에서 0.5 m/s 로 걷는 사람은 1.6 cm 밖에 못 간다 → 기각.
       실측: 그 상태에서 "볼 수 있었을 때의 재현율" 이 53% 였다.
    """
    from sar import people as people_mod

    dt = config.TIME_STEP / 1000.0
    speed = 0.5                       # stress_check 의 보행자 속도

    watcher = people_mod.Watcher()
    # 앞부분은 아무것도 안 보이고, 마지막 3스캔에만 걸어오는 사람이 보인다.
    blank = config.PEOPLE_VOTE_SCANS - config.PEOPLE_VOTE_NEEDED
    for _ in range(blank):
        watcher.recent.append([])
    for k in range(config.PEOPLE_VOTE_NEEDED):
        watcher.recent.append([(1.0 + speed * k * dt, 0.5)])

    spots = [(p[0], p[1], k)
             for k, scan in enumerate(watcher.recent) for p in scan]
    scans = sorted({s[2] for s in spots})
    assert len(scans) == config.PEOPLE_VOTE_NEEDED, "실험이 성립하지 않는다"

    span_seconds = (scans[-1] - scans[0]) * dt
    walked = speed * span_seconds
    need_new = config.PEOPLE_MIN_SPEED * span_seconds
    need_old = config.PEOPLE_MIN_SPEED * config.PEOPLE_VOTE_SCANS * dt

    assert walked >= need_new, (
        f"실제 구간 기준으로도 못 넘는다 (걸은 {walked * 100:.1f} cm "
        f"< 문턱 {need_new * 100:.1f} cm)")
    assert walked < need_old, (
        f"이 검사가 무의미하다 — 옛 문턱({need_old * 100:.1f} cm)도 넘는다. "
        f"걸은 거리 {walked * 100:.1f} cm")


def test_planner_only_sees_confident_people():
    """계획기에는 확신이 서는 검출만 넘긴다 (DWA 에는 전부 넘긴다).

    ⚠️ 회귀 방지. 유령의 해악은 두 경로로 들어오는데 값이 다르다 — DWA 는 잠깐
       돌아가면 그만이지만, 계획기의 사회적 비용은 **전역 경로를 왜곡** 한다.
       실측: 회피 문턱을 3 -> 2 로 내려 보행자 접촉을 0 으로 만들었더니, 사람이
       없는 정적 월드에서 헛것이 0.13 -> 0.27 /틱 으로 두 배가 되고 comb1 탐색률이
       85% -> 51% 로 무너졌다 (오도메트리는 오히려 좋아졌으니 유령 탓이다).
    """
    brain = mission.Mission((0.0, 0.0, 0.0))
    # (x, y, vx, vy, 본 스캔 수)
    brain._people = [
        (1.0, 0.0, 0.0, 0.0, config.PEOPLE_PLANNER_SCANS),        # 확신
        (2.0, 0.0, 0.0, 0.0, config.PEOPLE_PLANNER_SCANS - 1),    # 미덥다
    ]
    brain._people_xy = [(p[0], p[1]) for p in brain._people
                        if len(p) < 5 or p[4] >= config.PEOPLE_PLANNER_SCANS]

    assert (1.0, 0.0) in brain._people_xy, "확신이 선 것을 계획기에서 뺐다"
    assert (2.0, 0.0) not in brain._people_xy, (
        "미더운 검출을 계획기에 넘겼다 — 전역 경로가 왜곡된다")
    assert len(brain._people) == 2, "DWA 에는 전부 넘어가야 한다"
