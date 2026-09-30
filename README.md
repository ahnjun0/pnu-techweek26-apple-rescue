# sar_rescue — 2026 부산대 TECH WEEK: AMR 의 Search & Rescue

**대회 환경**: 주최 측 repo [kyu-rae-kim/PNU-TECHWEEK-260930](https://github.com/kyu-rae-kim/PNU-TECHWEEK-260930)
(커밋 `383de18`)의 **`worlds/apartment.wbt`** — Webots R2025a, TurtleBot3 Burger, `basicTimeStep 64`,
시작 $(-0.3, -7.5, \pi)$, 빨간 사과 2개, 보행자 1명(0.2 m/s).

**임무**: 시작점에서 출발해 빨간 사과를 찾아 가까이 간 뒤 시작점으로 돌아온다
(얼마나 가까이 가는지는 채점하지 않는다). GNSS·절대 위치 사용 금지, 기본 로봇 속도 초과 금지.

**제출물**: 파이썬 파일 하나 + README — [`submission/`](submission/) (주최 측 repo 와 같은 구조)

```
SCAN → EXPLORE ⇄ APPROACH → RETURN → DONE
```

## 현재 성적 (apartment, 보행자 있음, 9/30)

| 항목 | 결과 |
|---|---|
| 빨간 사과 | **2 / 2 방문** (실제 최근접 0.47 m, 0.68 m) |
| 보행자 접촉 | 0 초 |
| 걸린 시간 | 432.9 초 |
| 복귀 | ⚠️ 로봇은 도착했다고 판단했지만 실제로는 시작점에서 1.99 m — 복귀 중 바퀴 헛돔으로 위치 추정이 185 cm 틀어졌다 (해결 중) |

시뮬레이션은 결정적이라 같은 환경이면 같은 결과가 나온다. 측정은 `debug/mission_check.py` 가
Supervisor 로 정답 위치를 읽어 한다 (주행 판단에는 쓰지 않는다).

---

## 1. 대회 환경에서 돌리기

### 설치 — clone 하고 `./setup.sh` 한 번

준비물: macOS + **Webots R2025a** (`/Applications/Webots.app`) + [uv](https://docs.astral.sh/uv/).

```bash
git clone https://github.com/ahnjun0/robot_hack.git && cd robot_hack     # 기본 브랜치 competition
./setup.sh
```

`setup.sh` 가 하는 일 (처음 한 번 몇 분, 에셋 약 590 MB):

1. `uv sync` → `.venv` (numpy 2.2.6 · opencv-python 5.0.0.93 · matplotlib 3.10.9 · torch 2.14.0 · ultralytics 8.4.123, `uv.lock` 고정)
2. `controllers/*/runtime.ini` 에 이 컴퓨터의 `.venv` 절대경로를 적는다 → **Webots 전역 설정(Python command)을 안 건드려도** 우리 컨트롤러가 `.venv` 로 돈다
3. 라이브러리·YOLO 모델·Webots 설치를 확인한다
4. Webots 에셋 미러를 git 으로 받아 `competition/webots_assets/` 에 두고, 채점용 월드가 그걸 가리키게 한다 → **인터넷 없이 열린다**

라이브러리가 빠진 파이썬으로 컨트롤러가 돌면 시작하자마자 `ERROR: 필요한 라이브러리가 없습니다` 를 띄우고 멈춘다.

```bash
uv run pytest tests/ -q     # 389 개 통과 (~11초)
./check_rules.sh            # "전부 통과"
```

### 실행

| 무엇 | 어떻게 |
|---|---|
| **채점하며 눈으로 보기** | Webots 로 `worlds/apartment_check.wbt` 를 연다 (더블클릭해도 된다) |
| 채점만 (헤드리스) | `./run_headless.sh worlds/apartment_check.wbt 1500` |
| 보행자 없이 | `worlds/apartment_nopeople_check.wbt` |
| 설정 여러 개 동시 비교 | `bash debug/ab.sh worlds/apartment_check.wbt "기준=" "스캔매칭=SCANMATCH_ENABLED=True"` |
| **대회 그대로** (제출 컨트롤러, 채점 없음) | `python3 tools/build_submission.py` 후 Webots 로 `submission/worlds/apartment.wbt` 열기 (주최 측 원본 월드 — 에셋을 GitHub 에서 받는다) |

- 대회 설정(시작 위치, 지도 크기, 64 ms, 사람 검출·칼만, 카메라 거리 측정)은 월드의
  `controllerArgs ["--profile=apartment"]` 로 켜진다 (`boot.py`). 환경변수를 줄 필요가 없다.
- `SAR_SET="이름=값;..."` 으로 그 실행에만 설정을 덮는다 (파일을 고치지 않는다).
- 채점용 월드는 주최 측 원본에서 `python3 tools/make_check_worlds.py` 로 만든다 — 손으로 고치지 않는다.
- ⚠️ 실험이 도는 동안 `.wbt` 파일을 더블클릭하지 않는다 — 떠 있는 Webots 가 그 파일을 열어 실험이 끊긴다.

### 제출물

```bash
python3 tools/build_submission.py      # → submission/  (모듈을 한 파일 sar_rescue.py 로 합친다)
```

- 개발은 모듈로 하고 제출만 한 파일이다. 합친 파일과 모듈 실행의 상태 로그가 **완전히 같음**을 Webots 에서 확인했다.
- 제출 README = [`submission_README.md`](submission_README.md) + [`docs/기술_질의응답.md`](docs/기술_질의응답.md).
- 제출 파일에 GPS·Supervisor 흔적이 있으면 빌드와 `check_rules.sh` 가 실패한다.

### 문서

| 무엇 | 어디 |
|---|---|
| 위치 추정·GNSS·속도 제어·경로 계획 (수식) | [`docs/기술_질의응답.md`](docs/기술_질의응답.md) |
| 주최 측 제공 repo (383de18 스냅숏, 기록 제외) | [`competition/given/`](competition/given/) |
| 대회 계획안·질의응답 | 저장소 루트의 PDF·txt |
| 당일 절차·알려진 약점 | [`competition/README.md`](competition/README.md) |
| 확인된 사실 (공고·강의·월드) | [`competition/01_facts.md`](competition/01_facts.md) |
| 해 보고 뺀 것과 그 이유 | [`docs/무엇을-빼기로-했나.md`](docs/무엇을-빼기로-했나.md) |
| 설정값과 **왜 그 값인지** | `config.py` 주석 |

---

## 연습 환경 (대회 전 개발용)

아래는 대회 월드가 공개되기 전에 만든 연습 월드와 참고 자료다.

```bash
.venv/bin/python3 -m pytest tests/ -q -m ""      # 임무 전체를 돌려 보는 통합 테스트까지 (~3분)
./run_headless.sh worlds/practice_check.wbt              # 연습 월드 채점
./run_gui.sh worlds/practice.wbt run                     # 연습 월드 GUI
```

| 월드 | 컨트롤러 | 용도 |
|---|---|---|
| `worlds/practice.wbt` | `sar_controller` | 연습 과제 — 자율 탐색·구조 |
| `worlds/practice_check.wbt` | `mission_check` | 같은 월드 + 정답 위치(Supervisor)로 성능을 숫자로 잰다 |
| `worlds/empty.wbt` | `odometry_check` | 오도메트리 오차 측정 |
| `worlds/lidar_test.wbt` | `lidar_orientation` | LiDAR 인덱스 방향 측정 |
| `worlds/camera_test.wbt` | `camera_lidar_check` | 카메라와 LiDAR 대조, 색 임계값 측정 |

---

## 2. 당일 로봇이 바뀌었을 때 고쳐야 하는 것 — 체크리스트

순서대로 하면 된다. **각 단계마다 확인 방법이 있다. 추측하지 말고 재라.**

### ⓪ 월드의 `basicTimeStep` ★★ 제일 먼저 볼 것

```bash
grep basicTimeStep <대회에서 준 월드>.wbt
```

**이 한 줄이 성적을 좌우한다.** Webots 의 기본값은 32 다. 값이 클수록 보행자가
한 물리 스텝에 로봇을 깊이 파고들고, 그러면 물리 엔진이 로봇을 **한 틱에 1 m**
날려 버린다. 그 뒤로는 복구가 안 된다 (지도에 없는 곳에 떨어져 맞출 벽이 없다).

같은 코드, 같은 월드에서 스텝만 바꿔 5회씩 잰 값:

| basicTimeStep | 실패 | 시간 중앙값 | 오도메트리 중앙값 |
|---|---|---|---|
| **32 ms (Webots 기본값)** | **4/5** | 249초 | **199.7 cm** |
| 16 ms (우리 월드) | 0/5 | 245초 | 26.1 cm |
| **8 ms** | **0/5** | **132초** | **15.5 cm** (5회 전부 동일) |

**같은 코드다.** 알고리즘은 한 줄도 안 바뀌었는데 이 숫자 하나가 실패율을
0% 에서 80% 로 바꾼다. 8 ms 에서는 실행마다 결과가 갈리는 현상까지 사라진다.

**당일 할 일**
1. 값을 확인한다.
2. 크면 (32 이상) 주최 측에 줄여도 되는지 묻는다 — 채점의 "충돌 회피" 와 직결된다.
3. 못 바꾸면, 그 조건에서 미리 재 보고 기대치를 낮춘다.

⚠️ 우리 컨트롤러 주기(`config.TIME_STEP`)는 이 값과 분리돼 있다. 월드가 바뀌어도
   제어율은 안 변한다. 단 `TIME_STEP` 이 `basicTimeStep` 의 배수여야 한다
   (`test_controller_period_is_a_multiple_of_the_physics_step` 가 검사한다).

### ① 디바이스 이름 (`config.py` 맨 위)
```
LEFT_MOTOR_NAME / RIGHT_MOTOR_NAME / LEFT_ENCODER_NAME / RIGHT_ENCODER_NAME
LIDAR_NAME / CAMERA_NAME / COMPASS_NAME / GYRO_NAME
```
**확인**: 월드를 열면 컨트롤러가 디바이스 목록을 찍는다. 없는 이름은 "없음" 으로 나온다.

### ② 로봇 치수
```
WHEEL_RADIUS    바퀴 반지름 [m]      PROTO 의 Cylinder radius
WHEEL_BASE      축간 거리 [m]        좌우 HingeJoint anchor 의 y 차이
ROBOT_RADIUS    몸 반경 [m]          boundingObject 크기의 절반 + 여유
MAX_WHEEL_SPEED 바퀴 상한 [rad/s]    Motor 의 maxVelocity
```
**확인**: `sensors.verify_against_config()` 가 시작할 때 실제 디바이스와 대조해 찍는다.
바퀴 반지름·축간 거리는 코드로 읽을 수 없으니 PROTO 를 봐야 한다.

### ③ LiDAR 인덱스 방향 ★ 제일 잘 틀리는 곳
```
LIDAR_ANGLE_OFFSET   인덱스 0 이 가리키는 각도
LIDAR_ANGLE_SIGN     인덱스가 늘 때 반시계(+1)인가 시계(-1)인가
```
**재는 법**: `./run_headless.sh worlds/lidar_test.wbt` → 출력에 적을 값이 그대로 나온다.
로봇이 바뀌면 상자 3개의 좌표(`debug/lidar_orientation.py` 의 `BOXES`)와
월드의 상자 위치를 새 로봇 시작 위치에 맞춰 고칠 것.
**틀리면**: 지도가 뒤집히거나 거울상이 된다. 벽이 엉뚱한 곳에 찍힌다.

### ④ 엔코더 전용 각도 보정
```
WHEEL_BASE_ODOM   회전을 엔코더만으로 잴 때 쓰는 "유효" 축간 거리
```
**재는 법**: `./run_headless.sh worlds/empty.wbt` → "2. 제자리 360도 회전" 의 encoder theta.
잔차가 크면 `WHEEL_BASE_ODOM = WHEEL_BASE × (실제 회전각 / 엔코더가 말한 각)`.
나침반이 있으면 안 쓰이는 값이다.

### ⑤ 안전거리 — **하나만 고치면 된다**
```
ROBOT_CLEARANCE          몸 바깥으로 두고 싶은 안전거리 [m]
FOLLOW_TRACKING_BUDGET   주행기가 경로에서 벗어나는 양 [m] (측정값)
```
나머지는 전부 여기서 유도된다:
```
정지거리        = 로봇반경 + 안전거리 - 추종오차
평소 계획 여유   = 로봇반경 + 안전거리
좁은 계획 여유   = 정지거리 + SLACK
접근 판정 거리   = 평소 계획 여유 + 목표물 굵기 + 여유
```
**절대 개별 숫자를 직접 고치지 말 것.** `tests/test_config_consistency.py` 가 막는다.
**추종 오차 재는 법**: 로봇 위치와 `planner` 경로 사이 최단거리를 매 틱 기록해 최대값을 본다.

### ⑥ 카메라 색 임계값
```
DETECT_HUE_LOW / DETECT_HUE_HIGH / DETECT_SAT_MIN / DETECT_VALUE_MIN
```
**재는 법**: `./run_headless.sh worlds/camera_test.wbt` → 목표물과 바닥의 HSV 중앙값을 찍어 준다.
`DETECT_SAT_MIN` 을 그 둘 **사이** 값으로 잡는다.
**틀리면**: 바닥이 목표물로 잡힌다 (실제로 바닥의 29.9% 가 오검출된 적 있다).

### ⑦ 지도 크기
```
MAP_ORIGIN_X / MAP_ORIGIN_Y / MAP_WIDTH_CELLS / MAP_HEIGHT_CELLS / MAP_RESOLUTION
```
아레나보다 넉넉해야 하고, LiDAR 사거리의 2배보다 커야 한다 (테스트가 검사한다).

### ⑧ 시작 위치
```
START_X / START_Y / START_THETA
```
월드의 로봇 `translation` 과 반드시 같아야 한다 (테스트가 검사한다).

### ⑨ 목표물 개수
```
MISSION_TARGET_COUNT   몇 개를 찾으면 복귀할 것인가 (0 이면 끝없이 탐색)
```

---

## 3. 튜닝 파라미터 설명

### 지도
| 이름 | 뜻 | 키우면 | 줄이면 |
|---|---|---|---|
| `MAP_RESOLUTION` | 칸 한 변 [m] | 빠르지만 거칠다 | 정밀하지만 느리다 |
| `MAP_UPDATE_EVERY` | N 틱마다 지도 갱신 | 빠르다 | 지도가 빨리 자란다 |
| `LIDAR_RAY_STRIDE` | 광선 N개마다 하나 | 빠르다 | 벽이 촘촘히 찍힌다 |
| `LOG_ODDS_OCCUPIED/FREE` | 증거 세기 | 빨리 굳는다 | 사람 흔적이 잘 지워진다 |

### 경로 계획
| 이름 | 뜻 | 키우면 | 줄이면 |
|---|---|---|---|
| `ROBOT_CLEARANCE` | 안전거리 (**기준값**) | 안전하지만 좁은 문을 못 지난다 | 좁은 문을 지나지만 스친다 |
| `PLANNER_SQUEEZE_SLACK` | 좁게 갈 때 여유 | — | 0 에 가까우면 그 경로는 못 쓴다 |
| `PLANNER_UNKNOWN_COST` | 모르는 칸 비용 | 아는 길을 선호 | 모르는 곳으로 과감히 |
| `PLANNER_SOFT_WEIGHT` | 벽 근처 비용 (**기본 1.0=끔**) | 경로가 가운데로 | — |

### 주행
| 이름 | 뜻 | 키우면 | 줄이면 |
|---|---|---|---|
| `FOLLOW_USE_DWA` | **True 권장** | DWA (속도 후보 평가) | look-ahead 추종 + 정지 |
| `FOLLOW_MAX_SPEED` | 전진 상한 [m/s] | 빨리 지나가 노출이 준다 | **회전 여유도 같이 준다** |
| `FOLLOW_LOOKAHEAD` | 몇 m 앞을 보나 | 모퉁이를 자른다 | 좌우로 흔들린다 |
| `DWA_MAX_ACCEL` | 가속 한계 | 민첩하지만 거칠다 | 부드럽지만 굼뜨다 |
| `DWA_WEIGHT_CLEARANCE` | 안전거리 점수 비중 | 벽에서 멀리 | 목표로 직진 |
| `SAFETY_PINNED_DISTANCE` | "눌렸다" 판정 | 일찍 빠져나온다 | 늦게 알아챈다 |

### 탐색·임무
| 이름 | 뜻 |
|---|---|
| `FRONTIER_MIN_CLUSTER` | 이보다 작은 프론티어 덩어리는 잡음으로 버린다 |
| `FRONTIER_MAX_FAILURES` | 한 목표를 몇 번 실패하면 블랙리스트 (단, 시간 초과는 1번) |
| `MISSION_GOAL_TIMEOUT` | 한 목표에 이만큼 매달리면 포기 [s] — 못 가는 곳의 비용 상한 |
| `MISSION_SWEEP_POINTS` | 목표물이 모자랄 때 몇 곳에서 둘러볼 것인가 |
| `RETURN_TOLERANCE` | 복귀 완료 판정 반경 [m] |

---

## 4. 흔한 실패 증상별 원인표

| 증상 | 짚어볼 것 | 확인 방법 |
|---|---|---|
| **지도가 뒤집히거나 거울상** | `LIDAR_ANGLE_OFFSET/SIGN` | `worlds/lidar_test.wbt` |
| **지도가 90도 돌아감** | 나침반 공식, `START_THETA` | `worlds/empty.wbt` 의 공식 실증 |
| **회전 후 벽이 두 겹** | θ 추정 오차 (나침반 미사용?) | `odometry_check` 의 3개 추정기 비교 |
| **사람 흔적이 안 지워짐** | `LOG_ODDS_FREE` 가 너무 약함 | `test_mapping.py` 의 사람 흔적 테스트 |
| **로봇이 멀쩡한 곳에서 멈칫** | 정지거리 > 계획 여유 (**모순**) | `test_config_consistency.py` |
| **좁은 문 앞에서 얼어붙음** | `ROBOT_CLEARANCE` 가 문 폭의 절반보다 큼 | 문 폭 ≥ 2×(반경+안전거리) 인가 |
| **경로가 벽을 뚫음** | 대각선 모서리 통과 | `test_planner.py` 의 코너 테스트 |
| **탐색이 안 끝남** | 갈 수 없는 "유령 프론티어" | 프론티어 도달 판정(팽창 제외) |
| **탐색이 너무 일찍 끝남** | 프론티어 판정이 너무 빡빡 | `PLANNER_SQUEEZE_MARGIN` |
| **바닥이 목표물로 잡힘** | `DETECT_SAT_MIN` 이 낮음 | `worlds/camera_test.wbt` HSV 측정 |
| **목표물이 여러 개로 쪼개짐** | `DETECT_MERGE_RADIUS` 가 작음 | `test_detect.py` 병합 테스트 |
| **카메라엔 보이는데 거리가 없음** | 정상이다 — LiDAR 는 평면 한 층 | 아래 "LiDAR 의 한계" 참고 |
| **목표물을 다 못 찾음** | 카메라가 그쪽을 안 봤다 | `MISSION_SWEEP_POINTS` 를 늘린다 |
| **로봇이 아레나 밖으로 날아감** | `basicTimeStep` 이 커서 보행자가 로봇을 깊이 관통 | 16 이하로 줄인다 |
| **목표물이 엉뚱한 곳에 찍힘** | 보는 방향 앞으로 사람이 지나가 LiDAR 가 짧은 거리를 줌 | 각폭·거리 일관성 검사 (`DETECT_SIZE_TOLERANCE`) |
| **APPROACH/RETURN 이 안 끝남** | 탈출 중에 시계가 안 돌아 시간 초과가 안 걸림 | 시계는 탈출 중에도 돌려야 한다 |
| **오도메트리가 수십 m 로 폭주** | 바퀴가 헛돔 (끼임) | 위와 같은 원인. 탈출이 늦지 않은지 |
| **시뮬레이션이 1.0x 아래** | matplotlib 창 (한 번 32 ms) | `SAR_VIZ=0` 으로 끈다 |
| **한 자리에서 눌렸다 빠져나오길 반복** | 그 속도에서 바퀴가 낼 수 있는 회전이 남아 있나 | `(0.22 - v) / (WHEEL_BASE/2)`. 0.20 m/s 면 0.25 rad/s 뿐이다. 후보를 버리지 말고 비례로 줄일 것 |
| **한참 동안 지도가 안 늘어남** | 벽 바깥의 못 가는 프론티어를 쫓고 있나 | `debug/out/trace.csv` 의 `goal_x,goal_y` 가 아레나 밖인지 본다 |
| **멀쩡한 곳에서 자꾸 후진한다** | LiDAR 가 한 틱만 튄 것을 "눌렸다" 로 믿었나 | `trace.csv` 의 `near_d` 가 한 틱만 떨어졌다 돌아오는지 본다 (`SAFETY_PINNED_TICKS`) |
| **프론티어가 벽 너머에 찍힌다** | 광선이 벽을 지나쳐 그어졌나 | 벽 너머에 빈 칸이 있는지 격자를 본다. `mapping.update` 는 막힌 칸에서 멈춰야 한다 |

---

## 5. 알아 두어야 할 물리적 한계

### LiDAR 는 평면 한 층만 본다
LDS-01 은 z ≈ 0.173 m 높이의 **수평면 하나**만 스캔한다. 카메라는 z ≈ 0.213 m 에서
57° × 45° 원뿔로 본다. 그래서 **구조적으로** 이런 것들이 생긴다
(`worlds/camera_test.wbt` 에서 실증):

| 물체 | 카메라 | LiDAR |
|---|---|---|
| 높이 0.4 m 기둥 | 보인다 | 보인다 |
| **높이 0.10 m 물체** | 보인다 | **안 보인다** (평면보다 낮다) |
| **공중에 뜬 물체** | 보인다 | **안 보인다** (평면보다 높다) |
| **3.5 m 밖** | 보인다 | **안 보인다** (사거리 초과) |

→ `detect.py` 는 "봤지만 거리를 모른다" 를 정상 상황으로 다룬다. 가까이 가서 다시 본다.

### 보행자는 무한한 힘으로 민다
Webots 의 `Pedestrian` 은 physics 가 없는 운동학 물체다. 로봇을 벽 쪽으로 밀면
로봇이 벽 속으로 파고들고, 심하면 물리 엔진이 터져 로봇이 아레나 밖으로 날아간다.
→ 월드를 만들 때 **보행자 경로를 목표물·벽끝에서 충분히 떨어뜨릴 것.**
→ `SAFETY_PINNED_DISTANCE` 를 올려 일찍 빠져나오게 할 것.

---

## 6. 설계 규칙 (`CLAUDE.md` 와 같음)

1. **GPS·Supervisor 는 `debug/` 에서만.** 대회에서는 위치를 알 수 없다.
2. **Webots 의존 코드는 `sensors.py` / `controllers/` / `debug/` 에만.**
   나머지는 순수 numpy 라 `pytest` 로 단독 검증된다.
3. **로봇 상수는 `config.py` 한 곳에만.** 출처를 주석에 남긴다.
4. **좌표 변환은 `common.py` 의 `to_cell`/`to_world` 만.**
5. **추측 금지.** LiDAR 방향·카메라 부호·색 임계값은 전부 실측으로 정했다.

`./check_rules.sh` 가 1·2·4를 자동 검사한다.

---

## 7. 문서

| 문서 | 내용 |
|---|---|
| `docs/phase4-주행기-비교.md` | 국소 진동 추적 과정, DWA 도입 측정 |
| `docs/계획기-비교-Astar-vs-DstarLite.md` | A* vs D* Lite, 언제 무엇을 쓸 것인가 |
| `docs/phase5-보행자-대응.md` | 걸어다니는 사람 다루기 — 겪은 실패와 측정으로 정한 행동 |
| `docs/무엇을-빼기로-했나.md` | 넣었다가 뺀 기능들과 그 측정 근거, 선행연구 비교 |
| `docs/phase별-기록.md` | Phase 0~6 진행 기록과 각 단계에서 배운 것 |
