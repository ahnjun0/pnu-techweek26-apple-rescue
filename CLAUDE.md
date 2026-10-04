# CLAUDE.md — 특명: 집 나간 빨간 사과 찾아오기 (Webots 자율 탐색·구조)

## 프로젝트 목적
Webots 시뮬레이터에서 Differential Drive 로봇(TurtleBot3 Burger)이 처음 보는 아파트를 스스로
탐색해 빨간 사과를 찾아 방문한 뒤 시작 지점으로 복귀한다. 2026 부산대 TECH WEEK 해커톤 결과물이다.
코드를 고칠 때는 아래 규칙과 `docs/측정_기록.md` 의 근거를 먼저 본다.

## 절대 규칙

### 1. GPS와 Supervisor는 주행 로직에서 금지
- 대회에서는 로봇의 절대 위치를 알 수 없다.
- `GPS` 와 `Supervisor` API는 **`debug/` 아래 검증 스크립트에서만** 읽는다.
- `sar/` 의 모든 모듈과 `controllers/sar_controller/` 는 GPS·Supervisor 를 절대 참조하지 않는다.
- 위반 검사: `./check_rules.sh` (규칙 1·2·4 를 한꺼번에 본다).
  정답 위치는 `debug/truth.py` 가 Supervisor 로만 읽는다 (주최 측 `tb3_ground_truth.py` 와 같은 방식).

### 2. Webots 의존 코드 격리
- `from controller import ...` 는 `sar/sensors.py`, `controllers/`, `debug/` 에만 등장한다.
- 나머지 `sar/` 모듈(`mapping`, `planner`, `exploration`, `mission*`, `common`, `config` 등)은
  순수 numpy — Webots 없이 `pytest` 로 단독 테스트 가능해야 한다.

### 3. 로봇 상수는 `sar/config.py` 한 곳에만
- 디바이스 이름, 바퀴 반지름, 축간 거리 등을 모듈 안에 하드코딩하지 않는다.
- 값의 출처(PROTO 파일 URL/줄번호)나 측정 근거를 한 줄 주석으로 남긴다. 긴 실험 이야기는 `docs/측정_기록.md` 에.

### 4. 좌표 변환은 `sar/common.py` 의 to_cell/to_world 두 함수만
- 다른 곳에서 `int(x/res)` 같은 변환을 직접 쓰지 않는다.

### 5. 추측 금지
- 확실하지 않은 Webots API·상수는 추측하지 말고 PROTO 소스나 R2025a 문서를 먼저 읽고
  근거(파일/줄번호)를 주석에 남긴다.
- 특히 **LiDAR 배열 인덱스 방향은 문서만으로 확정하지 않는다.**
  `debug/lidar_orientation.py` 실험으로 확인한 뒤 `sar/config.py` 에 기록한다.

### 6. 동작을 바꾸지 않는 변경은 재생 비교로 확인한다
- 리팩터링·주석 정리처럼 동작이 같아야 하는 변경은 Webots 녹화(`SAR_RECORD=…`)를
  `python debug/replay_check.py 녹화.pkl.gz` 로 다시 넣어 **틱마다 같은지** 본다.
- `sar/sensors.py` 의 변환(바퀴 속도 환산, LiDAR 배열 정렬)은 재생 밖이다 — 고치면 실제 Webots 로 확인한다.

## 환경
- macOS (Apple Silicon), Webots **R2025a** — `/Applications/Webots.app`
- Python: 저장소 `.venv` (uv, Python 3.10, `uv.lock` 고정). `./setup.sh` 가 만들고,
  `controllers/*/runtime.ini` 의 `[python] COMMAND` 로 Webots 컨트롤러가 이걸 쓰게 한다 (절대경로 — 상대경로는 Webots 가 못 찾는다).
- 주최 측 자료(라이선스가 없어 재배포하지 않는다)·사과 PROTO·YOLO 가중치·채점 월드는 `./setup.sh` 가 받거나 만든다.
- Webots API import에는 `WEBOTS_HOME=/Applications/Webots.app` 필요 (GUI는 자동 설정)

### R2025a 주의: PROTO는 원격이다
로컬에 `projects/robots/robotis/turtlebot/protos/` 가 없다. 월드가 `EXTERNPROTO` URL로
GitHub에서 받아온다. 상수 확인은 아래 원본을 읽는다:
- https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects/robots/robotis/turtlebot/protos/TurtleBot3Burger.proto
- https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects/devices/robotis/protos/RobotisLds01.proto

### TurtleBot3Burger extensionSlot 함정
`extensionSlot` 의 기본값이 `[ RobotisLds01 { } ]` 이다. 센서를 추가하려고 이 필드를
덮어쓰면 **LiDAR가 사라진다.** 반드시 `RobotisLds01 { }` 를 다시 써 준 뒤 나머지를 추가한다.

### Compass / Gyro / Accelerometer 는 이미 로봇 안에 있다
`TurtleBot3Burger.proto:278-285` 가 이 셋을 기본 이름(`compass`, `gyro`, `accelerometer`)으로
이미 포함한다. `extensionSlot` 에 또 넣으면 `'name' field value should be unique` 경고가 나고
디바이스가 중복된다. **우리가 추가하는 것은 `Camera` 하나뿐이다.**

## 측정으로 알아낸 것 (추측 아님)

| 항목 | 값 | 근거 |
|---|---|---|
| LiDAR 인덱스 0 | 로봇 **뒤쪽**(180°) | `debug/lidar_orientation.py` |
| LiDAR 인덱스 증가 방향 | **시계 방향** | 같은 실험, 오른쪽 상자로 검산 |
| 나침반 heading | `atan2(v[0], v[1])` | ENU 규약에서 유도 + 이동 방향과 대조 |
| 오도메트리 오차 | 나침반/자이로 0.07 cm, 엔코더 6.1 cm | `debug/odometry_check.py` |
| 유효 축간 거리 | 0.180 (기하값 0.16 아님) | 360° 회전 측정 |
| apartment 바닥 바퀴 보정 | 바퀴로 잰 거리 × 0.983 = 실제 | 직진 38구간 중앙값 |

## ⚠️ 바퀴가 헛돌면 오도메트리가 폭주한다
로봇이 벽에 끼면 바퀴는 계속 도는데 몸은 안 움직인다. 엔코더는 그걸 "전진" 으로
읽으므로 추정 위치가 순식간에 수십 m 씩 튄다 (실제로 49 m, 108 m 까지 갔다).
→ 주행기는 벽에 닿기 전에 멈추고, 미끄러짐(바퀴 회전 vs 나침반 회전)을 감지하면 후진한다.
안전거리가 정확도 문제이기도 하다.

## 명령어
```bash
./setup.sh                      # 처음 한 번 — .venv, runtime.ini, 주최 측 자료·가중치, 에셋 미러, 채점 월드
./check_rules.sh                # 위 규칙 위반 검사
uv run pytest tests/ -q         # 순수 알고리즘 테스트 (Webots 불필요)
uv run pytest tests/ -q -m slow # 가짜 월드에서 임무 전체를 돌리는 느린 통합 테스트
./run_headless.sh               # 대회 조건 채점 월드를 헤드리스로 (기본 worlds/apartment_competition_check.wbt)
SAR_RECORD=rec.pkl.gz ./run_headless.sh && python debug/replay_check.py rec.pkl.gz   # 녹화 → 재생 비교
```
