# 당일 확인 사항

받는 즉시 채운다. **출처** 칸을 비우지 않는다 (누가, 어디서 — 구두/슬라이드/코드 주석).

## 이미 문서로 확인된 것

| 항목 | 내용 | 출처 |
|---|---|---|
| 형식 | 시뮬레이션만 (실물 로봇 없음) | 질의응답 1 |
| 심사 | 목표 달성 / 기술 구현 / 주행 안정성 / 창의성 | 질의응답 2 |
| 주행 안정성 | 충돌 회피 + **충분한 안전거리** | 질의응답 2 |
| 지도 | 사전 제공 없음 | 계획안 |
| 위치 | 현재 위치 알 수 없음, **시작 지점(위치·방향)은 제공** | 계획안 |
| 목표물 | 위치 모름, **종류·시각적 특징은 제공** | 계획안 |
| 충돌 | 정적 장애물·이동하는 사람과 충돌 금지 | 계획안 |
| 비전 | "매우 단순한 작업" | 질의응답 3 |
| 코드 | 필수 코드는 제공, 전체 틀은 없음 | 질의응답 3 |
| 로봇 | 단순 Differential Drive, IMU 없음 (IMU 구성 센서는 따로) | 질의응답 4 |
| 환경 | Webots, Ubuntu 22.04 & Python 3.10 기준, Win/Mac 가능, C++ 가능 | 계획안 |
| GPU | 선택. 필요시 IT관 309/310호 | 질의응답 5 |

## 받은 자료

| 무엇 | 어디 | 받은 시점 |
|---|---|---|
| 실습 repo | https://github.com/kyu-rae-kim/PNU-TECHWEEK-260930 → `00_given/PNU-TECHWEEK-260930/` | 커밋 `383de18` |

갱신 확인: `git -C competition/00_given/PNU-TECHWEEK-260930 pull` (우리 repo 에서는 제외 — 자체 .git 이 있다)

## 당일 들은 것 (소개 중)

| 항목 | 내용 | 출처 |
|---|---|---|
| 기본 센서 | **Wheel Encoder + LiDAR** | 소개 (구두) |
| IMU | 써도 되고 안 써도 됨 (나침반·자이로·가속도계 따로 탑재) | 소개 (구두) |
| GNSS | **사용 불가** | 소개 (구두) |
| 카메라 | **Object Detection 에만** 쓴다 | 소개 (구두) |
| LiDAR | 360° 스캔, 2D | 소개 (구두) |
| LiDAR 데이터 | 최소/최대 각도, 최소/최대 거리, 측정 개수, 해상도, 샘플링 주기 | 소개 (구두) |
| 비전 도구 | OpenCV | 소개 (구두) |

## 제공 repo 에서 확인한 것 (코드 근거)

| 항목 | 값 | 근거 | 우리와 |
|---|---|---|---|
| 로봇 | TurtleBot3Burger (R2025a) | worlds/*.wbt | 같음 ✅ |
| LiDAR 이름 | `LDS-01` | tb3_lidar.py:8 | 같음 ✅ |
| LiDAR 인덱스 | [180]=앞 [0]=뒤 [90]=왼 [270]=오 | tb3_lidar.py:22-25 | **우리 실측과 같음** ✅ |
| LiDAR 주기 | `enable(100)` = 100 ms | tb3_lidar.py:9 | 우리는 매 틱 |
| 모터·엔코더 | `left/right wheel motor`, `getPositionSensor()` | tb3_teleop_sensors.py | 같음 ✅ |
| 좌표계 | ENU (기본값) | WorldInfo.wrl:20 | 같음 ✅ |
| **물리 주기** | **basicTimeStep 미지정 → 기본 32 ms** | WorldInfo.wrl:12 | ⚠️ **우리 TIME_STEP=16 은 32 의 배수가 아니다** |
| **바닥 크기** | **약 12.9 × 7.7 m** (breakroom) | breakroom_sensor_test.wbt Floor | ⚠️ **우리 지도는 8×8 m 고정 (원점 -4,-4)** |
| 카메라 | 640×480, 시야각 1.0472 rad (60°) | breakroom_sensor_test.wbt | 확인 필요 |
| **목표물** | **사과 4색** (Red/Green/Orange/Purple), **반지름 0.05 m**, 바닥에 놓임 | protos/*Apple.proto | ⚠️ 아래 |
| 나침반 표시식 | `atan2(c[1], c[0]) + 180°` | tb3_teleop_sensors.py | 우리 실측식 `atan2(c[0], c[1])` 과 다름 (표시용) |
| 검출 교재 | LAB/HSV 색 분할, YOLO11n (COCO) | tb3_segmentation.py, 노트북 | |
| Ground truth | Supervisor 사용 (교육용) | tb3_ground_truth.py | 대회 주행엔 금지 |

## ⭐ 대회형 월드: `worlds/apartment.wbt` (제공 repo)

앞에서 breakroom 하나만 보고 "대회 월드는 아직 없다" 고 단정했다 — **틀렸다.**
apartment 에 사과와 보행자가 다 있다. 대회 조건(탐색·구조 + 움직이는 사람)과 맞는다.

| 항목 | 값 | 우리 설정 | 고칠 곳 |
|---|---|---|---|
| **물리 주기** | **`basicTimeStep 64`** | `TIME_STEP = 16` ❌ 64의 배수가 아니다 | `config.TIME_STEP` → 64. 틱 단위 상수 재확인 |
| **시작 자세** | **(-0.3, -7.5), 방향 π** (서쪽을 봄) | (-2.5, -2.5, 0) ❌ | `START_X/Y/THETA` |
| **아파트 범위** | 벽 27개: **x −12.4 ~ 0.0, y −13.1 ~ 0.0** (약 12.5×13 m) | 지도 −4 ~ +4 (8×8 m) ❌ 시작점부터 지도 밖 | `MAP_ORIGIN_*`, `MAP_*_CELLS` |
| **사과** | **7개** — 빨강 2, 초록 2, 보라 2, 주황 1 (전부 z=0.05, 반지름 0.05) | 빨강 3개 가정 | 목표 색·개수는 **당일 공개** 대기 |
| 사과 위치 | 빨강 (−12.02,−3.02) (−5.34,−10.54) / 초록 (−8.86,−7.67) (−8.37,−4.98) / 보라 (−2.84,−1.34) (−7.96,−11.68) / 주황 (−8.28,−4.81) | | 검증용 (Supervisor 는 debug 에서만) |
| **보행자** | 1명, **0.2 m/s**, 집 안을 긴 경로로 돈다 | `PEOPLE_ENABLED = False` | 켤지 판단 (1단계에서 헛것 100% 였다) |
| 카메라 | 640×480, 시야각 1.0472 rad (60°), 마운트 (0.05, 0, −0.08) | | |
| LiDAR | RobotisLds01 기본값 | 같음 ✅ | |
| 로봇 컨트롤러 | `tb3_teleop` (키보드) | | 우리 사본 월드에서 `sar_controller` 로 |

⚠️ 사과 셋이 서로 가깝다: 주황 (−8.28,−4.81) · 초록 (−8.37,−4.98) 은 **20 cm** 떨어져 있다.
색으로 가르지 못하면 한 덩어리로 합쳐진다 (우리 `detect.py` 는 병합 반경 안이면 같은 목표로 본다).

## 강의자료에서 확인한 것 (`부산대 TECH WEEK Physical AI.pdf`, 148쪽 — 규칙·채점 슬라이드는 없음)

| 쪽 | 내용 | 우리에게 |
|---|---|---|
| **p73** | **시뮬레이션 주요 전제: Kidnapped Robot Problem 다루지 않음(이동의 연속성), 초기 위치·방향을 아는 상태로 시작 — "Odometry 만으로도 위치 추정 가능"** | ⭐ 스캔매처를 끈 판단의 **공식 근거**. 심사에서 인용 |
| p75 | 예측·갱신 주기: 오도메트리 예측 30 Hz, Scan-to-Map 갱신 5~10 Hz | |
| p14 | LiDAR 데이터 구조: 최소/최대 각도·거리, 측정 개수, 해상도, 회전·샘플링 주기, 거리·강도 | 소개에서 들은 것과 같다 |
| p17 | Wheel Encoder: 미끄러짐 시 누적 오차 증가 | CLAUDE.md 의 "바퀴 헛돎" 경고와 같다 |
| p53~55 | 점유격자 (free/occupied/unknown), 베이즈 필터, log-odds | 우리 mapping.py 와 같은 방식 |
| p96~99 | Costmap 값: Unknown 255, Occupied 254, Inscribed 253, Inflation 1~252, Free 0 | ROS nav2 관례 |
| **p105** | **Frontier Exploration: ① 후보(Free 옆 Unknown) ② 목표 선택(거리·예상 cost·새로 볼 영역 크기) ③ Global Path ④ Local Planner** | ⭐ **우리 방식과 같다**. 우리는 ②를 "A* 경로 길이" 하나로 단순화 — 심사 설명거리 |
| p109 | Local Planner ~20 Hz (DWA / TEB / MPPI), Global ~1 Hz | 우리는 DWA |
| p116~126 | **Pure Pursuit (Look-ahead)**: 곡률 κ = 2y/(x²+y²), ω = vκ. 길면 모서리를 자르고, 짧으면 지그재그 | 교재의 기본 제어기. 우리는 DWA 에 look-ahead 조준점을 쓴다 |
| p127~147 | Decision Making: FSM, Behavior Tree (Reactive Fallback, Recovery Node 등) | 우리 mission.py 는 FSM (SCAN→EXPLORE⇄APPROACH→SWEEP→RETURN→DONE) |

### ⚠️ LiDAR 는 사과를 못 본다

LiDAR 스캔 평면 z = 0.153 m (config.py:60) 인데 사과 꼭대기는 z = 0.10 m 다.
- 우리 `detect.py` 는 "카메라 = 방위, LiDAR = 거리" 로 목표물 위치를 낸다 →
  사과 방향 LiDAR 는 **뒤의 벽까지** 재므로 **위치가 틀린다**
- LiDAR 지도에 사과가 없으므로 **사과에 부딪힐 수 있다** (심사: 충돌 감점)
→ 거리는 **카메라만으로** 내야 한다 (알려진 크기 0.05 m 또는 바닥 평면 가정)

## 당일 받을 것

| 항목 | 받은 값 | 출처 | config 반영 | 확인(import) |
|---|---|---|---|---|
| 시작 위치·방향 | | | `START_X/Y/THETA` | ☐ |
| 목표물 개수 | | | `MISSION_TARGET_COUNT` | ☐ |
| 목표물 생김새 (색·모양·크기) | | | `DETECT_*` | ☐ |
| 제한 시간 | | | `MISSION_TIME_LIMIT` | ☐ |
| 아레나 크기 | | | (지도 크기 확인) | ☐ |
| 이동하는 사람 유무 | | | `PEOPLE_ENABLED` | ☐ |
| 로봇 기종 / 디바이스 이름 | | | `*_NAME` | ☐ |
| 제공 코드 범위·데이터 형식 | | | `sensors.py` | ☐ |
| 제출 형식 (무엇을, 어떻게) | | | `04_submit/` | ☐ |
| 심사 시각·방식 (발표? 시연?) | | | `03_notes/` | ☐ |

## 당일 들은 것 (메모)

-
