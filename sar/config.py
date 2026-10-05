"""로봇 상수 · 지도 설정 · 튜닝 파라미터를 모두 모아 둔 곳.

받는 것: 없음 (상수만).
내놓는 것: 다른 모든 모듈이 import 하는 이름들.
핵심 아이디어: 숫자는 전부 여기에 둔다. 로봇 값은 PROTO 원본에서 읽었고 출처를 줄마다 적었다.
값마다 근거를 한 줄로 적는다. 그 값에 이르기까지의 실험은 docs/측정_기록.md 에 있다.
"""

import math

# ==========================================================================
# 1. 로봇 디바이스 이름
#    출처: TurtleBot3Burger.proto (R2025a) / RobotisLds01.proto (R2025a)
#    https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects/robots/robotis/turtlebot/protos/TurtleBot3Burger.proto
# ==========================================================================
LEFT_MOTOR_NAME = "left wheel motor"     # TurtleBot3Burger.proto:136
RIGHT_MOTOR_NAME = "right wheel motor"   # TurtleBot3Burger.proto:48
LEFT_ENCODER_NAME = "left wheel sensor"  # TurtleBot3Burger.proto:141
RIGHT_ENCODER_NAME = "right wheel sensor"  # TurtleBot3Burger.proto:53
LIDAR_NAME = "LDS-01"                    # RobotisLds01.proto:16 (name 기본값)

# Compass / Gyro 는 PROTO 몸체에 이미 들어 있다 (TurtleBot3Burger.proto:278-285).
# 월드에서 또 추가하면 이름이 충돌한다.
COMPASS_NAME = "compass"
GYRO_NAME = "gyro"

# 카메라만 월드의 extensionSlot 에 직접 추가한 디바이스다.
CAMERA_NAME = "camera"

# ==========================================================================
# 2. 로봇 기구 상수 [m]
# ==========================================================================
WHEEL_RADIUS = 0.033        # TurtleBot3Burger.proto:116,162 (Cylinder radius)
WHEEL_BASE = 0.16           # TurtleBot3Burger.proto:44,132 (좌우 anchor y = ±0.08)

# 엔코더만으로 각도를 구할 때 쓰는 "유효" 축간 거리. 기하값 0.16 이면 회전이 과소평가된다.
# 측정: empty.wbt 제자리 360.31° 회전에 엔코더 식 319.93° → 0.16 × 360.31 / 319.93 = 0.180
#       (debug/odometry_check.py "2. 제자리 360도 회전"). 로봇이 바뀌면 다시 잰다.
WHEEL_BASE_ODOM = 0.180
# 오도메트리용 바퀴 반지름 (주행 명령은 기하값 WHEEL_RADIUS). apartment 바닥에서는 바퀴로 잰
# 거리가 실제보다 1.7% 짧다 (직진 38구간 중앙값 0.983) — 바닥에 맞춰 보정한다.
WHEEL_RADIUS_ODOM = WHEEL_RADIUS / 0.983
# ⚠️ ROBOT_RADIUS 는 **계획용 패딩값** 이다. 충돌 판정에 쓰면 안 된다 (실제 몸체 외접 반경은
#    0.110 m — TurtleBot3Burger.proto:186-201). LiDAR 로도 충돌을 판정할 수 없다: 0.12 m 아래는
#    포화하고, LiDAR 는 원점보다 3 cm 뒤에 있다 (TurtleBot3Burger.proto:43-46).
ROBOT_RADIUS = 0.13         # 몸체 외접 반경 0.110 + 여유. 팽창 반경의 기준
MAX_WHEEL_SPEED = 6.67      # [rad/s] TurtleBot3 Burger 실제 상한 (≈0.22 m/s)

# LiDAR 스펙 — 출처: RobotisLds01.proto:192-197
LIDAR_RESOLUTION = 360      # horizontalResolution
LIDAR_FOV = 2.0 * math.pi   # fieldOfView 6.28318
LIDAR_MIN_RANGE = 0.12      # minRange
LIDAR_MAX_RANGE = 3.5       # maxRange

# --- LiDAR 배열 방향 (실험으로 확정) -----------------------------------------
# 문서(lidar.md:43)는 시작 각도를 못 박지 않는다. worlds/lidar_test.wbt 에서 상자를 정면 1.0 m /
# 왼쪽 1.5 m / 오른쪽 2.0 m 에 놓고 쟀다 (debug/lidar_orientation.py): 정면이 인덱스 180,
# 왼쪽 90, 오른쪽 270 → 인덱스 0 은 로봇 뒤쪽, 인덱스가 늘면 시계 방향.
# 광선 i 의 방향 angle(i) = wrap(LIDAR_ANGLE_OFFSET + LIDAR_ANGLE_SIGN * i * 2π/N) 은
# common.lidar_angles() 한 곳에만 구현한다. 로봇이 바뀌면 실험을 다시 한다.
LIDAR_ANGLE_OFFSET = -math.pi   # [rad] 인덱스 0 = 로봇 뒤쪽
LIDAR_ANGLE_SIGN = -1           # 인덱스가 늘면 시계 방향
LIDAR_ORIENTATION_VERIFIED = True

# ==========================================================================
# 3. 시뮬레이션
# ==========================================================================
# 대회 월드(apartment.wbt)의 WorldInfo basicTimeStep 이 64 다. 컨트롤러는 시작할 때
# apply_timestep(월드 주기) 로 이 값과 틱 단위 상수를 함께 맞춘다 (파일 끝).
TIME_STEP = 64              # [ms] 컨트롤러 주기. 월드 basicTimeStep 의 배수여야 한다

# ==========================================================================
# 4. 좌표계 · 지도
#    규약 전문은 common.py 상단 참고. 여기서는 숫자만 정한다.
# ==========================================================================
MAP_RESOLUTION = 0.05       # [m/cell] 셀 한 변의 길이
# 아파트 벽 27개가 x -12.4 ~ 0.0, y -13.1 ~ 0.0 에 있다 → 16x16 m 로 덮고 여유를 둔다.
MAP_ORIGIN_X = -14.0        # [m] 격자 (row=0, col=0) 셀의 왼쪽아래 모서리 월드 좌표
MAP_ORIGIN_Y = -15.0        # [m]
MAP_WIDTH_CELLS = 320       # col 개수 → x -14 ~ +2 m
MAP_HEIGHT_CELLS = 320      # row 개수 → y -15 ~ +1 m

# ==========================================================================
# 5. 시작 pose — 대회 월드(apartment.wbt)의 로봇 배치와 반드시 일치시킬 것
#    translation -0.3 -7.5 0 / rotation 0 0 1 3.14159 (서쪽을 봄)
# ==========================================================================
START_X = -0.3              # [m]
START_Y = -7.5              # [m]
START_THETA = math.pi       # [rad] -x 축을 바라봄

# ==========================================================================
# 6. 튜닝 파라미터
# ==========================================================================
MAP_UPDATE_EVERY = 4        # N 틱마다 지도 갱신 (성능)
LIDAR_RAY_STRIDE = 2        # LiDAR 광선을 N개마다 하나씩만 사용 (성능)

# --- 지도 log-odds ---------------------------------------------------------
# 셀마다 "막혔다는 확신" 을 덮어쓰지 않고 더한다. 지나간 사람의 흔적은 빈 공간 관측이 쌓이면 지워진다.
LOG_ODDS_OCCUPIED = 0.9     # 광선이 맞고 멈춘 셀에 더할 값
LOG_ODDS_FREE = -0.4        # 광선이 통과한 셀에 더할 값 (맞은 것보다 약하게)
LOG_ODDS_MAX = 5.0          # 위아래로 가둬 둔다. 안 그러면 한번 굳은 셀이 안 바뀐다
LOG_ODDS_MIN = -3.0
LOG_ODDS_OCCUPIED_THRESHOLD = 0.8   # 이 위면 "막힘" 으로 본다
LOG_ODDS_FREE_THRESHOLD = -0.4      # 이 아래면 "빈 공간" 으로 본다
                                    # 그 사이는 "모름" (프론티어 찾을 때 쓴다)

MAP_MAX_RAY_RANGE = 3.0     # [m] 이보다 먼 측정은 지도에 안 쓴다 (멀수록 부정확)

RETURN_TOLERANCE = 0.25     # [m] 시작점 복귀 완료 판정 반경

# ==========================================================================
# 안전거리 — 여기 있는 세 값에서 나머지가 유도된다
# ==========================================================================
# 흩어진 숫자를 각각 고치면 어긋난다 ("계획기는 0.21 만 떨어지면 된다는데 주행기는 0.32 안에
# 뭐가 있으면 선다" 는 모순으로 로봇이 자기 경로를 스스로 거부했다). 그래서 기준을 하나로 모았다.
#   정지거리 = 로봇반경 + 안전거리 - 추종오차
# 정지거리가 계획 여유와 같으면 추종 오차 1 mm 에도 정지가 걸린다. 둘의 차이가 추종 오차 예산이다.
# 추종 오차는 가짜 월드에서 쟀다 (중앙값 1.0~1.8 cm, 최대 9.2 cm). 로봇·속도가 바뀌면 다시 잰다.
ROBOT_CLEARANCE = 0.22          # [m] 몸 바깥으로 두고 싶은 안전거리
FOLLOW_TRACKING_BUDGET = 0.12   # [m] 측정된 추종 오차 최대치 + 약간의 여유

# --- 여기서 유도되는 값들 (직접 숫자를 쓰지 말 것) --------------------------
SAFETY_STOP_DISTANCE = ROBOT_RADIUS + ROBOT_CLEARANCE - FOLLOW_TRACKING_BUDGET

# 평소 계획 여유
PLANNER_INFLATION_MARGIN = ROBOT_CLEARANCE

# 프론티어가 장애물 **안** 인지만 보는 팽창 여유. 로봇은 프론티어를 볼 수 있는 곳에 서면 되고,
# 설 자리는 nearest_free 가 정한다. 여유를 크게 두면 벽 근처 프론티어가 거의 다 사라진다.
# 0.0 이어도 로봇 반경만큼은 팽창한다 (planner.inflate).
FRONTIER_REACHABLE_MARGIN = 0.0


# 좁은 문을 만났을 때만 쓰는 여유. 정지거리보다 조금 커야 주행기가 그 경로를 받아들인다
# (같게 두면 만들자마자 정지가 걸렸다).
PLANNER_SQUEEZE_SLACK = 0.05    # [m] 좁게 갈 때 정지거리 위로 남겨 둘 여유
PLANNER_SQUEEZE_MARGIN = (SAFETY_STOP_DISTANCE + PLANNER_SQUEEZE_SLACK
                          - ROBOT_RADIUS)

# --- 경로 계획 (planner.py) ------------------------------------------------
# ⚠️ 반드시 지켜야 하는 관계 (tests/test_config_consistency.py 가 검사한다):
#       SAFETY_STOP_DISTANCE  <  ROBOT_RADIUS + PLANNER_INFLATION_MARGIN
#    아니면 로봇이 자기가 만든 경로를 스스로 거부하고 정지→대기→후진을 반복한다.
# 모르는 칸은 기본으로 지나가지 않는다 — 지도 바깥 띠가 통째로 "모르는 칸" 이라, 허용하면 A* 가
# 벽 바깥을 빙 돌아간다. 프론티어는 정의상 빈 칸이므로 탐색에도 문제가 없다.
PLANNER_ALLOW_UNKNOWN = False       # 아직 모르는 칸을 지나가도 되는가 (호출부가 따로 허용할 수 있다)
PLANNER_UNKNOWN_COST = 1.8          # 모르는 칸을 지나는 비용 배수 (아는 길을 선호)
PLANNER_MAX_NODES = 60000           # A* 가 이만큼 까 보고도 못 찾으면 실패로 친다

# --- 벽 근처를 "비싸게" 만드는 비용층 -------------------------------------
# 갈 수는 있지만 굳이 가고 싶지 않은 곳(팽창 경계 바로 옆)에 비용을 매긴다.
PLANNER_SOFT_CLEARANCE = 0.20       # [m] 팽창 경계에서 이만큼 더 떨어지고 싶다
# 1.0 = 끔. 연습 월드와 apartment 모두에서 켜면 나빠져 1.0 으로 둔다.
PLANNER_SOFT_WEIGHT = 1.0           # 팽창 경계 바로 옆의 비용 배수 (1.0 = 끔)
# 뚫어 준 칸을 지나는 비용 배수. 1.0 = 끔 (켜면 느려졌다).
PLANNER_CARVED_COST = 1.0

# --- 프론티어 탐색 (exploration.py) ----------------------------------------
FRONTIER_MIN_CLUSTER = 4            # [칸] 이보다 작은 프론티어 덩어리는 잡음으로 본다
# 큰 경계(새 방 입구)를 가까운 작은 경계들보다 먼저 가게 한다 (대회 조건에서 서쪽 방 입구).
# 0.025 이상은 초반에 포기했다.
FRONTIER_SIZE_BONUS = 0.01          # [m/칸] 덩어리가 클수록 깎아 주는 거리
FRONTIER_MIN_DISTANCE = 0.25        # [m] 너무 가까운 프론티어는 무시 (제자리 맴돌기 방지)


# 목표는 도착 허용치의 몇 배 이상 떨어져 있어야 "가 볼 가치" 가 있는가.
# 1.0 = 사실상 끔 (FRONTIER_MIN_DISTANCE 가 그대로 쓰인다). 3.0 으로 올렸을 때 거리·시간이 늘었다.
FRONTIER_WORTH_DRIVING = 1.0
FRONTIER_BLACKLIST_RADIUS = 0.35    # [m] 실패한 목표 주변 이 반경은 다시 안 고른다
FRONTIER_MAX_FAILURES = 3           # 한 목표를 몇 번 실패하면 블랙리스트에 넣는가

# --- 경로 추종 (follower.py) ------------------------------------------------
# 느린 게 꼭 안전한 것은 아니다 — 사람이 더 빠르므로 통로를 빨리 건너 노출을 줄인다
# (가짜 보행자 월드: 0.15 → 완주 3/4, 0.20 → 4/4 로 가장 빠르고 접촉이 적었다).
FOLLOW_MAX_SPEED = 0.20         # [m/s] 전진 상한
FOLLOW_MAX_TURN = 1.2           # [rad/s] 회전 상한
FOLLOW_LOOKAHEAD = 0.20         # [m] 경로 위 이만큼 앞의 점을 보고 간다
                                # 크면 모퉁이를 자르고, 작으면 좌우로 흔들린다.
FOLLOW_WAYPOINT_TOLERANCE = 0.15  # [m] 이만큼 가까워지면 그 웨이포인트는 통과한 것
FOLLOW_GOAL_TOLERANCE = 0.20    # [m] 경로의 마지막 점에 이만큼 오면 도착
FOLLOW_TURN_GAIN = 1.8          # 방향 오차 → 회전 속도 비례 상수

# --- 안전 (follower.py) -----------------------------------------------------
# 정면은 "각도 부채꼴" 이 아니라 "로봇 폭만큼의 통로" 로 본다 — 부채꼴이면 옆으로 멀쩡히
# 떨어진 벽이 들어와 계획("가도 된다")과 주행("못 간다")이 어긋난다.
# ⚠️ 관계 (tests/test_config_consistency.py 가 검사): SAFETY_CORRIDOR_CLEARANCE < PLANNER_INFLATION_MARGIN
SAFETY_CORRIDOR_CLEARANCE = 0.03  # [m] 로봇 반경에 더할 통로 여유

# --- DWA (Dynamic Window Approach) — 주행기 ---------------------------------
# (전진속도, 회전속도) 후보를 뿌려 각각 몇 초 앞을 모의주행해 보고 점수가 제일 좋은
# 것을 고른다. 애초에 위험한 명령을 내지 않으므로 정지·대기·탈출 상태가 필요 없다.

# 너무 짧으면 사람을 예측해도 만날 지점까지 가 보지 못한다 (1.2 초 × 0.15 m/s = 18 cm).
DWA_SIM_TIME = 1.2              # [s] 후보 하나를 몇 초 앞까지 모의주행할 것인가
DWA_SIM_STEPS = 8               # 그 구간을 몇 점으로 나눌 것인가
DWA_SPEED_SAMPLES = 7           # 전진속도 후보 개수
DWA_TURN_SAMPLES = 17           # 회전속도 후보 개수
DWA_MAX_ACCEL = 0.6             # [m/s^2] 한 틱에 바꿀 수 있는 전진속도 (부드러움의 핵심)
DWA_MAX_ANG_ACCEL = 4.0         # [rad/s^2] 한 틱에 바꿀 수 있는 회전속도

DWA_REVERSE_SPEED = 0.06        # [m/s] 후보에 넣을 후진 속도 (0 이면 후진 안 함)
DWA_IDLE_SPEED = 0.01           # [m/s] 이 이하는 "전진이 아니다" 로 본다

# 후보 창의 폭은 제어 주기가 아니라 **결정 지평** 이어야 한다. 한 틱 폭이면 후진 중에는 전진
# 후보가 창에 아예 없어, 후진이 "유일해서" 뽑힌다. 가장 깊은 후진에서도 창이 전진 후보에 닿게
# 폭 > DWA_REVERSE_SPEED + DWA_IDLE_SPEED, 여유로 2배.
DWA_ACCEL_HORIZON = ((DWA_REVERSE_SPEED + 2.0 * DWA_IDLE_SPEED)
                     / DWA_MAX_ACCEL)   # [s] = 0.133
DWA_RAY_STRIDE = 3              # 장애물 점을 몇 개마다 하나씩 쓸지 (성능)

# 점수 가중치 — 셋의 상대 크기만 의미가 있다
DWA_WEIGHT_GOAL = 1.0           # 경로 위 목표점에 가까워지는가
DWA_WEIGHT_CLEARANCE = 0.8      # 장애물에서 멀리 떨어지는가
DWA_CLEARANCE_CAP = 0.8         # [m] 이보다 여유가 크면 더 쳐주지 않는다
# 후진은 "끼였을 때 빠져나오는 수단" 이지 이동 수단이 아니다 (벌점이 없으면 뒤로 기어간다).
DWA_REVERSE_PENALTY = None      # 아래에서 DWA_IDLE_PENALTY 와의 관계로 정한다

# --- 가만히 있는 후보에 매기는 벌점 -------------------------------------------
# 속도 0 후보는 궤적이 "점" 이라 여유 점수가 늘 만점이라, 목표가 멀면 "가만히 있기" 가 이겼다.
# 벌점이 (GOAL + CLEARANCE) 보다 크면 안전한 후보가 하나라도 있는 한 가만히 있기는 못 이기고,
# 안전한 후보가 없을 때만 선다. 사람이 가까이 있을 때는 벌점을 끈다 (follower.step 의 allow_idle).
DWA_IDLE_PENALTY = DWA_WEIGHT_GOAL + DWA_WEIGHT_CLEARANCE + 0.1

# 우선순위를 **관계** 로 못박는다:  전진  >  제자리 회전  >  후진
DWA_REVERSE_PENALTY = DWA_IDLE_PENALTY + 0.1

# DWA 가 후보를 "부딪힌다" 고 버리는 하한: 로봇 반경 + 이 값.
# 관계: DWA 하한 < 계획기 여유 (같으면 로봇이 자기 경로를 거부한다). 0.20 → 0.15 로 내려
# 좁은 곳에서 기어가던 시간을 줄였고, 몸체 여유는 1 cm 만 줄었다.
DWA_CLEARANCE_MARGIN = 0.15
# 그 여유로는 갈 데가 하나도 없을 때만 쓰는 최소 여유 (사람이 바짝 붙었을 때 등).
DWA_SQUEEZE_MARGIN = 0.03      # [m] 최후의 여유

# --- 임무 상태 머신 (mission.py) --------------------------------------------
# 경로를 다시 계획하는 주기. 경로 위에 새 장애물이 생기면 이와 별개로 즉시 다시 계획한다.
# ⚠️ PEOPLE_COST_RADIUS 와 짝이다 — 사람 비용은 자주 다시 계획할 때만 의미가 있다.
MISSION_REPLAN_EVERY = 0.3      # [s]
# 목표 시간 한도 = max(이 하한, 경로 길이 ÷ 실측 평균 속도 × 여유). 고정 시간은 먼 목표에 가혹했다.
MISSION_GOAL_TIMEOUT = 45.0     # [s] 가까운 목표의 하한 (아래 여유와 함께 쓴다)
MISSION_GOAL_TIMEOUT_SLACK = 2.5   # 경로 소요시간의 몇 배까지 기다려 줄 것인가
MISSION_STUCK_SECONDS = 4.0     # [s] 가라고 했는데 이만큼 안 움직이면 끼인 것이다
MISSION_STUCK_DISTANCE = 0.06   # [m] "안 움직였다" 의 기준
MISSION_BACKUP_SECONDS = 1.0    # [s] 끼였을 때 빠져나오는 시간

# --- 미끄러짐 감지 (mission._check_slip) ---------------------------------------
# apartment 카펫 가장자리에 바퀴가 걸리면 나침반 회전은 0~0.3 rad/s 인데 바퀴는 1.2 rad/s 로 돈다
# (평소 비율 0.87~1.14). 잡히면 1초 후진하고, 그 자리를 계획용 지도에 찍어 다시 가지 않는다.
SLIP_TURN_DIFF = 0.6            # [rad/s] 바퀴 회전 속도와 나침반 회전 속도의 차이
SLIP_SECONDS = 0.3              # [s] 이만큼 이어져야 미끄러짐으로 본다
SLIP_MARK_RADIUS = 0.10         # [m] 미끄러진 자리를 계획용 지도에 벽으로 찍는 반경

# 탈출이 아무리 해도 안 되면 바퀴가 헛도는 것이다. 계속 돌리면 엔코더만 쌓여 추정 위치가
# 폭주하므로 (사람에게 눌린 채 840초 → 108 m) 바퀴를 멈춘다. 성공한 실행의 최장 탈출 1.92초의 3배.
MISSION_ESCAPE_GIVEUP = 6.0     # [s] 탈출이 이만큼 안 끝나면 바퀴를 멈춘다
# 최대로 멈춰 있는 시간. 그 전에 LiDAR 가 트이면 곧바로 다시 움직인다.
MISSION_FREEZE_SECONDS = 20.0   # [s] 바퀴를 멈추고 기다리는 최대 시간
MISSION_FREEZE_CLEAR = 0.10     # [m] 눌림 판정보다 이만큼 더 트이면 다시 움직인다
# 로봇 몸이 이만큼까지 벽에 가까워지면 "눌렸다" 고 보고, 경로·목표를 제쳐 두고 빠져나온다.
# 정지거리 바로 아래 — 0.16 이면 보행자에게 밀릴 때 탈출이 늦었다.
SAFETY_PINNED_DISTANCE = 0.20   # [m] LiDAR(로봇 중심) 기준
SAFETY_PINNED_RAYS = 2          # 몇 개의 광선이 가까워야 진짜 눌린 것으로 보나 (1 이면 잡음에 속는다)
SAFETY_PINNED_TICKS = 2         # 몇 틱 연속이어야 진짜 눌린 것으로 보나 (1 이면 잡음에 속는다)
SAFETY_ESCAPE_SPEED = 1.0       # 탈출할 때 쓰는 속도 비율 (보행자가 미는 것보다 빨라야 한다)
# 탈출 방향으로 최소 이만큼은 갈 수 있어야 그쪽으로 간다 (아니면 뒤쪽 벽을 들이받는다).
SAFETY_ESCAPE_MIN_ROOM = 0.10   # [m]

# --- 움직이는 것(사람) 찾기 (people.py) --------------------------------------
# 다리로 보이는 조각을 찾는다. 문턱값은 Webots 실측 기록에 맞춰 골랐다
# (폭 0.04~0.35 / 짝 0.8 → 검출 40%, 유령 0.33/틱, 위치 오차 22.6 cm).
# 위치만 믿는다 — 한 스캔의 오차 23 cm 로는 두 스캔 차이로 속도를 낼 수 없다 (칼만이 거른다).
PEOPLE_SEGMENT_JUMP = 0.20      # [m] 이웃 광선이 이만큼 벌어지면 다른 물체로 본다
PEOPLE_WIDTH_RANGE = (0.04, 0.35)   # [m] 다리로 볼 조각의 폭
PEOPLE_PAIR_WITHIN = 0.8        # [m] 이 안에 있는 다리 둘을 사람 하나로 묶는다
# 시간 투표 — 여러 스캔에 걸쳐 같은 자리에 보인 것만 사람으로 친다.
# 투표는 "더 자주 보는 것" 이지 "더 정확히 보는 것" 이 아니다.

PEOPLE_VOTE_SCANS = 9           # 최근 몇 스캔을 함께 볼 것인가 (1 이면 투표 안 함)
# 회피는 2스캔이면 믿는다 (3 이면 빠르게 지나가는 사람을 놓쳐 접촉이 생겼다).
PEOPLE_VOTE_NEEDED = 2          # 그중 몇 번 보여야 믿을 것인가 (회피 기준)

# **계획기** 에는 더 엄하게 넘긴다 — 유령이 계획기에 들어가면 전역 경로가 왜곡된다.
# 회피는 2스캔, 계획은 3스캔.
PEOPLE_PLANNER_SCANS = 3
PEOPLE_VOTE_RADIUS = 0.30       # [m] 이 안에 있으면 같은 대상으로 본다

# --- 사람인가를 '모양' 이 아니라 '움직임' 으로 가른다 --------------------------
# 고정물(문틈·벽 끝)은 세계 좌표에서 제자리고 사람은 옮겨 간다. 가만히 선 사람은 못 잡지만,
# 그때는 고정 장애물이라 격자와 DWA 가 이미 피한다. 0.25 로 올리면 진짜 사람을 놓쳤다.
PEOPLE_MIN_SPEED = 0.15         # [m/s] 이보다 느리면 고정물로 본다

# --- 후보를 '모양' 이 아니라 '지도에 없음' 으로도 만든다 -----------------------
# 지도가 "빈 칸" 이라 아는 자리에서 광선이 돌아오면 지도에 없는 물체다. 헛것도 늘지만 놓침이
# 더 비싸다 (까다롭게 하면 진짜 사람 조각도 떨어져 더 오래 닿았다).
PEOPLE_MIN_BLOB_POINTS = 2      # 조각이 이보다 적은 점이면 잡음으로 본다
PEOPLE_OFF_MAP_SHARE = 0.5      # 조각의 점 중 이 비율 이상이 '빈 칸' 위에 있어야 한다
# 벽에서 이만큼 떨어진 덩어리만 사람 후보로 본다 (벽 표면의 양자화 잡음 제거).
PEOPLE_WALL_CLEARANCE = 0.25    # [m]
# 후보 생성기는 둘 다 쓴다 — 지도에 없는 덩어리(모양 무관)와 다리 모양(Arras et al. 단순판).
# 재현율이 올라가고, 어느 쪽이 잡든 움직임 투표가 거른다.

# --- 사람이 '갈 곳' 도 피한다 --------------------------------------------------
# Webots 보행자는 기구학적이라 궤적을 그대로 밀고 지나간다 — 우리가 미리 비켜야 한다.
# 예측을 켜면 사람과 닿은 시간이 5.9 → 4.2초로 줄었다 (한 번 스치는 것까지는 못 막는다).
DWA_PERSON_LOOKAHEAD = 1.2      # [s] 이만큼 앞의 자리까지 장애물로 본다 (0 = 끔)
DWA_PERSON_STEPS = 3            # 그 구간을 몇 점으로 나눌 것인가
# 이 거리 안의 사람만 미래 위치를 계산한다 (예측 시간 동안 로봇이 가는 거리 + 사람이 오는
# 거리 + 여유). 더 먼 사람은 우리 궤적과 만날 수 없다.
DWA_PERSON_REACH = ((FOLLOW_MAX_SPEED + 0.6) * DWA_PERSON_LOOKAHEAD
                    + ROBOT_RADIUS + 0.19)

# --- 사람 주변을 비싸게 만들기 (ROS 의 social costmap layer 와 같은 방식) ------
# 사람 둘레에 부드러운 비용을 얹어 경로 자체가 우회하게 한다. 딱딱하게 막으면 얼어붙는다.
# 반경 1.2 / 0.3초마다 다시 계획할 때 사람과 닿은 시간이 가장 짧았다 (6.6초, 끔은 12.8초).
# 이 거리 안에 사람이 있을 때만 "가만히 있어도 된다" 를 허용한다 (로봇 반경 + 사람 반경 + 여유).
PEOPLE_IDLE_RADIUS = 0.8        # [m]
PEOPLE_COST_RADIUS = 1.2        # [m] 0 이면 끔. 이 반경 안이 비싸진다

PEOPLE_COST_WEIGHT = 4.0        # 사람 바로 위의 비용 배수 (가장자리에서 1 로 잦아든다)

# --- 사람 추적: 등속 칼만 필터 (people.Tracker) -----------------------------
# 강의자료 p76-77: "Kalman Filter — CV(등속) tracking 에 유리". 여러 스캔을 누적해 속도를
# 걸러서 낸다. apartment: 사람 검출 + 칼만으로 보행자 접촉 0 초 (끄면 3.8 초).
PEOPLE_KF_MEAS_STD = 0.23       # [m] 측정(위치) 표준편차 — 실측 23 cm
PEOPLE_KF_ACCEL_STD = 1.0       # [m/s²] 등속 가정에서 벗어나는 정도 (걷다 멈추고 도는 사람)
PEOPLE_KF_GATE = 0.6            # [m] 예측 위치에서 이 안의 측정만 같은 사람으로 본다
PEOPLE_KF_FORGET = 1.0          # [s] 이만큼 못 보면 추적을 버린다
PEOPLE_KF_MIN_HITS = 3          # 이만큼 갱신된 추적만 내놓는다
PEOPLE_KF_COAST = 0.5           # [s] 한동안 안 보여도 예측 위치로 계속 내놓는다 (가려짐 대비)

# --- 스캔 정합 (scanmatch.py) ------------------------------------------------
# 바퀴가 헛돌아도 벽은 제자리에 있다. LiDAR 스캔을 지도에 맞춰 위치를 고친다
# (해커톤 공고의 "Localization — Scan Matching 기법 활용").
# ⚠️ 한 번에 고칠 수 있는 양을 좁게 둔다. 넓으면 지도가 조금만 틀어져도 엉뚱한 자리로 끌려간다.
SCANMATCH_EVERY = 8             # N 틱마다 한 번만 (성능)
SCANMATCH_RANGE = 0.10          # [m] 한 번에 x/y 로 고칠 수 있는 최대량
SCANMATCH_MIN_POINTS = 40       # 광선이 이보다 적으면 믿지 않는다
# --- 정밀 스캔 매칭 (scanmatch.match_fine) -----------------------------------
#   - 거리장을 보간해 **연속값**으로 x, y 만 고친다 (방향은 나침반이 이미 정확하다)
#   - 가우스-뉴턴. 정보행렬 JᵀJ 의 고윳값이 작은 방향(복도의 진행 방향)은 고치지 않는다
#   - 먼 점(지도에 없는 사람·새 물체)은 SCANMATCH_FINE_CAP 에서 자른다
# 문턱값 없이 언제나 조금씩 고친다 ("많이 틀어졌을 때만 고치기" 는 최악이었다).
SCANMATCH_FINE_ITERS = 6
SCANMATCH_FINE_CAP = 0.20          # [m] 이보다 벽에서 먼 점은 맞춤에 안 쓴다
SCANMATCH_FINE_MIN_EIG = 0.15      # 방향별 정보량 하한 (점 하나당, 1/칸² 단위의 평균)
SCANMATCH_FINE_BLEND = 1.0         # 찾은 보정을 이만큼만 적용한다 (1 = 전부)

# 시작하자마자 제자리에서 한 바퀴 돌아 주변 지도를 먼저 만든다.
MISSION_SCAN_TURNS = 1.15       # [바퀴] 조금 넘게 돌아 빈틈을 없앤다
MISSION_SCAN_SPEED = 0.9        # [rad/s] 시작 스캔 회전 속도
# 프론티어가 남았는데 전부 블랙리스트라면 블랙리스트를 비우고 다시 해 본다 (사람이 잠깐
# 막고 있었을 수도 있다). 목표물을 아직 다 못 찾았을 때는 더 끈질기게 매달린다.
MISSION_MAX_BLACKLIST_RESETS = 2
MISSION_MAX_BLACKLIST_RESETS_SEARCHING = 6
# 목표 프론티어 이 반경 안에 프론티어가 하나도 안 남았으면 목표를 버린다 (실패가 아니라 달성).
# 팽창 반경의 2배 — 모서리는 벽이 둘이라 설 자리까지의 거리가 겹친다.
FRONTIER_STALE_RADIUS = (ROBOT_RADIUS + PLANNER_INFLATION_MARGIN) * 2.0   # [m]

# --- 목표물 색 탐지 (detect.py) --------------------------------------------
# 빨강은 HSV 색상환의 양끝(0 근처와 179 근처)에 걸쳐 있어 두 구간으로 나눠 잡는다.
# 채도 하한이 가장 중요하다: 진짜 목표물 S 중앙값 208, 바닥 체크무늬 S 중앙값 118 (90% 131)
# — 그 사이를 가르는 150 (debug/camera_lidar_check.py 로 측정, worlds/camera_test.wbt).
# 조명이나 바닥이 다르면 다시 잰다.
DETECT_HUE_LOW = 10             # 0 ~ 이 값까지가 빨강 (0~179 척도)
DETECT_HUE_HIGH = 170           # 이 값 ~ 179 도 빨강
DETECT_SAT_MIN = 150            # 채도 하한 — 바닥과 목표물을 가르는 핵심 값
DETECT_VALUE_MIN = 60           # 명도 하한 — 너무 어두운 것은 색을 믿을 수 없다
DETECT_MIN_BLOB_PIXELS = 25     # 이보다 작은 덩어리는 잡음으로 버린다

# --- 목표물까지 거리 — 카메라만으로 잰다 (detect.camera_range) ---------------------
# 크기로 한 번, 바닥 위치로 한 번 재서 **둘이 맞아야** 받는다. 사과(꼭대기 0.10 m)는 LiDAR
# 평면(0.173 m = 슬롯 0.153 + LDS-01 0.02, TurtleBot3Burger.proto:38·RobotisLds01.proto:14)보다 낮아 LiDAR 로는 잴 수 없다.
# 녹화 프레임: 진짜 사과는 두 거리가 1~7% 안에서 맞고, 소화기는 81% 어긋났다.
TARGET_RADIUS = 0.05            # [m] 사과 반지름 (RedApple.proto:58-59 boundingObject Sphere, scale 1)
CAMERA_HEIGHT = 0.073           # [m] 확장슬롯 z 0.153 (TurtleBot3Burger.proto:38) + 대회 카메라 z -0.08 (apartment.wbt)
CAMERA_FORWARD = 0.02           # [m] 확장슬롯 x -0.03 + 대회 카메라 x 0.05
DETECT_RANGE_AGREEMENT = 0.30   # 크기 거리와 바닥 거리가 이 비율 안에서 맞아야 사과로 본다
# 먼 소화기(23x72 px)는 두 거리가 29% 로 통과했다 — 세로로 길어 모양으로 거른다 (사과 1.0~1.2).
DETECT_MAX_ASPECT = 1.5         # 세로/가로 가 이보다 크면 사과가 아니다
# 바닥에 누운 캔(26x11 px, 0.42)은 납작하다 — 사과 크기라 거리 검사를 통과했었다.
DETECT_MIN_ASPECT = 1.0 / DETECT_MAX_ASPECT   # 세로/가로 가 이보다 작으면 사과가 아니다

# --- YOLO 확인 (yolo_check.py) — 빨간 덩어리가 모양 검사를 통과했을 때만 돌린다 ---
# 녹화 프레임에서 yolo11n(conf 0.1): 사과 23장 → 모두 sports ball, 캔 26장 → bottle 22장.
# 판정은 셋: 사과 계열이면 받는다 / 병·컵이면 버린다 / 못 보면 단서(lead)로 남겨 가까이 가서 다시 본다
# (멀리 있는 사과는 YOLO 가 자주 놓친다 — 문지기로 쓰면 사과를 잃는다).
# 주최 측 예시(controllers/tb3_teleop_yolo/tb3_teleop_yolo.py:37)와 같은 자리.
# Webots 는 컨트롤러 폴더에서 실행하므로 이 상대경로가 저장소의 models/YOLO/ 를 가리킨다.
YOLO_MODEL_PATH = "../../models/YOLO/yolo11n.pt"
YOLO_DEVICE = "cpu"             # 예시와 같다. CPU 는 매번 같은 답을 낸다 (재현성)
YOLO_CONF = 0.10                # 이보다 낮은 확신의 상자는 버린다
YOLO_ACCEPT = ("sports ball", "apple", "orange")
YOLO_REJECT = ("bottle", "cup", "vase", "wine glass", "fire hydrant")

# --- 낮은 물체 (detect.LowObstacles) --------------------------------------------
# 바닥의 과일·캔은 LiDAR 평면보다 낮아 지도에도 DWA 에도 없다 — 로봇이 그대로 치고 다녔다.
# 카메라로 찾아 **계획용 지도**와 DWA 에 넣는다 (LiDAR 지도에 찍으면 광선이 지워 버린다).
YOLO_LOW_EVERY = 4               # 카메라 N 장마다 YOLO 로 낮은 물체를 찾는다 (64 ms 틱에서 약 0.5초)
YOLO_LOW_CLASSES = ("apple", "orange", "sports ball", "banana", "bottle", "cup", "vase")
# 멀리서 본 물체는 시선 방향으로 번진다 (2.5 m 로 켰을 때 점 29개 중 19개가 엉뚱한 자리였고
# 통로를 막았다). 녹화 재생으로 고른 값: 1.2 m / 0.3 m / 3회.
LOW_MAX_RANGE = 1.2              # [m] 바닥 접점으로 잰 거리가 이보다 멀면 믿지 않는다 (가까울수록 정확)
LOW_MERGE_RADIUS = 0.30          # [m] 이 안의 관측은 같은 물체로 합친다
LOW_MIN_SIGHTINGS = 3            # 이만큼 본 것만 장애물로 쓴다 (한 번 반짝한 잡음 제외)
LOW_OBSTACLE_RADIUS = 0.06       # [m] 지도에 벽으로 찍을 반경 (사과 반지름 0.05 + 여유)
# 바닥 접점 거리로 환산한 빨간 덩어리의 폭이 이보다 좁으면 낮은 물체로 치지 않는다.
# 과일·캔은 지름 6 cm 이상이다. 그보다 훨씬 좁으면 바로 앞 바닥의 물체가 아니라 멀리 있는
# 빨간 점이 수평선 아래에 걸린 것이다 — 대회 월드에서 소파 너머 9x9 점이 거실 북쪽 통로에
# 찍혀 길을 막았다. 녹화 재생(2026-10-05, 모드 T·X): 진짜 점 4/4·3/3 은 남고
# 가짜 점은 4→1·6→1 로 준다.
LOW_MIN_WIDTH = 0.02             # [m]

# --- 목표물 목록 관리 ------------------------------------------------------
DETECT_MERGE_RADIUS = 0.45      # [m] 이 안에 있으면 같은 물체로 본다
# 몇 번 봐야 "진짜 목표물" 로 인정하는가. 로그 후보 536개: 진짜는 본 횟수 하위10% 47회,
# 가짜는 중앙값 10회 → 25 에서 가짜 19/107 만 통과하고 진짜 415/429 를 지킨다.
DETECT_MIN_SIGHTINGS = 25
# 확정엔 못 미쳐도 이만큼 봤으면 "가서 확인해 볼 가치" 가 있다 (없으면 후보를 두고 복귀했다).
DETECT_VERIFY_SIGHTINGS = 4

# --- 카메라가 어디를 봤는지 기억한다 -------------------------------------------
# 탐색 종료는 LiDAR(360°) 커버리지로 판단하는데 목표물은 카메라(57°)로 찾는다 —
# 지도를 다 그려도 카메라가 안 본 곳의 목표물은 못 찾는다.
CAMERA_COVER_RAYS = 9           # 시야를 몇 갈래로 쪼개 표시할 것인가
CAMERA_COVER_EVERY = 4          # N 틱마다 표시 (성능)
DETECT_MAX_RANGE = 3.0          # [m] 이보다 먼 탐지는 위치가 부정확해 쓰지 않는다

# ── 방위만 아는 단서 (lead) ───────────────────────────────────────────────────
# 빨간 덩어리를 봤지만 위치를 못 정하면(화면 가장자리에 잘림, YOLO 가 못 봄) 방위만 남기고
# 그쪽으로 가까이 가서 다시 본다.
LEAD_MIN_SIGHTINGS = 3          # 이만큼 본 방향만 쫓는다 (한 번 반짝은 무시)
LEAD_MERGE_DEGREES = 15.0       # 이 안이면 같은 단서로 본다
LEAD_MERGE_DISTANCE = 0.6       # [m] 그리고 관측 위치가 이만큼 안이면 같은 단서
LEAD_MAX_TRIES = 2              # 같은 단서를 몇 번까지 쫓나 (헛된 추격 방지)
# 단서 방향으로 거리를 잴 수 있는 범위의 절반쯤 나아간 점을 목표로 삼는다.
LEAD_STEP_DISTANCE = DETECT_MAX_RANGE * 0.5

# --- 목표물 접근 (mission.py) ----------------------------------------------
# 계획기는 목표물에서 ROBOT_RADIUS + ROBOT_CLEARANCE 떨어진 곳까지만 데려다 준다.
# 거기에 목표물 반지름과 여유를 더해야 "도착했는데 방문 판정이 안 되는" 일이 없다.
APPROACH_TARGET_RADIUS = 0.10   # [m] 목표물이 이 정도 굵기라고 본다
APPROACH_DISTANCE = (ROBOT_RADIUS + ROBOT_CLEARANCE
                     + APPROACH_TARGET_RADIUS + 0.12)
APPROACH_TIMEOUT = 40.0         # [s] 한 목표물에 이만큼 매달리면 포기한다
# 확정 전 후보에 이만큼 가까워지면 더 다가가지 않고 그쪽을 바라보며 본 횟수를 쌓는다.
# 다가가는 경로를 따라 머리가 돌면 카메라 밖으로 놓친 채 "도착" 해 버린다 — 대회 월드
# (2026-10-05)에서 화장실 사과를 1.4 m 앞에서 10회 보고 놓친 뒤 방문으로 쳐 잃었다.
# 그때 정면으로 보는 동안 1초에 6회씩 쌓였다 (4 → 25 회면 3.5초).
VERIFY_LOOK_RANGE = 1.5         # [m]
VERIFY_LOOK_ALIGN = 0.35        # [rad] 이 안이면 카메라(반각 0.52)가 본다고 친다
VERIFY_LOOK_TIME = 6.0          # [s] 바라봐도 확정이 안 되면 건너뛴다
# 대회 월드 apartment 의 사과 7개 중 빨강이 2개다.
MISSION_TARGET_COUNT = 2        # 다 찾았다고 판단할 목표물 개수 (0 이면 끝없이 탐색)

# --- 전역 시간 예산 -----------------------------------------------------------
# 제한 시간을 다 쓰고 못 끝내면 복귀 점수까지 0 이다. 그래서 남은 시간이 "실제로 필요한 복귀
# 시간" 에 닿으면 찾은 것만 들고 돌아간다:
#   필요한 시간 = 계획기가 준 복귀 경로 길이 ÷ 지금까지의 평균 속도 × MISSION_RETURN_SLACK
# (고정 비율 0.75 로 뒀다가 복귀에 과하게 보수적이라 목표물을 잃었다.)
MISSION_TIME_LIMIT = 900.0      # [s] 0 이면 끈다. 대회 제한 시간에 맞춘다
MISSION_RETURN_SLACK = 1.5
MISSION_RETURN_RECHECK = 60     # [틱] 복귀 거리를 몇 틱마다 다시 잴 것인가 (A* 비용)
MISSION_RETURN_MIN = 30.0       # [s] 아무리 가까워도 이만큼은 남긴다
DETECT_EVERY = 6                # N 틱마다 카메라를 본다 (목표물은 갑자기 안 나타난다)

# --- 둘러보기 (SWEEP) ------------------------------------------------------
# 지도를 다 그려도 카메라(57°)가 한 번도 안 본 곳이 있을 수 있다. 탐색이 끝났는데 목표물이
# 모자라면 카메라가 안 본 곳 중 가까운 곳부터 몇 군데에서 제자리 한 바퀴를 돈다.
MISSION_SWEEP_POINTS = 20       # 최대 몇 곳에서 둘러볼 것인가
# 가까운 후보부터 이만큼만 "갈 수 있나" 를 물어본다 (미관측 칸이 수백 개일 수 있다).
MISSION_SWEEP_TRIES = 12
MISSION_SWEEP_SPACING = 1.5     # [m] 둘러보는 지점끼리 최소 이만큼 떨어뜨린다
# 탐색 **중** 둘러보기 — LiDAR 로는 다 그렸는데 카메라가 못 본 **가까운** 주머니는 떠나기 전에
# 본다. 대회 월드(2026-10-05): 110초에 화장실 입구에서 LiDAR 로 안을 다 그려 경계가 사라졌고,
# 사과는 안쪽 설비에 가려 카메라가 못 봤다 — 558초 뒤에야 돌아와 찾았다.
EXPLORE_LOOK_DISTANCE = 3.0     # [m] 로봇에서 주머니 중심까지 이 안이면 본다
EXPLORE_LOOK_POCKET = 0.25      # [m²] 이보다 넓은 주머니만
EXPLORE_LOOK_FRONTIER_CLEARANCE = 1.0   # [m] LiDAR 경계 이 안의 칸은 뺀다 (탐색이 어차피 간다)
EXPLORE_LOOK_MAX = 12           # 탐색 중 둘러보기 최대 횟수 (끝난 뒤 둘러보기와 따로 센다)
# 경계 목표에 도착했는데 경계가 그대로면, 실패로 치기 전에 같은 방향으로 아는 빈칸을 따라 이만큼까지
# 더 들어가 본다 (목표당 한 번). 좁은 문 바로 앞에서는 LiDAR 가 안쪽 모서리를 못 봐 경계가 남는다 —
# 대회 월드(2026-10-05): 110초에 화장실 문 앞에서 떠나, 화장실 사과를 558초 뒤에야 찾았다.
EXPLORE_PUSH_DISTANCE = 0.6     # [m]

# --- 복귀 (RETURN) ---------------------------------------------------------
# 경로를 못 찾으면 목표를 향해 직선을 하나 그어 DWA 에게 맡긴다 (DWA 는 부딪히는 명령을 안 낸다).
RETURN_DIRECT_AFTER = 3.0       # [s] 이만큼 경로를 못 찾으면 직선 접근으로 전환
# 오도메트리가 30 cm 쯤 틀어져 있으면 마지막 몇십 cm 를 좁히는 데 시간이 든다 (120초로는 모자랐다).
RETURN_TIMEOUT = 200.0          # [s] 이만큼 복귀하지 못하면 그 자리에서 끝낸다
# 지도에 길이 없으면 직선 대신 지나온 길을 되짚는다 — 이미 통과한 길이다
# (apartment: 문 한가운데 보행자가 남긴 15 cm 점 하나가 지도의 길을 막았다).
RETURN_CRUMB_SPACING = 0.10     # [m] 지나온 길을 이 간격으로 기록한다
# 되짚을 때 이만큼 가까운 두 점은 같은 자리로 보고 그 사이의 고리를 건너뛴다.
# 벽 두께 + 양쪽 정지거리보다 짧아야 벽 너머 점과 이어 붙이지 않는다.
RETURN_CRUMB_JOIN = 0.30        # [m]
# 지도에 길은 있는데 한 자리에서 맴돌면 들어온 길을 조금 되짚어 나온 뒤 다시 짠다.
# 대회 월드(2026-10-05): 화장실 사과 방문 자리(팽창 영역 안)에서 "주행 → 장애물 정지 →
# 제자리 회전" 을 되풀이하며 190초 동안 0.3 m 안에서 맴돌다 복귀 시간 초과로 멈췄다.
RETURN_STALL_TIME = 20.0        # [s] 이만큼
RETURN_STALL_RADIUS = 0.30      # [m] 이 반경 안에 머물면 막힌 것이다
RETURN_UNSTICK_DISTANCE = 1.5   # [m] 들어온 길을 이만큼 되짚는다
# 갈 경계가 안 보이는데 집까지도 길이 없으면 "다 봤다" 가 아니라 "갇혔다" 다 — 지나온 길로
# 빠져나온 뒤 목표물이 모자라고 마감 전이면 탐색을 이어 간다. 몇 번까지 이어 갈 것인가.
# (대회 월드 2026-10-05: 좁아진 문 안쪽에서 422초에 탐색을 끝냈다. 빠져나온 자리에서는
#  거실 쪽 경계 12개에 길이 있었다.) 0 이면 예전처럼 그 자리에서 탐색을 끝낸다.
EXPLORE_ESCAPE_RESUMES = 2

# --- 화면 표시 -------------------------------------------------------------
VIZ_UPDATE_EVERY = 20       # N 틱마다 matplotlib 창을 다시 그린다 (한 번 그리는 데 ~32 ms)
# 화면을 아예 끈다. 환경변수 SAR_VIZ=0 으로도 끌 수 있다 (run_headless.sh 가 그렇게 한다).
VIZ_ENABLED = True
VIZ_TRAIL_MAX = 3000        # 지나온 자취를 최대 몇 점까지 들고 있을지


# =============================================================================
# 틱 단위 상수를 실제 주기에 맞춘다
# =============================================================================
# 아래 "N 틱마다" 상수들은 **16 ms 주기** 에서 맞춘 값이다. 64 ms 월드에서는 같은 N 이 4배 긴
# 시간이 된다 (DETECT_EVERY=6 이 0.38초마다가 되어 사과를 3초 보고도 확정하지 못했다).
# 그래서 컨트롤러가 실제 주기를 정할 때 이 값들을 **같은 시간 간격** 이 되게 다시 맞춘다.
# 사람 검출의 틱 상수(PEOPLE_VOTE_*)는 "몇 번 봤나" 라는 뜻도 있어 넣지 않았다.
_TICK_BASE_MS = 16
_TICK_CONSTANTS = ("MAP_UPDATE_EVERY", "SAFETY_PINNED_TICKS", "CAMERA_COVER_EVERY",
                   "DETECT_EVERY", "VIZ_UPDATE_EVERY", "SCANMATCH_EVERY")
_TICK_ORIGINAL = {}


def apply_timestep(step_ms):
    """실제 컨트롤러 주기를 적고, 틱 단위 상수를 같은 시간 간격이 되게 맞춘다.

    여러 번 불러도 결과가 같다 (항상 16 ms 기준 원래 값에서 다시 계산한다).
    """
    g = globals()
    for name in _TICK_CONSTANTS:
        _TICK_ORIGINAL.setdefault(name, g[name])
        g[name] = max(1, int(round(_TICK_ORIGINAL[name] * _TICK_BASE_MS / step_ms)))
    g["TIME_STEP"] = int(step_ms)
