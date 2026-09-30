# sar_rescue — Autonomous Mobile Robot의 Search & Rescue

TurtleBot3 Burger 가 `apartment` 를 스스로 탐색해 **빨간 사과**를 찾아 가까이 간 뒤
**시작 지점으로 돌아온다.** 벽·가구·걸어 다니는 사람과 부딪히지 않는다.

- 제출물: `controllers/sar_rescue/sar_rescue.py` (파이썬 파일 하나) + 이 README
- YOLO 모델: `models/YOLO/yolo11n.pt` (주최 측 예시 `tb3_teleop_yolo.py` 와 같은 자리)
- GPS·Supervisor 는 쓰지 않는다. 위치는 바퀴 엔코더 + 나침반, 주변은 LiDAR + 카메라로만 안다.

## 구조 (주최 측 repo 와 같다)

```
controllers/sar_rescue/sar_rescue.py   ← 제출 컨트롤러
worlds/apartment.wbt                   ← 원본에서 controller 만 "tb3_teleop" → "sar_rescue"
models/  protos/  (나머지)             ← 주최 측 repo 그대로
```

## 재현 환경

아래 환경에서 만들고 측정했다. **시뮬레이션은 결정적이다** — 같은 환경·같은 월드면
몇 번을 돌려도 같은 결과가 나온다 (로그가 소수점까지 같다).

| 항목 | 버전 |
|---|---|
| Webots | **R2025a** |
| OS | macOS (Apple Silicon) |
| Python | 3.10.21 |
| numpy | 2.2.6 |
| opencv-python | 5.0.0.93 |
| matplotlib | 3.10.9 (지도 창 표시용) |
| torch | 2.14.0 (CPU 로 추론) |
| ultralytics | 8.4.123 (YOLO11n) |

```bash
python3.10 -m venv ~/sar-env
~/sar-env/bin/pip install numpy==2.2.6 opencv-python==5.0.0.93 matplotlib==3.10.9 \
    torch==2.14.0 ultralytics==8.4.123
```

Webots → **Preferences → General → Python command** 에 `~/sar-env/bin/python3` 의 절대경로를 넣는다.

## 실행

1. Webots R2025a 로 `worlds/apartment.wbt` 를 연다 (처음 열 때 인터넷으로 에셋을 받는다).
2. 재생(▶)을 누르면 스스로 움직인다. 조작할 것은 없다.
3. 콘솔에 상태가 한 줄씩 찍히고, 별도 창에 로봇이 그린 지도·경로·찾은 사과가 뜬다.
4. 시작 지점에 돌아오면 멈추고 `[[DONE]]` 을 찍는다.

**라이브러리가 없으면** 시작하자마자 콘솔에 `ERROR: 필요한 라이브러리가 없습니다` 와
빠진 것, 설치 명령(위 버전 그대로), 지금 쓰는 파이썬 경로를 크게 띄우고 멈춘다.
**YOLO 모델 파일이 없어도** 같은 방식으로 찾은 경로와 받는 법을 띄우고 멈춘다.
조용히 다르게 돌지 않는다.

⚠️ ultralytics 최신판은 numpy 2.2 를 받지 않아 numpy 를 1.26 으로 내린다. 위 버전을 그대로 쓴다.

## 규칙 준수

| 규칙 | 어떻게 |
|---|---|
| 기본 로봇 속도를 넘지 않는다 | 전진 상한 0.20 m/s (Burger 한계 약 0.22 m/s). 바퀴 명령은 매번 모터의 `maxVelocity`(6.67 rad/s, 시작 시 Webots 에서 읽어 대조)를 넘지 않게 같은 비율로 줄인다 |
| GNSS 사용 금지 | GPS 디바이스를 열지 않는다. 월드에 GPS 를 추가하지 않는다 |
| 절대 위치 금지 | Supervisor 를 쓰지 않는다 |

## 어떻게 동작하나

```
SCAN → EXPLORE ⇄ APPROACH → RETURN → DONE
```

| 단계 | 방법 | 강의자료 |
|---|---|---|
| 위치 추정 | 엔코더 오도메트리 + 나침반 방위 (바퀴 반지름은 apartment 바닥에 맞춰 0.0336 m 로 보정) | p57-62 |
| 지도 | LiDAR 점유 격자 (log-odds) | p53-55 |
| 탐색 | 프론티어 — 갈 수 있는 미탐색 경계 중 A* 경로가 가장 짧은 곳 | p103, p105 |
| 경로 | 장애물 팽창 + A*, 0.3 초마다·막히면 즉시 재계획 | p96-103 |
| 주행 | DWA (속도 후보를 굴려 부딪히지 않는 것 중 최선) | p109 |
| 사과 인식 | HSV 빨강 마스크 → 덩어리 → 크기로 잰 거리와 바닥 접점으로 잰 거리가 맞고, 세로/가로가 둥근 것(0.67~1.5)만 | p29-39 |
| 사과 확인 | **빨간 덩어리가 위 검사를 통과한 장에서만** YOLO11n 을 돌린다. sports ball·apple·orange 면 받고, bottle·cup 이면 버리고, 못 보면 가까이 가서 다시 본다 | p42-50 |
| 복귀 | A* 로 집까지. 지도에 길이 없으면 **지나온 길을 되짚는다** | |
| 위치 보정 | 정밀 스캔 매칭: LiDAR 스캔을 지도의 거리장에 맞춰 x, y 를 연속값으로 고친다 (가우스-뉴턴, 방향은 나침반) | p63-72 |
| 낮은 물체 | LiDAR(약 17 cm 높이)에 안 보이는 과일·캔을 카메라(빨간 덩어리, YOLO)로 찾아 계획용 지도·DWA 에 장애물로 넣는다 | |
| 미끄러짐 | 바퀴로 계산한 회전과 나침반 회전이 0.3초 넘게 어긋나면 후진하고 그 자리를 피한다 (카펫 턱) | |
| 보행자 | LiDAR 로 움직이는 물체를 찾고 등속 칼만 필터로 속도를 걸러 DWA 가 갈 곳을 피한다 | p76-77 |

## 측정 결과 (apartment, 보행자 있음)

빨간 사과 1 / 2 방문, 과일·캔 0 / 8 밀림, 위치 오차 최대 9.3 cm, 364초에 복귀(시작점까지 실제 0.24 m),
보행자 접촉 0초.

## 소스

한 파일은 모듈 여러 개를 빌드 스크립트로 합친 것이다. 읽기 쉬운 원본·테스트·측정 기록은
원본 저장소에 있다 (`tools/build_submission.py` 로 이 폴더를 다시 만든다).
