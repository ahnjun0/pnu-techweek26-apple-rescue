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

## 기술 질의응답 — 위치 추정·GNSS·속도 제어·경로 계획

아래 값은 대회 월드(apartment, `basicTimeStep 64`)와 `config.py` 기준이다. 괄호 안은 코드 위치다.

### 1. Localization 은 어떻게 하나

**바퀴 엔코더 오도메트리 + 나침반 방위**로 한다 (`localization.py`). 제어 주기 $\Delta t = 64\ \text{ms}$ 마다 다음을 계산한다.

**① 바퀴가 굴러간 거리** — 엔코더 누적각 $\phi_L, \phi_R$ 의 변화량과 바퀴 반지름 $r = 0.033\ \text{m}$:

$$
\Delta s_L = r\,\Delta\phi_L,\qquad
\Delta s_R = r\,\Delta\phi_R,\qquad
\Delta s = \tfrac{1}{2}\left(\Delta s_L + \Delta s_R\right)
$$

**② 방향 $\theta$** — 나침반 벡터 $\mathbf{m} = (m_x, m_y, m_z)$ (로봇 좌표계 기준 북쪽)에서 바로 읽는다.
ENU 규약에서 북쪽은 월드 $+y$ 이고, 로봇이 $\theta$ 만큼 돌아 있으면

$$
\begin{pmatrix} m_x \\ m_y \end{pmatrix}
= R(-\theta) \begin{pmatrix} 0 \\ 1 \end{pmatrix}
= \begin{pmatrix} \sin\theta \\ \cos\theta \end{pmatrix}
\quad\Longrightarrow\quad
\theta_k = \operatorname{atan2}(m_x,\ m_y)
$$

나침반은 절대 방위라 **누적 오차가 없다.** 검증 스크립트에서 정답 이동 방향과 대조해 확인했다.
(나침반이 없으면 자이로 적분 $\theta_k = \theta_{k-1} + \omega_z \Delta t$, 그것도 없으면 $\theta_k = \theta_{k-1} + (\Delta s_R - \Delta s_L)/B_{\text{odom}}$, $B_{\text{odom}} = 0.180\ \text{m}$ 순으로 대신 쓴다.)

**③ 위치** — 틱 **중간 각도**로 전진시킨다 (중점법, 곡선 주행 오차가 준다):

$$
\bar\theta = \theta_{k-1} + \tfrac{1}{2}\,\operatorname{wrap}\!\left(\theta_k - \theta_{k-1}\right),\qquad
x_k = x_{k-1} + \Delta s\cos\bar\theta,\qquad
y_k = y_{k-1} + \Delta s\sin\bar\theta
$$

시작 자세 $(x_0, y_0, \theta_0) = (-0.3,\ -7.5,\ \pi)$ 에서 누적한다.

**④ 지도** — 이 추정 위치에서 LiDAR 광선을 격자(5 cm)에 그어 칸마다 log-odds 를 누적한다 (`mapping.py`):

$$
l_i \leftarrow \operatorname{clip}\!\left(l_i + \begin{cases} +0.9 & \text{광선이 멈춘 칸 (맞음)} \\ -0.4 & \text{광선이 지나간 칸 (빔)} \end{cases},\ -3,\ +5\right),
\qquad
\text{막힘} \Leftrightarrow l_i > 0.8,\quad \text{빔} \Leftrightarrow l_i < -0.4
$$

- 측정된 오차: 대부분의 주행에서 최대 약 20 cm.
- 알려진 약점: 로봇이 걸려 **바퀴가 헛돌면** $\Delta\phi$ 는 커지는데 몸은 안 움직여, 엔코더가 "전진"으로 읽는다. apartment 한 실행에서 3초 만에 185 cm 틀어졌다 — 대응 중.
- **LiDAR 스캔 매칭**(`scanmatch.py`, 지도에 스캔을 맞춰 위치 보정)은 구현했지만 **꺼 두었다.**
  apartment 에서 켜면 오차가 21 cm → 164 cm 로 커지고 사과 방문이 1개 → 0개가 됐다.
  보정 결과로 위치를 **덮어쓰는** 방식이라 틀린 매칭 한 번이 그대로 누적되기 때문으로 본다.

### 2. GNSS 는 어떻게 쓰나 / Webots 절대 위치는?

**주행에는 GNSS 도, Webots 가 주는 절대 위치도 쓰지 않는다.** 로봇 판단에 들어가는 위치는 1번의 추정값뿐이다.

| 무엇                          | 주행 코드 (제출 파일)                            | 어디서만 쓰나                                               |
| ----------------------------- | ------------------------------------------------ | ----------------------------------------------------------- |
| GPS 디바이스                  | **코드 자체가 없다.** 월드에도 달지 않는다 | 쓰지 않는다                                                 |
| Supervisor (노드의 진짜 위치) | 쓰지 않는다                                      | 개발용 검증 스크립트(`debug/`)에서 **채점용**으로만 |

- 주최 측 예시 `controllers/tb3_ground_truth/tb3_ground_truth.py` 는 Supervisor 로 **Webots 의 절대 위치(정답)** 를 읽어
  0.1 초마다 Display 에 띄운다. 회전행렬 $R$ (행 우선 9개, `getOrientation()`)에서 방향은
  $\theta = \operatorname{atan2}(R_{10}, R_{00})$ 이다.
- 우리 검증 스크립트(`debug/truth.py`)도 **같은 방식**으로 정답을 읽는다. 채점용 월드 **사본**(`worlds/apartment_check.wbt`,
  `supervisor TRUE`)에서만 돌고, 제어 주기(64 ms)마다 다음을 기록한다. 그 값은 로봇 판단에 들어가지 않는다.

$$
e_{\text{pos}} = \left\lVert (\hat x, \hat y) - (x^\ast, y^\ast) \right\rVert,
\qquad
d_{\text{방문}} = \min_t \left\lVert (x^\ast_t, y^\ast_t) - \mathbf{p}_{\text{사과}} \right\rVert
$$

  ($\hat{\ }$ 는 추정, $^\ast$ 는 정답.)

- 이 분리는 자동 검사로 지킨다: `check_rules.sh` 가 `debug/` 밖에서 GPS·Supervisor 사용을 찾고,
  제출 파일에는 흔적이 0개인지 따로 검사한다.
- 센서 주기: 엔코더·나침반·LiDAR(360점, 최대 3.5 m)는 64 ms 로 켠다. 지도 갱신은 매 틱, 카메라는 128 ms 마다 본다.

### 3. 경로(좌표 리스트)를 받아 속도·바퀴는 어떻게 제어하나

경로는 월드 좌표 웨이포인트 $\{\mathbf{w}_0, \dots, \mathbf{w}_n\}$ 이다. 이걸 **DWA (Dynamic Window Approach)** 로 따라간다 (`follower.py`).

**① 조준점** — 경로 위에서 로봇에 가장 가까운 점부터 경로를 따라 $L = \max(0.20,\ v_{\max}T) = 0.24\ \text{m}$ 앞의 점 $\mathbf{g}$.

**② 동적 창** — 지금 속도 $(v_c, \omega_c)$ 에서 가속 한계 안에 드는 후보만 (창 폭 $\tau = 0.133\ \text{s}$):

$$
V_d = \Bigl\{ (v, \omega) \Bigm|
v \in [\max(-0.06,\ v_c - a\tau),\ \min(0.20,\ v_c + a\tau)],\ \
\omega \in [\max(-1.2,\ \omega_c - \alpha\tau),\ \min(1.2,\ \omega_c + \alpha\tau)] \Bigr\}
$$

$a = 0.6\ \text{m/s}^2$, $\alpha = 4.0\ \text{rad/s}^2$. $v$ 7개 × $\omega$ 17개 = 119 후보.

**③ 예측** — 후보마다 $T = 1.2\ \text{s}$ 동안 원호 궤적을 8 점 굴린다 (로봇 좌표계, $t_j = jT/8$):

$$
x(t) = \frac{v}{\omega}\sin(\omega t),\qquad
y(t) = \frac{v}{\omega}\bigl(1 - \cos(\omega t)\bigr)
\qquad (\omega \to 0 \text{ 이면 } x = vt,\ y = 0)
$$

**④ 거르기** — 궤적과 장애물 점(LiDAR 점 + 사람이 갈 곳의 예측점) 사이 최소 거리 $c$ 가

$$
c > r_{\text{robot}} + 0.15 = 0.28\ \text{m}
$$

인 후보만 남긴다.

**⑤ 점수** — 남은 후보 안에서 각 항을 $[0,1]$ 로 정규화($\mathcal{N}$)해 더한다:

$$
J(v,\omega) = 1.0\cdot\mathcal{N}\!\bigl(-\lVert \mathbf{p}_{8} - \mathbf{g} \rVert\bigr)
+ 0.8\cdot\mathcal{N}\!\bigl(\min(c,\ 0.8)\bigr)
- 2.0\cdot[v<0] - 1.9\cdot[\,|v| \le 0.01\,]
$$

($\mathbf{p}_8$: 궤적 끝점. 후진·정지는 감점.) $\arg\max J$ 를 고른다.

**⑥ 바퀴 속도** — 차동 구동 역기구학 (`sensors.py`, 축간 거리 $B = 0.16\ \text{m}$):

$$
\omega_L = \frac{v - \omega B/2}{r},\qquad
\omega_R = \frac{v + \omega B/2}{r},\qquad
s = \min\!\left(1,\ \frac{0.999\,\omega_{\max}}{\max(|\omega_L|, |\omega_R|)}\right),\qquad
(\omega_L, \omega_R) \leftarrow s\,(\omega_L, \omega_R)
$$

$\omega_{\max} = 6.67\ \text{rad/s}$ 는 모터의 `maxVelocity` 다 (시작할 때 Webots 에서 읽어 대조). **두 바퀴를 같은 비율로** 줄이므로
회전 반경 $v/\omega$ 는 그대로 두고 느려지기만 한다 — 어떤 경우에도 기본 로봇 속도를 넘지 않는다.

안전장치: 0.20 m 안으로 눌리면 경로를 무시하고 트인 쪽으로 빠져나온다. 안전한 후보가 하나도 없으면 멈추고 제자리에서 돈다.

### 4. 글로벌 경로 플래너는 어떻게 경로를 정하나

**점유 격자 위의 A\*** 다 (`planner.py`).

**① 못 가는 칸 (팽창)** — 벽 칸까지의 거리가 로봇 반경 + 여유 이하이면 막는다. 모르는 칸도 막는다.

$$
\text{blocked}(n) \Leftrightarrow d\bigl(n,\ \text{벽}\bigr) \le r_{\text{robot}} + m = 0.13 + 0.22 = 0.35\ \text{m}
\ \ \lor\ \ n \in \text{미지}
$$

이렇게 하면 로봇을 **점**으로 보고 계획해도 몸통이 벽에 닿지 않는다.

**② 칸 비용** — 기본 1. 사람 $\mathbf{q}$ 가 있으면 반경 $R_p = 1.2\ \text{m}$ 안을 선형으로 비싸게 매긴다:

$$
C(n) = 1 + (4.0 - 1)\cdot\max\!\left(0,\ 1 - \frac{\lVert n - \mathbf{q} \rVert}{R_p}\right)
$$

**③ A\*** — 8방향 이동. 대각선은 옆 두 칸이 모두 열려 있을 때만(모서리 뚫기 금지).

$$
f(n) = g(n) + h(n),\qquad
g(n') = g(n) + \delta\cdot C(n'),\ \ \delta \in \{1,\ \sqrt{2}\},\qquad
h(n) = \max(|\Delta r|, |\Delta c|) + (\sqrt{2} - 1)\min(|\Delta r|, |\Delta c|)
$$

$h$ 는 옥타일 거리로, $C \ge 1$ 이므로 과대추정하지 않는다 (최단 경로 보장).

**④ 다듬기** — 칸 경로에서 직선으로 보이는(막힌 칸을 지나지 않고, 원래 경로보다 비싸지 않은) 가장 먼 점까지 건너뛰어 꺾이는 곳만 웨이포인트로 남긴다.

**⑤ 다시 계획** — 0.3 초마다, 그리고 새로 본 장애물이 경로를 막으면 즉시. 평소 여유로 길이 없으면 여유를 줄여($r_{\text{robot}} + 0.15 = 0.28\ \text{m}$) 한 번 더 찾는다 (좁은 문).

**목적지는 누가 정하나** (`exploration.py`, `mission.py`):

- 탐색 중: **프론티어**(아는 빈 칸과 모르는 칸의 경계) 덩어리 $F_k$ 마다 설 자리 $\mathbf{s}_k$ 를 잡고, A\* 경로 길이가 가장 짧은 곳으로 간다.

$$
k^\ast = \arg\min_k \Bigl( \lVert \mathbf{x}_{\text{robot}} - \mathbf{w}^{(k)}_0 \rVert + \sum_{j} \lVert \mathbf{w}^{(k)}_{j+1} - \mathbf{w}^{(k)}_{j} \rVert \Bigr)
$$

- 빨간 사과를 확정하면 그쪽이 먼저다 (카메라로 본 것 우선).
- 복귀: 시작점까지 A\*. 지도상 길이 없으면(예: 보행자가 지나간 자리가 문을 막은 것처럼 기록됨) **지나온 길을 거꾸로** 따라간다 — 실제로 지나온 길은 통과할 수 있다는 것이 이미 증명된 길이다.
