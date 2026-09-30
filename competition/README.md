# 대회 당일 작업 공간

연습 코드는 저장소 루트에 그대로 있다. 여기는 **당일에 새로 생기는 것만** 둔다.

## 돌아갈 곳

당일에 무엇이 망가지든 여기로 돌아온다 (16월드 47/48, 처음 보는 지도 50/54):

```bash
git diff pre-competition -- '*.py'            # 무엇을 바꿨는지 먼저 본다
git checkout pre-competition -- '*.py'        # 파이썬 코드만 대회 직전 상태로
```

`'*.py'` 로 좁힌 이유: `.` 로 되돌리면 `.gitignore` 와 이 폴더의 메모까지 옛날로
돌아간다. 메모·월드·당일 확인 사항은 남기고 **코드만** 되돌려야 한다.
하나만 되돌릴 땐 `git checkout pre-competition -- config.py` 처럼 파일을 적는다.

당일 작업은 브랜치 `competition` 에서 한다. 잘 되는 상태가 나올 때마다 **바로 커밋**한다.

## 환경 (uv)

**환경이 둘이다. 섞지 않는다.**

| 환경 | 어디 | 무엇을 | 만드는 법 |
|---|---|---|---|
| 로봇 | `.venv/` (저장소 루트) | Webots 컨트롤러, 시험, 벤치 | `uv sync` |
| 노트북 | `competition/notebook/.venv/` | 제공 노트북 (YOLO 포함) | `cd competition/notebook && uv sync` |

노트북 실행: `cd competition/notebook && uv run jupyter lab` → 커널 **"TECH WEEK 노트북 (py3.10)"**

⚠️ **반드시 `uv run` 으로 띄운다** (그냥 `jupyter lab` 금지). 8번 셀의 `%%bash pip` 은
커널이 아니라 **Jupyter 서버의 PATH** 를 쓴다. 환경을 활성화하지 않고 띄웠더니 그 `pip` 이
**miniconda base (Python 3.14)** 를 가리켜 거기에 numpy 1.23.5 를 깔려고 했다 (빌드 실패로
다행히 아무것도 안 깔렸다). `uv run` 은 `.venv/bin` 을 PATH 앞에 붙이므로 8번 셀이
"이미 설치됨" 으로 끝난다. 확인: 55개 코드 셀 전부 오류 없이 실행 (YOLO 추론 포함, 9초).

⚠️ **왜 나눴나** — 노트북 8번 셀이 `pip install numpy==1.23.5 opencv-python==4.8.0.74 ...`
를 실행한다. 로봇 환경에서 돌리면 numpy 2.2.6 → 1.23.5, OpenCV 5.0 → 4.8 로 **내려가서**
시험을 통과시킨 환경이 조용히 바뀐다. 노트북 환경은 그 버전 그대로 고정해 뒀으므로 8번
셀을 실행해도 아무것도 안 바뀐다.

⚠️ 노트북은 **사본** (`competition/notebook/`) 에서 돌린다. `00_given/` 원본은 건드리지 않는다.
YOLO 가중치 `yolo11n.pt` 도 사본 옆 `models/YOLO/` 에 받아 두었다 (COCO 에 `apple` = 47번).

## 폴더

| 폴더 | 무엇을 | 규칙 |
|---|---|---|
| `00_given/` | 주최 측이 준 것 (코드·월드·문서) | **원본 그대로. 절대 수정하지 않는다.** 고칠 땐 복사본을 고친다 |
| `01_facts.md` | 당일 확인 사항 | 받는 즉시 채운다. 출처(누가/어디서)를 같이 적는다 |
| `02_runs/` | 실행 로그 | 이름: `HHMM_무엇을바꿨나.log` (git 에 안 올라간다) |
| `03_notes/` | 팀 메모, 심사 설명 초안 | |
| `04_submit/` | 제출물 최종본 | 제출 직전에만 넣는다 |

⚠️ **대회 월드(.wbt)는 사본을 저장소의 `worlds/` 에 둔다.** Webots 는 월드 파일
기준 `../controllers/` 에서 컨트롤러를 찾는다 — `competition/` 안에 두면 우리
`sar_controller` 를 못 찾는다. 원본은 `00_given/` 에 그대로 남긴다.

## 당일 순서

1. **받은 것을 전부 `00_given/` 에 원본 그대로 넣고 커밋** ("주최 측 원본")
2. **`01_facts.md` 를 채운다** — 목표물 개수·생김새·제한 시간·제공 코드 범위·제출 형식
3. **제공 코드를 읽는다** — 디바이스 이름, 데이터 형식, 실행 주기가 우리와 같은지.
   다르면 `sensors.py` 에서 맞춘다 (Webots 를 만지는 곳은 거기뿐이다)
4. **대회 월드 사본을 `worlds/` 에** 두고, 로봇의 `controller` 필드를 `"sar_controller"` 로
5. **`config.py` 를 당일 값으로** — 아래 표. 고친 뒤엔 **반드시 import 로 확인**:
   ```bash
   find . -name "*.pyc" -delete
   ~/webots-env/bin/python3 -c "import config as c; print(c.START_X, c.START_Y, c.START_THETA, c.MISSION_TARGET_COUNT, c.MISSION_TIME_LIMIT)"
   ```
   (파일을 grep 하는 것으로는 부족하다 — 캐시가 옛 값을 물고 있어 측정을 날린 적이 있다)
6. **시험·규칙 확인 후 실행**
   ```bash
   ./check_rules.sh && ~/webots-env/bin/python3 -m pytest tests/ -q
   ./run_gui.sh worlds/<대회월드>.wbt
   ```

## 당일 바꿀 config 값

| 무엇 | config.py | 지금 값 (연습 가정) | 근거 |
|---|---|---|---|
| 시작 위치·방향 | `START_X` `START_Y` `START_THETA` | -2.5, -2.5, 0.0 | **당일 제공** (계획안: position & orientation 제공) |
| 목표물 개수 | `MISSION_TARGET_COUNT` | 3 | 연습 가정. 문서에 없음 |
| 목표물 색 | `DETECT_HUE_LOW/HIGH` `DETECT_SAT_MIN` `DETECT_VALUE_MIN` | 빨강 | 연습 가정. **당일 공개** |
| 제한 시간 | `MISSION_TIME_LIMIT` | 900.0 초 | 연습 가정. 문서에 없음 |
| 모터·센서 이름 | `LEFT/RIGHT_MOTOR_NAME` `LEFT/RIGHT_ENCODER_NAME` `LIDAR_NAME` `CAMERA_NAME` `COMPASS_NAME` `GYRO_NAME` | TurtleBot3 Burger 기준 | 로봇이 다르면 제공 코드에서 확인 |
| 바퀴 반지름·축간 | `WHEEL_RADIUS` `WHEEL_BASE` `WHEEL_BASE_ODOM` | 0.033, 0.16, 0.180 | 로봇이 다르면 PROTO 에서 확인 |
| 사람 회피 | `PEOPLE_ENABLED` | False | 사람이 있는 월드면 켤지 판단 (1단계에선 헛것 100% 였다) |

⚠️ `MISSION_TARGET_COUNT` 는 **상한** 으로 쓰인다 (오검출 억제 + "다 찾았으니 복귀").
실제 개수보다 작게 두면 **남은 목표물을 아예 안 찾고** 복귀한다. 모르면 크게 두는 쪽이 안전하다.

## 알려진 약점 (당일 조심할 것)

- **처음 보는 지도에서 92.6%** (50/54) — 연습 16월드의 100% 보다 낮다
- **좁은 시작 주머니에서 한 발도 못 움직일 수 있다** (무작위 시드 222: 0.0 m, 1/3).
  로봇이 시작 스캔 후 안 움직이면 이것이다. 로그의 `[포기]` 줄이 사유를 보여 준다
- **스캔매처는 꺼 두었다** (보정 15번 중 11번이 위치를 더 틀리게 했다).
  계획안이 Localization 에 Scan Matching 을 명시하므로 심사에서 이유를 물으면
  `docs/무엇을-빼기로-했나.md` 의 숫자로 답한다
- 근거 전부: `docs/무엇을-빼기로-했나.md`
