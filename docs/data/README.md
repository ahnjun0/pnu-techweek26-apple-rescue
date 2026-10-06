# 재현용 데이터

다시 돌려서는 얻을 수 없는 기록과, 커버리지 비교의 기준을 둔다. 쓰는 법은 저장소 README 의 "재현하기".

## `2026-09-30_발표본기록/`

README "결과" 표의 발표본 기록이다 — 9/30 21:52, 발표본 코드, 에셋 미러 월드(`worlds/apartment_check.wbt`).
사과 2/2, 609.1초 복귀, 시작점까지 실제 0.22 m.

Webots 는 켤 때마다 몇 가지 시작 모드 중 하나로 뜨고, 모드마다 결과가 틱까지 같다. 이 실행의 모드는
그 뒤(10/4~5) 33번 켜도 다시 나오지 않았다 — 그래서 재실행 대신 이 기록으로 비교한다.

| 파일 | 내용 |
|---|---|
| `tape.npz` | 틱(64 ms)마다: `t` 시각, `pose` 추정 위치 (x, y, θ), `ranges` LiDAR 360개, `truth` 진짜 위치 (x, y), `cmd` 명령 (v, ω) |
| `trace.csv` | 틱마다 상태 — 추정·진짜 위치, 명령, 상태 기계, 목표, 주행기 사정, 상태 문구 |
| `summary.txt` | 채점 요약 (`debug/mission_check.py` 가 찍은 것) |

⚠️ `tape.npz` 의 `person` 은 보행자가 아니다. "첫 번째 움직이는 물체" 를 적었는데 apartment 에서는
탁자 위 노트북이었다. 보행자 위치는 월드의 궤적·속도로 계산한다 (`docs/발표/make_gifs.py` 의 `pedestrian_clock`).

## `coverage_reference.npz`

카메라 커버리지를 셀 공통 기준 지도 (`debug/coverage_check.py`). `free` 는 320×320 (5 cm 칸) 의 빈 칸 표시,
106 m². 네 실행 — 위 발표본 기록, main 대회 조건, detour-hold 의 3091f2d, detour-hold 의 f117ecb — 의 마지막
지도에서 둘 이상이 빈 칸이라고 한 칸이다. 같은 방법으로 다시 만들려면:

```bash
uv run python debug/coverage_check.py --make-reference <저장할 .npz> <실행 폴더> ...
```
