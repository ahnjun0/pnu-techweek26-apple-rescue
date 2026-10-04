#!/bin/bash
# clone 한 뒤 **이것 하나**만 실행하면 된다.  쓰는 법:  ./setup.sh
#
# 1. uv sync        → .venv (numpy·OpenCV·matplotlib·torch·ultralytics, 버전 고정 — uv.lock)
# 2. runtime.ini    → controllers/*/runtime.ini 에 이 컴퓨터의 .venv 절대경로를 적는다.
#                     Webots 가 우리 컨트롤러를 **이 .venv 로** 돌린다 (전역 설정을 안 건드린다).
#                     ⚠️ 상대경로는 Webots 가 못 찾는다 (실측: "../../.venv/bin/python3 was not found").
#                     근거: Webots R2025a docs/guide/controller-programming.md "Languages Settings".
#                     컴퓨터마다 달라서 저장소에는 올리지 않는다 (.gitignore).
# 3. 받기          → 공개 저장소에 넣을 수 없는 것: 주최 측 저장소(external/, 라이선스 없음 — 재배포 안 함),
#                     사과 PROTO 4개(protos/), YOLO11n 가중치(models/YOLO/, AGPL-3.0). tools/fetch_external.sh
# 4. 확인           → 라이브러리·모델 파일·Webots 설치.
# 5. 에셋·월드      → Webots 에셋 미러(약 590 MB, 처음 한 번 몇 분)를 git 으로 받고, 주최 측 apartment.wbt 에서
#                     채점용 월드 셋을 만든다 (tools/make_check_worlds.py).
#                     ⚠️ 월드가 GitHub 주소를 직접 가리키면 Webots 가 150여 개를 한꺼번에 받다가
#                        연결이 끊겨 멈출 때가 있다 (9/30 새 clone 시험). 그래서 미러판 월드도 만든다.
set -e
cd "$(dirname "$0")"
ROOT="$PWD"

command -v uv >/dev/null || { echo "❌ uv 가 없다: curl -LsSf https://astral.sh/uv/install.sh | sh"; exit 1; }

echo "== 1. uv sync"
uv sync

echo "== 2. 컨트롤러가 쓸 파이썬 (runtime.ini)"
PY="$ROOT/.venv/bin/python3"
for dir in controllers/*/; do
  name=$(basename "$dir")
  [ -f "$dir/$name.py" ] || continue
  cat > "$dir/runtime.ini" <<INI
; ./setup.sh 가 만든 파일 — 이 컴퓨터 전용 (저장소에 올리지 않는다).
[python]
COMMAND = $PY
INI
  echo "  $dir → $PY"
done

echo "== 3. 주최 측 자료·가중치"
bash tools/fetch_external.sh

echo "== 4. 확인"
"$PY" -c "from sar import deps; deps.check('setup'); print('  라이브러리 OK')"
[ -f models/YOLO/yolo11n.pt ] && echo "  YOLO 모델 OK (models/YOLO/yolo11n.pt)" || echo "  ❌ models/YOLO/yolo11n.pt 가 없다"
WEBOTS=/Applications/Webots.app/Contents/MacOS/webots
[ -x "$WEBOTS" ] && echo "  Webots OK ($WEBOTS)" || echo "  ⚠️ Webots R2025a 가 /Applications 에 없다 — https://cyberbotics.com 에서 R2025a 설치"

echo "== 5. Webots 에셋·채점 월드"
bash tools/setup_assets.sh || echo "  ⚠️ 에셋 미러를 받지 못했다 — 대회 조건 월드만 만든다 (미러판은 bash tools/setup_assets.sh 를 다시 돌린 뒤 python tools/make_check_worlds.py)"
"$PY" tools/make_check_worlds.py

cat <<MSG

준비 끝. 대회 월드(채점하며 보기):
  Webots 로 worlds/apartment_competition_check.wbt 를 연다   (대회와 같은 조건 — Webots 에셋 캐시 필요)
  인터넷 없이:  worlds/apartment_check.wbt
  헤드리스:     ./run_headless.sh            (기본이 대회 조건 월드)
MSG
