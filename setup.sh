#!/bin/bash
# clone 한 뒤 **이것 하나**만 실행하면 된다.  쓰는 법:  ./setup.sh
#
# 1. uv sync        → .venv (numpy·OpenCV·matplotlib·torch·ultralytics, 버전 고정 — uv.lock)
# 2. runtime.ini    → controllers/*/runtime.ini 에 이 컴퓨터의 .venv 절대경로를 적는다.
#                     Webots 가 우리 컨트롤러를 **이 .venv 로** 돌린다 (전역 설정을 안 건드린다).
#                     ⚠️ 상대경로는 Webots 가 못 찾는다 (실측: "../../.venv/bin/python3 was not found").
#                     근거: Webots R2025a docs/guide/controller-programming.md "Languages Settings".
#                     컴퓨터마다 달라서 저장소에는 올리지 않는다 (.gitignore).
# 3. 확인           → 라이브러리·모델 파일·Webots 설치.
# 4. 에셋          → Webots 에셋 미러(약 590 MB, 처음 한 번 몇 분)를 git 으로 받고, 채점용 월드가 그걸 가리키게 한다.
#                     ⚠️ 월드가 GitHub 주소를 직접 가리키면 Webots 가 150여 개를 한꺼번에 받다가
#                        연결이 끊겨 멈춘다 (9/30 새 clone 시험). git 으로 받는 쪽이 안정적이다.
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

echo "== 3. 확인"
"$PY" -c "from sar import deps; deps.check('setup'); print('  라이브러리 OK')"
[ -f models/YOLO/yolo11n.pt ] && echo "  YOLO 모델 OK (models/YOLO/yolo11n.pt)" || echo "  ❌ models/YOLO/yolo11n.pt 가 없다"
WEBOTS=/Applications/Webots.app/Contents/MacOS/webots
[ -x "$WEBOTS" ] && echo "  Webots OK ($WEBOTS)" || echo "  ⚠️ Webots R2025a 가 /Applications 에 없다 — https://cyberbotics.com 에서 R2025a 설치"

echo "== 4. Webots 에셋"
bash tools/setup_assets.sh
"$PY" tools/make_check_worlds.py

cat <<MSG

준비 끝. 대회 월드(채점하며 보기):
  Webots 로 worlds/apartment_check.wbt 를 연다   (인터넷 없이 열린다)
  또는  ./run_headless.sh worlds/apartment_check.wbt 1500
MSG
