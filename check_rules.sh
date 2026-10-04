#!/bin/sh
# CLAUDE.md 의 절대 규칙을 실제로 검사한다.  사용:  ./check_rules.sh
# 통과하면 exit 0, 위반이 있으면 위반 줄을 찍고 exit 1.
fail=0

# ⚠️ 우리 코드만 검사한다. setup 이 받는 주최 측 원본(external/)은 교육용으로
#    Supervisor 를 쓰고(tb3_ground_truth), 가상환경(.venv/)·에셋 미러는 남의 코드다.
EXCLUDE="--exclude-dir=.claude --exclude-dir=external --exclude-dir=webots_assets --exclude-dir=.venv --exclude-dir=.git"

echo "규칙 1 — GPS/Supervisor 는 debug/ 밖에서 쓰면 안 된다"
hits=$(grep -rn $EXCLUDE --include="*.py" -E "read_gps|enable_gps|getDevice\\(.gps.\\)|Supervisor" . \
       | grep -vE "^\.?/?(debug/|tools/)")
if [ -n "$hits" ]; then echo "$hits"; fail=1; else echo "  OK"; fi

echo "규칙 2 — Webots import 는 sar/sensors.py / controllers/ / debug/ 에만"
hits=$(grep -rn $EXCLUDE --include="*.py" -E "^from controller import|^import controller" . \
       | grep -vE "^\.?/?(debug/|controllers/|sar/sensors\.py:)")
if [ -n "$hits" ]; then echo "$hits"; fail=1; else echo "  OK"; fi

echo "규칙 4 — 좌표 변환은 sar/common.py 에만 (MAP_RESOLUTION 직접 나눗셈 금지)"
hits=$(grep -rn $EXCLUDE --include="*.py" "MAP_RESOLUTION" . \
       | grep -vE "^\.?/?(sar/common\.py:|sar/config\.py:|tests/)")
if [ -n "$hits" ]; then echo "$hits"; fail=1; else echo "  OK"; fi

[ "$fail" -eq 0 ] && echo "\n전부 통과" || echo "\n위반 있음"
exit $fail
