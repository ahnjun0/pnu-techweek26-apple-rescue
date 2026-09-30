#!/bin/sh
# CLAUDE.md 의 절대 규칙을 실제로 검사한다.  사용:  ./check_rules.sh
# 통과하면 exit 0, 위반이 있으면 위반 줄을 찍고 exit 1.
fail=0

# ⚠️ 우리 코드만 검사한다. 주최 측 원본(competition/00_given/)은 교육용으로
#    Supervisor 를 쓰고(tb3_ground_truth), 가상환경(.venv/)은 남의 패키지다.
#    둘 다 검사에 넣으면 "위반" 이 떠서 진짜 위반을 가린다.
EXCLUDE="--exclude-dir=.claude --exclude-dir=00_given --exclude-dir=given --exclude-dir=submission --exclude-dir=webots_assets --exclude-dir=.venv --exclude-dir=.git"

echo "규칙 1 — GPS/Supervisor 는 debug/ 밖에서 쓰면 안 된다"
hits=$(grep -rn $EXCLUDE --include="*.py" -E "read_gps|enable_gps|getDevice\\(.gps.\\)|Supervisor" . \
       | grep -vE "^\.?/?(debug/|tools/)")
if [ -n "$hits" ]; then echo "$hits"; fail=1; else echo "  OK"; fi

echo "규칙 2 — Webots import 는 sensors.py / controllers/ / debug/ 에만"
hits=$(grep -rn $EXCLUDE --include="*.py" -E "^from controller import|^import controller" . \
       | grep -vE "^\.?/?(debug/|controllers/|sensors\.py:)")
if [ -n "$hits" ]; then echo "$hits"; fail=1; else echo "  OK"; fi

echo "규칙 4 — 좌표 변환은 common.py 에만 (MAP_RESOLUTION 직접 나눗셈 금지)"
hits=$(grep -rn $EXCLUDE --include="*.py" "MAP_RESOLUTION" . \
       | grep -vE "^\.?/?(common\.py:|config\.py:|tests/)")
if [ -n "$hits" ]; then echo "$hits"; fail=1; else echo "  OK"; fi

echo "규칙 1b — 제출 파일(submission/)에는 GPS·Supervisor 흔적이 하나도 없어야 한다"
SUB=submission/controllers/sar_rescue/sar_rescue.py
if [ -f "$SUB" ]; then
  n=$(grep -cE 'GPS_NAME|read_gps|enable_gps|Supervisor|supervisor|[.]gps[^a-z_]' "$SUB")
  if [ "$n" -gt 0 ]; then echo "  $n 줄 발견 — python3 tools/build_submission.py 로 다시 만든다"; fail=1; else echo "  OK"; fi
else echo "  (아직 없음 — python3 tools/build_submission.py)"; fi

[ "$fail" -eq 0 ] && echo "\n전부 통과" || echo "\n위반 있음"
exit $fail
