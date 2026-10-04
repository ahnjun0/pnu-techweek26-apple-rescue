#!/bin/bash
# 공개 저장소에 넣을 수 없는 것을 받는다 (./setup.sh 가 부른다).
#  1. 주최 측 저장소 kyu-rae-kim/PNU-TECHWEEK-260930 (라이선스가 없어 재배포하지 않는다) → external/
#  2. 사과 PROTO 4개 + 아이콘 → protos/ (채점 월드가 ../protos/RedApple.proto 를 가리킨다)
#  3. YOLO11n 가중치 → models/YOLO/yolo11n.pt (Ultralytics, AGPL-3.0)
set -euo pipefail
cd "$(dirname "$0")/.."
ORG=external/PNU-TECHWEEK-260930
COMMIT=383de18
if [ ! -d "$ORG/.git" ]; then
  git clone -q https://github.com/kyu-rae-kim/PNU-TECHWEEK-260930.git "$ORG"
fi
git -C "$ORG" checkout -q "$COMMIT"
mkdir -p protos/icons
cp "$ORG"/protos/{Red,Green,Orange,Purple}Apple.proto protos/
cp "$ORG"/protos/icons/Apple.png protos/icons/
mkdir -p models/YOLO
if [ ! -f models/YOLO/yolo11n.pt ]; then
  curl -fsSL -o models/YOLO/yolo11n.pt \
    https://github.com/ultralytics/assets/releases/download/v8.3.0/yolo11n.pt
fi
echo "  주최 측 자료($COMMIT), 사과 PROTO, YOLO 가중치 OK"
