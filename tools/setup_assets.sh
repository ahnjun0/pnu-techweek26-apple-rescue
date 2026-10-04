#!/bin/bash
# Webots R2025a 에셋을 competition/webots_assets 에 받는다 (약 590 MB, 몇 분 걸린다).
#
# 왜 필요한가: apartment 월드는 여는 순간 GitHub 에서 PROTO·텍스처 150여 개를 받는다.
#   인터넷이 느리거나 끊기면 다운로드 실패(GOAWAY)로 Webots 가 튕긴다.
#   worlds/apartment_check.wbt 는 이 폴더를 가리키므로 **먼저 이걸 받아야 열린다.**
# 쓰는 법: bash tools/setup_assets.sh
set -e
cd "$(dirname "$0")/.."
ROOT="$PWD"
if [ -d competition/webots_assets/.git ]; then echo "이미 있다: competition/webots_assets"; exit 0; fi
mkdir -p competition
git clone --depth 1 --branch R2025a --filter=blob:none --sparse \
  https://github.com/cyberbotics/webots.git competition/webots_assets
cd competition/webots_assets
git sparse-checkout set \
  projects/appearances/ \
  projects/devices/robotis/ \
  projects/humans/pedestrian/ \
  projects/objects/animals/ \
  projects/objects/apartment_structure/ \
  projects/objects/backgrounds/ \
  projects/objects/balls/ \
  projects/objects/bathroom/ \
  projects/objects/bedroom/ \
  projects/objects/cabinet/ \
  projects/objects/chairs/ \
  projects/objects/computers/ \
  projects/objects/drinks/ \
  projects/objects/factory/containers/ \
  projects/objects/factory/fire_extinguisher/ \
  projects/objects/factory/pipes/ \
  projects/objects/fruits/ \
  projects/objects/garden/ \
  projects/objects/kitchen/breakfast/ \
  projects/objects/kitchen/components/ \
  projects/objects/kitchen/fridge/ \
  projects/objects/kitchen/oven/ \
  projects/objects/kitchen/utensils/ \
  projects/objects/lights/ \
  projects/objects/living_room_furniture/ \
  projects/objects/paintings/ \
  projects/objects/plants/ \
  projects/objects/school_furniture/ \
  projects/objects/solids/ \
  projects/objects/stairs/ \
  projects/objects/tables/ \
  projects/objects/telephone/ \
  projects/objects/television/ \
  projects/objects/toys/ \
  projects/objects/traffic/ \
  projects/objects/trees/ \
  projects/robots/robotis/turtlebot/ \
  projects/default/worlds/textures/ \
  projects/bounding_objects/ \
  projects/objects/factory/tools/ \
  projects/objects/geometries/ \
  projects/objects/shapes/ \
  projects/samples/environments/indoor/worlds/textures/ \
  projects/objects/floors/ \
  >/dev/null
cd "$ROOT" && python3 tools/localize_assets.py mirror
echo "완료: $(du -sh competition/webots_assets | cut -f1)"
