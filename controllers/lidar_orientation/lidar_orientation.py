"""debug.lidar_orientation.py 를 Webots 컨트롤러로 띄우기만 하는 얇은 껍데기.

Webots 는 controllers/<이름>/<이름>.py 만 컨트롤러로 인식하므로,
실제 내용은 debug/ 에 두고 여기서는 불러 쓰기만 한다.
"""

import os
import sys

_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

from sar import boot  # noqa: E402  (config 보다 먼저 — 라이브러리 확인)
boot.start("lidar_orientation")

from debug.lidar_orientation import main

main()
