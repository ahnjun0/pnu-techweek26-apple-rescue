"""debug/mover.py 를 Webots 컨트롤러로 띄우기만 하는 얇은 껍데기.

Webots 는 controllers/<이름>/<이름>.py 만 컨트롤러로 인식하므로,
실제 내용은 debug/ 에 두고 여기서는 불러 쓰기만 한다 (mission_check 와 같은 방식).

⚠️ 이 컨트롤러는 "움직이는 장애물" 자신을 옮긴다. 로봇의 판단 코드가 아니다.
   Webots 의 Pedestrian 도 자기 컨트롤러에서 스스로를 옮기는 방식이다.
"""

import os
import sys

_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

import boot  # noqa: E402  (config 보다 먼저 — 대회 설정·라이브러리 확인)
boot.start("mover")

from debug.mover import main

main()
