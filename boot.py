"""컨트롤러 공통 시작 — config 를 import 하기 **전에** 부른다.

1. 월드의 controllerArgs 에서 --profile=이름 을 읽어 SAR_PROFILE 로 둔다
   (환경변수가 이미 있으면 그쪽이 이긴다 — run_headless / ab.sh 가 덮을 수 있게).
   그래서 GUI 에서 월드를 그냥 열어도 대회 설정으로 돈다.
2. 어떤 파이썬으로 도는지 찍는다 (환경 문제를 바로 알아보게).
3. 라이브러리를 확인한다 (deps.py). 없으면 크게 알리고 멈춘다.
"""
import os
import sys


def start(who):
    for arg in sys.argv[1:]:
        if arg.startswith("--profile="):
            os.environ.setdefault("SAR_PROFILE", arg.split("=", 1)[1])
    print(f"[{who}] python {sys.executable}  profile={os.environ.get('SAR_PROFILE') or '(연습)'}",
          flush=True)
    import deps
    deps.check(who)
