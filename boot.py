"""컨트롤러 공통 시작 — config 를 import 하기 **전에** 부른다.

1. 어떤 파이썬으로 도는지 찍는다 (환경 문제를 바로 알아보게).
2. 라이브러리를 확인한다 (deps.py). 없으면 크게 알리고 멈춘다.
"""
import sys


def start(who):
    print(f"[{who}] python {sys.executable}", flush=True)
    import deps
    deps.check(who)
