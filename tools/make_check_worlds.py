"""채점용 월드를 주최 측 원본(external/PNU-TECHWEEK-260930/worlds/apartment.wbt)에서 만든다.

  worlds/apartment_competition_check.wbt  원본 + 채점. 에셋은 원본 그대로 GitHub 주소 —
                                          **대회와 같은 조건** (발표 성적이 이 월드에서 나왔다).
                                          Webots 에셋 캐시에 R2025a 에셋이 있어야 멈추지 않고 열린다.
  worlds/apartment_check.wbt              위와 같고 에셋만 로컬 미러 — 인터넷 없이 열린다
  worlds/apartment_nopeople_check.wbt     로컬 미러판에서 보행자만 뺀 것

원본과 다른 곳은 아래 세 가지뿐이다 — 손으로 고치지 말고 이 스크립트를 고친다.
  1. controller "tb3_teleop" → "mission_check"   (우리 주행 + 채점)
  2. supervisor TRUE                             (정답 위치 채점 — debug/truth.py 만 쓴다)
  3. 빨간 사과 두 개에 DEF TARGET_RED_1/2        (채점 스크립트가 진짜 위치를 찾는다)

⚠️ 로컬 미러판: GitHub 주소 그대로 두면 Webots 가 150여 개를 한꺼번에 받다가 연결이 끊길 때가
   있다 ("stream N finished with error: Connection closed" — 9/30 새 clone 시험에서 10분 넘게 멈춤).
   git 으로 받은 미러(competition/webots_assets, ./setup.sh 가 받는다)를 가리키게 바꾼다.
   --remote 를 주면 셋 다 GitHub 주소로 둔다.

쓰는 법:  python3 tools/make_check_worlds.py [--remote]
"""
import importlib.util
import os
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SOURCE = os.path.join(ROOT, "external", "PNU-TECHWEEK-260930", "worlds", "apartment.wbt")
_spec = importlib.util.spec_from_file_location(
    "localize_assets", os.path.join(ROOT, "tools", "localize_assets.py"))
localize = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(localize)


def one(pattern, repl, text, what, flags=0):
    new, n = re.subn(pattern, repl, text, flags=flags)
    assert n == 1, f"원본에서 {what} 를 {n} 번 찾았다 (1 번이어야 한다)"
    return new


def with_scoring(text):
    text = one(r'\n  controller "tb3_teleop"\n',
               '\n  controller "mission_check"\n'
               '  # ⚠️ 채점용 사본이다 (원본: external/PNU-TECHWEEK-260930/worlds/apartment.wbt,\n'
               '  #    tools/make_check_worlds.py 가 만든다). 정답 위치를 재려고 supervisor 를 켰다 —\n'
               '  #    주행 로직은 쓰지 않는다 (CLAUDE.md 규칙 1).\n'
               '  supervisor TRUE\n',
               text, "로봇 컨트롤러 줄")
    count = [0]

    def name(m):
        count[0] += 1
        return f"\nDEF TARGET_RED_{count[0]} RedApple {{"
    text = re.sub(r"\nRedApple \{", name, text)
    assert count[0] == 2, f"빨간 사과를 {count[0]} 개 찾았다 (2 개여야 한다)"
    return text


def without_people(text):
    return one(r"\nPedestrian \{\n(?:  .*\n)*?\}\n", "\n", text, "보행자 노드")


def main():
    with open(SOURCE, encoding="utf-8") as f:
        base = with_scoring(f.read())
    outputs = {   # 이름: (내용, 로컬 미러로 바꿀까)
        "apartment_competition_check.wbt": (base, False),
        "apartment_check.wbt": (base, True),
        "apartment_nopeople_check.wbt": (without_people(base), True),
    }
    remote = "--remote" in sys.argv
    if not remote and not os.path.isdir(localize.MIRROR):
        sys.exit("❌ 에셋 미러가 없다 — bash tools/setup_assets.sh 먼저 (./setup.sh 가 한다)")
    for name, (text, use_mirror) in outputs.items():
        path = os.path.join(ROOT, "worlds", name)
        missing = set()
        if use_mirror and not remote:
            text = localize.rewrite(text, os.path.dirname(path), missing)
        with open(path, "w", encoding="utf-8") as f:
            f.write(text)
        left = len(localize.REMOTE.findall(text))
        print(f"만들었다: {os.path.relpath(path, ROOT)}  (원격 주소 {left} 개)")
        for m in sorted(missing):
            print("  ⚠️ 미러에 없음:", m)

if __name__ == "__main__":
    main()
