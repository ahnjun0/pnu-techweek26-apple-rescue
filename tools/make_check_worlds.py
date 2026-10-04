"""채점용 월드 사본을 주최 측 원본(competition/given/worlds/apartment.wbt)에서 만든다.

  worlds/apartment_check.wbt           원본 + 채점 (보행자 있음)
  worlds/apartment_nopeople_check.wbt  위에서 보행자만 뺀 것

원본과 다른 곳은 아래 네 가지뿐이다 — 손으로 고치지 말고 이 스크립트를 고친다.
  1. controller "tb3_teleop" → "mission_check"   (우리 주행 + 채점)
  2. controllerArgs ["--profile=apartment"]      (대회 설정. boot.py 가 읽는다)
  3. supervisor TRUE                             (정답 위치 채점 — debug/truth.py 만 쓴다)
  4. 빨간 사과 두 개에 DEF TARGET_RED_1/2        (채점 스크립트가 진짜 위치를 찾는다)

에셋은 **로컬 미러**(competition/webots_assets, ./setup.sh 가 받는다)를 가리키게 바꾼다.
⚠️ GitHub 주소 그대로 두면 Webots 가 150여 개를 한꺼번에 받다가 연결이 끊긴다
   ("stream N finished with error: Connection closed" — 9/30 새 clone 시험에서 10분 넘게 멈춤).
   git 으로 미러를 받는 쪽이 안정적이다. 원본 주소가 필요하면 --remote.

쓰는 법:  python3 tools/make_check_worlds.py [--remote]
"""
import importlib.util
import os
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SOURCE = os.path.join(ROOT, "competition", "given", "worlds", "apartment.wbt")
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
               '  controllerArgs [\n    "--profile=apartment"\n  ]\n'
               '  # ⚠️ 채점용 사본이다 (원본: competition/given/worlds/apartment.wbt,\n'
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
    outputs = {
        "apartment_check.wbt": base,
        "apartment_nopeople_check.wbt": without_people(base),
    }
    remote = "--remote" in sys.argv
    if not remote and not os.path.isdir(localize.MIRROR):
        sys.exit("❌ 에셋 미러가 없다 — bash tools/setup_assets.sh 먼저 (./setup.sh 가 한다)")
    for name, text in outputs.items():
        path = os.path.join(ROOT, "worlds", name)
        missing = set()
        if not remote:
            text = localize.rewrite(text, os.path.dirname(path), missing)
        with open(path, "w", encoding="utf-8") as f:
            f.write(text)
        left = len(localize.REMOTE.findall(text))
        print(f"만들었다: {os.path.relpath(path, ROOT)}  (원격 주소 {left} 개)")
        for m in sorted(missing):
            print("  ⚠️ 미러에 없음:", m)

if __name__ == "__main__":
    main()
