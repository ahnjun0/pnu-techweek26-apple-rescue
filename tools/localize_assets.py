"""Webots 원격 에셋 주소를 로컬 미러(competition/webots_assets) 경로로 바꾼다.

왜: 원격 주소면 여는 순간 GitHub 에서 받는다. 느리거나 끊기면 Webots 가 튕긴다.

쓰는 법:
  python3 tools/localize_assets.py mirror
      미러 안 PROTO 들의 "webots://projects/..." 를 상대경로로 바꾼다 (setup_assets.sh 가 부른다).
  python3 tools/localize_assets.py world 원본.wbt 사본.wbt
      월드의 원격 주소를 사본 위치 기준 상대경로로 바꿔 사본을 쓴다. 미러에 없는 파일은 알려 준다.

큰따옴표 안 주소만 바꾼다 — TexturedBackground 의 JS 템플릿('webots://...')은 건드리지 않는다.
"""
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
MIRROR = os.path.join(os.path.dirname(HERE), "competition", "webots_assets")
REMOTE = re.compile(r'"(?:webots://|https://raw\.githubusercontent\.com/cyberbotics/webots/R2025a/)'
                    r'(projects/[^"]+)"')


def rewrite(text, out_dir, missing):
    def repl(m):
        target = os.path.join(MIRROR, m.group(1))
        if not os.path.exists(target):
            missing.add(m.group(1))
            return m.group(0)
        return '"' + os.path.relpath(target, out_dir).replace(os.sep, "/") + '"'
    return REMOTE.sub(repl, text)


# PROTO 안 JS 템플릿이 만드는 주소 ('webots://projects/...'). Webots 는 이걸 **설치 폴더** 에서
# 찾는데 R2025a 설치본에는 에셋이 없다 — TexturedBackground 의 배경 조명 텍스처가 빠져
# 카메라 색이 달라졌다 (9/30). 미러 안의 상대경로로 바꾼다.
TEMPLATE = re.compile(r"'webots://(projects/[^']+)'")


def rewrite_template(text, out_dir, missing):
    def repl(m):
        target = os.path.join(MIRROR, m.group(1))
        if not os.path.exists(target):
            missing.add(m.group(1))
            return m.group(0)
        return "'" + os.path.relpath(target, out_dir).replace(os.sep, "/") + "'"
    return TEMPLATE.sub(repl, text)


def mirror():
    changed, missing = 0, set()
    for root, _, files in os.walk(os.path.join(MIRROR, "projects")):
        for name in files:
            if not name.endswith(".proto"):
                continue
            path = os.path.join(root, name)
            with open(path, encoding="utf-8") as f:
                old = f.read()
            new = rewrite_template(rewrite(old, root, missing), root, missing)
            if new != old:
                with open(path, "w", encoding="utf-8") as f:
                    f.write(new)
                changed += 1
    print(f"PROTO {changed} 개를 고쳤다. 미러에 없는 참조 {len(missing)} 개")
    for m in sorted(missing)[:20]:
        print("  없음:", m)


def world(src, dst):
    missing = set()
    with open(src, encoding="utf-8") as f:
        text = f.read()
    text = rewrite(text, os.path.dirname(os.path.abspath(dst)), missing)
    with open(dst, "w", encoding="utf-8") as f:
        f.write(text)
    left = len(REMOTE.findall(text))
    print(f"{dst}: 원격 주소 {left} 개 남음")
    for m in sorted(missing):
        print("  미러에 없음:", m)


if __name__ == "__main__":
    if sys.argv[1:2] == ["mirror"]:
        mirror()
    elif sys.argv[1:2] == ["world"] and len(sys.argv) == 4:
        world(sys.argv[2], sys.argv[3])
    else:
        print(__doc__)
        sys.exit(1)
