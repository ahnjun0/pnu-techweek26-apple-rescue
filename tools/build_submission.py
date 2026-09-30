"""제출물을 주최 측 repo 구조 그대로 만든다: submission/

  submission/
    README.md                              ← 재현 방법 (라이브러리 버전 포함)
    controllers/sar_rescue/sar_rescue.py   ← 파이썬 파일 **하나** (우리 모듈 전부 포함)
    worlds/apartment.wbt                   ← 컨트롤러만 sar_rescue 로 바꾼 원본
    models/ protos/ ...                    ← 주최 측 repo 그대로 (competition/given/)

쓰는 법:  python3 tools/build_submission.py

왜 이렇게 합치나: 개발은 모듈로 한다 (테스트 383개가 모듈 단위다). 제출만 한 파일이다.
각 모듈 소스를 **그대로** 문자열로 넣고, 실행할 때 모듈 객체로 되살려 sys.modules 에
등록한다. 코드를 고쳐 쓰지 않으므로 합친 파일과 원래 코드의 동작이 같다
(tools/check_submission.sh 가 두 실행의 로그를 비교해 확인한다).
"""
import os
import re
import shutil
import subprocess
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
GIVEN = os.path.join(ROOT, "competition", "given")
OUT = os.path.join(ROOT, "submission")
NAME = "sar_rescue"

# 의존 순서 (앞의 것만 import 한다). viz 는 matplotlib 창 — 없으면 알린다.
MODULES = ["deps", "boot", "config", "common", "mapping", "planner", "exploration", "follower",
           "localization", "detect", "people", "scanmatch", "mission",
           "sensors", "viz", "yolo_check"]

# 라이브러리 목록은 deps.py 하나에서 관리한다 (pyproject.toml·README 와 같은 버전).
sys.path.insert(0, ROOT)
from deps import REQUIRED  # noqa: E402
MODEL = os.path.join(ROOT, "models", "YOLO", "yolo11n.pt")


# 정답 위치는 debug/truth.py(Supervisor)에서만 읽는다 — 주행 모듈에는 GPS 코드가 아예 없다.
# 그래도 실수로 들어오면 여기서 막는다.
# 합친 파일에 이것이 하나라도 남으면 빌드 실패 (check_rules.sh 도 같은 패턴을 본다)
FORBIDDEN = re.compile(r"GPS_NAME|read_gps|enable_gps|Supervisor|supervisor|\.gps\b")


def git(*args):
    return subprocess.run(["git", "-C", ROOT, *args], capture_output=True,
                          text=True).stdout.strip()


def controller_source():
    parts = [f'''# -*- coding: utf-8 -*-
"""{NAME} — 2026 부산대 TECH WEEK Search & Rescue 제출 컨트롤러 (파일 하나).

빨간 사과를 찾아 가까이 간 뒤 시작 지점으로 돌아온다. GNSS·절대 위치는 쓰지 않는다.
이 파일은 tools/build_submission.py 가 만들었다 — 직접 고치지 말 것.
원본 모듈: https://github.com/ahnjun0/robot_hack (커밋 {git("rev-parse", "--short", "HEAD")})
실행 방법과 재현 환경은 저장소 루트의 README.md 를 본다.
"""
import os
import sys
import types


# ── 1. 라이브러리 확인: 없으면 경고를 크게 띄우고 멈춘다 ────────────────────────
def _check_libraries():
    missing = []
    for module, package in {REQUIRED!r}:
        try:
            __import__(module)
        except ImportError as error:
            missing.append((module, package, error))
    if not missing:
        return
    line = "!" * 70
    lines = [line, "ERROR: 필요한 라이브러리가 없습니다 — {NAME} 를 시작할 수 없습니다.", ""]
    for module, package, error in missing:
        lines.append(f"  - {{module}}  (설치: pip install {{package}})   [{{error}}]")
    lines += ["", f"  지금 쓰는 파이썬: {{sys.executable}}",
              "  Webots > Preferences > General > Python command 가 위 라이브러리를 설치한",
              "  파이썬을 가리키는지 확인하세요. 버전은 README.md '재현 환경' 표를 따릅니다.",
              line]
    message = "\\n".join(lines)
    print(message, file=sys.stderr)
    sys.stderr.flush()
    raise ImportError(message)


_check_libraries()

# 대회 월드(apartment) 설정으로 돈다 (config.py 끝의 SAR_PROFILE 참고).
os.environ.setdefault("SAR_PROFILE", "apartment")

# ── 2. 모듈 소스 (원본 그대로) ───────────────────────────────────────────────
_SOURCES = {{}}
''']
    for name in MODULES + ["sar_main"]:
        path = (os.path.join(ROOT, "controllers", "sar_controller", "sar_controller.py")
                if name == "sar_main" else os.path.join(ROOT, name + ".py"))
        with open(path, encoding="utf-8") as f:
            src = f.read()
        parts.append(f"\n_SOURCES[{name!r}] = {src!r}\n")
    parts.append(f'''

# ── 3. 모듈로 되살린다 ───────────────────────────────────────────────────────
def _load(name):
    module = types.ModuleType(name)
    module.__file__ = os.path.abspath(__file__)
    sys.modules[name] = module
    exec(compile(_SOURCES[name], f"<{NAME}:{{name}}.py>", "exec"), module.__dict__)
    return module


for _name in {MODULES!r}:
    _load(_name)

if __name__ == "__main__":
    _load("sar_main").main()
''')
    return "".join(parts)


def main():
    if os.path.exists(OUT):
        shutil.rmtree(OUT)
    shutil.copytree(GIVEN, OUT, ignore=shutil.ignore_patterns(".DS_Store"))

    ctl_dir = os.path.join(OUT, "controllers", NAME)
    os.makedirs(ctl_dir)
    ctl = os.path.join(ctl_dir, NAME + ".py")
    source = controller_source()
    left = sorted(set(FORBIDDEN.findall(source)))
    assert not left, f"제출 파일에 GPS/Supervisor 흔적이 남았다: {left}"
    with open(ctl, "w", encoding="utf-8") as f:
        f.write(source)

    world = os.path.join(OUT, "worlds", "apartment.wbt")
    with open(world, encoding="utf-8") as f:
        text = f.read()
    text, n = re.subn(r'controller "tb3_teleop"', f'controller "{NAME}"', text)
    assert n == 1, f"apartment.wbt 에서 tb3_teleop 컨트롤러를 {n} 개 찾았다 (1 개여야 한다)"
    with open(world, "w", encoding="utf-8") as f:
        f.write(text)

    # README = 제출용 본문 + 기술 질의응답 (강사님 질문 네 가지, docs/ 에서 관리)
    with open(os.path.join(ROOT, "submission_README.md"), encoding="utf-8") as f:
        readme = f.read().rstrip("\n")
    with open(os.path.join(ROOT, "docs", "기술_질의응답.md"), encoding="utf-8") as f:
        readme += "\n\n" + f.read()
    with open(os.path.join(OUT, "README.md"), "w", encoding="utf-8") as f:
        f.write(readme)
    # YOLO 모델 — 주최 측 예시와 같은 자리 (models/YOLO/yolo11n.pt)
    shutil.copy(MODEL, os.path.join(OUT, "models", "YOLO", "yolo11n.pt"))
    print(f"만들었다: {os.path.relpath(OUT, ROOT)}/")
    print(f"  controllers/{NAME}/{NAME}.py  {os.path.getsize(ctl) // 1024} KB")
    print(f"  worlds/apartment.wbt  (controller tb3_teleop → {NAME})")


if __name__ == "__main__":
    main()
