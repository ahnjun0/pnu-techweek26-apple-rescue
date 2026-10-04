"""필요한 라이브러리 목록과 확인. 없으면 **크게 알리고 멈춘다** (조용히 다르게 돌지 않는다).

버전은 검증한 것 그대로다 — pyproject.toml / uv.lock / 제출 README 표와 같다.
제출 파일(tools/build_submission.py)도 이 목록을 그대로 쓴다.
"""
import sys

# (import 이름, 설치 이름==버전)
REQUIRED = [
    ("numpy", "numpy==2.2.6"),
    ("cv2", "opencv-python==5.0.0.93"),
    ("matplotlib", "matplotlib==3.10.9"),
    ("torch", "torch==2.14.0"),
    ("ultralytics", "ultralytics==8.4.123"),
]


def check(who="controller"):
    missing = []
    for module, package in REQUIRED:
        try:
            __import__(module)
        except ImportError as error:
            missing.append((module, package, error))
    if not missing:
        return
    bar = "!" * 70
    lines = [bar, f"ERROR: 필요한 라이브러리가 없습니다 — {who} 를 시작할 수 없습니다.", ""]
    for module, package, error in missing:
        lines.append(f"  - {module}  (설치: pip install {package})   [{error}]")
    lines += ["", f"  지금 쓰는 파이썬: {sys.executable}",
              "  저장소에서 ./setup.sh 를 실행하면 .venv 를 만들고 컨트롤러가 그걸 쓰게 한다.",
              bar]
    message = "\n".join(lines)
    print(message, file=sys.stderr)
    sys.stderr.flush()
    raise ImportError(message)
