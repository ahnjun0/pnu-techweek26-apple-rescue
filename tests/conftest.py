"""pytest 설정: 프로젝트 루트를 import 경로에 넣고, 시험용 작은 아레나 값을 config 에 넣는다."""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
sys.path.insert(0, HERE)

import arena  # noqa: E402

arena.install()
