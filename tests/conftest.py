"""pytest 가 프로젝트 루트를 import 경로에 넣도록 해 준다."""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
