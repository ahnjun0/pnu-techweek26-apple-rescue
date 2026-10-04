"""YOLO 로 "이 빨간 덩어리가 무엇인가" 를 묻는다.

받는 것: 카메라 BGR 영상.
내놓는 것: [(x1, y1, x2, y2, 라벨, 확신), ...] — detect.yolo_verdict 가 판정한다.
핵심 아이디어: YOLO 는 **빨간 덩어리가 모양 검사를 통과한 장** 에서만 돈다 (detect.py).
ultralytics(수백 MB)가 필요하므로 순수 모듈(detect, mission)은 이걸 import 하지 않는다.
컨트롤러가 load() 로 만들어 mission.classify 에 꽂는다.
"""

import os
import sys

from . import config


def _alert(lines):
    bar = "!" * 70
    message = "\n".join([bar, *lines, bar])
    print(message, file=sys.stderr)
    sys.stderr.flush()
    return message


def load():
    """모델을 **시작할 때** 불러 둔다. 모델 파일이 없으면 크게 알리고 멈춘다.

    ⚠️ 주행 도중에 처음 부르면 1~2초 멈칫하고, 파일이 없다는 걸 그때서야 안다.
    """
    path = os.path.abspath(config.YOLO_MODEL_PATH)   # Webots 는 컨트롤러 폴더에서 실행한다
    if not os.path.exists(path):
        raise FileNotFoundError(_alert([
            "ERROR: YOLO 모델 파일이 없습니다.",
            f"  찾은 곳: {path}",
            "  받는 법: python3 -c \"from ultralytics import YOLO; YOLO('yolo11n.pt')\"",
            "           (지금 폴더에 yolo11n.pt 가 생긴다) → 위 경로로 옮긴다.",
        ]))
    from ultralytics import YOLO
    model = YOLO(path)
    model.to(config.YOLO_DEVICE)
    print(f"[yolo] 모델을 불러왔다: {path} ({config.YOLO_DEVICE})")

    def classify(image_bgr):
        result = model(image_bgr, verbose=False, conf=config.YOLO_CONF,
                       device=config.YOLO_DEVICE)[0]
        names = result.names
        return [(x1, y1, x2, y2, names[int(cls)], float(conf))
                for (x1, y1, x2, y2), cls, conf in zip(result.boxes.xyxy.tolist(),
                                                       result.boxes.cls.tolist(),
                                                       result.boxes.conf.tolist())]
    return classify
