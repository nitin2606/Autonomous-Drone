import cv2
from typing import Any, Dict, Optional, Tuple


class Camera:
    def __init__(self, cfg: Dict[str, Any]):
        self.cfg = cfg
        src = cfg.get("source", 0)
        self.cap = cv2.VideoCapture(src)
        w = int(cfg.get("width", 640))
        h = int(cfg.get("height", 480))
        fps = int(cfg.get("fps", 30))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
        self.cap.set(cv2.CAP_PROP_FPS, fps)

        if not self.cap.isOpened():
            raise RuntimeError(f"Failed to open camera source: {src}")

    def read(self) -> Tuple[bool, Optional[any]]:
        return self.cap.read()

    def release(self):
        try:
            self.cap.release()
        except Exception:
            pass


