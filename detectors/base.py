from abc import ABC, abstractmethod
from typing import List, Optional, Tuple, Dict, Any


Detection = Dict[str, Any]


class Detector(ABC):
    
    @abstractmethod
    def detect(self, frame) -> List[Detection]:
        pass

    def pick_target(self, frame_shape: Tuple[int, int], detections: List[Detection]) -> Optional[Detection]:
        if not detections:
            return None
        height, width = frame_shape[:2]
        cx_img, cy_img = width // 2, height // 2
        best_det = None
        best_dist2 = float("inf")
        for det in detections:
            cx, cy = det.get("center", (None, None))
            if cx is None:
                x, y, w, h = det["bbox"]
                cx, cy = x + w // 2, y + h // 2
                det["center"] = (cx, cy)
            dx, dy = cx - cx_img, cy - cy_img
            d2 = dx * dx + dy * dy
            if d2 < best_dist2:
                best_dist2 = d2
                best_det = det
        return best_det


