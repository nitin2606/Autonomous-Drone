import cv2
import numpy as np
from typing import List, Dict, Any

from .base import Detector, Detection


class ColorContourDetector(Detector):
    def __init__(self, cfg: Dict[str, Any]):
        self.cfg = cfg
        self.min_area = int(cfg.get("min_area", 300))
        self.aspect_min = float(cfg.get("aspect_ratio_min", 0.5))
        self.aspect_max = float(cfg.get("aspect_ratio_max", 1.6))
        self.draw_viz = bool(cfg.get("draw_viz", True))
        self.color_cfg = cfg.get("color", {})

    def _masks(self, hsv):
        masks = []
        def arr(o):
            return np.array([o["h_min"], o["s_min"], o["v_min"]]), np.array([o["h_max"], o["s_max"], o["v_max"]])
        for key in ["blue", "yellow", "black", "red1", "red2"]:
            if key in self.color_cfg:
                lo, hi = arr(self.color_cfg[key])
                masks.append(cv2.inRange(hsv, lo, hi))
        if not masks:
            return None
        mask = masks[0]
        for m in masks[1:]:
            mask = cv2.bitwise_or(mask, m)
        return mask

    def detect(self, frame) -> List[Detection]:
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = self._masks(hsv)
        if mask is None:
            return []
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        detections: List[Detection] = []
        for c in contours:
            x, y, w, h = cv2.boundingRect(c)
            area = cv2.contourArea(c)
            if w == 0 or h == 0:
                continue
            aspect = float(w) / float(h)
            if area < self.min_area or aspect < self.aspect_min or aspect > self.aspect_max:
                continue
            det: Detection = {
                "bbox": (x, y, w, h),
                "score": 1.0,
                "class_name": "color_target",
                "center": (x + w // 2, y + h // 2)
            }
            detections.append(det)
        return detections


