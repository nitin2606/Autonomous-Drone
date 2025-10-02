from typing import List, Dict, Any

from .base import Detector, Detection


class YoloV8Detector(Detector):
    def __init__(self, cfg: Dict[str, Any]):
        self.cfg = cfg
        self.draw_viz = bool(cfg.get("draw_viz", True))
        ycfg = cfg.get("yolov8", {})
        self.model_path = ycfg.get("model_path", "yolov8n.pt")
        self.conf = float(ycfg.get("conf_threshold", 0.25))
        self.iou = float(ycfg.get("iou_threshold", 0.45))
        self.classes = ycfg.get("classes", None)
        try:
            from ultralytics import YOLO  
        except Exception as e:  
            raise RuntimeError("ultralytics is required for YOLOv8 backend. Install via `pip install ultralytics`.\n"+str(e))
        self._model = YOLO(self.model_path)

    def detect(self, frame) -> List[Detection]:
        results = self._model.predict(source=frame, conf=self.conf, iou=self.iou, classes=self.classes, verbose=False)
        detections: List[Detection] = []
        if not results:
            return detections
        res = results[0]
        for b in res.boxes:  
            xyxy = b.xyxy[0].tolist()
            x1, y1, x2, y2 = [int(v) for v in xyxy]
            w, h = x2 - x1, y2 - y1
            cls_id = int(b.cls.item())
            score = float(b.conf.item())
            class_name = res.names.get(cls_id, str(cls_id))  
            detections.append({
                "bbox": (x1, y1, w, h),
                "score": score,
                "class_name": class_name,
                "center": (x1 + w // 2, y1 + h // 2)
            })
        return detections


