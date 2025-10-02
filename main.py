import argparse
import yaml

from detectors.color_detector import ColorContourDetector
from detectors.yolov8_detector import YoloV8Detector
from tracker.orchestrator import TrackerOrchestrator


def load_config(path: str):
    with open(path, "r") as f:
        return yaml.safe_load(f)


def build_detector(det_cfg):
    backend = det_cfg.get("backend", "color").lower()
    if backend == "color":
        return ColorContourDetector(det_cfg)
    if backend in ("yolo", "yolov8"):
        return YoloV8Detector(det_cfg)
    raise ValueError(f"Unknown detection backend: {backend}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", default="NEW_autonomy/configs/default.yaml")
    args = parser.parse_args()

    cfg = load_config(args.config)
    mission_cfg = cfg.get("mission", {})
    cam_cfg = cfg.get("camera", {})
    det_cfg = cfg.get("detection", {})
    ctrl_cfg = cfg.get("control", {})

    detector = build_detector(det_cfg)
    orchestrator = TrackerOrchestrator(mission_cfg, cam_cfg, det_cfg, ctrl_cfg, detector)
    orchestrator.run()


if __name__ == "__main__":
    main()


