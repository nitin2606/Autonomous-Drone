from typing import Any, Dict, Tuple
import cv2

from ..controllers.drone import DroneController
from ..controllers.camera import Camera
from ..controllers.pid import PID, PIDGains
from ..detectors.base import Detector


class TrackerOrchestrator:
    def __init__(self, mission_cfg: Dict[str, Any], cam_cfg: Dict[str, Any], det_cfg: Dict[str, Any], ctrl_cfg: Dict[str, Any], detector: Detector):
        self.mission_cfg = mission_cfg
        self.cam_cfg = cam_cfg
        self.det_cfg = det_cfg
        self.ctrl_cfg = ctrl_cfg
        self.detector = detector
        self.camera = Camera(cam_cfg)
        self.drone = DroneController(mission_cfg)

        self.center_box = ctrl_cfg.get("center_box", {"x_min": 285, "x_max": 365, "y_min": 190, "y_max": 320})
        self.deadzone_px = int(ctrl_cfg.get("deadzone_px", 10))
        lims = ctrl_cfg.get("limits", {"vx_max": 2.0, "vy_max": 2.0, "vz_max": 1.0})
        pid_cfg = ctrl_cfg.get("pid", {"x": {"kp": 0.003, "ki": 0.0, "kd": 0.001}, "y": {"kp": 0.003, "ki": 0.0, "kd": 0.001}})

        self.pid_x = PID(PIDGains(**pid_cfg["x"]), output_limit=float(lims.get("vy_max", 2.0)))  # pixel error in x -> body vy
        self.pid_y = PID(PIDGains(**pid_cfg["y"]), output_limit=float(lims.get("vx_max", 2.0)))  # pixel error in y -> body vx

    def _decide_velocity(self, frame_shape, target_center) -> Tuple[float, float, float]:
        h, w = frame_shape[:2]
        cx_img, cy_img = w // 2, h // 2
        cx, cy = target_center
        err_x = cx - cx_img   # +right, -left (pixels)
        err_y = cy - cy_img   # +down,  -up   (pixels)

        # deadzone to avoid jitter
        if abs(err_x) < self.deadzone_px:
            err_x = 0.0
        if abs(err_y) < self.deadzone_px:
            err_y = 0.0

        # Map pixel error to body velocities: vy for lateral, vx for forward/back
        vy_cmd = self.pid_x.update(err_x)     # +right -> +vy
        vx_cmd = -self.pid_y.update(err_y)    # +down  -> -vx (move forward when target above center)

        return float(vx_cmd), float(vy_cmd), 0.0

    def run(self):
        self.drone.arm_and_takeoff(float(self.mission_cfg.get("search_altitude_m", 10.0)))
        loiter = int(self.mission_cfg.get("loiter_seconds", 2))
        # Optional small loiter
        for _ in range(max(loiter, 0)):
            self.drone.set_velocity_body(0.0, 0.0, 0.0)

        while True:
            ok, frame = self.camera.read()
            if not ok or frame is None:
                continue
            detections = self.detector.detect(frame)
            target = self.detector.pick_target(frame.shape, detections)
            if target:
                cx, cy = target["center"]
                vx, vy, vz = self._decide_velocity(frame.shape, (cx, cy))
                self.drone.set_velocity_body(vx, vy, vz)
                if self.det_cfg.get("draw_viz", True):
                    x, y, w, h = target["bbox"]
                    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 255), 2)
                    cv2.circle(frame, (cx, cy), 3, (0, 255, 0), -1)
            # draw center box
            if self.det_cfg.get("draw_viz", True):
                cb = self.center_box
                cv2.rectangle(frame, (cb["x_min"], cb["y_min"]), (cb["x_max"], cb["y_max"]), (0, 255, 0), 2)
                cv2.line(frame, (frame.shape[1]//2, 0), (frame.shape[1]//2, frame.shape[0]), (0, 250, 0), 1)
                cv2.line(frame, (0, frame.shape[0]//2), (frame.shape[1], frame.shape[0]//2), (0, 250, 0), 1)
                cv2.imshow("Autonomy", frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

        self.camera.release()
        self.drone.close()
        cv2.destroyAllWindows()


