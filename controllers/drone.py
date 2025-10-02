import time
from typing import Any, Dict

from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil


class DroneController:
    def __init__(self, cfg: Dict[str, Any]):
        self.cfg = cfg
        conn = cfg.get("connection_string", "127.0.0.1:14550")
        baud = int(cfg.get("baud", 921600))
        self.vehicle = connect(conn, baud=baud, wait_ready=True)
        self.gnd_speed = float(cfg.get("gnd_speed_mps", 1.0))

    def arm_and_takeoff(self, altitude_m: float):
        while not self.vehicle.is_armable:
            time.sleep(1)
        self.vehicle.mode = VehicleMode("GUIDED")
        self.vehicle.armed = True
        while not self.vehicle.armed:
            time.sleep(1)
        self.vehicle.simple_takeoff(altitude_m)
        while True:
            v_alt = self.vehicle.location.global_relative_frame.alt
            if v_alt >= altitude_m * 0.95:
                break
            time.sleep(1)

    def set_velocity_body(self, vx: float, vy: float, vz: float):
        msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
            0,
            0, 0,
            mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
            0b0000111111000111,
            0, 0, 0,
            vx, vy, vz,
            0, 0, 0,
            0, 0
        )
        self.vehicle.send_mavlink(msg)
        self.vehicle.flush()

    def goto_relative(self, lat: float, lon: float, alt_m: float):
        self.vehicle.simple_goto(LocationGlobalRelative(lat, lon, alt_m))

    def close(self):
        try:
            self.vehicle.close()
        except Exception:
            pass


