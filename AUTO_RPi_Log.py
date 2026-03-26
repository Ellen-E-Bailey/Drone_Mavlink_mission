# -*- coding: utf-8 -*-
"""
Created on Wed Mar 25 22:23:32 2026

@author: ellis
"""

from pymavlink import mavutil
import csv
from datetime import datetime
import os
import math as m
import time as t


# ============================================================
#   LOGGER (uses existing MAVLink connection)
# ============================================================

class MavlinkLogger:
    def __init__(self, mav_connection, log_dir='logs'):
        self.connection = mav_connection
        self.log_dir = os.path.abspath(log_dir)
        self._setup_logfile()

    def _setup_logfile(self):
        os.makedirs(self.log_dir, exist_ok=True)
        filename = f"mavlog_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        self.path = os.path.join(self.log_dir, filename)

        self.logfile = open(self.path, 'w', newline='')
        self.writer = csv.writer(self.logfile)

        self.writer.writerow([
            "timestamp",
            "msg_type",
            "lat", "lon", "alt",
            "roll", "pitch", "yaw",
            "voltage", "current", "battery_remaining",
            "throttle"
        ])
        self.logfile.flush()

        print(f"[LOGGER] Logging to {self.path}")

    def step(self):
        msg = self.connection.recv_match(
            type=['GLOBAL_POSITION_INT', 'ATTITUDE', 'SYS_STATUS', 'RC_CHANNELS'],
            blocking=False
        )
        if msg is None:
            return

        timestamp = datetime.now().isoformat()

        if msg.get_type() == 'GLOBAL_POSITION_INT':
            lat = msg.lat / 1e7
            lon = msg.lon / 1e7
            alt = msg.alt / 1000.0

            self.writer.writerow([timestamp, "GLOBAL_POSITION_INT",
                                  lat, lon, alt,
                                  "", "", "",
                                  "", "", "",
                                  ""])
            self.logfile.flush()

        elif msg.get_type() == 'ATTITUDE':
            self.writer.writerow([timestamp, "ATTITUDE",
                                  "", "", "",
                                  msg.roll, msg.pitch, msg.yaw,
                                  "", "", "",
                                  ""])
            self.logfile.flush()

        elif msg.get_type() == 'SYS_STATUS':
            voltage = msg.voltage_battery / 1000.0
            current = msg.current_battery / 100.0
            remaining = msg.battery_remaining

            self.writer.writerow([timestamp, "SYS_STATUS",
                                  "", "", "",
                                  "", "", "",
                                  voltage, current, remaining,
                                  ""])
            self.logfile.flush()

        elif msg.get_type() == 'RC_CHANNELS':
            throttle = msg.chan3_raw

            self.writer.writerow([timestamp, "RC_CHANNELS",
                                  "", "", "",
                                  "", "", "",
                                  "", "", "",
                                  throttle])
            self.logfile.flush()

    def close(self):
        try:
            self.logfile.flush()
        except:
            pass
        try:
            self.logfile.close()
        except:
            pass
        print(f"[LOGGER] Closed log file: {self.path}")


# ============================================================
#   MISSION (single AUTO + LAND)
# ============================================================

class AutoMission:
    def __init__(self, connection):
        self.master = connection

    def clear_mission(self):
        print("[MISSION] Clearing mission…")
        self.master.mav.mission_clear_all_send(
            self.master.target_system,
            self.master.target_component
        )
        t.sleep(0.3)

    def get_global_position(self):
        msg = self.master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        return msg.lat / 1e7, msg.lon / 1e7, msg.relative_alt / 1000.0

    def compute_global_waypoint(self, bearing_deg, distance_m):
        """
        Option A assumption:
        - ATTITUDE.yaw ≈ 0° when pointing North
        - positive yaw clockwise
        - bearing_deg: clockwise from nose
        """
        lat0, lon0, alt0 = self.get_global_position()

        att = self.master.recv_match(type='ATTITUDE', blocking=True)
        yaw_deg = m.degrees(att.yaw)

        move_heading_deg = (yaw_deg + bearing_deg) % 360.0
        move_heading_rad = m.radians(move_heading_deg)

        dN = distance_m * m.cos(move_heading_rad)
        dE = distance_m * m.sin(move_heading_rad)

        R = 6378137.0
        d_lat = dN / R
        d_lon = dE / (R * m.cos(m.radians(lat0)))

        lat_target = lat0 + m.degrees(d_lat)
        lon_target = lon0 + m.degrees(d_lon)
        alt_target = alt0

        print(f"[MISSION] yaw={yaw_deg:.1f}°, bearing={bearing_deg:.1f}°, move={move_heading_deg:.1f}°")
        print(f"[MISSION] dN={dN:.2f}, dE={dE:.2f}")
        print(f"[MISSION] Target lat={lat_target:.7f}, lon={lon_target:.7f}, alt={alt_target:.1f}")

        return lat_target, lon_target, alt_target

    def upload_waypoint_and_land(self, lat, lon, alt):
        print("[MISSION] Uploading waypoint + LAND mission…")

        self.master.mav.mission_count_send(
            self.master.target_system,
            self.master.target_component,
            2
        )

        # WP 0: waypoint
        req = self.master.recv_match(type='MISSION_REQUEST', blocking=True)
        self.master.mav.mission_item_send(
            self.master.target_system,
            self.master.target_component,
            0,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            1, 1,
            0, 0, 0, 0,
            lat, lon, alt
        )

        # WP 1: LAND
        req = self.master.recv_match(type='MISSION_REQUEST', blocking=True)
        self.master.mav.mission_item_send(
            self.master.target_system,
            self.master.target_component,
            1,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            mavutil.mavlink.MAV_CMD_NAV_LAND,
            0, 1,
            0, 0, 0, 0,
            lat, lon, 0
        )

        ack = self.master.recv_match(type='MISSION_ACK', blocking=True)
        print(f"[MISSION] Mission ACK: {ack.type}")

    def wait_for_auto(self, logger):
        print("[MISSION] Waiting for AUTO mode…")
        while True:
            logger.step()
            hb = self.master.recv_match(type='HEARTBEAT', blocking=True)            
            mode = mavutil.mode_string_v10(hb)
            if mode == "AUTO":
                print("[MISSION] AUTO detected — mission running")
                return
            t.sleep(0.1)

    def wait_until_not_auto(self, logger):
        print("[MISSION] Waiting for pilot to exit AUTO…")
        while True:
            logger.step()
            hb = self.master.recv_match(type='HEARTBEAT', blocking=True)         
            mode = mavutil.mode_string_v10(hb)
            if mode != "AUTO":
                print(f"[MISSION] Pilot switched to {mode}")
                return
            t.sleep(0.1)


# ============================================================
#   MAIN 
# ============================================================

if __name__ == "__main__":
    # For SITL on Windows:
    # master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
    # For RPi:
    # master = mavutil.mavlink_connection('/dev/serial0', baud=921600)

    print("[SYSTEM] Connecting to FC…")
    #master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
    master = mavutil.mavlink_connection('/dev/serial0', baud=921600)
    master.wait_heartbeat()
    print(f"[SYSTEM] Connected: sys={master.target_system}, comp={master.target_component}")

    logger = None
    try:
      
        logger = MavlinkLogger(master)
        

        mission = AutoMission(master)

        bearing = 90
        distance = 20.0

        lat, lon, alt = mission.compute_global_waypoint(bearing, distance)

        mission.clear_mission()
        mission.upload_waypoint_and_land(lat, lon, alt)

        print("[SYSTEM] Mission uploaded. Pilot must switch to AUTO to execute.")

        mission.wait_until_not_auto(logger)
        mission.wait_for_auto(logger)
        mission.wait_until_not_auto(logger)

        print("[SYSTEM] Mission cycle complete. Exiting.")

    except KeyboardInterrupt:
        print("\n[SYSTEM] Ctrl+C — shutting down cleanly…")

    finally:
        if logger is not None:
            logger.close()
        try:
            master.close()
        except:
            pass
        print("[SYSTEM] MAVLink connection closed.")
