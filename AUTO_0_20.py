# -*- coding: utf-8 -*-
"""
Created on Wed Mar 25 19:20:46 2026

@author: ellis
"""
from pymavlink import mavutil
import math as m
import time as t




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
            hb = self.master.recv_match(type='HEARTBEAT', blocking=False)            
            mode = mavutil.mode_string_v10(hb)
            if mode == "AUTO":
                print("[MISSION] AUTO detected — mission running")
                return
            t.sleep(0.1)

    def wait_until_not_auto(self, logger):
        print("[MISSION] Waiting for pilot to exit AUTO…")
        while True:
            logger.step()
            hb = self.master.recv_match(type='HEARTBEAT', blocking=False)            
            mode = mavutil.mode_string_v10(hb)
            if mode != "AUTO":
                print(f"[MISSION] Pilot switched to {mode} — mission ended")
                return
            t.sleep(0.1)


if __name__ == "__main__":
  
    master = mavutil.mavlink_connection('/dev/serial0', baud=921600)
    #master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
    master.wait_heartbeat()

    mission = AutoMission(master)
    bearing_deg = 0   # relative to nose, +right
    distance_m = 20.0

    # Compute waypoint
    lat, lon, alt = mission.compute_global_waypoint(bearing_deg, distance_m)

    mission.clear_mission()
    mission.upload_waypoint_and_land(lat, lon, alt)
  
    
    print("Mission uploaded. When ready, switch to AUTO to execute it.")

 
