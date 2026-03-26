# -*- coding: utf-8 -*-
"""
Created on Mon Mar 23 17:20:15 2026

@author: ellis
"""

from pymavlink import mavutil
import math as m
import time as t

bearing= 0 #deg
distance= 20 #m


class GuidedNavigator:
    def __init__(self):
        myport = '/dev/serial0'   # or '/dev/ttyAMA0'
        mybaudrate = 921600     
        self.master = mavutil.mavlink_connection(myport,mybaudrate)
        self.master.wait_heartbeat()
        print("Connected to vehicle")

    # ---------------------------------------------------------
    # WAIT FOR PILOT TO ENABLE GUIDED MODE
    # ---------------------------------------------------------
    
    def wait_for_guided(self):
        print("Waiting for GUIDED mode…")
        while True:
            hb = self.master.recv_match(type='HEARTBEAT', blocking=True)
            mode = mavutil.mode_string_v10(hb)
            print(f"Current mode: {mode}")
            
            if mode == "GUIDED":
                print("GUIDED mode detected — inititating autonomous flight")
                return
            
            t.sleep(0.5)

    def set_mode(self, mode_name):
        mode_id = self.master.mode_mapping()[mode_name]
    
        # Send mode change request
        self.master.mav.set_mode_send(
            self.master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id
        )
    
        # Wait for confirmation
        print(f"Switching to {mode_name}…")
        while True:
            hb = self.master.recv_match(type='HEARTBEAT', blocking=True)
            mode = mavutil.mode_string_v10(hb)
            if mode == mode_name:
                print(f"Mode changed to {mode_name}")
                return

    # ---------------------------------------------------------
    # ARM
    # ---------------------------------------------------------
    def arm(self):
        print("Arming…")
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1, 0, 0, 0, 0, 0, 0
        )
        self.master.recv_match(type='COMMAND_ACK', blocking=True)
        print("Armed")

    # ---------------------------------------------------------
    # TAKEOFF + WAIT UNTIL AIRBORNE
    # ---------------------------------------------------------
    def takeoff(self, alt):
        print(f"Taking off to {alt} m…")
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,
            0, 0, 0, float('nan'),
            0, 0, alt
        )
        self.master.recv_match(type='COMMAND_ACK', blocking=True)
        self.wait_until_airborne(min_alt=alt)

    def wait_until_airborne(self, min_alt=10, timeout=30):
        print("Waiting for climb…")
        start = t.time()
        while True:
            if t.time() - start > timeout:
                print("Takeoff timeout")
                return False

            msg = self.master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
            alt = msg.relative_alt / 1000.0
            vz = msg.vz / 100.0

            print(f"Alt: {alt:.2f} m | Vz: {vz:.2f} m/s")

            if alt > min_alt:
                print("Airborne")
                return True

            t.sleep(0.2)

    # ---------------------------------------------------------
    # RELATIVE MOVEMENT USING LOCAL_NED
    # ---------------------------------------------------------
    def goto_ned(self, x, y, z):
        print(f"Going to NED: x={x:.2f}, y={y:.2f}, z={z:.2f}")
        self.master.mav.set_position_target_local_ned_send(
            0,
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            0b110111111000,  # position only
            x, y, z,
            0, 0, 0,
            0, 0, 0,
            0, 0
        )

    # ---------------------------------------------------------
    # ARRIVAL DETECTOR (LOCAL_NED)
    # ---------------------------------------------------------
    def wait_until_arrived_local(self, target_x, target_y, target_z,
                                 dist_thresh=1.0,
                                 vel_thresh=0.3,
                                 stable_count_required=5,
                                 timeout=60):

        stable_count = 0
        start = t.time()

        while True:
            if t.time() - start > timeout:
                print("Arrival timeout")
                return False

            msg = self.master.recv_match(type='LOCAL_POSITION_NED', blocking=True)
            x = msg.x
            y = msg.y
            z = msg.z
            vx = msg.vx
            vy = msg.vy
            speed = m.sqrt(vx**2 + vy**2)

            dx = x - target_x
            dy = y - target_y
            dz = z - target_z
            dist = m.sqrt(dx*dx + dy*dy + dz*dz)

            print(f"Dist: {dist:.2f} m | Speed: {speed:.2f} m/s")

            if dist < dist_thresh and speed < vel_thresh:
                stable_count += 1
            else:
                stable_count = 0

            if stable_count >= stable_count_required:
                print("Arrived at relative target")
                return True

            t.sleep(0.3)

    # ---------------------------------------------------------
    # HIGH-LEVEL: MOVE USING RELATIVE BEARING + DISTANCE
    # ---------------------------------------------------------
    def goto_relative_bearing(self, bearing_deg, distance_m, alt_hold=True):
        """
        bearing_deg: relative to drone nose (body frame)
        distance_m: how far to move
        """
    
        # 1. Get drone yaw (ENU frame)
        att = self.master.recv_match(type='ATTITUDE', blocking=True)
        yaw_rad = att.yaw               # radians, ENU
        yaw_deg = m.degrees(yaw_rad)
    
        # 2. Convert relative → global heading
        global_heading_deg = yaw_deg + bearing_deg
        global_heading_rad = m.radians(global_heading_deg)
    
        # 3. Convert global heading → NED movement
        north = distance_m * m.cos(global_heading_rad)
        east  = distance_m * m.sin(global_heading_rad)
    
        # 4. Get current NED position
        pos = self.master.recv_match(type='LOCAL_POSITION_NED', blocking=True)
        x0 = pos.x
        y0 = pos.y
        z0 = pos.z   # negative = above ground
    
        # 5. Compute absolute target
        target_x = x0 + north
        target_y = y0 + east
        target_z = z0  # keep altitude constant
    
        print(f"Drone yaw: {yaw_deg:.1f}°")
        print(f"Relative bearing {bearing_deg}° → global {global_heading_deg:.1f}°")
        print(f"Move N={north:.2f} E={east:.2f}")
        print(f"Target NED: x={target_x:.2f}, y={target_y:.2f}, z={target_z:.2f}")
    
        # 6. Send absolute NED target
        self.goto_ned(target_x, target_y, target_z)
    
        # 7. Wait until arrival
        return self.wait_until_arrived_local(target_x, target_y, target_z)
    
    # LAND
    # ---------------------------------------------------------
    def land(self):
        print("Landing…")
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_LAND,
            0, 0, 0, 0, 0, 0, 0, 0
        )
        
    def disconnect_SITL(self):
        self.master.close()

#Create Class instance (Connect)
nav = GuidedNavigator()

#ensure drone is in stabilise
#nav.set_mode("STABILIZE")


#Waits for pilot to change to guided mode before mission starts
nav.wait_for_guided()

#Goes to new coordinates
nav.goto_relative_bearing(bearing_deg=bearing, distance_m=distance)

#Lands at new position
nav.land()


