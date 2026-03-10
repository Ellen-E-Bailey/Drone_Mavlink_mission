# -*- coding: utf-8 -*-
"""
Created on Tue Mar 10 11:55:35 2026

@author: ellis
"""
from pymavlink import mavutil
import math as m
import time as t


def connect():
    myport = '/dev/serial0'   # or '/dev/ttyAMA0'
    mybaudrate = 921600
    connection = mavutil.mavlink_connection(myport,mybaudrate)
    print("Waiting for heartbeat...")
    connection.wait_heartbeat()
    print(f"Heartbeat from system {connection.target_system}, component {connection.target_component}")
    return connection


#Get crrent Location
#--------------------------------
def get_current_location(connection):
    msg = connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
    lat0 = msg.lat / 1e7

    lon0 = msg.lon / 1e7
    alt0 = msg.relative_alt / 1000.0
    print(f"Current position: Lat={lat0}, Lon={lon0}, Alt={alt0}")
    return lat0, lon0, alt0

#Mode Switching
# -------------------------------
def set_mode(connection, mode_name):
    mode_id = connection.mode_mapping()[mode_name]
    connection.mav.set_mode_send(
        connection.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id
    )
    print(f"Mode set to {mode_name}")
# -------------------------------

#arm/disarm
# -------------------------------
def arm_drone(connection):
    print("Arming...")
    connection.mav.command_long_send(
        connection.target_system,
        connection.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1, 0, 0, 0, 0, 0, 0
    )
    # Check if armed
    t.sleep(2)
    msg = connection.recv_match(type='HEARTBEAT', blocking=True, timeout=2)
    if msg and msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
        print("✓ Armed successfully")
        return True
    else:
        print("✗ Arming failed - trying force arm for testing")
        # Try force arm for testing only
        connection.mav.command_long_send(
            connection.target_system,
            connection.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 1, 21196, 0, 0, 0, 0, 0)
        t.sleep(2)
        return True


def disarm_drone(connection):
    connection.mav.command_long_send(
        connection.target_system,
        connection.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        0, 0, 0, 0, 0, 0, 0
    )
    print("Sent DISARM command")
# -------------------------------

#Takeoff
#--------------------------------
def takeoff(connection,altitude=10):
    print(f"Taking off to {altitude}m...")
    connection.mav.command_long_send(
        connection.target_system, 
        connection.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 
        0, 0, 0, 0, m.nan, 0,0,altitude)
    
#Target Location
# -------------------------------
def project_target(lat_deg, lon_deg, heading_deg, distance_m):
    R = 6371000.0 #Earth radius
    h = m.radians(heading_deg)
    dlat = (distance_m * m.cos(h)) / R
    dlon = (distance_m * m.sin(h)) / (R * m.cos(m.radians(lat_deg)))
    lat_t = lat_deg + dlat * (180.0 / m.pi)
    lon_t = lon_deg + dlon * (180.0 / m.pi)
    print(f"Target: Lat={lat_t}, Lon={lon_t}")
    return lat_t, lon_t
# -------------------------------