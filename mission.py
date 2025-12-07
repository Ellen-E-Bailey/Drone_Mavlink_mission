# -*- coding: utf-8 -*-
"""
Created on Sun Dec  7 15:31:57 2025

@author: ellis
"""

from pymavlink import mavutil
import math as m
import time as t

#FUNCTIONS
# -------------------------------


#Connection
# -------------------------------
def connect():
    port = '/dev/serial0'
    baud = 57600
    connection = mavutil.mavlink_connection(port, baud=baud)
    connection.wait_heartbeat()
    print(f"Heartbeat from system {connection.target_system}, component {connection.target_component}")
    return connection
# -------------------------------


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
    connection.mav.command_long_send(
        connection.target_system,
        connection.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1, 0, 0, 0, 0, 0, 0
    )
    print("Sent ARM command")

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


#Target Location
# -------------------------------
def project_target(lat_deg, lon_deg, heading_deg, distance_m):
    R = 6371000.0
    h = m.radians(heading_deg)
    dlat = (distance_m * m.cos(h)) / R
    dlon = (distance_m * m.sin(h)) / (R * m.cos(m.radians(lat_deg)))
    lat_t = lat_deg + dlat * (180.0 / m.pi)
    lon_t = lon_deg + dlon * (180.0 / m.pi)
    return lat_t, lon_t
# -------------------------------


#Guided Movement
# -------------------------------
def goto_global_relalt(connection, lat, lon, alt_m):
    connection.mav.set_position_target_global_int_send(
        0,
        connection.target_system,
        connection.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        0b0000111111111000,  # position only
        int(lat * 1e7),
        int(lon * 1e7),
        alt_m,
        0,0,0, 0,0,0, 0,0
    )
    print(f"Sent target: Lat={lat}, Lon={lon}, Alt={alt_m}")
# -------------------------------


#Ground detection, ***need check for optical flow sensor***
# -------------------------------
def read_rangefinder(connection, samples=10):
    readings = []
    start = t.time()
    while len(readings) < samples and (t.time() - start) < 5:
        msg = connection.recv_match(type=['RANGEFINDER','DISTANCE_SENSOR'], blocking=True, timeout=0.5)
        if msg is None: continue
        if msg.get_type() == 'RANGEFINDER':
            readings.append(msg.distance)
        elif msg.get_type() == 'DISTANCE_SENSOR':
            readings.append(msg.current_distance / 100.0)
    if not readings:
        return None, None
    avg = sum(readings)/len(readings)
    var = sum((r-avg)**2 for r in readings)/len(readings)
    return avg, var
"""
def check_optical_flow(connection, samples=20):
    flows_x, flows_y, qualities = [], [], []
    for _ in range(samples):
        msg = connection.recv_match(type=['OPTICAL_FLOW','OPTICAL_FLOW_RAD'], blocking=True, timeout=0.5)
        if msg is None:
            continue
        if msg.get_type() == 'OPTICAL_FLOW':
            flows_x.append(msg.flow_x)
            flows_y.append(msg.flow_y)
            qualities.append(msg.quality)
        elif msg.get_type() == 'OPTICAL_FLOW_RAD':
            flows_x.append(msg.flow_x)
            flows_y.append(msg.flow_y)
            qualities.append(msg.quality)
    if not flows_x:
        return None, None, None
    avg_quality = sum(qualities)/len(qualities)
    drift_mag = (sum(abs(x) for x in flows_x)/len(flows_x),
                 sum(abs(y) for y in flows_y)/len(flows_y))
    return avg_quality, drift_mag[0], drift_mag[1]

"""
# -------------------------------


#Landing
# -------------------------------
def land_here(connection):
    set_mode(connection, "LAND")
    print("Landing initiated")

# -------------------------------

# -------------------------------
#Main routine

def main():
    connection = connect()

    # Step 1: Arm
    arm_drone(connection)
    t.sleep(2)

    # Step 2: Set initial mode
    set_mode(connection, "GUIDED")

    # Step 3: Get current GPS
    msg = connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
    lat0 = msg.lat / 1e7
    lon0 = msg.lon / 1e7
    alt0 = msg.relative_alt / 1000.0
    print(f"Current position: Lat={lat0}, Lon={lon0}, Alt={alt0}")

    # Step 4: Input direction + distance
    heading_deg = float(input("Enter heading (deg): "))
    distance_m = float(input("Enter distance (m): "))

    # Step 5: Compute target
    lat_t, lon_t = project_target(lat0, lon0, heading_deg, distance_m)

    # Step 6: Fly to target at safe altitude
    goto_global_relalt(connection, lat_t, lon_t, 15)
    t.sleep(10)  # wait to arrive
    
    # Step 7: Check landing suitability
    avg, var = read_rangefinder(connection)
    if avg is None:
        print("No rangefinder data, aborting landing")
        return
    print(f"Rangefinder avg={avg:.2f}m, var={var:.4f}")
    
    if var < 0.5:  # heuristic: low variance means flat ground
        print("Landing site suitable")
        land_here(connection)
    else:
        print("Unsafe to land, holding position")




