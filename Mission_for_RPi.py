# -*- coding: utf-8 -*-
"""
Created on Mon Mar  9 19:03:32 2026

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
    
def land(connection):
    print("Landing...")
    set_mode('LAND')
    

#Return to Launch
#-----------------------------------
def RTL(connection):
    connection.mav.command_long_send(
        connection.target_system,
        connection.target_component,
        mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,
        0, 0, 0, 0, 0, 0, 0, 0
    )
    print("RTL command sent")
#-----------------------------------

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

#Guided Movement
# -------------------------------
def goto_global_relalt(connection, lat, lon, alt_m=10):
    connection.mav.set_position_target_global_int_send(
        0,
        connection.target_system,
        connection.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        0b0000111111111000,  
        int(lat * 1e7),
        int(lon * 1e7),
        alt_m,
        0,0,0, 0,0,0, 0,0
    )
    print(f"Sent target: Lat={lat}, Lon={lon}, Alt={alt_m}")

def fly_to_target(connection, lat_t, lon_t, alt_t, threshold=1.0):
    reached = False
    while not reached:
        # Send target
        connection.mav.set_position_target_global_int_send(
            0,
            connection.target_system,
            connection.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            0b0000111111111000,
            int(lat_t * 1e7),
            int(lon_t * 1e7),
            alt_t,
            0, 0, 0, 0, 0, 0, 0, 0
        )

        # Report position
        msg = connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1)
        if msg:
            lat = msg.lat / 1e7
            lon = msg.lon / 1e7
            alt = msg.relative_alt / 1000.0
            print(f"Current position: Lat={lat:.6f}, Lon={lon:.6f}, Alt={alt:.1f}")

            # Distance to target
            dlat = (lat - lat_t) * 111139
            dlon = (lon - lon_t) * 111139 * m.cos(m.radians(lat_t))
            dist = m.sqrt(dlat**2 + dlon**2)
            print(f"Distance to target: {dist:.2f} m")

            if dist < threshold:
                print("Target reached!")
                reached = True
"""
def plot_on_map(lat0, lon0, lat_t, lon_t):
    m = folium.Map(location=[lat0, lon0], zoom_start=17)
    folium.Marker([lat0, lon0], popup="Current Location", icon=folium.Icon(color='blue')).add_to(m)
    folium.Marker([lat_t, lon_t], popup="Target Location", icon=folium.Icon(color='red')).add_to(m)
    folium.PolyLine([(lat0, lon0), (lat_t, lon_t)], color="black", dash_array="5").add_to(m)
    folium.Circle([lat_t, lon_t], radius=10, color="green", fill=True, fill_opacity=0.2).add_to(m)
    map_file = "target_map.html"
    m.save(map_file)
    webbrowser.open(map_file)

    return m
"""

# -------------------------------

def mission(connection, heading, distance):
    
    lat0,lon0,alt0=get_current_location(connection)
    set_mode(connection, "GUIDED")  
    lat_t, lon_t = project_target(lat0, lon0, heading, distance)
    
    #Send location to pilot for review    
    print(f"Candidate target location: Lat={lat_t:.6f}, Lon={lon_t:.6f}")
   # plot_on_map(lat0, lon0, lat_t, lon_t)
    confirm = input("Pilot confirm safe to move? (y/n): ").strip().lower()
    if confirm == "y":
        arm_drone(connection)
        takeoff(connection)
        t.sleep(5)
        print("Pilot confirmed. Moving to target...")
        fly_to_target(connection, lat_t, lon_t, 15)
        print("Holding at target for ...")
        i=1
        for i in range(5):
            print(5-i)
            t.sleep(1)           
        #RTL(connection)
        land(connection)
    else:
        print("Pilot rejected target. Handing control back.")
        set_mode(connection, "LOITER")
    
   
def main():
    connection=connect()
    heading=float(input("Confirm heading angle [deg]: "))
    distance=float(input("Confirm distance [m]: "))
    try:
        mission(connection, heading, distance)
    finally:
        connection.close()


if __name__ == "__main__":
    main()
