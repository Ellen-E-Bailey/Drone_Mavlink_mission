# -*- coding: utf-8 -*-
"""
Created on Tue Mar 10 10:54:05 2026

@author: ellis
"""

from pymavlink import mavutil
import csv
from datetime import datetime

# ---connection ---
myport = '/dev/serial0'
mybaudrate = 921600
connection = mavutil.mavlink_connection(myport, mybaudrate)

print("Connecting...")
connection.wait_heartbeat()
print(f"Heartbeat from system {connection.target_system}, component {connection.target_component}")

#---Log file---
log_filename = f"mavlog_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"

with open(log_filename, mode='w', newline='') as logfile:
    writer = csv.writer(logfile)
    writer.writerow(["timestamp", "msg_type", "lat", "lon", "alt", "roll", "pitch", "yaw"])

    print(f"Logging to {log_filename}")

    # --- get msg ---
    while True:
        msg = connection.recv_match(type=['GLOBAL_POSITION_INT', 'ATTITUDE'], blocking=False)

        if msg is None:
            continue

        timestamp = datetime.now().isoformat()

        if msg.get_type() == 'GLOBAL_POSITION_INT':
            lat = msg.lat / 1e7
            lon = msg.lon / 1e7
            alt = msg.alt / 1000.0
            print(f"Global Position: Lat={lat}, Lon={lon}, Alt={alt} m")

            writer.writerow([timestamp, "GLOBAL_POSITION_INT", lat, lon, alt, "", "", ""])
            logfile.flush()

        elif msg.get_type() == 'ATTITUDE':
            roll = msg.roll
            pitch = msg.pitch
            yaw = msg.yaw
            print(f"Attitude: Roll={roll}, Pitch={pitch}, Yaw={yaw}")

            writer.writerow([timestamp, "ATTITUDE", "", "", "", roll, pitch, yaw])
            logfile.flush()