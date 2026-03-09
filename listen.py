# -*- coding: utf-8 -*-
"""
Created on Wed Oct 15 14:48:29 2025

@author: ellis
"""
from pymavlink import mavutil


# Start a connection listening on a USB port
"""
myport = 'COM4'
mybaudrate = 9600
connection = mavutil.mavlink_connection(myport,mybaudrate)
"""
# Start a connection listening on a UDP port
#connection = mavutil.mavlink_connection('tcp:206.189.60.90:33999')

#start a connection listening on RPi
myport = '/dev/serial0'   # or '/dev/ttyAMA0'
mybaudrate = 921600
connection = mavutil.mavlink_connection(myport,mybaudrate)

print("Connecting")
# Wait for the first heartbeat
#   This sets the system and component ID of remote system for the link
connection.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (connection.target_system, connection.target_component))

# Once connected, use 'connection' to get and send messages

# Step 3: Retrieve and display MAVLink messages from the drone
while True:
    # Fetch the next message
    
    msg = connection.recv_match(type=['GLOBAL_POSITION_INT', 'ATTITUDE'], blocking=False)
    
    if msg is None:
        continue

    # Process and display Global Position data
    if msg.get_type() == 'GLOBAL_POSITION_INT':
        print(f"Global Position: Lat={msg.lat / 1e7}, Lon={msg.lon / 1e7}, Alt={msg.alt / 1000.0} meters")

    # Process and display Attitude data
    elif msg.get_type() == 'ATTITUDE':
        print(f"Attitude: Roll={msg.roll}, Pitch={msg.pitch}, Yaw={msg.yaw}")
        
    elif msg.get_type()== 'SYS_STATUS':
        print(f"Battery: {msg.battery_remaining}%")
        
