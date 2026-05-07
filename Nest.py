# -*- coding: utf-8 -*-
"""
Created on Fri Apr 10 15:50:53 2026

@author: ellis
"""

from pymavlink import mavutil
import csv
from datetime import datetime
import os
import math as m
import time as t
import numpy as np
import matplotlib.pyplot as plt

class NestEstimator:
    def __init__(self, lat0, lon0, grid_size=1500, span=1000):
        self.lat0 = lat0
        self.lon0 = lon0

        #Hornet speed values from Hornet Handbook, Dr. Sarah Bunker 2022
        self.u_mean = 5.36 #mean flight speed
        self.u_std = 1.825 #deviation from flight speed
        self.path_std = 200 #Tune this uncertainty 
        self.t_n=45 #unloading time (how long hornets spend at nest)

        self.grid_size = grid_size
        self.span = span #Foraging distance of hornet + extra to avoid cutoff
        self.certainty_limit=0.98
        self.nest_error=10

        x = np.linspace(-span, span, grid_size)
        y = np.linspace(-span, span, grid_size)
        self.X, self.Y = np.meshgrid(x, y)

        self.P = np.ones_like(self.X, dtype=float)
        self.measurements = []


    # Coordinate transforms
    # ----------------------------
    def latlon_to_xy(self, lat, lon):
        R = 6371000
        dlat = np.radians(lat - self.lat0)
        dlon = np.radians(lon - self.lon0)
        x = R * dlon * np.cos(np.radians(self.lat0))
        y = R * dlat
        return x, y

    def xy_to_latlon(self, x, y, lat0, lon0):
        R = 6371000
        dlon = x / (R * np.cos(np.radians(lat0)))
        dlat = y / R
        lat = lat0 + np.rad2deg(dlat)
        lon = lon0 + np.rad2deg(dlon)
        return lat, lon

    def polar_to_latlon(self, r, g, lat0, lon0):
        x = r * np.sin(np.radians(g))
        y = r * np.cos(np.radians(g))
        return self.xy_to_latlon(x, y,lat0,lon0)

    
    # Likelihood model
    # ----------------------------
    def radius_stats(self, dt):
        r_mean = (dt-self.t_n) * self.u_mean / 2
        r_std_speed = (dt-self.t_n) * self.u_std / 2
        r_std = np.sqrt(r_std_speed**2 + self.path_std**2)
        return r_mean, r_std
    
    def bearing_field(self, dx, dy):
        """Returns bearing (deg) from drone position (dx,dy) to each grid point."""
        angles = np.degrees(np.arctan2(self.X - dx, self.Y - dy))  
        return angles


    def gaussian_likelihood(self, dist, r_mean, r_std):
        return np.exp(-0.5 * ((dist - r_mean) / r_std)**2)
    
    def angular_likelihood(self,theta, theta_meas, theta_std):
        dtheta = (theta - theta_meas + 180) % 360 - 180 #handles angle wrapping
        return np.exp(-0.5 * (dtheta / theta_std)**2)

   
    # Add measurement
    # ----------------------------
    def add_measurement(self, lat, lon , heading_abs , heading_rel , heading_std , dt, label,distance=None):
        dx, dy = self.latlon_to_xy(lat, lon)
        r_mean, r_std = self.radius_stats(dt)

        #Grid set up from drone location
        dist = np.sqrt((self.X - dx)**2 + (self.Y - dy)**2)
        thetas=self.bearing_field(dx,dy)
        
        meas_bearing_abs = (heading_abs + heading_rel) % 360.0
               
        radial= self.gaussian_likelihood(dist, r_mean, r_std)
        angular= self.angular_likelihood(thetas, meas_bearing_abs, heading_std)
   
        self.P = self.P* radial*angular
        self.measurements.append((dx, dy, label,heading_abs,heading_rel))
        
        return 

    # Compute next waypoint 
    # ----------------------------
    def compute_next_waypoint(self, heading_deg, r_mean, lat, lon):
        # Move r_mean metres along heading_deg
        return self.polar_to_latlon(r_mean, heading_deg, lat, lon)
    
    def check_certainty(self,confidence=None):
        if confidence==None:
            confidence=self.certainty_limit
        P_norm = self.P / np.max(self.P)
        cells_90=np.where(P_norm>confidence)[0]
        cell_size=((2*self.span)**2)/(self.grid_size**2)
        Area=len(cells_90)*cell_size
        return Area
    
    # Save plot
    # ----------------------------
    def save_plot(self, filename,true_nest=None,show=True):
        P_norm = self.P / np.max(self.P)

        plt.figure(figsize=(8, 6))
        cs = plt.contourf(self.X, self.Y, P_norm, levels=20, cmap='viridis')
        plt.colorbar(cs, label="Nest location probability")
        
        ax = plt.gca()

        for item in self.measurements:
            
            if len(item) == 3:
                dx, dy, label = item
                heading = None
            else:
                dx, dy, label,heading_abs, heading_rel = item
    
            plt.scatter(dx, dy, c='red', s=30, edgecolor='k', zorder=5)
            plt.text(dx + 10, dy + 10, label, color='white', fontsize=9, weight='bold', zorder=6)
       
            rad = np.radians(heading_abs)
            dx_arrow = self.span/10 * np.sin(rad)   # East component
            dy_arrow = self.span/10 * np.cos(rad)   # North component
            from matplotlib.patches import FancyArrowPatch
            arr = FancyArrowPatch((dx, dy), (dx + dx_arrow, dy + dy_arrow),
                                  arrowstyle='-|>', mutation_scale=12,
                                  color='red', linewidth=1.5, zorder=8)
            ax.add_patch(arr)

            
        #Plot Simulated Nest location
        if true_nest is not None:
            tx, ty = true_nest
            if abs(tx) <= 90 and abs(ty) <= 180:                
                tx, ty = self.latlon_to_xy(tx, ty) 
                
            plt.scatter(tx, ty, c='yellow', marker='*', s=200, edgecolor='black', linewidth=1.2, zorder=10)
            plt.text(tx + 10, ty + 10, "NEST", color='black', fontsize=10, weight='bold', zorder=11)
            mm=self.get_map_max_xy()
            map_x, map_y, _, _, _ = mm
            dist = m.hypot(map_x - tx, map_y - ty)
            self.nest_error=dist
            print("Distance between probability peak and nest: ", np.round(dist,2), "m")
        
        plt.xlabel("East (m)")
        plt.ylabel("North (m)")
        plt.title(f"Probability Field at {datetime.now().strftime('%Y%m%d_%H%M%S')}")
        plt.axis('equal')
        plt.tight_layout()
        plt.savefig(filename, dpi=200)
        
        if show:
            plt.show()

        
        plt.close()

   
    # Save visited waypoints
    # ----------------------------
    def save_waypoints(self, filename, waypoint_list):
        os.makedirs(os.path.dirname(filename), exist_ok=True)
        with open(filename, "w") as f:
            for lat, lon in waypoint_list:
                f.write(f"{lat},{lon}\n")
                
    def get_map_max_xy(self):
        """Return (x_m, y_m, row, col, p_norm) of the single highest-probability cell."""
        if np.all(self.P == 0):
            return None
        P_norm = self.P / np.max(self.P)
        idx_flat = np.argmax(P_norm)
        row, col = np.unravel_index(idx_flat, P_norm.shape)
        x_m = float(self.X[row, col])
        y_m = float(self.Y[row, col])
        return x_m, y_m, int(row), int(col), float(P_norm[row, col])
    
    
    def get_nest_error(self):
        return self.nest_error



    def _confidence_area_radius(self, P_norm, confidence_level=0.30):
        mask = (P_norm >= confidence_level)
        if not mask.any():
            return 0.0
        cell_area = ((2.0 * self.span) ** 2) / (self.grid_size ** 2)
        area = mask.sum() * cell_area
        return m.sqrt(area / m.pi)


    
    def get_nest_location(self):
        mm=self.get_map_max_xy()
        map_x, map_y, _, _, _ = mm
        nest_lat,nest_lon=self.xy_to_latlon(map_x, map_y, lat0, lon0)
        return nest_lat, nest_lon
    
    def confidence_area_extreme_points(self, confidence_level = 0.70):
        #Find shape of probability area
        P_norm = self.P/np.max(self.P)
        mask = P_norm >= confidence_level
        Xi, Yi = np.nonzero(mask)
        #Find maximum and minimum edges of bounding box
        Xmaxi = np.max(Xi)
        Xmini = np.min(Xi)
        Ymaxi = np.max(Yi)
        Ymini = np.min(Yi)
        #Find value of probability at each corner point
        combs = [(Xmaxi, Ymaxi), (Xmini, Ymaxi), (Xmini, Ymini), (Xmaxi, Ymini)]
        vals = [P_norm[comb] for comb in combs]
        #Most diagonal point is assumed to be corner point with highest probability (increase computational efficiency)
        diag_point_1 = vals.index(max(vals))
        #Find other diagonal point
        if diag_point_1 == 0 or diag_point_1 == 2:
            diag_point_1 = 0
            diag_point_2 = 2
        else:
            diag_point_1 = 1
            diag_point_2 = 3
        
        #convert from indices to actual numbers:
        diag_point_1 = np.array([self.X[combs[diag_point_1]], self.Y[combs[diag_point_1]]])
        diag_point_2 = np.array([self.X[combs[diag_point_2]], self.Y[combs[diag_point_2]]])
        
        
        return diag_point_1, diag_point_2
    
    
    
    @staticmethod
    def perp_points(point_1, point_2, ang = 90):
        theta = np.deg2rad(ang)/2
        #Find middle point between two points
        point_mid = np.array([(point_1[0]+point_2[0])/2,(point_1[1]+point_2[1])/2])
        #Find distance to move perpendicular to line between points based on angle
        perp_dist = np.array([(point_1[1]-point_mid[1])/np.tan(theta),(-point_1[0]+point_mid[0])/np.tan(theta)])
        
        perp_point_1 = point_mid + perp_dist
        perp_point_2 = point_mid - perp_dist
        
        return perp_point_1, perp_point_2
        
    def get_waypoint(self,current_lat,current_lon,confidence_level=0.7,ang=90):
        ext_points=self.confidence_area_extreme_points(confidence_level)
        angle_points=self.perp_points(ext_points[0],ext_points[1],ang)
        px, py = self.latlon_to_xy(current_lat, current_lon)
        tx,ty=angle_points[0][0], angle_points[0][1]
        tx2,ty2=angle_points[1][0], angle_points[1][1]
        dx=tx-px
        dy=ty-py
        dx2=tx2-px
        dy2=ty2-py
        dist=m.hypot(dx,dy)
        dist2=m.hypot(dx2,dy2)
        if dist>dist2:
            dx=dx2
            dy=dy2
            tx=tx2
            ty=ty2
            dist=dist2
        waypoint_lat, waypoint_lon = self.xy_to_latlon(tx, ty, self.lat0, self.lon0)
        heading_abs = m.degrees(m.atan2(dx, dy)) % 360.0
        
        return float(waypoint_lat), float(waypoint_lon), (tx, ty), dist, heading_abs
    
class NestSimulator:
    def __init__(self, lat0, lon0, u_mean=5.36, u_std=1.825, t_n=45, path_std=200, rng=None):
        """
        lat0, lon0: reference origin
        u_mean, u_std: hornet speed mean and std (m/s)
        t_n: unloading time at nest (s)
        path_std: extra path uncertainty (m) (added noise)
        rng:random generator
        """
        self.lat0 = lat0
        self.lon0 = lon0
        self.u_mean = u_mean
        self.u_std = u_std
        self.t_n = t_n
        self.path_std = path_std
        self.rng = rng if rng is not None else np.random.default_rng()
        self.nest_xy = None  # (x,y) in metres relative to lat0,lon0


    def latlon_to_xy(self, lat, lon):
        R = 6371000.0
        dlat = np.radians(lat - self.lat0)
        dlon = np.radians(lon - self.lon0)
        x = R * dlon * np.cos(np.radians(self.lat0))
        y = R * dlat
        return x, y

    def xy_to_latlon(self, x, y):
        R = 6371000.0
        dlon = x / (R * np.cos(np.radians(self.lat0)))
        dlat = y / R
        lat = self.lat0 + np.rad2deg(dlat)
        lon = self.lon0 + np.rad2deg(dlon)
        return lat, lon

  
    def place_random_nest(self, radius=800.0, min_radius=50.0):
        """
        Place nest uniformly in angle and uniformly in radius between min_radius and radius (metres).
        """
        ang = self.rng.uniform(0, 2 * m.pi)
        r = m.sqrt(self.rng.uniform(min_radius**2, radius**2))
        x = r * m.sin(ang)
        y = r * m.cos(ang)
        self.nest_xy = (x, y)
        return self.xy_to_latlon(x, y)
    
    def place_set_nest(self, x,y):
        self.nest_xy=(x,y)
        return self.xy_to_latlon(x, y)

    def set_nest_latlon(self, lat, lon):
        self.nest_xy = self.latlon_to_xy(lat, lon)

    def get_true_nest_latlon(self):
        if self.nest_xy is None:
            return None
        return self.xy_to_latlon(self.nest_xy[0], self.nest_xy[1])

    #Get measurements
    def _true_distance_and_bearing(self, drone_lat, drone_lon):
        if self.nest_xy is None:
            raise RuntimeError("Nest not set. Call place_random_nest or set_nest_latlon first.")
        px, py = self.latlon_to_xy(drone_lat, drone_lon)
        nx, ny = self.nest_xy
        dx = nx - px
        dy = ny - py
        dist = m.hypot(dx, dy)  # metres
        bearing = m.degrees(m.atan2(dx, dy))
        return dist, bearing

    def sample_dt_and_bearing(self, drone_lat, drone_lon,drone_heading,
                              speed_noise=True,
                              dt_noise_std=5.0,
                              bearing_noise_std=15.0,
                              return_true=False):
     
        dist, true_bearing = self._true_distance_and_bearing(drone_lat, drone_lon)

        # sample hornet speed for this simulated trip
        if speed_noise:
            u = max(0.1, self.rng.normal(self.u_mean, self.u_std))
        else:
            u = self.u_mean

        # path noise (models extra path length due to meandering)
        extra_path = abs(self.rng.normal(-50, self.path_std))

        # round-trip time: unloading + travel out + travel back + small measurement noise
        travel_time = 2.0 * (dist + extra_path) / u
        dt = self.t_n + travel_time + self.rng.normal(-dt_noise_std, dt_noise_std)

        # measured bearing with noise
        meas_bearing_abs = true_bearing + self.rng.normal(-bearing_noise_std, bearing_noise_std)
        meas_bearing = (meas_bearing_abs - drone_heading + 180.0) % 360.0 - 180.0

    
        bearing_std = bearing_noise_std

        if return_true:
            return dt, meas_bearing, bearing_std, dist, true_bearing
        return dt, meas_bearing, bearing_std

 
    def simulate_path(self, waypoints_latlon, heading_at_waypoint=None,
                      dt_noise_std=5.0, bearing_noise_std=10.0):
        out = []
        for (lat, lon) in waypoints_latlon:
            out.append(self.sample_dt_and_bearing(lat, lon,
                                                 dt_noise_std=dt_noise_std,
                                                 bearing_noise_std=bearing_noise_std))
        return out
    
    
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



    def upload_waypoint_and_land(self, lat, lon, alt=0):
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

    
    def get_current_pose(self):
        lat,lon,alt=self.get_global_position()
        att = self.master.recv_match(type='ATTITUDE', blocking=True)
        yaw_deg = m.degrees(att.yaw)
        return lat,lon,yaw_deg


    def wait_for_start_then_disarm(self, final_seq=None, overall_timeout=600, per_recv_timeout=5.0):
        """
         wait for mission start, then wait for disarm.
        Phase 1: block until mission starts (MISSION_CURRENT changes from initial or mode==AUTO).
        Phase 2: print mission_current updates and block until motors are disarmed.
        Returns True if disarmed observed before overall_timeout, False on timeout.
        """
        start = t.time()
        print("[MISSION] Waiting for mission to start...")

        # try to read initial mission_current (may be None)
        try:
            initial_idx = getattr(self.master, "mission_current", None)
        except Exception:
            initial_idx = None

        # Phase 1: wait for mission start
        while True:
            if t.time() - start > overall_timeout:
                print("[MISSION] Timeout waiting for mission start.")
                return False
            try:
                msg = self.master.recv_match(blocking=True, timeout=per_recv_timeout)
            except Exception as e:
                print("[MISSION] recv_match exception:", e)
                msg = None

            if msg is None:
                continue

            mtype = msg.get_type()
            if mtype == 'MISSION_CURRENT':
                seq = int(getattr(msg, 'seq', -1))
                #print(f"[MISSION] mission_current = {seq}")
                if initial_idx is None:
                    initial_idx = seq
                elif seq != initial_idx:
                    print("[MISSION] mission_current changed -> mission started.")
                    break
            elif mtype == 'HEARTBEAT':
                # some FCs switch to AUTO when mission starts
                try:
                    mode = getattr(self.master, 'mode', None) or self.master.get_mode()
                except Exception:
                    mode = None
                if mode and mode.upper() == 'AUTO':
                    print("[MISSION] Mode AUTO detected -> mission started.")
                    break
            else:
                # print other mission-related messages for visibility
                if mtype in ('COMMAND_ACK', 'MISSION_ITEM_REACHED'):
                    print(f"[MISSION] {mtype}: {msg}")

        # Phase 2: mission started — print mission_current updates and wait for disarm
        print("[MISSION] Waiting for motors to be disarmed (mission complete)...")
        while True:
            if t.time() - start > overall_timeout:
                print("[MISSION] Overall timeout waiting for disarm.")
                return False
            try:
                msg = self.master.recv_match(blocking=True, timeout=per_recv_timeout)
            except Exception as e:
                print("[MISSION] recv_match exception:", e)
                msg = None

            if msg is not None:
                mtype = msg.get_type()
                if mtype == 'MISSION_CURRENT':
                    seq = int(getattr(msg, 'seq', -1))
                    #print(f"[MISSION] mission_current = {seq}")
                elif mtype == 'MISSION_ITEM_REACHED':
                    seq = int(getattr(msg, 'seq', -1))
                    print(f"[MISSION] MISSION_ITEM_REACHED seq={seq}")
                elif mtype == 'COMMAND_ACK':
                    cmd = int(getattr(msg, 'command', -1))
                    res = int(getattr(msg, 'result', -1))
                    print(f"[MISSION] COMMAND_ACK cmd={cmd} result={res}")
                else:
                  
                    lat,lon,heading=self.get_current_pose()
                    print(lat,lon,heading)
                    #print(f"[TELEM] {mtype}")

            # check disarm state after consuming messages
            try:
                armed = self.master.motors_armed()
            except Exception:
                armed = True  # assume still armed if can't query
            if not armed:
                print("[MISSION] Motors disarmed -> mission complete.")
                return True

            # loop continues until disarmed or timeout

             
        



t.sleep(7)

#CONNECT TO FLIGHT CONTROLLER
#-------------------------------------------------------
print("[SYSTEM] Connecting to FC…")
master = mavutil.mavlink_connection('udpin:0.0.0.0:14550') #Windows SITL
#master = mavutil.mavlink_connection('/dev/serial0', baud=921600) #RPi
master.wait_heartbeat()
print(f"[SYSTEM] Connected: sys={master.target_system}, comp={master.target_component}")
#-------------------------------------------------------
 

try:   
    #Create Mission Class and Initilise
    #----------------------------------------------------
    mission = AutoMission(master)
    mission.clear_mission()
    #----------------------------------------------------
    
 
    #Initialise Position and create Nest Estimator and Simulator Classes
    #-----------------------------------------------------------------------
    lat0,lon0,heading0=mission.get_current_pose()
    sim=NestSimulator(lat0, lon0)
    Nest=NestEstimator(lat0, lon0)
    
    #true_lat, true_lon=sim.place_set_nest(-20, 15) #(East, North (m)) #Place a nest in a set location
    true_lat,true_lon=sim.place_random_nest(radius=800,min_radius=100) #Place a random nest
    #-----------------------------------------------------------------------
    
    
    #Use estimator to determine first waypoint
    #--------------------------------------------------------
    #Measurement 1
    label="M1"
    dt, bearing, bearing_std = sim.sample_dt_and_bearing(lat0, lon0,heading0)
    Nest.add_measurement(lat0, lon0,heading0,bearing,bearing_std, dt, label)
    #Check Area of 95% probability
    Area=Nest.check_certainty(confidence=0.95)
    print("[NEST] Area of certainty: ", Area)
    folder = f"Probability_Plots/{datetime.now().strftime('%Y%m%d_%H%M%S')}/"
    os.makedirs(folder, exist_ok=True)
    Nest.save_plot(folder+"M1",true_nest=(true_lat,true_lon))
    lat,lon,t_xy, dist_move, heading_next=Nest.get_waypoint(lat0,lon0,confidence_level=0.7, ang=100)
    print("[NEST] Next drone Location: ", lat,lon)
    print("[NEST] Distance: ", dist_move)
    print("[NEST] Heading: ", heading_next)
    
    
    
    #Upload waypoint to FC and checks if mission has been completed
    #---------------------------------------------------
    mission.upload_waypoint_and_land(lat, lon)
    
    #CHECK IF LANDED AT WAYPOINT BEFORE CONTINUING
    ok = mission.wait_for_start_then_disarm(final_seq=None, overall_timeout=300, per_recv_timeout=5.0)
    if not ok:
        print("Mission did not complete within timeout; handle retry/abort.")
    else:
        print("Mission cycle finished — get next measurement.")



         
    
    #-------------------------------------------------
    
    i=2 #Measurement index
       
    
    while ( Area>10 and i<30):
        print("-----------------------------------------------------------")
        print(f"Measurement {i}")
        print("-----------------------------------------------------------") 
        label=f"M{i}"
        
 
        lat,lon,heading=mission.get_current_pose()
        dt, bearing, bearing_std = sim.sample_dt_and_bearing(lat, lon,heading)
        Nest.add_measurement(lat, lon,heading,bearing,bearing_std, dt, label)
        Area=Nest.check_certainty(confidence=0.95)
        print("[NEST] Area of certainty: ", Area)
        
        Nest.save_plot(folder+f"M{i}", true_nest=(true_lat, true_lon))
        lat,lon,t_xy, dist_move,heading_next=Nest.get_waypoint(lat,lon, confidence_level=0.7,ang=100)
        
        print("[NEST] Next drone Location: ", lat,lon)
        print("[NEST] Distance: ", dist_move)
        print("[NEST] Heading: ", heading_next)
        
        nest_error=Nest.get_nest_error()
        
        #CLEAR PREVIOUS WAYPOINT AND SEND NEW
        mission.clear_mission()
        mission.upload_waypoint_and_land(lat, lon)
        #CHECK IF LANDED AT WAYPOINT BEFORE CONTINUE
        ok = mission.wait_for_start_then_disarm(final_seq=None, overall_timeout=600, per_recv_timeout=5.0)
        if not ok:
            print("Mission did not complete within timeout; handle retry/abort.")
        else:
            print("Mission cycle finished — proceed to next waypoint.")
      
        i+=1
        
    print("[NEST] Nest found!")
    nest_estimate_lat, nest_estimate_lon=Nest.get_nest_location()
    print("[NEST] Nest estimate lat, lon: ",float(nest_estimate_lat),",", float(nest_estimate_lon) )
    print("[SYSTEM] Moving to estimated nest position")
    mission.clear_mission()
    mission.upload_waypoint_and_land(nest_estimate_lat,nest_estimate_lon)
    
    ok = mission.wait_for_start_then_disarm(final_seq=None, overall_timeout=600, per_recv_timeout=5.0)
    if not ok:
        print("Mission did not complete within timeout; handle retry/abort.")
    else:
        print("Mission cycle finished — get next measurement.")
        
    Nest.save_plot("Mission_Ended",true_nest=(true_lat,true_lon))
    print("[NEST] Actual distance to simulated nest: ", nest_error)
    

except KeyboardInterrupt:
    print("\n[SYSTEM] Ctrl+C — shutting down cleanly…")

finally:
    try:
        master.close()
    except:
        pass
    print("[SYSTEM] MAVLink connection closed.")
