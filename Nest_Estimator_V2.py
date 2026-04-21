# -*- coding: utf-8 -*-
"""
Created on Wed Apr  8 13:10:09 2026

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
    def __init__(self, lat0, lon0, grid_size=600, span=200):
        self.lat0 = lat0
        self.lon0 = lon0

        #Hornet speed values from Hornet Handbook, Dr. Sarah Bunker 2022
        self.u_mean = 5.36 #mean flight speed
        self.u_std = 1.825 #deviation from flight speed
        self.path_std = 200 #Tune this uncertainty (based of Rojas-Nossa)
        self.t_n=45 #unloading time (how long hornets spend at nest)

        self.grid_size = grid_size
        self.span = span #Foraging distance of hornet
        self.certainty_limit=0.98
        self.nest_error=800

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

   #--------------------------------

    def add_measurement(self, lat, lon , heading_abs , heading_rel , heading_std , dt, label,distance=None):
        dx, dy = self.latlon_to_xy(lat, lon)
        r_mean, r_std = self.radius_stats(dt)

        #Grid set up from drone location
        dist = np.sqrt((self.X - dx)**2 + (self.Y - dy)**2)
        thetas=self.bearing_field(dx,dy)
        
        meas_bearing_abs = (heading_abs + heading_rel) % 360.0

        radial= self.gaussian_likelihood(dist, r_mean, r_std)
        angular= self.angular_likelihood(thetas, meas_bearing_abs, heading_std)
   
        self.P = radial*angular*self.P
        self.measurements.append((dx, dy, label,heading_abs,heading_rel))
        

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
    

    def save_plot(self, filename,true_nest=None,show=True,save=True):
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
        plt.title("Probability Field")
        plt.axis('equal')
        plt.tight_layout()
        if save:
            plt.savefig(filename, dpi=200)
        
        if show:
            plt.show()

       
        plt.close()

   

    def save_waypoints(self, filename, waypoint_list):
        os.makedirs(os.path.dirname(filename), exist_ok=True)
        with open(filename, "w") as f:
            for lat, lon in waypoint_list:
                f.write(f"{lat},{lon}\n")
                
    #Nest Error
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
        """Returns area that has a probability greater than the confidence level"""
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
        #Most diagonal point is likely to be corner point with highest probability (bodge)
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
        extra_path = abs(self.rng.normal(0.0, self.path_std))

        # round-trip time: unloading + travel out + travel back + small measurement noise
        travel_time = 2.0 * (dist + extra_path) / u
        dt = self.t_n + travel_time + self.rng.normal(0.0, dt_noise_std)

        # measured bearing with noise
        meas_bearing_abs = true_bearing + self.rng.normal(0.0, bearing_noise_std)
        meas_bearing = (meas_bearing_abs - drone_heading + 180.0) % 360.0 - 180.0 #relative to drone nose
        
    
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

def get_pose(): #Replace with AutoMission() get_pose
    lat,lon,heading=45,56,0
    return lat,lon,heading

Num_measurements=[]
Distances=[]
Errors=[]
iterations=1
save=False

for its in range(iterations):
    #Initialise
    lat0,lon0,heading_abs=get_pose()
    Total_distance=0
    Nest=NestEstimator(lat0, lon0)
    sim = NestSimulator(lat0, lon0)
    #true_lat,true_lon=sim.place_random_nest(radius=800,min_radius=100)
    true_lat, true_lon=sim.place_set_nest(60, 100)
    sign=1
    #Measurement 1
    label="M1"
    print("-----------------------------------------------------------")
    print("Measurement 1")
    print("-----------------------------------------------------------") 

    #RELATIVE MEASUREMENT
    dt, measure_heading_rel, heading_std = sim.sample_dt_and_bearing(lat0, lon0,heading_abs)
    
    #ABSOLUTE DRONE HEADING WITH RELATIVE MEASUREMENT
    Nest.add_measurement(lat0, lon0,heading_abs,measure_heading_rel,heading_std, dt, label)
    Area=Nest.check_certainty()
    print("Area of certainty: ", Area)
    
    #New Position based on low probability region
    lat,lon,t_xy, dist_move, heading_next=Nest.get_waypoint(lat0, lon0,confidence_level=0.7,ang=100/sign)
    folder = f"Probability_Plots/{datetime.now().strftime('%Y%m%d_%H%M%S')}/"
    if save:       
        os.makedirs(folder, exist_ok=True)
    Nest.save_plot(folder+"M1",save=save,true_nest=(true_lat,true_lon))

    print("Next drone Location: ", lat,lon)
    
    
    i=2
    nest_error=Nest.get_nest_error()
    move_drone=None
    Total_distance+=dist_move
    heading=heading_next
   
    while ( Area>10 and i<30):
        print("----------------------------------------------------------")
        print(f"Measurement {i}")
        print("-----------------------------------------------------------") 
        label=f"M{i}"
        
        # 1) SENSOR MEASUREMENT (relative bearing)
        dt, bearing_rel, bearing_std = sim.sample_dt_and_bearing(lat, lon, heading)
        bearing_std=20
        # 2) UPDATE POSTERIOR (convert rel to abs inside add_measurement)
        Nest.add_measurement(
            lat, lon,
            heading,        # absolute drone heading
            bearing_rel,    # relative sensor bearing
            bearing_std,
            dt,
            label,
            move_drone
        )
        
        Area=Nest.check_certainty(confidence=0.95)
        print("Area of certainty: ", Area)
        Nest.save_plot(folder+f"M{i}",save=save,true_nest=(true_lat,true_lon))
        lat,lon,t_xy, dist_move,heading_next=Nest.get_waypoint(lat,lon, confidence_level=0.7,ang=100/sign)
   
        # 4) MOVE DRONE
        Total_distance += dist_move         
        print("Next drone Location: ", lat,lon)
        print("Drone is moving ", np.round(dist_move), "m at ", np.round(heading_next), "degrees from North")
    
        
        nest_error=Nest.get_nest_error()
        heading=heading_next    
        sign=sign*(-1)
        i+=1
       
        
    Num_measurements.append((i))
    Distances.append(Total_distance)
    Errors.append(nest_error)
    nest_estimate_lat, nest_estimate_lon=Nest.get_nest_location()
    print("Final drone lat, lon: ",float(nest_estimate_lat),",", float(nest_estimate_lon) )
    print("Nest lat, lon:", true_lat, true_lon)
    print("Nest Error: ", np.round(nest_error,2), "m" )
    
Average_measurements=np.mean(Num_measurements)
Variance_measurements=np.var(Num_measurements)
Average_distance=np.mean(Distances)
Variance_distance=np.var(Distances)
Average_error=np.mean(Errors)
Variance_error=np.var(Errors)

print("----------------------------------")
print("Average number of measurement points: ", Average_measurements)
print("Variance of number of measurement points: ", Variance_measurements)
print("Average Drone distance: ", Average_distance)
print("Variance of Drone distance: ", Variance_distance)
print("Average Nest Error: ", Average_error)
print("Variance of Nest error: ", Variance_error)


