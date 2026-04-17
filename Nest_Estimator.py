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
    def __init__(self, lat0, lon0, grid_size=1000, span=1000):
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
   
        self.P = self.P*radial*angular
        self.measurements.append((dx, dy, label,heading_abs,heading_rel))
        move_heading= meas_bearing_abs
        if distance==None:
            dist_move=r_mean+r_std/2
        else:
            dist_move=distance
        if dist_move>700:
            print("Likely overshoot, reduced to 700 m movement")
            dist_move=700
        return dist_move,move_heading

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
            dx_arrow = 100 * np.sin(rad)   # East component
            dy_arrow = 100 * np.cos(rad)   # North component
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
                
            plt.scatter(tx, ty, c='yellow', marker='*', s=self.span*2/10, edgecolor='black', linewidth=1.2, zorder=10)
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

    def _local_minima_mask(self, P_norm):
        pad = np.pad(P_norm, pad_width=1, mode='constant', constant_values=np.inf)
        center = pad[1:-1, 1:-1]
        neighs = [
            pad[0:-2, 0:-2], pad[0:-2, 1:-1], pad[0:-2, 2:],
            pad[1:-1, 0:-2],                 pad[1:-1, 2:],
            pad[2:  , 0:-2], pad[2:  , 1:-1], pad[2:  , 2:]
        ]
        comp = np.ones_like(center, dtype=bool)
        for n in neighs:
            comp &= (center < n)
        return comp

    def _confidence_area_radius(self, P_norm, confidence_level=0.30):
        """Returns area that has a probability greater than the confidence level"""
        mask = (P_norm >= confidence_level)
        if not mask.any():
            return 0.0
        cell_area = ((2.0 * self.span) ** 2) / (self.grid_size ** 2)
        area = mask.sum() * cell_area
        return m.sqrt(area / m.pi)

    def select_closest_low_prob(self, current_lat, current_lon, current_heading,
                                measurement_index,
                                confidence_level=0.30,
                                low_mode='fraction',   # 'fraction'|'percentile'|'local_minima'
                                fraction=0.5,          # for 'fraction': P < fraction * P_peak
                                percentile=20.0,       # for 'percentile': P_norm < percentile-th percentile
                                min_separation=20.0):
        """
        Return (dist_m, heading_abs_deg, target_xy) or None.
        - measurement_index: if <3 returns None (use old behaviour).
        - low_mode:
            'fraction' -> choose cells with P_norm < fraction * P_peak
            'percentile' -> choose cells with P_norm below given percentile
            'local_minima' -> choose local minima inside circle (strictly less than 8 neighbors)
        - selects the candidate with smallest distance to the MAP (closest low-P to peak).
        - returns absolute heading (0 = North, clockwise). target_xy is (x_m, y_m) in metres.
        """
        if measurement_index is None:
            measurement_index = 0
        if measurement_index < 4:
            return None

        if np.max(self.P) <= 0:
            return None
        P_norm = self.P / np.max(self.P)

        map_info = self.get_map_max_xy()
        if map_info is None:
            return None
        map_x, map_y, _, _, p_peak = map_info

        radius = self._confidence_area_radius(P_norm, confidence_level=confidence_level)
        if radius <= 0:
            return None

        # mask of cells inside the circle
        dist_from_map = np.sqrt((self.X - map_x)**2 + (self.Y - map_y)**2)
        inside_circle = (dist_from_map <= radius)
        if not inside_circle.any():
            return None

        # build candidate mask according to low_mode
        if low_mode == 'fraction':
            thresh = fraction * p_peak
            candidate_mask = (P_norm < thresh) & inside_circle
        elif low_mode == 'percentile':
            # compute percentile threshold over cells inside circle
            vals = P_norm[inside_circle]
            if vals.size == 0:
                return None
            thr = np.percentile(vals, percentile)
            candidate_mask = (P_norm <= thr) & inside_circle
        elif low_mode == 'local_minima':
            minima_mask = self._local_minima_mask(P_norm)
            candidate_mask = minima_mask & inside_circle
        else:
            raise ValueError("low_mode must be 'fraction','percentile', or 'local_minima'")

        # if no candidates found, relax criteria: use the lowest-value cell inside circle
        if not candidate_mask.any():
            masked = np.where(inside_circle, P_norm, np.inf)
            idx_flat = np.argmin(masked)
            r_idx, c_idx = np.unravel_index(idx_flat, P_norm.shape)
            tx = float(self.X[r_idx, c_idx])
            ty = float(self.Y[r_idx, c_idx])
        else:
            ys, xs = np.where(candidate_mask)
            # compute distances from MAP to each candidate and pick the closest to the MAP
            dists_to_map = np.sqrt((self.X[ys, xs] - map_x)**2 + (self.Y[ys, xs] - map_y)**2)
            sel = int(np.argmin(dists_to_map))
            r_idx, c_idx = int(ys[sel]), int(xs[sel])
            tx = float(self.X[r_idx, c_idx])
            ty = float(self.Y[r_idx, c_idx])

        # compute vector from current drone pose to target
        px, py = self.latlon_to_xy(current_lat, current_lon)
        dx = tx - px
        dy = ty - py
        dist_to_target = m.hypot(dx, dy)
        if dist_to_target < min_separation:
            return None

      
        waypoint_lat, waypoint_lon=self.xy_to_latlon(tx, ty, self.lat0, self.lon0)
        heading_abs = m.degrees(m.atan2(dx, dy)) % 360.0
    
        return float(waypoint_lat), float(waypoint_lon), (tx, ty)
        
    def get_nest_location(self):
        mm=self.get_map_max_xy()
        map_x, map_y, _, _, _ = mm
        nest_lat,nest_lon=self.xy_to_latlon(map_x, map_y, lat0, lon0)
        return nest_lat, nest_lon
        

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
iterations=15

for its in range(iterations):
    #Initialise
    lat0,lon0,heading_abs=get_pose()
    Total_distance=0
    Nest=NestEstimator(lat0, lon0)
    sim = NestSimulator(lat0, lon0)
    true_lat,true_lon=sim.place_random_nest(radius=800,min_radius=100)    
    
    
    #Measurement 1
    label="M1"
    print("-----------------------------------------------------------")
    print("Measurement 1")
    print("-----------------------------------------------------------") 

    #RELATIVE MEASUREMENT
    dt, measure_heading_rel, heading_std = sim.sample_dt_and_bearing(lat0, lon0,heading_abs)
    #ABSOLUTE DRONE HEADING WITH RELATIVE MEASUREMENT. RETURNS ABSOLUTE HEADING
    dist_move,move_heading_abs=Nest.add_measurement(lat0, lon0,heading_abs,measure_heading_rel,heading_std, dt, label)
    folder = f"Probability_Plots/{datetime.now().strftime('%Y%m%d_%H%M%S')}/"
    os.makedirs(folder, exist_ok=True)
    Nest.save_plot(folder+"M1",true_nest=(true_lat,true_lon))
    lat,lon=Nest.compute_next_waypoint(move_heading_abs, dist_move, lat0, lon0)
    print("Next drone Location: ", lat,lon)
    Area=Nest.check_certainty()
    print("Area of certainty: ", Area)
    
    i=2
    nest_error=Nest.get_nest_error()
    move_drone=None
    Total_distance+=dist_move
    heading=move_heading_abs
    while ( Area>10 and i<30):
        print("----------------------------------------------------------")
        print(f"Measurement {i}")
        print("-----------------------------------------------------------") 
        label=f"M{i}"
        
        # 1) SENSOR MEASUREMENT (relative bearing)
        dt, bearing_rel, bearing_std = sim.sample_dt_and_bearing(lat, lon, heading)
        
        # 2) UPDATE POSTERIOR (convert rel to abs inside add_measurement)
        dist_move, move_heading_abs = Nest.add_measurement(
            lat, lon,
            heading,        # absolute drone heading
            bearing_rel,    # relative sensor bearing
            bearing_std,
            dt,
            label,
            move_drone
        )

        
        # 3) LOW-PROBABILITY TARGETING AFTER 3 MEASUREMENTS
        if i < 4:
            # first 3 movements use estimator heading
            heading_next = move_heading_abs+bearing_std/i  
        else:
            res = Nest.select_closest_low_prob(
                lat, lon,
                heading,
                measurement_index=i,
                confidence_level=0.9,
                low_mode='local_minima',
            )
    
            if res is None:
                heading_next = move_heading_abs
            else:
                lat,lon,t_xy = res
                heading_next=move_heading_abs +bearing_std/(i+3) #Scuffed recalculation of heading that somehow works
        
        #LAT LON OF PEAK PROBABILITY FIELD
        if i>15:
            nest_estimate_lat, nest_estimate_lon=Nest.get_nest_location()
            lat, lon= nest_estimate_lat, nest_estimate_lon
            x,y=Nest.latlon_to_xy(lat, lon)
            heading_next=np.atan(x/y)

                              
        # 4) MOVE DRONE
        Total_distance += dist_move
        Nest.save_plot(folder+f"M{i}", true_nest=(true_lat, true_lon))
       
        # 5) Use absolute heading, direct distance and current position to find next waypoint.
        #For this sim works best with giving heading and distance
        #but on drone can just move to a lat/lon that is optimal and .get_current_pose() returns the new heading 
        """
        lat, lon = Nest.compute_next_waypoint(
            heading_next,   # absolute heading
            dist_move,
            lat, lon
        )
        """
       
            
        print("Next drone Location: ", lat,lon)
        print("Drone is moving ", np.round(dist_move), "m at ", np.round(heading_next), "degrees from North")
        Area=Nest.check_certainty(confidence=0.94)
        print("Area of certainty: ", Area)
        nest_error=Nest.get_nest_error()
        heading=heading_next
        i+=1
       
        
    Num_measurements.append((i))
    Distances.append(Total_distance)
    Errors.append(nest_error)
    nest_estimate_lat, nest_estimate_lon=Nest.get_nest_location()
    print("Final drone lat, lon: ",float(nest_estimate_lat),",", float(nest_estimate_lon) )
    print("Nest lat, lon:", true_lat, true_lon)
    
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


