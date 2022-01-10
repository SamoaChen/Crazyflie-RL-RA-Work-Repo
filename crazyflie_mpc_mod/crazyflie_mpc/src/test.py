#!/usr/bin/env python

import numpy as np
import waypoint_traj as wt

curr_pos = np.zeros([3,])

radius = 0.5
t_final = 12
t_plot = np.linspace(0, t_final, num=500)
x_traj = radius * np.cos(t_plot) + 0.2172
y_traj = radius * np.sin(t_plot) + 4.5455
  
z_traj = np.zeros((len(t_plot),)) + 0.4
points = np.stack((x_traj, y_traj, z_traj), axis=1)
print("points shape", points.shape)
#points[-1, 2] = 0.2 #final landing height - > convert to a rosservice
print("final points shape", points.shape)
#points = np.array([  # points for generating trajectory
#                   [-1.409, 2.826, 0.55],
#                   [1.609, 2.826, 0.55],
#                   [1.609, 5.826, 0.55],
#                   [-1.409, 5.826, 0.55],
#                   [-1.409, 2.826, 0.55],
#                   [-1.409, 2.826, 0.3],
#                   [-1.409, 2.826, 0.0]])
#points = np.array([[0.,-2.,0.],
#                   [0.,-2.,0.4],
#                   [0.,-2.,0.]])
#-----GENERATE TAKEOFF AND LANDING TRAJECTORY
takeoff_points = np.vstack((curr_pos, points[0, :]))
traj_takeoff = wt.WaypointTraj(takeoff_points)
land_points = np.vstack((points[-1, :], np.array([points[-1, 0], points[-1, 1], 0.2]))) # let the ending point be at the same x y location with height 0.2
traj_land = wt.WaypointTraj(land_points)
print(traj_land.update(0))
print(points[-1, :])
