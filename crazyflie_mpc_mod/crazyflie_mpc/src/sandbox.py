import matplotlib.pyplot as plt
from casadi import *
import time
import sys
sys.path.insert(1, '../')

from flightsim.simulate import Quadrotor, Quadrotor_nom, simulate
from flightsim.world import World
from flightsim.axes3ds import Axes3Ds
from flightsim.crazyflie_params import quad_params
from flightsim import hover_traj

import waypoint_traj
import se3_control
import mpc_control
import hybrid_control

from tslearn.metrics import dtw, dtw_path

np.random.seed(3)

# This object defines the quadrotor dynamical model and should not be changed.
quadrotor = Quadrotor(quad_params)
# 'NODE', 'GP', or 'None'
quadrotor_nom = Quadrotor_nom(quad_params, None)

# You will complete the implementation of the SE3Control object.
se3_control = se3_control.SE3Control(quad_params)
mpc_control = mpc_control.MPControl(quad_params)
hybrid_control = hybrid_control.NODEControl(quad_params)

# This simple hover trajectory is useful for tuning control gains.
my_traj = hover_traj.HoverTraj()

t_final = 10
radius = 1.0
t_plot = np.linspace(0, t_final, num=500)
# circle
x_traj = radius * np.cos(t_plot)
y_traj = radius * np.sin(t_plot)

z_traj = np.zeros((len(t_plot),))
points = np.stack((x_traj, y_traj, z_traj), axis=1)

my_traj = waypoint_traj.WaypointTraj(points)

# Set simulation parameters.
#
# You may use the initial condition and a simple hover trajectory to examine the
# step response of your controller to an initial disturbance in position or
# orientation.

w = radius * 1.2
world = World.empty((-w, w, -w, w, -w * 0.1, w))
# test_cube
initial_state = {'x': np.array([radius, 0, 0]),
                 'v': np.array([0, 0, 0]),
                 'q': np.array([0, 0, 0., 1.]),  # [i,j,k,w]
                 'w': np.zeros(3, )}

# Perform simulation.
#
# This function performs the numerical simulation.  It returns arrays reporting
# the quadrotor state, the control outputs calculated by your controller, and
# the flat outputs calculated by you trajectory.
start_time = time.time()
print('Simulate.')
(time_sim, state, control, flat, exit, noiseT, noiseM, nominalState, state_dot, nominalStateDot,
 rotor_speeds) = simulate(initial_state,
                          quadrotor,
                          quadrotor_nom,
                          mpc_control,
                          my_traj,
                          t_final)
print(exit.value)
end_time = time.time()
print(f'Solved in {end_time - start_time:.2f} seconds')

quats = state['q']
from scipy.spatial.transform import Rotation

q1 = np.squeeze(quats)
r = Rotation.from_quat(q1)
rot_mat = r.as_matrix()

t11 = rot_mat[:, 0, 0]
t12 = rot_mat[:, 0, 1]
t13 = rot_mat[:, 0, 2]
t21 = rot_mat[:, 1, 0]
t22 = rot_mat[:, 1, 1]
t23 = rot_mat[:, 1, 2]
t31 = rot_mat[:, 2, 0]
t32 = rot_mat[:, 2, 1]
t33 = rot_mat[:, 2, 2]
# Get Euler angles from rotation matrix
phi = np.arcsin(t32)
tet = np.arctan2(np.divide(-t31, np.cos(phi)), np.divide(t33, np.cos(phi)))
psi = np.arctan2(np.divide(-t12, np.cos(phi)), np.divide(t22, np.cos(phi)))

quats = state['q']
vel_i = state['v']
# r           = Rotation.from_quat(quats)
# rot_mat     = r.as_matrix()
q1 = np.squeeze(quats)
r = Rotation.from_quat(q1)
rot_mat = r.as_matrix()
vel_b = np.zeros_like(vel_i)
for i in range(rot_mat.shape[0]):
    vel_b[i, :] = rot_mat[i, :, :].T @ vel_i[i, :]

quats = nominalState['q']
vel_i = nominalState['v']
q1 = np.squeeze(quats)
r = Rotation.from_quat(q1)
rot_mat = r.as_matrix()
vel_b_nom = np.zeros_like(vel_i)
for i in range(rot_mat.shape[0]):
    vel_b_nom[i, :] = rot_mat[i, :, :].T @ vel_i[i, :]

# Plot Results

# 3D Path
N = time_sim.shape[0]
fig = plt.figure('3D Path')
ax = Axes3Ds(fig)
world.draw(ax)
ax.plot([state['x'][0, 0]], [state['x'][0, 1]], [state['x'][0, 2]], 'g.', markersize=12, markeredgewidth=2,
        markerfacecolor='none')
ax.plot([state['x'][N - 1, 0]], [state['x'][N - 1, 1]], [state['x'][N - 1, 2]], 'r.', markersize=12, markeredgewidth=2,
        markerfacecolor='none')
ax.plot3D(state['x'][:, 0], state['x'][:, 1], state['x'][:, 2], 'b')
ax.plot3D(flat['x'][:, 0], flat['x'][:, 1], flat['x'][:, 2], 'k')
ax.legend(('start', 'end', 'flown traj', 'planned traj'))

# Position and Velocity vs. Time
(fig, axes) = plt.subplots(nrows=4, ncols=2, sharex=True, num='States')
x = state['x']
x_des = flat['x']
x_nom = nominalState['x']
ax = axes[0, 0]
ax.plot(time_sim, x[:, 0], 'r', time_sim, x[:, 1], 'g', time_sim, x[:, 2], 'b')
ax.plot(time_sim, x_des[:, 0], 'r--', time_sim, x_des[:, 1], 'g--', time_sim, x_des[:, 2], 'b--')
# ax.plot(time_sim, x_nom[:,0], 'r--',    time_sim, x_nom[:,1], 'g--',    time_sim, x_nom[:,2], 'b--')
ax.legend(('x', 'y', 'z'))
ax.set_ylabel('Pos, m')
ax.grid('major')
# ax.set_title('Position')
v = state['v']
v_des = flat['x_dot']
v_nom = nominalState['v']
aerr_x = (vel_b[:, 0] - vel_b_nom[:, 0]) * 500
aerr_y = (vel_b[:, 1] - vel_b_nom[:, 1]) * 500
aerr_z = (vel_b[:, 2] - vel_b_nom[:, 2]) * 500
ax = axes[0, 1]
ax.plot(time_sim, v[:, 0], 'r', time_sim, v[:, 1], 'g', time_sim, v[:, 2], 'b')
ax.plot(time_sim, v_des[:, 0], 'r--', time_sim, v_des[:, 1], 'g--', time_sim, v_des[:, 2], 'b--')
# ax.plot(time_sim, v_nom[:,0], 'r--', time_sim, v_nom[:,1], 'g--', time_sim, v_nom[:,2], 'b--')
# ax.plot(time_sim, aerr_x, 'r', time_sim, aerr_y, 'g', time_sim, aerr_z, 'b')
# ax.plot(time_sim, vel_b[:,0], 'r', time_sim, vel_b[:,1], 'g', time_sim, vel_b[:,2], 'b')
# ax.legend(('x', 'y', 'z'))
ax.set_ylabel('Vel,m/s')
ax.set_xlabel('time, s')
ax.grid('major')

# Orientation and Angular Velocity vs. Time
# (fig, axes) = plt.subplots(nrows=2, ncols=1, sharex=True, num='Orientation vs Time')
# q_des = control['cmd_q']
q = state['q']
ax = axes[1, 0]
# ax.plot(time, q_des[:,0], 'r--', time, q_des[:,1], 'g--', time, q_des[:,2], 'b', time, q_des[:,3], 'k--')
# ax.plot(time, q[:,0], 'r',    time, q[:,1], 'g',    time, q[:,2], 'b',    time, q[:,3],     'k')
# ax.plot(time, np.degrees(phi_des), 'r--', time, np.degrees(tet_des), 'g--', time, np.degrees(psi_des), 'b--')
ax.plot(time_sim, np.degrees(phi), 'r', time_sim, np.degrees(tet), 'g', time_sim, np.degrees(psi), 'b')
# ax.legend(('i', 'j', 'k', 'w'))
# ax.set_ylabel('quaternion')
ax.legend(('roll', 'pitch', 'yaw'))
ax.set_ylabel('Euler,deg')
# ax.set_xlabel('time, s')
ax.grid('major')
w = state['w']
ax = axes[1, 1]
ax.plot(time_sim, np.degrees(w[:, 0]), 'r', time_sim, np.degrees(w[:, 1]), 'g', time_sim, np.degrees(w[:, 2]), 'b')
# ax.legend(('x', 'y', 'z'))
ax.set_ylabel('AngVel,deg/s')
# ax.set_xlabel('time, s')
ax.grid('major')

# Commands vs. Time
# (fig, axes) = plt.subplots(nrows=4, ncols=1, sharex=True, num='Commands vs Time')
s = control['cmd_motor_speeds']
accel_des = control['r_ddot_des']
accel_des = np.squeeze(accel_des)
ax = axes[2, 0]
ax.plot(time_sim, s[:, 0], 'r', time_sim, s[:, 1], 'g', time_sim, s[:, 2], 'b', time_sim, s[:, 3], 'k')
ax.plot(time_sim, rotor_speeds[:, 0], 'r--', time_sim, rotor_speeds[:, 1], 'g--', time_sim, rotor_speeds[:, 2], 'b--',
        time_sim, rotor_speeds[:, 3], 'k--')
ax.legend(('1', '2', '3', '4'))
ax.set_ylabel('motorSpd, rad/s')
ax = axes[2, 1]
ax.plot(time_sim, accel_des[:, 0], 'r', time_sim, accel_des[:, 1], 'g', time_sim, accel_des[:, 2], 'b')
# ax.legend(('x', 'y', 'z'))
ax.set_ylabel('accelCmds, m/s^2')
ax.grid('major')
# ax.set_title('Commands')
M = control['cmd_moment']
ax = axes[3, 0]
ax.plot(time_sim, M[:, 0] + 0 * noiseM[0:M.shape[0], 0], 'r',
        time_sim, M[:, 1] + 0 * noiseM[0:M.shape[0], 1], 'g',
        time_sim, M[:, 2] + 0 * noiseM[0:M.shape[0], 2], 'b')
# ax.legend(('x', 'y', 'z'))
ax.set_ylabel('moment, N*m')
ax.grid('major')
T = control['cmd_thrust']
ax = axes[3, 1]
ax.plot(time_sim, T + 0 * noiseT[0:T.shape[0], 0], 'k')
ax.set_ylabel('thrust, N')
ax.set_xlabel('time, s')
ax.grid('major')


# Metric to compare between two time sequences, dynamic time warping (DTW)
dtw_score_pos = dtw(x, x_des)
dtw_score_vel = dtw(v, v_des)
print('dtw_score, pos:', dtw_score_pos, 'vel:', dtw_score_vel)

pos_rmse = np.sqrt(np.sum(np.square(np.subtract(x, x_des))) / (x.shape[0] * x.shape[1]))
vel_rmse = np.sqrt(np.sum(np.square(np.subtract(v, v_des))) / (v.shape[0] * v.shape[1]))
print('reference RMSE: pos:', pos_rmse, 'vel:', vel_rmse)

# pos_rmse = np.sqrt(np.sum(np.square(np.subtract(x, x_nom)))/(x.shape[0]*x.shape[1]))
# vel_rmse = np.sqrt(np.sum(np.square(np.subtract(v, v_nom)))/(v.shape[0]*v.shape[1]))
# print('open-loop RMSE, pos:', pos_rmse, 'vel:', vel_rmse)

# time, pos_x, pos_y, pos_z, vel_x, vel_y, vel_z, quat0, quat1, quat2, quat3, omega_x, omega_y, omega_z, accel_cmd_x, accel_cmd_y, accel_cmd_z, thrust, moment_x, moment_y, moment_z
# data = np.hstack((time_sim, x[:,0], x[:,1], x[:,2], v[:,0], v[:,1], v[:,2], q[:,0], q[:,1], q[:,2], q[:,3], w[:,0], w[:,1], w[:,2],
#                   accel_des[:,0], accel_des[:,1], accel_des[:,2], T, M[:,0], M[:,1], M[:,2]))
# with open('traj_data.npy', 'wb') as f:
#     np.save(f, data)
# with open('traj_data.npy', 'rb') as f:
#     data1 = np.load(f)
# print(data1)

# time, pos_x, pos_y, pos_z, vel_x, vel_y, vel_z, quat0, quat1, quat2, quat3,
# omega_x, omega_y, omega_z, thrust, moment_x, moment_y, moment_z,
# pos_x_des, pos_y_des, pos_z_des, vel_x_des, vel_y_des, vel_z_des
# data = np.vstack((time_sim, x[:,0], x[:,1], x[:,2], v[:,0], v[:,1], v[:,2], q[:,0], q[:,1], q[:,2], q[:,3], w[:,0], w[:,1], w[:,2],
#                   T+0*noiseT[0:T.shape[0],0], M[:,0]+0*noiseM[0:M.shape[0],0], M[:,1]+0*noiseM[0:M.shape[0],1], M[:,2]+0*noiseM[0:M.shape[0],2],
#                   x_des[:,0], x_des[:,1], x_des[:,2], v_des[:,0], v_des[:,1], v_des[:,2]))
# with open('../../KNODE/Data/uncertain_params/traj_data_mpc_aerodrag_radius4.npy', 'wb') as f:
#     np.save(f, data.T)


# data = np.vstack((time_sim, vel_b[:,0], vel_b[:,1], vel_b[:,2], vel_b_nom[:,0], vel_b_nom[:,1], vel_b_nom[:,2], aerr_x, aerr_y, aerr_z))
# with open('../../GP/Data/vel_accel_drag.npy', 'wb') as f:
#    np.save(f, data.T)

# data = np.vstack((time_sim,
#                   x[:,0], x[:,1], x[:,2], v[:,0], v[:,1], v[:,2], q[:,0], q[:,1], q[:,2], q[:,3], w[:,0], w[:,1], w[:,2], T, M[:,0], M[:,1], M[:,2],
#                   state_dot[:,0], state_dot[:,1], state_dot[:,2], state_dot[:,3], state_dot[:,4], state_dot[:,5], state_dot[:,6], state_dot[:,7], state_dot[:,8], state_dot[:,9], state_dot[:,10], state_dot[:,11], state_dot[:,12],
#                   nominalStateDot[:,0], nominalStateDot[:,1], nominalStateDot[:,2], nominalStateDot[:,3], nominalStateDot[:,4], nominalStateDot[:,5], nominalStateDot[:,6], nominalStateDot[:,7], nominalStateDot[:,8], nominalStateDot[:,9], nominalStateDot[:,10], nominalStateDot[:,11], nominalStateDot[:,12]))
# with open('./GP_data/full_aerodrag_radius6.npy', 'wb') as f:
#     np.save(f, data.T)

# Animation (Slow)
# Instead of viewing the animation live, you may provide a .mp4 filename to save.
# R = Rotation.from_quat(state['q']).as_matrix()
# ani = animate(time_sim, state['x'], R, world=world, filename="explore_statespace.mp4")

plt.show()
