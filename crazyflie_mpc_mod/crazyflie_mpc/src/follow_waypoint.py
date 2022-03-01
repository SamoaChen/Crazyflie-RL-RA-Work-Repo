#!/usr/bin/env python
import numpy as np
import rospy
import tf2_ros as tf
import time
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Imu
from tf import TransformListener
from std_msgs.msg import String
from std_srvs.srv import Empty, EmptyResponse

from scipy.spatial.transform import Rotation
import waypoint_traj as wt
from mpc_control import MPControl
from hybrid_control import HybridControl
from geometric_control import GeometriControl


class MPCDemo():
    def __init__(self):
        rospy.init_node('mpc_demo', anonymous=True)  # initializing node
        # self.m_serviceLand = rospy.Service('land', , self.landingService)
        # self.m_serviceTakeoff = rospy.Service('takeoff', , self.takeoffService)
        # frames and transforms
        self.worldFrame = rospy.get_param("~world_frame", "world")
        quad_name = "crazy_mpc/base_link"
        quad_name_optitrack = "crazy_mpc"
        self.frame = quad_name
        #self.frame = rospy.get_param("~frame")
        self.tf_listener = TransformListener()
        
        # subscribers and publishers
        self.rate = rospy.Rate(250)
        self.angular_vel = np.zeros([3,])  # angular velocity updated by imu subscriber
        self.curr_pos = np.zeros([3,])
        self.curr_quat = np.zeros([4,])
        self.est_vel_pub = rospy.Publisher('est_vel', TwistStamped, queue_size=1)  # publishing estimated velocity
        self.u_pub = rospy.Publisher('u_euler', TwistStamped, queue_size=1)  # publishing stamped 
        self.cmd_stamped_pub = rospy.Publisher('cmd_vel_stamped', TwistStamped, queue_size=1)  # publishing time stamped cmd_vel
        self.imu_sub = rospy.Subscriber('/crazyflie/imu', Imu, self.imu_callback)  # subscribing imu
        self.cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)  # publishing to cmd_vel to control crazyflie
        self.goal_pub = rospy.Publisher('goal', TwistStamped, queue_size=1)  # publishing waypoints along the trajectory        
        self.vicon_sub = rospy.Subscriber("/mocap_node/" + quad_name_optitrack + "/pose", PoseStamped, self.vicon_callback)
        self.tf_pub = rospy.Publisher('tf_pos', PoseStamped, queue_size=1) 
        self.serviceTakeoff = rospy.Service('takeoff', Empty, self.takeoffService)
        self.serviceLand = rospy.Service('land', Empty, self.landingService)

        # controller and waypoints
        self.m_state = 0 # Idle: 0, Automatic: 1, TakingOff: 2, Landing: 3
        self.m_thrust = 0
        self.m_startZ = 0
        
        t_final = 11
        radius = 0.3
        t_plot = np.linspace(0, t_final, num=500)
        # center of the circle is 0.2172, 4.5455
        x_traj = radius * np.cos(t_plot) + -0.311
        y_traj = radius * np.sin(t_plot) + -1.511
  
        z_traj = np.zeros((len(t_plot),)) + 0.4111
        points = np.stack((x_traj, y_traj, z_traj), axis=1)
        #print(points)
        #print("points shape", points.shape)
        #points[-1, 2] = 0.2 #final landing height - > convert to a rosservice
        #print("final points shape", points.shape)
        #points = np.array([  # points for generating trajectory
        #                   [-0.3, -1.5, 0.1],
        #                   [-0.3, -1.5, 0.2],
        #                   [-0.3, -1.5, 0.1]])
        #points = np.array([[0.,-2.,0.],
        #                   [0.,-2.,0.4],
        #                   [0.,-2.,0.]])
        #points = np.array([[0.2172, 4.5455, 0.5],
        #                   [0.2172, 4.5455, 0.6]])
        #-----GENERATE TAKEOFF AND LANDING TRAJECTORY
        self.takeoff_z = points[0, :]
        takeoff_points = np.vstack((np.array([points[0, 0], points[0, 1], 0]), self.takeoff_z))
        self.traj_takeoff = self.generate_traj(takeoff_points)
        land_points = np.vstack((points[-1, :], np.array([points[-1, 0], points[-1, 1], 0.2]))) # let the ending point be at the same x y location with height 0.2
        self.traj_land = self.generate_traj(land_points)
        
        #-----GENERATE MAIN TRAJECTORY
        self.traj = self.generate_traj(points)  # input points should have size [traj_len, 3]
        
        self.controller = GeometriControl()  # controller
        
        self.initial_state = {'x': np.array([0, 0, 0]), # positions
                              'v': np.array([0, 0, 0]), # velocities
                              'q': np.array([0, 0, 0, 1]), # quaternion
                              'w': np.zeros(3,)} # angular vel
        self.t0 = rospy.get_time()
        self.prev_time = rospy.get_time()
        self.prev_pos = self.initial_state['x']
        self.prev_vel = np.zeros([3,])
        rospy.loginfo("=============== MPC Demo Initialized ===============")
    
    def imu_callback(self, data):
        '''
        call back function for getting current angular velocity
        '''
        imu_angular_vel = Vector3()
        imu_angular_vel = data.angular_velocity
        self.angular_vel[0] = imu_angular_vel.x
        self.angular_vel[1] = imu_angular_vel.y
        self.angular_vel[2] = imu_angular_vel.z

    def vicon_callback(self, data):
        self.curr_pos[0] = data.pose.position.x
        self.curr_pos[1] = data.pose.position.y
        self.curr_pos[2] = data.pose.position.z
        self.curr_quat[0] = data.pose.orientation.x
        self.curr_quat[1] = data.pose.orientation.y
        self.curr_quat[2] = data.pose.orientation.z
        self.curr_quat[3] = data.pose.orientation.w
        
            
    #-----TRAJECTORY CONTROL 
    def traj_control(self, traj):
        curr_time = rospy.get_time()
        dt = curr_time - self.prev_time
        flat = self.sanitize_trajectory_dic(traj.update(curr_time-self.t0))

        #transform = TransformStamped()  # for getting transforms
        self.tf_listener.waitForTransform(self.worldFrame, self.frame, rospy.Time(), rospy.Duration(20.0))
        t = self.tf_listener.getLatestCommonTime(self.frame, self.worldFrame)
        (tf_pos, tf_quat) = self.tf_listener.lookupTransform(self.worldFrame, self.frame, t)  # position and quaternion in world frame
        vicon_pos = self.curr_pos
        vicon_quat = self.curr_quat
        pos = tf_pos
        quat = tf_quat

        v = (np.array(pos)-np.array(self.prev_pos))/dt  # velocity estimate
        #print("prev pos\n", self.prev_pos) 
        v_est_sum = np.sum(v)
        if v_est_sum == 0.0:
            v = self.prev_vel
        # clipping
        v[0:2] = np.clip(v[0:2], -0.7, 0.6)
        v[2] = np.clip(v[2], -0.2, 0.2)
        #print(v)
        curr_state = {
                    'x': np.array(pos),
                    'v': v,
                    'q': np.array(quat),
                    'w': self.angular_vel}
 
        # controller update
        u = self.controller.update(curr_time, curr_state, flat)
        roll = u['euler'][0]
        pitch = u['euler'][1]
        yaw = u['euler'][2]
        thrust = u['cmd_thrust']
        r_ddot_des = u['r_ddot_des']
        u1 = u['cmd_thrust']

  
        def map_u1(u1):
            # u1 ranges from -0.2 to 0.2
            trim_cmd = 42000
            min_cmd = 10000
            u1_trim = 0.327
            c = min_cmd
            m = (trim_cmd - min_cmd)/u1_trim
            mapped_u1 = u1*m + c
            if mapped_u1 > 60000:
                mapped_u1 = 60000
            return mapped_u1


        # publish command
        msg = Twist()
        msg.linear.x = np.clip(np.degrees(pitch), -10., 10.)  # pitch
        msg.linear.y = np.clip(np.degrees(roll), -10., 10.)  # roll
        msg.linear.z = map_u1(thrust)
        msg.angular.z = np.degrees(0.) # hardcoding yawrate to be 0 for now
        self.cmd_pub.publish(msg) # publishing msg to the crazyflie

        self.log_ros_info(roll, pitch, yaw, r_ddot_des, v, msg, flat, tf_pos, tf_quat, u1)
        if v_est_sum != 0:
            self.prev_vel = v
            self.prev_time = curr_time
            self.prev_pos = pos  
        return traj     



    def generate_traj(self, points):
        '''
        returns trajectory object generated from points
        '''
        return wt.WaypointTraj(points) 
    
    #-----TAKEOFF & LANDING SERVICE CALL FUNCTIONS, WHEN CALLED SWITCH STATE TO TAKING OFF AND LANDING
    def takeoffService(self, req):
        rospy.loginfo("Takeoff requested!")

        self.m_state = 2  # set state to taking off
        self.prev_time = rospy.get_time()
        self.t0 = rospy.get_time()
        return EmptyResponse()

    def landingService(self, req):
        rospy.loginfo("Land requested!")

        self.m_state = 3  # set state to land
        self.prev_time = rospy.get_time()
        self.t0 = rospy.get_time()
        return EmptyResponse()

    def takeoff(self):
        rospy.loginfo("Taking off")
        self.traj_takeoff = self.traj_control(self.traj_takeoff)
        #rospy.loginfo(self.takeoff_z[2])
        #rospy.loginfo(self.prev_pos[2])
        # listen for quad position
        self.tf_listener.waitForTransform(self.worldFrame, self.frame, rospy.Time(), rospy.Duration(20.0))
        t = self.tf_listener.getLatestCommonTime(self.frame, self.worldFrame)
        (tf_pos, tf_quat) = self.tf_listener.lookupTransform(self.worldFrame, self.frame, t)  # position and quaternion in world frame
        if self.prev_pos[2] >= tf_pos[2]: # lowering the height temporarily
            print('automatic state reached')
            self.m_state = 1 # switch to automatic
            self.prev_time = rospy.get_time()
            self.t0 = rospy.get_time()

    def land(self):
        rospy.loginfo("landing")
        self.traj_land = self.traj_control(self.traj_land)
        if self.prev_pos[2] <= 0.2:
            self.m_state = 0
            msg = Twist()
            self.cmd_pub.publish(msg)
    
    
    def automatic(self):
        curr_time = rospy.get_time()
        dt = curr_time - self.prev_time
        flat = self.sanitize_trajectory_dic(self.traj.update(curr_time-self.t0))

        transform = TransformStamped()  # for getting transforms
        self.tf_listener.waitForTransform(self.worldFrame, self.frame, rospy.Time(), rospy.Duration(20.0))
        t = self.tf_listener.getLatestCommonTime(self.frame, self.worldFrame)
        (tf_pos, tf_quat) = self.tf_listener.lookupTransform(self.worldFrame, self.frame, t)  # position and quaternion in world frame
        vicon_pos = self.curr_pos
        vicon_quat = self.curr_quat
        pos = tf_pos
        quat = tf_quat

        v = (np.array(pos)-np.array(self.prev_pos))/dt  # velocity estimate
        #print("prev pos\n", self.prev_pos) 
        v_est_sum = np.sum(v)
        if v_est_sum == 0.0:
            v = self.prev_vel
        # clipping
        v[0:2] = np.clip(v[0:2], -0.7, 0.6)
        v[2] = np.clip(v[2], -0.2, 0.2)
        #print(v)
        curr_state = {
                    'x': np.array(pos),
                    'v': v,
                    'q': np.array(quat),
                    'w': self.angular_vel}
 
        # controller update
        u = self.controller.update(curr_time, curr_state, flat)
        roll = u['euler'][0]
        pitch = u['euler'][1]
        yaw = u['euler'][2]
        thrust = u['cmd_thrust']
        r_ddot_des = u['r_ddot_des']
        u1 = u['cmd_thrust']

  
        def map_u1(u1):
            # u1 ranges from -0.2 to 0.2
            trim_cmd = 42000
            min_cmd = 10000
            u1_trim = 0.327
            c = min_cmd
            m = (trim_cmd - min_cmd)/u1_trim
            mapped_u1 = u1*m + c
            if mapped_u1 > 60000:
                mapped_u1 = 60000
            return mapped_u1


        # publish command
        msg = Twist()
        msg.linear.x = np.clip(np.degrees(pitch), -10., 10.)  # pitch
        msg.linear.y = np.clip(np.degrees(roll), -10., 10.)  # roll
        msg.linear.z = map_u1(thrust)
        msg.angular.z = np.degrees(0.) # hardcoding yawrate to be 0 for now
        self.cmd_pub.publish(msg) # publishing msg to the crazyflie

        self.log_ros_info(roll, pitch, yaw, r_ddot_des, v, msg, flat, tf_pos, tf_quat, u1)
        if v_est_sum != 0:
            self.prev_vel = v
            self.prev_time = curr_time
            self.prev_pos = pos

    def idle(self):
        while rospy.get_time() - self.t0 <= 3:
            msg = Twist()
            self.cmd_pub.publish(msg)
        #self.m_state = 1
        self.prev_time = rospy.get_time()
        self.t0 = rospy.get_time()
        #print(self.curr_pos)

        # temporarily print out stuff for debugging
        #self.tf_listener.waitForTransform(self.worldFrame, self.frame, rospy.Time(), rospy.Duration(20.0))
        #t = self.tf_listener.getLatestCommonTime(self.frame, self.worldFrame)
        #(tf_pos, tf_quat) = self.tf_listener.lookupTransform(self.worldFrame, self.frame, t)  # position and quaternion in world frame
        #print(tf_pos[2])

    def takeoff0(self):
        imsg = Twist()

        while z_ <= 0.2:
            if self.m_thrust > 50000:
                break
            transform = TransformStamped()  # for getting transforms
            self.tf_listener.waitForTransform(self.worldFrame, self.frame, rospy.Time(), rospy.Duration(20.0))
            t = self.tf_listener.getLatestCommonTime(self.frame, self.worldFrame)
            (pos, quat) = self.tf_listener.lookupTransform(self.worldFrame, self.frame, t)
            self.m_thrust += 10000 * 0.002
            self.cmd_pub.publish(msg)
            z_ = pos[2]
        
        self.m_state = 1
        self.prev_time = rospy.get_time()
        self.t0 = rospy.get_time()


    def run(self):
        while not rospy.is_shutdown():
            if self.m_state == 0:
                self.idle()
            
            elif self.m_state == 3:
                self.land()
            
            elif self.m_state == 1:
                self.automatic()
            
            elif self.m_state == 2:
                self.takeoff()
            
            #self.rate.sleep()
            
 
    def sanitize_trajectory_dic(self, trajectory_dic):
        """
        Return a sanitized version of the trajectory dictionary where all of the elements are np arrays
        """
        trajectory_dic['x'] = np.asarray(trajectory_dic['x'], np.float).ravel()
        trajectory_dic['x_dot'] = np.asarray(trajectory_dic['x_dot'], np.float).ravel()
        trajectory_dic['x_ddot'] = np.asarray(trajectory_dic['x_ddot'], np.float).ravel()
        trajectory_dic['x_dddot'] = np.asarray(trajectory_dic['x_dddot'], np.float).ravel()
        trajectory_dic['x_ddddot'] = np.asarray(trajectory_dic['x_ddddot'], np.float).ravel()

        return trajectory_dic

    def log_ros_info(self, roll, pitch, yaw, r_ddot_des, est_v, cmd_msg, flat, tf_pos, tf_quat, u1):
        '''
        logging information from this demo
        '''
        # logging controller outputs
        curr_log_time = rospy.Time.now()
        u_msg = TwistStamped()
        u_msg.header.stamp = curr_log_time
        # roll, pitch, and yaw are mapped to TwistStamped angular
        u_msg.twist.angular.x = roll          
        u_msg.twist.angular.y = pitch
        u_msg.twist.angular.z = yaw
        # r_ddot_des is mapped to TwistStamped linear
        u_msg.twist.linear.x = r_ddot_des[0]         
        u_msg.twist.linear.y = r_ddot_des[1]
        u_msg.twist.linear.z = r_ddot_des[2]
        
        # logging estimate velocities
        est_v_msg = TwistStamped()
        est_v_msg.header.stamp = curr_log_time
        # estimated velocities are mapped to TwistStampedow()
        est_v_msg.twist.linear.x = est_v[0]  
        est_v_msg.twist.linear.y = est_v[1]
        est_v_msg.twist.linear.z = est_v[2]
        
        # logging time stamped cmd_vel
        cmd_stamped_msg = TwistStamped()
        cmd_stamped_msg.header.stamp = curr_log_time
        cmd_stamped_msg.twist.linear.x = cmd_msg.linear.x
        cmd_stamped_msg.twist.linear.y = cmd_msg.linear.y
        cmd_stamped_msg.twist.linear.z = cmd_msg.linear.z
        cmd_stamped_msg.twist.angular.z = cmd_msg.angular.z
        cmd_stamped_msg.twist.angular.x = u1

        # logging waypoints
        traj_msg = TwistStamped()
        traj_msg.header.stamp = curr_log_time
        traj_msg.twist.linear.x = flat['x'][0]
        traj_msg.twist.linear.y = flat['x'][1]
        traj_msg.twist.linear.z = flat['x'][2]
        traj_msg.twist.angular.x = flat['x_dot'][0]
        traj_msg.twist.angular.y = flat['x_dot'][1]
        traj_msg.twist.angular.z = flat['x_dot'][2]

        # logging position from tf
        tf_pose_msg = PoseStamped()
        tf_pose_msg.header.stamp = curr_log_time
        tf_pose_msg.pose.position.x = tf_pos[0]
        tf_pose_msg.pose.position.y = tf_pos[1]
        tf_pose_msg.pose.position.z = tf_pos[2]
        tf_pose_msg.pose.orientation.x = tf_quat[0]
        tf_pose_msg.pose.orientation.y = tf_quat[1]
        tf_pose_msg.pose.orientation.z = tf_quat[2]
        tf_pose_msg.pose.orientation.w = tf_quat[3]


        # publishing the messages
        self.u_pub.publish(u_msg)
        self.est_vel_pub.publish(est_v_msg)
        self.cmd_stamped_pub.publish(cmd_stamped_msg)
        self.goal_pub.publish(traj_msg)
        self.tf_pub.publish(tf_pose_msg)

if __name__ == '__main__':
    mpc_demo = MPCDemo()
    mpc_demo.run()
    rospy.spin()
        
