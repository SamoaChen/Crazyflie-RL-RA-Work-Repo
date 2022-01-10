#!usr/bin/env python
#-----IMPORT PACKAGES
import numpy as np
import rospy
import tf2_ros as tf
import time
from tf import TransformListener

#-----IMPORT MESSAGES
from sensor_msgs.msg import Temperature
from std_srvs.srv import Empty, EmptyResponse

#-----CREATE ANEMOMETER CLASS
class Anemometer(object):
    """writing anemometer reading to csv based on service request"""
    def __init__(self):
        self.record_srv = rospy.Service('record', Empty, self.recordService)
        self.vel_sub = rospy.Subscriber('flow_vel', Temperature, self.vel_sub_callback, queue_size=1)
        self.tf_listener = TransformListener()

        # PUBLIC PARAMETERS
        self.world_frame = 'world'
        self.target_frame = 'crazyflie_test'
        self.record = False
        self.start_time = None
        self.record_duration = 10
        self.write_data = None
        self.folder_path = '/home/samoa/catkin_ws/src/flow_field/data/'
    
    def recordService(self, req):
        rospy.loginfo('Recording Started')
        self.record = True

        #self.tf_listener.waitForTransform(self.world_frame, self.target_frame, rospy.Time(), rospy.Duration(10.0))
        #(tf_pos, tf_quat) = self.tf_listener.lookupTransform(self.world_frame, self.target_frame, rospy.Time(0))
        file_name = self.folder_path + 'level3_flow.txt'
        self.write_data = open(file_name, 'w+')

        # RECORD DATA
        #x_str, y_str, z_str = str(tf_pos[0]), str(tf_pos[1]), str(tf_pos[2])
        #pos_str = x_str + ',' + y_str + ',' + z_str + '\n'
        time_str = str(time.time()) + '\n'
        #self.write_data.write(pos_str + time_str) #debug
        self.write_data.write(time_str)
        self.start_time = time.time()
        while ((time.time() - self.start_time) < self.record_duration):
            pass
        self.record = False
        self.write_data.close()
        rospy.loginfo('Recording Stopped')
        return EmptyResponse()
    
    def vel_sub_callback(self, flow_vel):
        if self.record:
            vel = flow_vel.temperature
            stamp = flow_vel.header.stamp.to_sec()
            data = str(stamp) + ',' + str(vel) + '\n'
            self.write_data.write(data)

if __name__ == "__main__":
    rospy.init_node('data_recording_node', anonymous=True)
    anemometer = Anemometer()
    rospy.spin()