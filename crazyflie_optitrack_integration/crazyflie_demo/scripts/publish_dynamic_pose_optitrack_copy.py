#!/usr/bin/env python
#-----IMPORT PACKAGES
import math
import rospy
import tf
from geometry_msgs.msg import PoseStamped

#-----IMPORT MESSAGES
from std_srvs.srv import Empty, EmptyResponse

#-----CREATE COMMANDER CLASS
class Commander(object):
    """publish command to the drone"""
    def __init__(self):
        # INPUT PARAMETERS
        self.name = rospy.get_param("~name", "goal")
        self.worldFrame = rospy.get_param("~worldFrame", "world")
        self.targetName = rospy.get_param("~targetName", "flat_plate/base_link") #crazy_mpc/base_link for optitrack  flat_plate
        self.r = rospy.get_param("~rate", 30)
        self.x = rospy.get_param("~x", -0.2851)
        self.y = rospy.get_param("~y", -1.556)
        self.z = rospy.get_param("~z", 0.2)

        # ROS OBJECTS
        self.follow_srv = rospy.Service('follow', Empty, self.followService)
        self.hover_srv = rospy.Service('hover', Empty, self.hoverService)
        self.goal_pub = rospy.Publisher(self.name, PoseStamped, queue_size=1)
        self.tf_listener = tf.TransformListener()
        self.rate = rospy.Rate(self.r)

        # PUBLIC PARAMETERS
        self.dynamic = False
        self.msg = PoseStamped()
        self.msg.header.seq = 0
        self.msg.header.stamp = rospy.Time.now()
        self.msg.header.frame_id = self.worldFrame
        self.msg.pose.position.x = self.x
        self.msg.pose.position.y = self.y
        self.msg.pose.position.z = self.z
        quaternion = tf.transformations.quaternion_from_euler(0, 0, 0)
        self.msg.pose.orientation.x = quaternion[0]
        self.msg.pose.orientation.y = quaternion[1]
        self.msg.pose.orientation.z = quaternion[2]
        self.msg.pose.orientation.w = quaternion[3]

    def hoverService(self, req):
        rospy.loginfo('Hover at Current Position')
        self.dynamic = False
        return EmptyResponse()

    def followService(self, req):
        rospy.loginfo('Dynamic Target Tracking')
        self.dynamic = True
        return EmptyResponse()

    def goal_publisher(self):
        while not rospy.is_shutdown():
            if self.dynamic:
                self.tf_listener.waitForTransform(self.worldFrame, self.targetName, rospy.Time(), rospy.Duration(10))
                (trans, rot) = self.tf_listener.lookupTransform(self.worldFrame, self.targetName, rospy.Time(0))
                self.msg.header.seq += 1
                self.msg.header.stamp = rospy.Time.now()
                self.msg.pose.position.x = trans[0]
                self.msg.pose.position.y = trans[1]
                self.msg.pose.position.z = self.z # make sure the drone hover above the target boat
                self.goal_pub.publish(self.msg)
                self.rate.sleep()
            else:
                self.msg.header.seq += 1
                self.msg.header.stamp = rospy.Time.now()
                self.goal_pub.publish(self.msg)
                self.rate.sleep()
                
if __name__ == '__main__':
    rospy.init_node('publish_pose', anonymous=True)
    commander = Commander()
    commander.goal_publisher()
