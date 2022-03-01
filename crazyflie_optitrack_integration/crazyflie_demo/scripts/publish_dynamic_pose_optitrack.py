#!/usr/bin/env python
import math
import rospy
import tf2_ros as tf
from tf import TransformListener
from tf import transformations
from geometry_msgs.msg import PoseStamped

if __name__ == '__main__':
    rospy.init_node('publish_pose', anonymous=True)
    worldFrame = rospy.get_param("~worldFrame", "world")
    targetName = rospy.get_param("~targetName", "crazy_mpc/base_link")
    name = rospy.get_param("~name", "goal_test")
    r = rospy.get_param("~rate", 30)
    x = rospy.get_param("~x", "1")
    y = rospy.get_param("~y", "1")
    z = rospy.get_param("~z", "1")


    rate = rospy.Rate(r)

    msg = PoseStamped()
    msg.header.seq = 0
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = worldFrame
    msg.pose.position.x = x
    msg.pose.position.y = y
    msg.pose.position.z = z
    quaternion = transformations.quaternion_from_euler(0, 0, 0)
    msg.pose.orientation.x = quaternion[0]
    msg.pose.orientation.y = quaternion[1]
    msg.pose.orientation.z = quaternion[2]
    msg.pose.orientation.w = quaternion[3]

    pub = rospy.Publisher(name, PoseStamped, queue_size=1)

    # initialize listener
    listener = TransformListener()

    while not rospy.is_shutdown():
        # get updated target frame position
        listener.waitForTransform(worldFrame, targetName, rospy.Time(), rospy.Duration(5.0))
        (trans,rot) = listener.lookupTransform(worldFrame, targetName, rospy.Time(0))
        # update msg
        msg.header.seq += 1
        msg.header.stamp = rospy.Time.now()
        msg.pose.position.x = trans[0]
        msg.pose.position.y = trans[1]
        msg.pose.position.z = trans[2]
        pub.publish(msg)
        rate.sleep()