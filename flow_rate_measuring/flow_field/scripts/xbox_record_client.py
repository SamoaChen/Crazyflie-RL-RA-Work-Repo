#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
from std_srvs.srv import Empty

class Controller():
    def __init__(self, use_controller, joy_topic):

        if use_controller:
            rospy.loginfo("waiting for record service")
            rospy.wait_for_service('record')
            rospy.loginfo("found record service")
            self._record = rospy.ServiceProxy('record', Empty)
        else:
            self._record = None

        # subscribe to the joystick at the end to make sure that all required
        # services were found
        self._buttons = None
        rospy.Subscriber(joy_topic, Joy, self._joyChanged)

    def _joyChanged(self, data):
        for i in range(0, len(data.buttons)):
            if self._buttons == None or data.buttons[i] != self._buttons[i]:
                if i == 2 and data.buttons[i] == 1 and self._record != None:
                    self._record()

        self._buttons = data.buttons

if __name__ == '__main__':
    rospy.init_node('xbox_record_client_node', anonymous=True)
    use_controller = rospy.get_param("~use_crazyflie_controller", False)
    joy_topic = rospy.get_param("~joy_topic", "joy")
    controller = Controller(use_controller, joy_topic)
    rospy.spin()