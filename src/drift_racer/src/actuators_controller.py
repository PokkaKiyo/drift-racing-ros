#!/usr/bin/env python

# This node is responsible for controlling the actuators of the vehicle


import rospy
from geometry_msgs.msg import Twist

class ActuatorsController:
    def __init__(self):
        self._cmd_vel_sub = rospy.Subscriber("/cmd_vel_final", Twist, self.cmd_vel_callback)
    
    def cmd_vel_callback(self, msg):
        print(msg)
        


if __name__ == '__main__':
    rospy.init_node('actuators_controller')
    ActuatorsController()
    rospy.spin()
