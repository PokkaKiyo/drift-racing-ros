#!/usr/bin/env python

'''

This node is responsible for controlling the actuators of the vehicle.
It receives a cmd_vel message of type geometry_msgs/Twist, and controls the actuators accordingly.

Currently, the actuators in our system are:
1. H-bridge for velocity control (open loop)
2. Servo for steering control (open loop)

Callbacks:
1. cmd_vel_callback

'''

import rospy
from geometry_msgs.msg import Twist

class ActuatorsController:
    def __init__(self):
        self._cmd_vel_sub = rospy.Subscriber("/cmd_vel_final", Twist, self.cmd_vel_callback)
    
    def cmd_vel_callback(self, msg):
        if msg.linear.z != 0:
            # TODO: write brake code here (set speed to 0)
            print("Braking!") # stub
        else:
            desired_speed = msg.linear.x
            # TODO: write speed control code here
            print("The desired speed is", desired_speed)
        
        desired_steering_angle = msg.angular.z
        # TODO: write steering controller code here
        print("The desired steering angle is", desired_steering_angle)
    
if __name__ == '__main__':
    rospy.init_node('actuators_controller')
    ActuatorsController()
    rospy.spin()
