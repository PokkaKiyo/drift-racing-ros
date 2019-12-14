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

import RPi.GPIO as IO
from gpiozero import AngularServo


class ActuatorsController:
    def __init__(self):
        self._cmd_vel_sub = rospy.Subscriber("/cmd_vel_final", Twist, self.cmd_vel_callback)

        self._max_speed = 2.0
        self._max_steering = 0.8

        IO.setwarnings(False)
        IO.setmode(IO.BCM)
        IO.setup(12, IO.OUT)
        IO.setup(23, IO.OUT, initial=1)
        self._p = IO.PWM(12, 500)
        self._p.start(0)

        self._servo = AngularServo(13, min_angle=-90, max_angle=90)


    def cmd_vel_callback(self, msg):
        # TODO: add a limiter to the speed and steering

        # Step 1: Control the speed
        if msg.linear.z != 0:
            desired_speed = 0
            print("Actuators: Braking!")
        else:
            desired_speed = msg.linear.x

        print("Actuators: The desired speed is", desired_speed)
        self._p.ChangeDutyCycle(desired_speed / self._max_speed * 50 + 50)

        # Step 2: Control the steering
        desired_steering_angle = msg.angular.z
        self._servo.angle = desired_steering_angle / self._max_steering * 180
        print("Actuators: The desired steering angle is", desired_steering_angle)
    

if __name__ == '__main__':
    rospy.init_node('actuators_controller')
    rospy.logwarn("Actuators Controller: This code is untested, do not put on the car!")
    ActuatorsController()
    rospy.spin()
