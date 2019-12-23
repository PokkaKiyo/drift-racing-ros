#!/usr/bin/env python

'''

This node is responsible for controlling the actuators of the vehicle.
It receives a cmd_vel message of type geometry_msgs/Twist, and controls the actuators accordingly.



Currently, the actuators in our system are:
1. H-bridge for velocity control (open loop)
2. Servo for steering control (open loop)

Callbacks:
1. cmd_vel_callback - Input: A message of type geometry_msgs/Twist, where:
    - msg.linear.x is the desired speed of the vehicle, in the range [0.0, 0.9]
    - msg.linear.z is 1 if the brakes are activated, 0 otherwise
    - msg.angular.z is the desired steering angle of the vehicle
    - All other fields should be 0

'''

import rospy
from geometry_msgs.msg import Twist

import RPi.GPIO as IO
from gpiozero import AngularServo


class ActuatorsController:
    def __init__(self):
        self._cmd_vel_sub = rospy.Subscriber("/cmd_vel_final", Twist, self.cmd_vel_callback)

        self._max_speed = 0.9
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
            print("Actuators Controller: Braking!")
            self._p.ChangeDutyCycle(0)
        else:
            desired_speed = msg.linear.x
            print("Actuators Controller: The desired speed is", desired_speed)
            if desired_speed == 0:
                self._p.ChangeDutyCycle(0)
            else:
                self._p.ChangeDutyCycle(desired_speed / self._max_speed * 50 + 50) # range: [0.0, 100.0]

        # Step 2: Control the steering
        desired_steering_angle = msg.angular.z
        self._servo.angle = desired_steering_angle / self._max_steering * -85
        print("Actuators: The desired steering angle is", desired_steering_angle)
    

if __name__ == '__main__':
    rospy.init_node('actuators_controller')
    ActuatorsController()
    rospy.spin()
