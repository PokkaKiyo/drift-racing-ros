#!/usr/bin/env python

'''

This node is responsible for capturing the training data

'''

import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

import cv2
import csv

class TrainingDataCollector:
    def __init__(self):
        self._cam_image_sub = rospy.Subscriber("/cam_image", Image, self.cam_image_callback)
        self._cmd_vel_sub = rospy.Subscriber("/cmd_vel_final", Twist, self.cmd_vel_callback)

        self._speed = 0
        self._steering_angle = 0

        self._image_count = 0

        with open('data.csv', 'w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['filename', 'steering_angle', 'speed'])
    
    def cam_image_callback(self, msg):
        image_data = msg.data
        save_path = str(self._image_count) + '.jpg'
        cv2.imwrite(save_path, image_data)

        with open('data.csv', 'a', newline='') as file:
            writer = csv.writer(file)
            writer.writerow([save_path, self._steering_angle, self._speed])

        self._image_count += 1

    def cmd_vel_callback(self, msg):
        self._speed = msg.linear.x
        self._steering_angle = msg.angular.z



if __name__ == "__main__":
    rospy.init_node('picamera_image_publisher')
    TrainingDataCollector()
    rospy.spin()
