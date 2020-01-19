#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist

import picamera
import io
import csv
from PIL import Image

class TrainingDataCollector:
    def __init__(self):
        self._cmd_vel_sub = rospy.Subscriber("/cmd_vel_final", Twist, self.cmd_vel_callback)
        self._image_count_sub = rospy.Subscriber("/image_count", String, self.image_count_callback)

        self._steering_angle = 0
        self._speed = 0

        with open('/home/pi/catkin_ws/data_log.csv', 'w+') as file:
            writer = csv.writer(file)
            writer.writerow(['image_path', 'steering_angle', 'speed'])

        rospy.loginfo("TrainingDataCollector: started")

    def cmd_vel_callback(self, msg):
        self._steering_angle = msg.angular.z
        self._speed = msg.linear.x
    
    def image_count_callback(self, msg):
        image_path = msg.data + ".jpeg"

        with open('/home/pi/catkin_ws/data_log.csv', 'a+') as file:
            writer = csv.writer(file)
            writer.writerow([image_path, self._steering_angle, self._speed])



if __name__ == "__main__":
    rospy.init_node('training_data_collector')
    TrainingDataCollector()
    rospy.spin()
