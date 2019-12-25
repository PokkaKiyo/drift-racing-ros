#!/usr/bin/env python

'''

This node is responsible for capturing images using the camera on the vehicle, and publishing it on a topic

'''

import rospy
from sensor_msgs.msg import Image

import numpy as np

import picamera

class PiCameraImagePublisher:
    def __init__(self):
        self._image_pub = rospy.Publisher('/cam_image', Image, queue_size=1)

        self._camera_timer = rospy.Timer(rospy.Duration(0.2), camera_timer_callback)

        self._camera  = picamera.camera.PiCamera()
        self._camera.resolution = (320, 240)
    
    def camera_timer_callback(self, event):
        print("camera_timer_callback_called")

        msg = Image()
        msg.data = np.empty((240 * 320 * 3,), dtype=np.uint8)
        self._camera.capture(msg.data)
        msg.data = msg.data.reshape((240, 320, 3))

        self._image_pub.publish(msg)
        


if __name__ == '__main__':
    rospy.init_node('picamera_image_publisher')
    PiCameraImagePublisher()
    rospy.spin()    

