#!/usr/bin/env python

'''

This node is responsible for capturing images using the camera on the vehicle.

https://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython

https://picamera.readthedocs.io/en/release-1.10/recipes1.html

'''

import rospy
from sensor_msgs.msg import Image

import picamera

class CameraCapture:
    def __init__(self):
        self._image_pub = rospy.Publisher('/cam_image', Image, queue_size=1)
        self._camera  = picamera.camera.PiCamera()
        self.start_camera()
    
    def start_camera(self):
        # for testing
        self._camera.capture('example.jpg')

        # TODO: send the message over the /cam_image topic

        # TODO: put in a while loop



if __name__ == '__main__':
    rospy.init_node('camera_capture')
    CameraCapture()
    rospy.spin()
