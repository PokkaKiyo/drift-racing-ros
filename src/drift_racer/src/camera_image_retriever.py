#!/usr/bin/env python

''' This is just a skeleton file for learning '''
import rospy
from sensor_msgs.msg import Image


class CameraImageRetriever:
    def __init__(self):
        self.retrieve_images()
    
    def retrieve_images(self):
        image_data = None
        failure_count = 0
        while image_data is None:
            try:
                image_data = rospy.wait_for_message('/cam_image', Image, timeout=1)
            except Exception as e:
                failure_count += 1
                if failure_count >= 10:
                    rospy.logerr("Image Retriever: Not receiving any images!")



if __name__ == '__main__':
    rospy.init_node('camera_image_retriever')
    CameraImageRetriever()
    rospy.spin()