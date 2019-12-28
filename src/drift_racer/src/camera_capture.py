#!/usr/bin/env python

import rospy
from std_msgs.msg import String

import picamera
import io
from PIL import Image

class CameraCapture:
    def __init__(self):
        self._image_count_pub = rospy.Publisher('/image_count', String, queue_size=1)
        self._image_count = 0
        rospy.loginfo("CameraCapture: starting camera stream.")
        self.start_camera()
    
    def start_camera(self):
        with picamera.PiCamera(resolution=(640, 480), framerate=30) as camera:
            try:
                stream = io.BytesIO()
                for _ in camera.capture_continuous(stream, format='jpeg', use_video_port=True):
                    stream.seek(0)
                    image = Image.open(stream).convert('RGB')
                    img_save_path = "/media/pi/tamano/data/" + str(self._image_count) + ".jpg"
                    image.save(img_save_path)

                    self._image_count_pub.publish(str(self._image_count))
                    self._image_count += 1
            except KeyboardInterrupt:
                rospy.loginfo("keyboard interrupt")
            finally:
                rospy.loginfo("done")

if __name__ == '__main__':
    rospy.init_node('camera_capture')
    CameraCapture()
    rospy.spin()
