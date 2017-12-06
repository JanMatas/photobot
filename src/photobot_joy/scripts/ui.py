#!/usr/bin/env python
import cv2
import base64
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String

class UiPublisher(object):
    def __init__(self):
        self.imagePub = rospy.Publisher("/image_base64", String)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/event_trigger",String,self.callback)#/camera/rgb/image_color
        self.counter = 0
    def callback(self, msg):
        if ("taken" in msg.data):
            filename='/home/human4/photobot/camera_image.jpg'
            img_data = open(filename, 'rb').read()
            jpg_as_text = base64.b64encode(img_data)
            self.imagePub.publish(str(jpg_as_text))

def main():
    rospy.init_node('ui')
    UiPublisher()
    rospy.spin()

if __name__ == '__main__':
    main()
