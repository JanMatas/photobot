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
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw",Image,self.callback)#/camera/rgb/image_color
        self.counter = 0
    def callback(self, msg):

        try:
            cv2_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            import cv2
            retval, buffer = cv2.imencode('.jpg', cv2_img)
            jpg_as_text = base64.b64encode(buffer)
            self.imagePub.publish(str(jpg_as_text))
        except CvBridgeError, e:
            print e

def main():
    rospy.init_node('ui')
    UiPublisher()
    rospy.spin()

if __name__ == '__main__':
    main()
