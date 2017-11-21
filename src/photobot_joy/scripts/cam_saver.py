#! /usr/bin/python
# Copyright (c) 2015, Rethink Robotics, Inc.

# Using this CvBridge Tutorial for converting
# ROS images to OpenCV2 images
# http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython

# Using this OpenCV2 tutorial for saving Images:
# http://opencv-python-tutroals.readthedocs.org/en/latest/py_tutorials/py_gui/py_image_display/py_image_display.html

# rospy for the subscriber
import rospy
# ROS Image message
from sensor_msgs.msg import Image
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError
# OpenCV2 for saving an image
import cv2

from std_msgs.msg import String







class CamSaver(object):

    def __init__(self):
        rospy.init_node('cam_saver')
        # Define your image topic
        image_topic = "/usb_cam/image_raw"
        trigger_topic = "/event_output"
        # Set up your subscriber and define its callback
        rospy.Subscriber(image_topic, Image, self.image_callback)
        rospy.Subscriber(trigger_topic, String,self.trigger_callback)
        self.event_trigger = rospy.Publisher("/event_trigger", String)
        self.ui_publish = rospy.Publisher("/image_ui", Image)


        self.bridge = CvBridge()
        self.last_image = None




    def image_callback(self, msg):

        self.last_image = msg

    def trigger_callback(self, _msg):
        if not "photo" in _msg.data:
            print ("Received unknown trigger")
            return
        print("Received trigger!")
        if not self.last_image:
            print("No image received yet, trigger failed")
            return
        msg = self.last_image

        
        try:    
            # Convert your ROS Image message to OpenCV2
            cv2_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        except CvBridgeError, e:
            print(e)
        else:
            # Save your OpenCV2 image as a jpeg 
            cv2.imwrite('/home/human4/photobot/camera_image.jpeg', cv2_img)
            self.event_trigger.publish("Photo taken")
            self.ui_publish.publish(msg)


# Instantiate CvBridge


def main():
    CamSaver()
    rospy.spin()

if __name__ == '__main__':
    main()
