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
import piggyphoto

from std_msgs.msg import String

from sensor_msgs.msg import Image
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError
# OpenCV2 for saving an image
import cv2





class CamSaver(object):

    def __init__(self):
        rospy.init_node('cam_saver')
        # Define your image topic
        trigger_topic = "/event_output"
        # Set up your subscriber and define its callback
        rospy.Subscriber(trigger_topic, String,self.trigger_callback)
        self.event_trigger = rospy.Publisher("/event_trigger", String)
        #self.ui_publish = rospy.Publisher("/image_ui", Image)



    def trigger_callback(self, _msg):
        if not "photo" in _msg.data:
            print ("Received unknown trigger")
            return
        print("Received trigger!")

        try:    
        	# Convert your ROS Image message to OpenCV2
			C = piggyphoto.camera()
			print C.abilities
			C.capture_image('/home/human4/photobot/camera_image.jpeg')
        except e:
            print(e)
        else:
            # Save your OpenCV2 image as a jpeg 
            self.event_trigger.publish("Photo taken")

            #self.ui_publish.publish(img)


# Instantiate CvBridge


def main():
    CamSaver()
    rospy.spin()

if __name__ == '__main__':
    main()
