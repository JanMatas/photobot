#!/usr/bin/env python
import rospy
import apiai

from std_msgs.msg import Int32MultiArray
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Vector3


class CenteringNode(object):
    """
    ROS node for the Dialogflow natural language understanding.
    Dialogflow (previously API.ai) parses natural language into a json string containing the semantics of the text. This
    nose wraps that into a ROS actionlib interface.
    """
    def __init__(self):
        rospy.init_node('centering_node')
        rospy.Subscriber("/filter_faceCoord", Int32MultiArray, self.callback, queue_size=10)
        rospy.Subscriber("/event_output", String, self.callback_trigger, queue_size=10)
        self.enabled = False
        
        self.trigger_pub = rospy.Publisher("/event_output", String, queue_size=10)
        self.result_pub = rospy.Publisher("RosAria/cmd_vel", Twist, queue_size=10)
        #self.speech_pub = rospy.Publisher("speech_output", String, queue_size=10)
        #self.picture_pub = rospy.Publisher("take_picture", String, queue_size=10) 
        #self.request = None
        

    def callback_trigger(self, msg):
        if "Center" in msg.data:
            self.enabled = True
        
    def callback(self, msg):
        if not self.enabled:
            return
        data = msg.data
        twist = Twist()
        numOfFaces = data[1]
        #twist.linear.x = 4*data.axes[1]
        #twist.angular.z = 4*data.axes[0]

        if numOfFaces == 1:
            
            width = data[8]
            faceCenter_x = data[6] + data[8]/2
            faceCenter_y = data[7] + data[9]/2

            left_x = data[2]/3
            right_x = 2*data[2]/3

            if faceCenter_x < left_x:
                twist.linear.x = 0.05
                twist.angular.z = 3
                self.result_pub.publish(twist)
                
            elif faceCenter_x > right_x:
                twist.linear.x = 0.05
                twist.angular.z = -3
                self.result_pub.publish(twist)
            else:
                #self.result_pub.publish(twist)
                if (width < 70):
                    twist.linear.x = -0.1
                    self.result_pub.publish(twist)
                else:
                    self.result_pub.publish(twist)
                    self.enabled = False
                    msg = String()
                    msg.data = "photo"
                    self.trigger_pub.publish(msg)
 
        elif numOfFaces > 1:
            meanFaceCenter_x = 0.0
            leftMost = data[2]
            rightMost = 0 
            for i in range(numOfFaces):
                thisCenter = data[6 + 6*i] + data[8 + 6*i]/2.0       
                meanFaceCenter_x += thisCenter/numOfFaces
                if thisCenter > rightMost:
                    rightMost = thisCenter
                if thisCenter < leftMost:
                    leftMost = thisCenter
            
            left_x = data[2]/3
            right_x = 2*data[2]/3

            if meanFaceCenter_x < left_x:
                twist.linear.x = 0.05
                twist.angular.z = 3
                self.result_pub.publish(twist)
                
            elif meanFaceCenter_x > right_x:
                twist.linear.x = 0.05
                twist.angular.z = -3
                self.result_pub.publish(twist)

            else:
                if(leftMost > (data[2]/5) and rightMost < (4*data[2]/5)):
                    #cuvaj
                    twist.linear.x = -0.1
                else:
                    twist.linear.x = 0.0        
                self.result_pub.publish(twist)
                self.enabled = False
                msg = String()
                msg.data = "photo"
                self.trigger_pub.publish(msg)
 
        else:
            self.result_pub.publish(twist)
   
       
if __name__ == "__main__":
    _node = CenteringNode()
    rospy.spin()
