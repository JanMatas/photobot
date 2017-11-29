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
        rospy.Subscriber("/faceCoord", Int32MultiArray, self.callback, queue_size=10)
      

        self.result_pub = rospy.Publisher("RosAria/cmd_vel", Twist, queue_size=10)
        #self.speech_pub = rospy.Publisher("speech_output", String, queue_size=10)
        #self.picture_pub = rospy.Publisher("take_picture", String, queue_size=10) 
        #self.request = None
        

    def callback(self, msg):
        
        data = msg.data
        twist = Twist()
        #twist.linear.x = 4*data.axes[1]
        #twist.angular.z = 4*data.axes[0]

        if data[1] == 1:
            
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

        else:
            self.result_pub.publish(twist)
        
       
    
        

        #rospy.logdebug("Waiting for response...")
        #rospy.logdebug("Got response, and publishing it.")

        
        
        
        
        
        
        
        #self.result_pub.publish(result_msg)
        
        #speech_object.data = response_string
        #self.speech_pub.publish(speech_object)
if __name__ == "__main__":
    _node = CenteringNode()
    rospy.spin()
