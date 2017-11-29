#!/usr/bin/env python
import rospy
import apiai

from std_msgs.msg import Int32MultiArray
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist, Vector3


class FilterFacesNode(object):
    """
    ROS node for the Dialogflow natural language understanding.
    Dialogflow (previously API.ai) parses natural language into a json string containing the semantics of the text. This
    nose wraps that into a ROS actionlib interface.
    """
    def __init__(self):
        rospy.init_node('facefilter_node')
        rospy.Subscriber("/faceCoord", Int32MultiArray, self.callback, queue_size=10)
        self.facePresentCounter = 0
        self.SentTrigger = False

        self.faceCoord_pub = rospy.Publisher("/filter_faceCoord", Int32MultiArray, queue_size=10)
        self.facePresent_pub = rospy.Publisher("/event_trigger", Bool, queue_size=10)
        #self.speech_pub = rospy.Publisher("speech_output", String, queue_size=10)
        #self.picture_pub = rospy.Publisher("take_picture", String, queue_size=10) 
        #self.request = None
        

    def callback(self, msg):
        
        data = msg.data
        numOfFaces = data[1]
        frameRate = data[0]
        pictureWidth = data[2]
        pictureHeight = data[3]

        filteredData = []
        filteredData.append(data[0]) #frame rate
        filteredData.append(0) #number of faces
        filteredData.append(data[2]) #width
        filteredData.append(data[3]) #height
        
        faceCounter = 0
        for i in range(numOfFaces):
            
            if data[i*6 + 5] > frameRate/2 and data[i*6 + 8] > pictureWidth/15 and data[i*6 + 9] > pictureHeight/15:

                filteredData.append(data[6*i + 4]) #face index
                filteredData.append(data[6*i + 5]) #face duration
                filteredData.append(data[6*i + 6]) #face corner x
                filteredData.append(data[6*i + 7]) #face corner y
                filteredData.append(data[6*i + 8]) #face width
                filteredData.append(data[6*i + 9]) #face height
                
                faceCounter += 1
        
        
        #print filteredData
        filteredData[1] = faceCounter
        dataForPublish = Int32MultiArray(data=filteredData)
        self.faceCoord_pub.publish(dataForPublish)
        
        if not(self.SentTrigger) and faceCounter > 0:
            self.facePresentCounter += 1

        if self.SentTrigger and faceCounter == 0 :
            self. facePresentCounter += 1

        if self.facePresentCounter  > 2*frameRate and not(self.SentTrigger):
            self.facePresent_pub.publish(True)
            self.SentTrigger = True
            self.facePresentCounter = 0

        if self.facePresentCounter > 2*frameRate and self.SentTrigger :
            self.facePresent_pub.publish(False)
            self.SentTrigger = False
            self.facePresentCounter = 0
        
        
        #speech_object.data = response_string
        #self.speech_pub.publish(speech_object)
if __name__ == "__main__":
    _node = FilterFacesNode()
    rospy.spin()
