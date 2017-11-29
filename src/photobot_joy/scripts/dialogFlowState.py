#!/usr/bin/env python
import rospy
import apiai
import json

from std_msgs.msg import String

# Authentication
SESSION_ID = ''


class DialogflowNode(object):
    """
    ROS node for the Dialogflow natural language understanding.
    Dialogflow (previously API.ai) parses natural language into a json string containing the semantics of the text. This
    nose wraps that into a ROS actionlib interface.
    """
    def __init__(self):
        rospy.init_node('dialogflow_node')
        rospy.Subscriber("speech_input", String, self.speech_callback, queue_size=10)
        rospy.Subscriber("event_trigger", String, self.event_callback, queue_size=10)

        try:
            self._client_access_token = "bc9d7b5841464fb8b17ce8ef89b11669" 
        except rospy.ROSException:
            rospy.logfatal("Missing required ROS parameter client_access_token")
            exit(1)

        self.ai = apiai.ApiAI(self._client_access_token)

        self.speech_pub = rospy.Publisher("speech_output", String, queue_size=10)
        self.event_pub = rospy.Publisher("event_output", String, queue_size=10) 
        self.request = None

    def publish_msgs_events(self, json_msg):

        parsed = json.loads(json_msg)
        response_string = parsed["result"]["fulfillment"]["speech"]
        print (response_string)
        action = parsed["result"]["action"]
        if (action == "take_photo"):
            trigger_msg = "Take photo, type: " + parsed["result"]["parameters"]["photo_type"]
            print (trigger_msg)
            event_object = String()
            event_object.data = trigger_msg
            self.event_pub.publish(event_object)
        if (action == "apply_filter"):
            trigger_msg = "Apply filter, type: " + parsed["result"]["parameters"]["filter_type"]
            print (trigger_msg)
            event_object = String()
            event_object.data = trigger_msg
            self.event_pub.publish(event_object)
        if (action == "send_email"):
            trigger_msg = "Send email " + parsed["result"]["parameters"]["email"]
            print (trigger_msg)
            event_object = String()
            event_object.data = trigger_msg
            self.event_pub.publish(event_object)

        speech_object = String()
        speech_object.data = response_string
        self.speech_pub.publish(speech_object) 


    def event_callback(self, msg):
        if (msg.data == "Face detected"):
            return
            self.request = self.ai.event_request(apiai.events.Event("face-trigger"))
            #self.request = self.ai.text_request()
            #self.request.query = "Hello"
        if (msg.data == "Photo taken"):
            self.request = self.ai.event_request(apiai.events.Event("photo-taken"))
        if (msg.data == "Email taken"):
            self.request = self.ai.event_request(apiai.events.Event("email-taken"))

        rospy.logdebug("Waiting for response...")
        response = self.request.getresponse()
        rospy.logdebug("Got response, and publishing it.")

        self.publish_msgs_events(response.read())



    def speech_callback(self, msg):
        self.request = self.ai.text_request()
        self.request.query = msg.data

        rospy.logdebug("Waiting for response...")
        response = self.request.getresponse()
        rospy.logdebug("Got response, and publishing it.")

        self.publish_msgs_events(response.read())
        
       
        #try :
        #     if 'wants-picture' in parsed["result"]["contexts"]["name"]:
        #          picture_object = String()
        #          picture_object.data = "trigger"
        #          self.picture_pub.publish(picture_object)
        #except:
        #     pass


if __name__ == "__main__":
    dialogflow_node = DialogflowNode()
    rospy.spin()
