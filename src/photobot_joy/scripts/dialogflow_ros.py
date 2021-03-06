#!/usr/bin/env python
import rospy
import apiai

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
        try:
            self._client_access_token = "bc9d7b5841464fb8b17ce8ef89b11669" 
        except rospy.ROSException:
            rospy.logfatal("Missing required ROS parameter client_access_token")
            exit(1)

        self.ai = apiai.ApiAI(self._client_access_token)

        self.result_pub = rospy.Publisher("command", String, queue_size=10)
        self.speech_pub = rospy.Publisher("speech_output", String, queue_size=10)
        self.picture_pub = rospy.Publisher("take_picture", String, queue_size=10)
        self.sub_hints = rospy.Publisher("speech_input_hints", String, queue_size=10)
        self.request = None
        self.hint_map = {'greeting-followup':'yes,no', 'greeting-yes-followup':'selfie,group photo',
                        'greeting-yes-photo-now-photo-taken-followup':'it’s great,I love it',
                        'greeting-yes-photo-now-photo-taken-yes-followup':'yes,no',
                        'greeting-yes-photo-now-photo-taken-no-followup':'yes,no'}

    def speech_callback(self, msg):
        self.request = self.ai.text_request()
        self.request.query = msg.data

        rospy.logdebug("Waiting for response...")
        response = self.request.getresponse()
        rospy.logdebug("Got response, and publishing it.")

        result_msg = String()
        response_string = response.read()
        result_msg.data = response_string
        import json
        parsed = json.loads(response_string)
        response_string = parsed["result"]["fulfillment"]["speech"] 
        try :
            if 'wants-picture' in map(lambda x: x["name"], parsed["result"]["contexts"]):
                  picture_object = String()
                  picture_object.data = "trigger"
                  self.picture_pub.publish(picture_object)
        except:
             pass
        try :
            for context_hint in self.hint_map.keys():
                if context_hint in map(lambda x: x["name"], parsed["result"]["contexts"]):
                      hint_object = String()
                      hint_object.data = self.hint_map[context_hint]
                      self.sub_hints.publish(hint_object)
        except:
             pass
        print (response_string)
        self.result_pub.publish(result_msg)
        speech_object = String()
        speech_object.data = response_string
        self.speech_pub.publish(speech_object)
if __name__ == "__main__":
    dialogflow_node = DialogflowNode()
    rospy.spin()
