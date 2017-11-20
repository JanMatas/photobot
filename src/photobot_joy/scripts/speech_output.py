#!/usr/bin/env python
from gtts import gTTS
import os
import uuid
from std_msgs.msg import Bool, String
import rospy
from playsound import playsound

class SpeechOutput(object):
    def __init__(self):
        rospy.init_node('speech_output', anonymous=True)
        rospy.Subscriber("speech_output", String, self.callback)
        self.pub = rospy.Publisher("speech_input_enable", Bool, queue_size=10)


    def callback(self, data):
        rospy.loginfo(rospy.get_caller_id() + "Synthetising %s", data.data)
        tts = gTTS(text=data.data, lang='en-uk')
        id = uuid.uuid4()
        filename = '/tmp/photobot_temp_%s.mp3' % (id) 
        tts.save(filename)
        try:
            msg = Bool()
            msg.data = False
            self.pub.publish(msg)
            playsound(filename)
            msg.data = True
            self.pub.publish(msg)
        finally:
            os.remove(filename)

if __name__ == '__main__':
    speech_output = SpeechOutput()
    rospy.spin()
