#!/usr/bin/env python
from gtts import gTTS
import os
import uuid
from std_msgs.msg import String
import rospy
from playsound import playsound
def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "Synthetising %s", data.data)
    tts = gTTS(text=data.data, lang='en-uk')
    id = uuid.uuid4()
    filename = '/tmp/photobot_temp_%s.mp3' % (id) 
    tts.save(filename)
    try:
        playsound(filename)
    finally:
        os.remove(filename)

def speech_main():
    rospy.init_node('speech_output', anonymous=True)
    rospy.Subscriber("speech_output", String, callback)
    rospy.spin()
if __name__ == '__main__':
    speech_main()
