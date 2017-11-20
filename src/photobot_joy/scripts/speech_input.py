#!/usr/bin/env python
# NOTE: this example requires PyAudio because it uses the Microphone class

import time
import rospy
import speech_recognition as sr
from std_msgs.msg import String, Bool
class SpeechInput(object):
    def __init__(self):
        rospy.init_node('speech_input_node')
        self.pub = rospy.Publisher("speech_input", String, queue_size=10)
        rospy.Subscriber("speech_input_enable", Bool, self.speech_input_enable)
        self.r = sr.Recognizer()
        self.m = sr.Microphone()
        rospy.logdebug("Calibrating...")
        with self.m as source:
            self.r.adjust_for_ambient_noise(source)
        rospy.logdebug("Done calibration")
        
        self.stop_listening = self.r.listen_in_background(self.m, self.callback)

    def speech_input_enable(self, data):
        if data.data and not self.stop_listening:
            self.stop_listening = self.r.listen_in_background(self.m, self.callback)
	elif not data.data and self.stop_listening:
	    self.stop_listening()
            self.stop_listening = None

    def callback(self,recognizer, audio):
        print "audio in"
        try:
            recognized_speech =  recognizer.recognize_google(audio)
            speech_input_msg = String()
            speech_input_msg.data = recognized_speech
            self.pub.publish(speech_input_msg)

        except sr.UnknownValueError:
            print("Google Speech Recognition could not understand audio")
        except sr.RequestError as e:
            print("Could not request results from Google Speech Recognition service; {0}".format(e))
    
    def __del__(self):
        #TODO Make sure this is ok!
        self.stop_listening()
if __name__ == "__main__":
    sr_node = SpeechInput()
    rospy.spin()
