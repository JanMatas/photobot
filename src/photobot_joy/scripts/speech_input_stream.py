#!/usr/bin/env python


# [START import_libraries]
from __future__ import division
import time
import collections

import re
import sys
import rospy
from google.cloud import speech
from google.cloud.speech import enums
from google.cloud.speech import types
import pyaudio
import threading
import math
import struct
from six.moves import queue
# [END import_libraries]

# Audio recording parameters
RATE = 16000
CHUNK = int(RATE / 10)  # 100ms
TAIL = 30

class MicrophoneStream(object):
    def __init__(self, rate, chunk, queue):
        self._rate = rate
        self._chunk = chunk
        self._buff = queue
        self.closed = True
        self.tail = TAIL


    def __enter__(self):
        self._audio_interface = pyaudio.PyAudio()
        self._audio_stream = self._audio_interface.open(
            format=pyaudio.paInt16,
            # The API currently only supports 1-channel (mono) audio
            # https://goo.gl/z757pE
            channels=1, rate=self._rate,
            input=True, frames_per_buffer=self._chunk,
            # Run the audio stream asynchronously to fill the buffer object.
            # This is necessary so that the input device's buffer doesn't
            # overflow while the calling thread makes network requests, etc.
            stream_callback=self._fill_buffer,
        )

        self.closed = False

        return self

    def __exit__(self, type, value, traceback):
        self._audio_stream.stop_stream()
        self._audio_stream.close()
        self.closed = True
        # Signal the generator to terminate so that the client's
        # streaming_recognize method will not block the process termination.
        self._buff.put(None)
        self._audio_interface.terminate()

    def _fill_buffer(self, in_data, frame_count, time_info, status_flags):
        """Continuously collect data from the audio stream, into the buffer."""
        self._buff.put(in_data)
        return None, pyaudio.paContinue

# [END audio_stream]



class SpeechRecognizer(object):

    def __init__(self):
        self.activated = False
        self.stop_listening = False
        rospy.init_node('speech_input_node', disable_signals=True)
        self.pub = rospy.Publisher("speech_input", String, queue_size=10)
        self.sub = rospy.Subscriber("speech_input_enable", Bool, self.speech_input_enable)
        self.sub_hints = rospy.Subscriber("speech_input_hints", String, self.speech_input_hints)
        self.hints = []

        self.client = speech.SpeechClient()

        self.streaming_config = types.StreamingRecognitionConfig(
            config=config,
            interim_results=False
        )
        self.thread_handle = None

    def rms(self, data):
        if not data:
            return 0
        count = len(data)/2
        format = "%dh"%(count)
        shorts = struct.unpack( format, data )
        sum_squares = 0.0
        for sample in shorts:
            n = sample * (1.0/32768)
            sum_squares += n*n
        return math.sqrt( sum_squares / count )
#
    def chunk_generator(self, window, q, init_time):
        tail = TAIL
        data = list(window)
        while True:
            chunk = q.get()
            if chunk is None:
                return
            
            data.append(chunk)
            # Now consume whatever other data's still buffered.
            while True:
                try:
                    chunk = q.get(block=False)
                    if chunk is None:
                        return
                    data.append(chunk)
                except queue.Empty:
                    break
            if sum(map(self.rms, data)) / len(data) >  0.01:
               tail = TAIL
            
            if tail == 0:
               time_since_init = time.time() - init_time
               if time_since_init > 15:
                   print "No voice detected, closing unused stream."
                   return
               else:
                   print "No voice detected but stream has not been used for 15 secs so there is no savings in closing it."
            if self.stop_listening: 
                return
            tail = max(0, tail - len(data))
            ret = b''.join(data)
            data = []
            yield ret

    def start_thread(self):
        q = queue.Queue()

  
        window = collections.deque(maxlen=10)
        self.activated = True
        with MicrophoneStream(RATE, CHUNK, q) as stream:
            while self.activated:   
                 window.append(q.get())
                 if self.stop_listening:
                     print "not listening"
                     window = collections.deque(maxlen=10)
                     continue
                 print "rms " + str(self.rms(window[-1]))
                 if self.rms(window[-1]) > 0.01:
                      print "Sound detected, opening stream."
                      audio_generator = self.chunk_generator(window, q, time.time())
                      window = collections.deque(maxlen=10)
                      requests = (types.StreamingRecognizeRequest(audio_content=content)
                            for content in audio_generator)
                      config = types.RecognitionConfig(
                          encoding=enums.RecognitionConfig.AudioEncoding.LINEAR16,
                          sample_rate_hertz=RATE,
                          language_code='en-us',
                          speech_contexts=[speech.types.SpeechContext(
                            phrases=self.hints,
                          )],
                      )
                      response_generator = self.client.streaming_recognize(self.streaming_config, requests)
                      self.response_loop(response_generator)
                      continue
                 print "silent chunk"
    

    def speech_input_hints(self, data):
         self.hints =  str(data.data).split(',')      
        
    def speech_input_enable(self, data):
         print "listening is: "+ str(data.data)           
         self.stop_listening = not data.data           
    def stop(self):
        print "Waiting for thread to finish"

        self.activated = False 
        self.thread_handle.join()

        print "Thread finished"



    def response_loop(self, responses):
       for response in responses:
           print "response"           
           if not self.activated:
               return
           if not response.results:
               continue

           result = response.results[0]
           if not result.alternatives:
               continue

           transcript = result.alternatives[0].transcript

           speech_input_msg = String()
           speech_input_msg.data = transcript 
           self.pub.publish(speech_input_msg)

    def start_waiting(self):
       while True:
           print "loop"
           if self.pub.get_num_connections() == 0 and self.activated:
               print "No subscribers, stopping!"
               self.stop()
           if self.pub.get_num_connections() > 0 and not self.thread_handle:
               self.thread_handle = threading.Thread(name='daemon', target=self.start_thread)
               self.thread_handle.setDaemon(True)
               self.activated = True
               self.thread_handle.start()             
               print "Subscriber connected, started thread"
           time.sleep(1)

import rospy
from std_msgs.msg import String, Bool





if __name__ == '__main__':
   
    sr = SpeechRecognizer()
    sr.start_waiting()
