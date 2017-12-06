#!/usr/bin/env python
import rospy
import time
from std_msgs.msg import String
from geometry_msgs.msg import Twist

global star_time
global acc
acc = []
def callback(data):
    global acc
    acc.append(time.time() - start_time)
    print data.data
    if data.data == "exit":
        print sum(acc)/float(len(acc))
def callback2(data):
    global start_time
    start_time = time.time()
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber('/speech_input', String, callback)
    
    rospy.Subscriber('/cmd_vel', Twist, callback2)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
