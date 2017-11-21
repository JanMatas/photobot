#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String, Int32MultiArray
from collections import deque

pub = rospy.Publisher('event_trigger', String, queue_size=10)
# rospy.init_node('talker', anonymous=True)

buffer = deque(maxlen=100)
avg = 0
def callback(data):
    global buffer
    global avg
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    rospy.loginfo('There are faces: ' +  str(data.data[1]))
    rospy.loginfo('avg faces: ' +  str(avg))
    faces_n = data.data[1]
    buffer.append(faces_n)
    avg = sum(buffer)/len(buffer)
    if avg > 0.5:
        pub.publish("Face detected")
        exit(0)
#         avg = 0
#         buffer.clear()

    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    # TODO: proper type
    rospy.Subscriber("/faceCoord", Int32MultiArray, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


# talker
def talker():
    print "ahoj"
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
