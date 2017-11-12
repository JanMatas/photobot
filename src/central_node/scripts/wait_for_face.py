#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String, Int32MultiArray
from collections import deque

#pub = rospy.Publisher('face_detect', String, queue_size=10)
pub = rospy.Publisher('speech_input', String, queue_size=10)
# rospy.init_node('talker', anonymous=True)

buffer = deque(maxlen=100)
avg = 0
stop = False
def callback(data):
    global buffer
    global avg
    global stop
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    rospy.loginfo('There are faces: ' +  str(data.data[1]))
    rospy.loginfo('avg faces: ' +  str(avg))
    faces_n = data.data[1]
    buffer.append(faces_n)
    avg = float(sum(buffer))/len(buffer)
    if avg > 0.5 and stop == False:
        #pub.publish("Face detected")
        rospy.loginfo("Face detected")
        pub.publish("Hello")
        stop = True
#         avg = 0
#         buffer.clear()

    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listenCamera', anonymous=True)

    # TODO: proper type
    rospy.Subscriber("/faceCoord", Int32MultiArray, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
