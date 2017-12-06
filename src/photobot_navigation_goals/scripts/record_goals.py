#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped

clicked_points = []
fname = 'clicked_points.txt'


def callback(data):
    global clicked_point
    #  import pdb; pdb.set_trace();
    x = data.point.x
    y = data.point.y
    data_point = "{} {}\n".format(x, y)
    rospy.loginfo("Got x: %f, y: %f", x, y)
    clicked_points.append(data_point)
    with open(fname, 'w') as f:
        f.writelines(clicked_points)

    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('navigation_goals_recorder', anonymous=True)

    #  rospy.Subscriber("/clicked_point", String, callback)
    rospy.Subscriber("/clicked_point", PointStamped, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
