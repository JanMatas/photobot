#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped

clicked_points = []
fname = 'clicked_points.txt'

px = 0.0
py = 0.0

tolerance = 0.1
initialized = False

def callback(data):
    """
    Save points clicked on the map.
    Points are saved as (dx, dy) from last point.
    Initial point is assumed to be (0, 0).
    The accumulated clicked points are dumped to file on every callback.
    """
    global clicked_point
    global px
    global py
    global initialized

    x = data.point.x
    y = data.point.y
    dx = x - px
    dy = y - py

    # Make sure subsequent points make sense
    # First point must be the robot itself
    if not initialized:
        if dx > tolerance or dy > tolerance:
            rospy.loginfo("Ignoring point (%.2f, %.2f), too far away from O")
        else:
            initialized = True
        return

    px = x
    py = y

    data_point = "{} {}\n".format(dx, dy)
    rospy.loginfo("Got x: %f, y: %f", x, y)
    rospy.loginfo("Saving as dx: %f, dy: %f", dx, dy)
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
