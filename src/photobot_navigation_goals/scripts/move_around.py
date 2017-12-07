#!/usr/bin/env python

from time import sleep

import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import *

fname = "clicked_points.txt"
face_detected_topic = "/face_detected"

face_detected = False


def load_goals():
    def to_xy(s):
        s_ = s.split(' ')
        return (float(s_[0]), float(s_[1]))

    with open(fname) as f:
        points = f.readlines()
        return [to_xy(p) for p in points]


def make_xy_goal(point):
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = 'base_link'
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = point[0]
    goal.target_pose.pose.position.y = point[1]
    goal.target_pose.pose.orientation.w = 1.0
    return goal


#  def face_detected_callback(data):
def face_detected_callback(data, args):
    global face_detected
    move_base = args[0]
    if data.data == "Face detected":
        face_detected = True
        move_base.cancel_goal()
        rospy.loginfo("Cancelling current goal")


def start_node():
    rospy.init_node('wander_around_node', anonymous=False)

    move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    rospy.Subscriber(face_detected_topic, String,
                     face_detected_callback, (move_base))
    rospy.on_shutdown(shutdown)

    # Tell the action client that we want to spin a thread by default
    rospy.loginfo("wait for the action server to come up")

    move_base.wait_for_server(rospy.Duration(5))
    return move_base


def goto_goal(move_base, point):
    timeout = 60
    goal = make_xy_goal(point)
    move_base.send_goal(goal)
    #  start_time = time()
    # Allow robot up to `timeout` seconds to complete the task
    success = move_base.wait_for_result(rospy.Duration(timeout))
    #  while move_base.get_state() != GoalStatus.SUCCEEDED \
    #   and time()-start_time < timeout:
    #      if face_detected:
            #  move_base.cancel_goal()
            #  rospy.loginfo("Face detected, cancelling goal")
    #  success = move_base.get_state() == GoalStatus.SUCCEEDED
    if not success:
        move_base.cancel_goal()
        rospy.loginfo("The base failed to reach goal in %d seconds", timeout)
    else:
        state = move_base.get_state()
        if state == GoalStatus.SUCCEEDED:
            rospy.loginfo("Moved to %f %f", point[0], point[1])


def shutdown(self):
    rospy.loginfo("Stop")


def move_around():
    global face_detected
    #  goals = [(3, 0)]
    goals = load_goals()
    move_base = start_node()
    for point in goals:
        if not face_detected:
            goto_goal(move_base, point)


if __name__ == '__main__':
    try:
        move_around()
    except rospy.ROSInterruptException:
        rospy.loginfo("Exception thrown")
