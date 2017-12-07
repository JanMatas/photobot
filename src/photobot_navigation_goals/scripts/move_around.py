#!/usr/bin/env python

from time import sleep

import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import Bool
import actionlib
from actionlib_msgs.msg import *

fname = "clicked_points.txt"
face_detected_topic = "/face_detected"


def shutdown():
    rospy.loginfo("Stop")


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


class MoveAround:
    # This method has a bit weird signature because it's a callback

    def __init__(self):
        self.face_detected = False

    @staticmethod
    def face_detected_callback(data, self):
        if data.data == True:
            self.face_detected = True
            self.move_base.cancel_goal()
            rospy.loginfo("Face detected: cancelling current goal")


    def start_node(self):
        rospy.init_node('wander_around_node', anonymous=False)

        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.Subscriber(face_detected_topic, Bool,
                         self.face_detected_callback, self)
        rospy.on_shutdown(shutdown)

        # Tell the action client that we want to spin a thread by default
        rospy.loginfo("wait for the action server to come up")
        self.move_base.wait_for_server(rospy.Duration(5))

    def goto_goal(self, point):
        #  global face_detected
        timeout = 60
        dt = 3
        elapsed = 0
        goal = make_xy_goal(point)
        self.move_base.send_goal(goal)
        rospy.loginfo("Goal sent!")
        # Allow robot up to `timeout` seconds to complete the task
        success = False
        while not self.face_detected and elapsed < timeout:
            success = self.move_base.wait_for_result(rospy.Duration(dt))
            elapsed += dt

        rospy.loginfo("Stopped waiting for goal")
        if not success and not self.face_detected:
            self.move_base.cancel_goal()
            rospy.loginfo("The base failed to reach goal in %d seconds", timeout)
        else:
            state = self.move_base.get_state()
            if state == GoalStatus.SUCCEEDED:
                rospy.loginfo("Moved to %f %f", point[0], point[1])


def move_around():
    goals = load_goals()
    move = MoveAround()
    move.start_node()
    for point in goals:
        if not move.face_detected:
            move.goto_goal(point)
        else:
            rospy.loginfo("Ignoring goal")


if __name__ == '__main__':
    try:
        move_around()
    except rospy.ROSInterruptException:
        rospy.loginfo("Exception thrown")
