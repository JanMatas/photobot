#!/usr/bin/env python

from time import sleep
from itertools import cycle

import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import Bool
import actionlib
from actionlib_msgs.msg import *

fname = "clicked_points.txt"
face_detected_topic = "/face_detected"
interaction_complete_topic = "/interaction_complete"



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
    def __init__(self):
        self.want_to_stop = False
        self.killed = False

    # This method has a bit weird signature because it's a callback
    @staticmethod
    def face_detected_callback(data, self):
        if data.data == True:
            self.want_to_stop = True
            self.move_base.cancel_goal()
            rospy.loginfo("Face detected: cancelling current goal")

    @staticmethod
    def interaction_complete_callback(data, self):
        if data.data == True:
            self.want_to_stop = False
            rospy.loginfo("Resuming motion")

    def start_node(self):
        rospy.init_node('move_around_node', anonymous=False)

        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.Subscriber(face_detected_topic, Bool,
                         self.face_detected_callback, self)
        rospy.Subscriber(interaction_complete_topic, Bool,
                         self.interaction_complete_callback, self)
        rospy.on_shutdown(self.shutdown)

        # Tell the action client that we want to spin a thread by default
        rospy.loginfo("Waiting for move_base")
        self.move_base.wait_for_server(rospy.Duration(5))
        rospy.loginfo("Done")

    def goto_goal(self, point):
        timeout = 60
        dt = 0.1
        elapsed = 0
        goal = make_xy_goal(point)
        self.move_base.send_goal(goal)
        rospy.loginfo("Goal sent!")
        # Allow robot up to `timeout` seconds to complete the task
        success = False
        while not self.want_to_stop and elapsed < timeout:
            success = self.move_base.wait_for_result(rospy.Duration(dt))
            elapsed += dt

        rospy.loginfo("Stopped waiting for goal")
        if not success and not self.want_to_stop:
            self.move_base.cancel_goal()
            rospy.loginfo("The base failed to reach goal in %d seconds", timeout)
        else:
            state = self.move_base.get_state()
            if state == GoalStatus.SUCCEEDED:
                rospy.loginfo("Moved to %f %f", point[0], point[1])

    def loop(self, goals):
        for point in goals:
            while self.want_to_stop:
                sleep(0.1)
            if self.killed:
                return
            self.goto_goal(point)

    def shutdown(self):
        rospy.loginfo("Stop")
        self.killed = True


def move_around():
    goals = cycle(load_goals())
    move = MoveAround()
    move.start_node()
    move.loop(goals)


if __name__ == '__main__':
    try:
        move_around()
    except rospy.ROSInterruptException:
        rospy.loginfo("Exception thrown")
