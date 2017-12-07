#!/usr/bin/env python

'''
Copyright (c) 2015, Mark Silliman
All rights reserved.
Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
'''

#Code is inspired by http://wiki.ros.org/navigation/Tutorials/SendingSimpleGoals (written in C++).
#TurtleBot must have minimal.launch & amcl_demo.launch running prior to starting this script.

import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import *

fname = 'clicked_points.txt'


def load_goals():
    def to_xy(s):
        s_ = s.split(' ')
        return (float(s_[0]), float(s_[1]))

    with open(fname) as f:
        points = f.readlines()
        import pdb; pdb.set_trace()
        return [to_xy(p) for p in points]


def make_xy_goal(point):
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = 'base_link'
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = point[0]
    goal.target_pose.pose.position.y = point[1]
    goal.target_pose.pose.orientation.w = 1.0 #go forward
    return goal


def start_node():
    rospy.init_node('wander_around_node', anonymous=False)

    #what to do if shut down (e.g. ctrl + C or failure)
    rospy.on_shutdown(shutdown)
    #tell the action client that we want to spin a thread by default
    move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    rospy.loginfo("wait for the action server to come up")
    #allow up to 5 seconds for the action server to come up
    move_base.wait_for_server(rospy.Duration(5))
    return move_base


def goto_goal(move_base, point):
    timeout = 20
    goal = make_xy_goal(point)
    #start moving
    move_base.send_goal(goal)
    #allow robot up to `timeout` seconds to complete the task
    success = move_base.wait_for_result(rospy.Duration(timeout)) 
    if not success:
        move_base.cancel_goal()
        rospy.loginfo("The base failed to reach goal in %d seconds", timeout)
    else:
        # We made it!
        state = move_base.get_state()
        if state == GoalStatus.SUCCEEDED:
            rospy.loginfo("Hooray, the base moved to %f %f", point[0], point[1])


def shutdown(self):
    rospy.loginfo("Stop")


def move_around():
    #  goals = [(3, 0)]
    goals = load_goals()
    move_base = start_node()
    for point in goals:
        goto_goal(move_base, point)


if __name__ == '__main__':
    try:
        move_around()
    except rospy.ROSInterruptException:
        rospy.loginfo("Exception thrown")
