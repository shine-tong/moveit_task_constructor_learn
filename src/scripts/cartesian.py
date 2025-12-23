#! /usr/bin/env python3
# -*- coding: utf-8 -*-
import time
import rospy
import convert_msgs
from std_msgs.msg import Header
from geometry_msgs.msg import TwistStamped, Twist, Vector3Stamped, Vector3
from moveit.task_constructor import core, stages
from math import pi
from py_binding_tools import roscpp_init

def mtc_test():
    # moveit_commander.roscpp_initialize(args=[])
    roscpp_init("mtc_tutorial")
    
    # [cartesianTut1]
    group = "manipulator"
    # [cartesianTut1]

    # [cartesianTut2]
    # Cartesian and joint-space interpolation planners
    cartesian = core.CartesianPath()
    # rospy.loginfo(dir(cartesian))
    cartesian.max_velocity_scaling_factor = 0.1  # m/s
    cartesian.max_acceleration_scaling_factor = 0.1  # m/s²
    jointspace = core.JointInterpolationPlanner()
    # [cartesianTut2]

    # [cartesianTut3]
    task = core.Task()
    task.name = "cartesian"

    # start from current robot state
    task.add(stages.CurrentState("current state"))
    # [cartesianTut3]

    # [initAndConfigMoveRelative]
    # move along x
    move = stages.MoveRelative("x +0.2", cartesian)
    move.group = group
    header = Header(frame_id="world")
    move.setDirection(Vector3Stamped(header=header, vector=Vector3(0.2, 0, 0)))
    task.add(move)
    # [initAndConfigMoveRelative]

    # [cartesianTut4]
    # move along y
    move = stages.MoveRelative("y -0.3", cartesian)
    move.group = group
    move.setDirection(Vector3Stamped(header=header, vector=Vector3(0, -0.3, 0)))
    task.add(move)
    # [cartesianTut4]

    # [cartesianTut5]
    # rotate about z
    move = stages.MoveRelative("rz +45°", cartesian)
    move.group = group
    move.setDirection(TwistStamped(header=header, twist=Twist(angular=Vector3(0, 0, pi / 4.0))))
    task.add(move)
    # [cartesianTut5]

    # [cartesianTut6]
    # Cartesian motion, defined as joint-space offset
    move = stages.MoveRelative("joint offset", cartesian)
    move.group = group
    move.setDirection(dict(joint1=pi / 18.0))
    task.add(move)
    # [cartesianTut6]

    # [initAndConfigMoveTo]
    # moveTo named posture, using joint-space interplation
    move = stages.MoveTo("moveTo home", jointspace)
    move.group = group
    move.setGoal("home")
    task.add(move)
    # [initAndConfigMoveTo]

    if task.plan():
        solutions = task.solutions
        rospy.loginfo("Found {} solution(s).".format(len(solutions)))
        for i, sol in enumerate(solutions):
            rospy.loginfo("Solution {}: cost {}".format(i, sol.cost))
        trajj = convert_msgs.Sol_to_JointTrajMsg(solutions)
        # rospy.loginfo(type(trajj))
        # rospy.loginfo(trajj)
        task.publish(task.solutions[0])
        task.execute(task.solutions[0])
    time.sleep(50)

def main():
    rospy.init_node("moveit_mtc_cartesian_tutorial", anonymous=True)
    mtc_test()
    
if __name__ == "__main__":
    main()