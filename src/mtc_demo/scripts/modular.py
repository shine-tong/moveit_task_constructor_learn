#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import time
import convert_msgs
from std_msgs.msg import Header
from geometry_msgs.msg import TwistStamped, Twist, Vector3Stamped, Vector3
from moveit.task_constructor import core, stages
from py_binding_tools import roscpp_init
from math import pi

def creat_module(group_name):
    serial_container = core.SerialContainer("Cartesian Path")   # 子任务名称
    cartesian = core.CartesianPath()
    cartesian.max_velocity_scaling_factor = 0.1  
    cartesian.max_acceleration_scaling_factor = 0.1  
    jointspace = core.JointInterpolationPlanner()
    
    move1 = stages.MoveRelative("x +0.2", cartesian)
    move1.group = group_name
    header1 = Header(frame_id="world")
    move1.setDirection(Vector3Stamped(header=header1, vector=Vector3(0.2, 0, 0)))
    serial_container.insert(move1)

    move2 = stages.MoveRelative("y -0.3", cartesian)
    move2.group = group_name
    header2 = Header(frame_id="world")
    move2.setDirection(Vector3Stamped(header=header2, vector=Vector3(0, -0.3, 0)))
    serial_container.insert(move2)
    
    move3 = stages.MoveRelative("rz +45°", cartesian)
    move3.group = group_name
    header3 = Header(frame_id="world")
    move3.setDirection(TwistStamped(header=header3, twist=Twist(angular=Vector3(0, 0, pi / 4.0))))
    serial_container.insert(move3)
    
    move4 = stages.MoveRelative("joint offset", cartesian)
    move4.group = group_name
    move4.setDirection(dict(joint1=pi / 18.0))
    serial_container.insert(move4)
    
    move5 = stages.MoveTo("moveTo home", jointspace)
    move5.group = group_name
    move5.setGoal("home")
    serial_container.insert(move5)
    
    return serial_container

def creat_task():
    task = core.Task()
    task.name = "cartesian" # 顶层任务名称
    task.add(stages.CurrentState("current state"))
    
    group = "manipulator"
    task.add(creat_module(group))
    task.add(creat_module(group))
    task.add(creat_module(group))
    
    return task

def main():
    rospy.init_node("moveit_mtc_cartesian_tutorial", anonymous=True)
    roscpp_init("mtc_tutorial")
    task = creat_task()
    
    if task.plan():
        solutions = task.solutions
        task.publish(task.solutions[0])
        task.execute(task.solutions[0])
        trajj = convert_msgs.Sol_to_JointTrajMsg(solutions)
    time.sleep(30)
    
if __name__ == "__main__":
    main()