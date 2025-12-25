#! /usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Conversion functions between MTC core types and ROS message types
"""

def Sol_to_SolMsg(solution):
    """
    Convert moveit.task_constructor.core.Solution to 
    moveit_task_constructor_msgs.msg._Solution.Solution
    """
    return solution[0].toMsg()

def SolMsg_to_SubTrajMsg(sol_msg):
    """
    Convert moveit_task_constructor_msgs.msg._Solution.Solution to 
    moveit_task_constructor_msgs.msg._SubTrajectory.SubTrajectory
    """
    return sol_msg.sub_trajectory[2]

def SubTrajMsg_to_RobotTrajMsg(subtraj_msg):
    """
    Convert moveit_task_constructor_msgs.msg._SubTrajectory.SubTrajectory to 
    moveit_msgs.msg._RobotTrajectory.RobotTrajectory
    """
    return subtraj_msg.trajectory

def SubTrajMsg_to_JointTrajMsg(subtraj_msg):
    """
    Convert moveit_task_constructor_msgs.msg._SubTrajectory.SubTrajectory to 
    trajectory_msgs.msg._JointTrajectory.JointTrajectory
    """
    return subtraj_msg.trajectory.joint_trajectory

def Sol_to_JointTrajMsg(solution):
    """
    Convert moveit.task_constructor.core.Solution to 
    trajectory_msgs.msg._JointTrajectory.JointTrajectory
    """
    sol_msg = Sol_to_SolMsg(solution)
    subtraj_msg = SolMsg_to_SubTrajMsg(sol_msg)
    joint_traj_msg = SubTrajMsg_to_JointTrajMsg(subtraj_msg)
    return joint_traj_msg