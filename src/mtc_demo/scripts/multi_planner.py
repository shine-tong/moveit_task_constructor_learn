#! /usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import rospy
from moveit.task_constructor import core, stages
from py_binding_tools import roscpp_init

def multi_planners():
    ompl_planner = core.PipelinePlanner("ompl")
    ompl_planner.planner = "RRTConnectkConfigDefault"
    pilz_planner = core.PipelinePlanner("pilz_industrial_motion_planner")
    pilz_planner.planner = "PTP"
    multi_planner = core.MultiPlanner()
    multi_planner.add(ompl_planner, pilz_planner)
    
    task = core.Task()
    task.name = "multi_planner"
    task.add(stages.CurrentState("current_state"))
    
    alternatives = core.Alternatives("Alternatives")
    
    # goal 1
    goalConfig1 = {
        "panda_joint1": 1.0,
        "panda_joint2": -1.0,
        "panda_joint3": 0.0,
        "panda_joint4": -2.5,
        "panda_joint5": 1.0,
        "panda_joint6": 1.0,
        "panda_joint7": 1.0,
    }
    # goal 2
    goalConfig2 = {
        "panda_joint1": -3.0,
        "panda_joint2": -1.0,
        "panda_joint3": 0.0,
        "panda_joint4": -2.0,
        "panda_joint5": 1.0,
        "panda_joint6": 2.0,
        "panda_joint7": 0.5,
    }
    # First motion plan to compare
    moveTo1 = stages.MoveTo("Move To Goal Configuration 1", multi_planner)
    moveTo1.group = "panda_arm"
    moveTo1.setGoal(goalConfig1)
    alternatives.insert(moveTo1)

    # Second motion plan to compare
    moveTo2 = stages.MoveTo("Move To Goal Configuration 2", multi_planner)
    moveTo2.group = "panda_arm"
    moveTo2.setGoal(goalConfig2)
    alternatives.insert(moveTo2)

    # Add the alternatives stage to the task hierarchy
    task.add(alternatives)
    if task.plan():
        task.publish(task.solutions[0])
        task.execute(task.solutions[0])
    time.sleep(1)

def main():
    rospy.init_node("moveit_mtc_tutorial", anonymous=True)
    roscpp_init("mtc_tutorial")
    multi_planners()
    
if __name__ == "__main__":
    main()