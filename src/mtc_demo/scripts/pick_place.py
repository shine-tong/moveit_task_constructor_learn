#! /usr/bin/env python3
# -*- coding: utf-8 -*-
import time
import math
import rospy
import convert_msgs
from geometry_msgs.msg import PoseStamped, TwistStamped
from moveit_msgs.msg import Constraints, OrientationConstraint
from moveit_commander import PlanningSceneInterface
from moveit.task_constructor import core, stages
from py_binding_tools import roscpp_init

def crea_object(name: str, size: list):
    # 清除场景中的旧对象
    psi = PlanningSceneInterface(synchronous=True)
    psi.remove_world_object()
    
    # 定义抓取对象的属性
    object_pose = PoseStamped()
    object_pose.header.frame_id = "world"
    object_pose.pose.position.x = 0.30702
    object_pose.pose.position.y = 0.0
    object_pose.pose.position.z = 0.285
    object_pose.pose.orientation.w = 1.0
    
    # 将对象添加到场景中
    psi.add_box(name, object_pose, size)
    
def pick_and_place():
    arm = "panda_arm"
    eef = "hand"
    jointspace = core.JointInterpolationPlanner()
    object_name = "object"
    object_size = [0.1, 0.05, 0.03]
    crea_object(object_name, object_size)
    
    task = core.Task()
    task.name = "pick_and_place"
    
    task.add(stages.CurrentState("current_state"))
    
    # 创建规划器
    piperline = core.PipelinePlanner()
    piperline.planner = "RRTConnectkConfigDefault"
    piperline.num_planning_attempts = 3
    piperline.goal_position_tolerance = 0.01
    piperline.goal_orientation_tolerance = 0.01
    piperline.goal_joint_tolerance = 0.01
    piperline.max_velocity_scaling_factor = 1.0
    piperline.max_acceleration_scaling_factor = 1.0
    planners = [(arm, piperline)]
    
    # 连接到指定的规划组和规划器
    task.add(stages.Connect("connect", planners))
    
    # 创建抓握生成器
    grasp_generator = stages.GenerateGraspPose("Generate Grasp Pose")
    grasp_generator.angle_delta = math.pi / 2   # 在辐射中采样物体周围位置的角度步进距离
    grasp_generator.pregrasp = "open"   # 预握姿式为 open
    grasp_generator.grasp = "close"     # 抓握姿式为 close
    grasp_generator.setMonitoredStage(task["current_state"])  # 生成所有可能的抓握姿式
    
    # 计算抓握时手臂姿势和末端的 IK
    simple_grasp = stages.SimpleGrasp(grasp_generator, "Simple Grasp")
    # 将 IK 计算的框架设置在手指之间的中心
    ik_frame = PoseStamped()
    ik_frame.header.frame_id = "panda_hand"
    ik_frame.pose.position.z = 0.1034   # 手指之间的中心位置
    ik_frame.pose.orientation.x = 1.0   # 从上方抓握
    simple_grasp.setIKFrame(ik_frame)
    
    # 创建 Pick 容器，实现接近、抓取和举起物体的功能
    pick = stages.Pick(simple_grasp, "Pick")
    pick.eef = eef
    pick.object = object_name
    
    # 扭转并接近物体
    approach = TwistStamped()
    approach.header.frame_id = "world"
    approach.twist.linear.z = -1.0
    pick.setApproachMotion(approach, 0.03, 0.1)
    
    # 扭转并抬起物体
    lift = TwistStamped()
    lift.header.frame_id = "panda_hand"
    lift.twist.linear.z = -1.0
    pick.setLiftMotion(lift, 0.03, 0.1)
    
    task.add(pick)
    
    # 定义方向约束保持物体竖直
    oc = OrientationConstraint()
    oc.parameterization = OrientationConstraint.ROTATION_VECTOR
    oc.header.frame_id = "world"
    oc.link_name = "panda_hand"
    oc.orientation.w = 1.0
    oc.absolute_x_axis_tolerance = 0.5
    oc.absolute_y_axis_tolerance = 0.5
    oc.absolute_z_axis_tolerance = math.pi
    oc.weight = 1.0  # 硬约束
    
    constraint = Constraints()
    constraint.name = "panda_hand:upright"
    constraint.orientation_constraints.append(oc)
    
    # 将 Pick 阶段与以下 Place 阶段连接
    con = stages.Connect("connect", planners)
    # con.properties["path_constraints"] = constraint
    task.add(con)
    
    # 定义物体放置后的姿态
    place_pose = PoseStamped()
    place_pose.header.frame_id = "world"
    place_pose.pose.position.x = -0.2
    place_pose.pose.position.y = -0.6
    place_pose.pose.position.z = 0.0
    place_pose.pose.orientation.w = 1.0
    
    # 生成放置物体的笛卡尔位置姿态
    place_generator = stages.GeneratePlacePose("Generate Place Pose")
    place_generator.setMonitoredStage(task["Pick"])
    place_generator.object = object_name
    place_generator.pose = place_pose
    
    # 释放物体
    simple_ungrasp = stages.SimpleUnGrasp(place_generator, "UnGrasp") 
    
    # 创建 Place 容器，实现放置、松开和收回的功能
    place = stages.Place(simple_ungrasp, "Place")
    place.eef = eef
    place.object = object_name
    place.eef_frame = "panda_link8"
    
    # 扭转并离开物体
    retract = TwistStamped()
    retract.header.frame_id = "world"
    retract.twist.linear.z = 1.0
    place.setRetractMotion(retract, 0.03, 0.1)
    
    # 扭转并放下物体
    place_motion = TwistStamped()
    place_motion.header.frame_id = "panda_hand"
    place_motion.twist.linear.z = 1.0
    place.setPlaceMotion(place_motion, 0.03, 0.1)
    
    task.add(place)
    
    move = stages.MoveTo("moveTo ready", jointspace)
    move.group = arm
    move.setGoal("ready")
    task.add(move)
    
    if task.plan():
        solutions = task.solutions
        task.publish(task.solutions[0])
        task.execute(task.solutions[0])
        trajj = convert_msgs.Sol_to_JointTrajMsg(solutions)
    
    # 避免 ClassLoader 警告    
    del piperline
    del planners
    
    time.sleep(50)
    
def main():
    rospy.init_node("moveit_mtc_pick_place_tutorial", anonymous=True)
    roscpp_init("mtc_tutorial")
    pick_and_place()
    
if __name__ == "__main__":
    main()