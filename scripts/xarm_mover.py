#!/usr/bin/env python

from __future__ import print_function

import rospy

import sys
import copy
import math
import moveit_commander

import moveit_msgs.msg
from moveit_msgs.msg import Constraints, JointConstraint, PositionConstraint, OrientationConstraint, BoundingVolume
from sensor_msgs.msg import JointState
from moveit_msgs.msg import RobotState
import geometry_msgs.msg
from geometry_msgs.msg import Quaternion, Pose
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

from xarm_moveit.srv import MoverService, MoverServiceRequest, MoverServiceResponse


joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
# Between Melodic and Noetic, the return type of plan() changed. moveit_commander has no __version__ variable, so checking the python version as a proxy
if sys.version_info >= (3, 0):
    def planCompat(plan):
        return plan[1]
else:
    def planCompat(plan):
        return plan
"""
    Given the start angles of the robot, plan a trajectory that ends at the destination pose.
"""
def plan_trajectory(move_group, destination_pose, start_joint_angles): 
    current_joint_state = JointState()
    current_joint_state.name = joint_names
    current_joint_state.position = start_joint_angles

    moveit_robot_state = RobotState()
    moveit_robot_state.joint_state = current_joint_state
    move_group.set_start_state(moveit_robot_state)
    
    move_group.set_pose_target(destination_pose)
    plan = move_group.plan()

    if not plan:
        exception_str = """
            Trajectory could not be planned for a destination of {} with starting joint angles {}.
            Please make sure target and destination are reachable by the robot.
        """.format(destination_pose, destination_pose)
        raise Exception(exception_str)

    return planCompat(plan)

def go_home(arm,a=1,v=1):
        arm.set_max_acceleration_scaling_factor(a)
        arm.set_max_velocity_scaling_factor(v)
        # 你可以使用“home”或者其他姿态
        arm.set_named_target('home')
        plan=arm.plan()
        # if not plan:
        #     exception_str = """
        #         Trajectory could not be planned for a destination of {} with starting joint angles {}.
        #         Please make sure target and destination are reachable by the robot.
        #     """.format(destination_pose, destination_pose)
        #     raise Exception(exception_str)

        return planCompat(plan)
        # rospy.sleep(1)
def move_j(self, joint_configuration=None,a=1,v=1):
        # 设置机械臂的目标位置，使用六轴的位置数据进行描述（单位：弧度）
        if joint_configuration==None:
            joint_configuration = [0, -1.5707, 0, -1.5707, 0, 0]
        self.arm.set_max_acceleration_scaling_factor(a)
        self.arm.set_max_velocity_scaling_factor(v)
        self.arm.set_joint_value_target(joint_configuration)
        rospy.loginfo("move_j:"+str(joint_configuration))
        self.arm.go()
        rospy.sleep(1)
def testRobot(self):
        try:
            print("Test for robot...")
            # self.go_home()
            self.move_j([0.391410, -0.676384, -0.376217, 0.0, 1.052834, 0.454125],a=0.5,v=0.5)           
            rospy.sleep(2)
            print("test movejoint complete")

            # self.move_p([0.3,0,0.3,0,-np.pi/2,0])
            # print("test movepoint complete")
            # rospy.sleep(5)   
        except:
            print("Test fail! ")     
def plan_pick_and_place(req):
    response = MoverServiceResponse()

    rospy.loginfo("Starting pick and place planning!!!")

    group_name = "xarm6"
    arm = moveit_commander.MoveGroupCommander(group_name)
    arm.set_goal_joint_tolerance(0.001)
    arm.set_goal_position_tolerance(0.001)
    arm.set_goal_orientation_tolerance(0.01)

    rospy.loginfo("SET UP!!!")

    end_effector_link = arm.get_end_effector_link()
        # 设置机械臂基座的参考系
    reference_frame = 'base_link'
    arm.set_pose_reference_frame(reference_frame)

        # 设置最大规划时间和是否允许重新规划
    arm.set_planning_time(5)
    arm.allow_replanning(True)
    arm.set_planner_id("RRTconnect")

        # 设置允许的最大速度和加速度（范围：0~1）
    arm.set_max_acceleration_scaling_factor(1)
    arm.set_max_velocity_scaling_factor(1)

    rospy.loginfo("planning 1")

    current_robot_joint_configuration = req.joints_input.joints

    # Pre grasp - position gripper directly above target object
    pre_grasp_pose = plan_trajectory(arm, req.pick_pose, current_robot_joint_configuration)
    
    # If the trajectory has no points, planning has failed and we return an empty response
    if not pre_grasp_pose.joint_trajectory.points:
        return response

    rospy.loginfo("trajectory 1 length :"+ str(len(pre_grasp_pose.joint_trajectory.points)))
    
    previous_ending_joint_angles = pre_grasp_pose.joint_trajectory.points[-1].positions

    # # Grasp - lower gripper so that fingers are on either side of object
    # pick_pose = copy.deepcopy(req.pick_pose)
    # pick_pose.position.z -= 0.05  # Static value coming from Unity, TODO: pass along with request
    # grasp_pose = plan_trajectory(arm, pick_pose, previous_ending_joint_angles)
    
    # if not grasp_pose.joint_trajectory.points:
    #     return response

    # previous_ending_joint_angles = grasp_pose.joint_trajectory.points[-1].positions

    # # Pick Up - raise gripper back to the pre grasp position
    # pick_up_pose = plan_trajectory(arm, req.pick_pose, previous_ending_joint_angles)
    
    # if not pick_up_pose.joint_trajectory.points:
    #     return response

    # previous_ending_joint_angles = pick_up_pose.joint_trajectory.points[-1].positions


    rospy.loginfo("planning 2")
    # Place - move gripper to desired placement position
    place_pose = plan_trajectory(arm, req.place_pose, previous_ending_joint_angles)

    if not place_pose.joint_trajectory.points:
        return response

    previous_ending_joint_angles = place_pose.joint_trajectory.points[-1].positions

    rospy.loginfo(previous_ending_joint_angles)

    rospy.loginfo("trajectory 2 length :"+ str(len(place_pose.joint_trajectory.points)))

    # arm.execute(pre_grasp_pose)
    # 机械臂初始姿态
    # pre_gohome=go_home(arm)
    # # If the trajectory has no points, planning has failed and we return an empty response
    # if not pre_gohome.joint_trajectory.points:
    #     return response
    

    response.trajectories.append(pre_grasp_pose)
    # response.trajectories.append(grasp_pose)
    # response.trajectories.append(pick_up_pose)
    response.trajectories.append(place_pose)

    
    arm.clear_pose_targets()
    rospy.loginfo("trajectories complete")
    return response
    # testRobot()

def plan_waypoints(req):
    # will replaced by new defined service
    response = MoverServiceResponse()

    group_name = "xarm6"
    arm = moveit_commander.MoveGroupCommander(group_name)
    arm.set_goal_joint_tolerance(0.001)
    arm.set_goal_position_tolerance(0.001)
    arm.set_goal_orientation_tolerance(0.01)

    end_effector_link = arm.get_end_effector_link()
        # 设置机械臂基座的参考系
    reference_frame = 'base_link'
    arm.set_pose_reference_frame(reference_frame)

        # 设置最大规划时间和是否允许重新规划
    arm.set_planning_time(5)
    arm.allow_replanning(True)
    arm.set_planner_id("RRTconnect")

        # 设置允许的最大速度和加速度（范围：0~1）
    arm.set_max_acceleration_scaling_factor(1)
    arm.set_max_velocity_scaling_factor(1)



def moveit_xarm():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('xarm_moveit_server')
    rospy.loginfo("Starting xarm_moveit_server!!!!")
    s = rospy.Service('xarm', MoverService, plan_pick_and_place)
    
    rospy.spin()


if __name__ == "__main__":
    moveit_xarm()
