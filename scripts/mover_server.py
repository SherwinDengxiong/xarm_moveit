#! mover_server.py

#!/usr/bin/env python

from __future__ import print_function

import rospy
import tf

import sys
import copy
import math
import moveit_commander
import threading
import pickle
import csv
import os

# import pyrealsense2 as rs
from cv_bridge import CvBridge

import moveit_msgs.msg
from moveit_msgs.msg import Constraints, JointConstraint, PositionConstraint, OrientationConstraint, BoundingVolume
from sensor_msgs.msg import JointState, Image, CameraInfo
from moveit_msgs.msg import RobotState
import geometry_msgs.msg
from geometry_msgs.msg import Quaternion, Pose
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

from xarm_moveit.msg import RGBD,Extrinsics
from xarm_moveit.srv import MoverService, MoverServiceRequest, MoverServiceResponse
from xarm_moveit.srv import TrajectoryService, TrajectoryServiceRequest, TrajectoryServiceResponse
from xarm_moveit.srv import DetectionService,DetectionServiceRequest,DetectionServiceResponse
class Subscriber:
    def __init__(self,mode) :
        self.unregister_event=threading.Event()
        self.image=None
        self.K=[]
        self.R=[]
        self.P=[]
        self.rotation=[]
        self.translation=[]
        self.msg=None
        if mode==0:
            # rgb
            self.subscriber=rospy.Subscriber('/camera/color/image_raw',Image, self.colorcallback,queue_size=1)
            
            
        elif mode==1:
            #depth
            self.subscriber=rospy.Subscriber('/camera/aligned_depth_to_color/image_raw',Image, self.colorcallback,queue_size=1)
            self.image=None
            
        elif mode==2:
            # color camera info
            self.subscriber=rospy.Subscriber('/camera/color/camera_info',CameraInfo, self.camerainfo_callback,queue_size=1)
            
        elif mode==3:
            # depth camera info
            self.subscriber=rospy.Subscriber('/camera/aligned_depth_to_color/camera_info',CameraInfo, self.camerainfo_callback,queue_size=1)
            
        elif mode==4:
            #depth to color extrinsics
            self.subscriber=rospy.Subscriber('/camera/extrinsics/depth_to_color',Extrinsics, self.extrinsics_callback,queue_size=1)
            
        
    def camerainfo_callback(self,msgs):
        self.msg=msgs
        self.K=msgs.K
        self.R=msgs.R
        self.P=msgs.P

        self.subscriber.unregister()
        self.unregister_event.set()
        
    def colorcallback(self,msg):
        self.msg=msg
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        # rospy.loginfo(cv_image)
        rospy.loginfo(cv_image.shape)
        self.image=cv_image
        
        self.subscriber.unregister()
        self.unregister_event.set()
    def extrinsics_callback(self,msg):
        self.msg=msg
        self.rotation=msg.rotation
        self.translation=msg.translation

        self.subscriber.unregister()
        self.unregister_event.set()
    def get_msg(self):
        return self.msg
    def get_image(self):
        rospy.loginfo("get image")
        return self.image
    def get_camerainfo(self):
        print(self.K)
        print(self.R)
        print(self.P)
        return [self.K,self.R,self.P]
    def get_rotation(self):
        return self.rotation
    def get_translation(self):
        return self.translation
    
class Detection_client:
    def __init__(self,service_name,request) :
        self.finish_event=threading.Event()
        self.detection_info = rospy.ServiceProxy(service_name, DetectionService)
        self.request=request
        self.response=None

        self.send_request()
    def send_request(self):
		# request data
        self.response = self.detection_info(self.request)
        self.finish_event.set()
    def get_response(self):
        return self.response
pose_o=Pose()
pose_o.position.x = 0.0
pose_o.position.y = 0.0
pose_o.position.z = 0.0

# Set the orientation values
pose_o.orientation.x = 1.0
pose_o.orientation.y = 0.0
pose_o.orientation.z = 0.0
pose_o.orientation.w = 0.0


prepared_pose=copy.deepcopy(pose_o)
prepared_pose.position.x=0.0
prepared_pose.position.y=-0.3
prepared_pose.position.z=0.45
prepared_pose_joint_value=[0,0,0,0,0,0]

place_pose=copy.deepcopy(prepared_pose)
place_pose.position.x=0.5
place_pose.position.y=-0.15
place_pose.position.z=0.3

previous_joint=prepared_pose_joint_value
detection_pose=copy.deepcopy(prepared_pose)

detection_configuration=prepared_pose_joint_value
image_list=[]
depth_map_list=[]
picture_pose=[]
recorded_traj=[]
executed_traj=[]
entire_pose_list=[]
gripper_status=[]
current_gripper_status=1
demonstration_info_path="/home/rover/catkin_ws/src/FastSAM/weights/demoinfo.csv"
demonstration_number=0
joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
color_intrinsic=[]
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
def plan_trjectory_forward(move_group,destination_configuration,start_joint_angles):
    """
    Given the start angles of the robot, plan a trajectory that ends at the destination configuration
"""
    current_joint_state = JointState()
    current_joint_state.name = joint_names
    current_joint_state.position = start_joint_angles

    moveit_robot_state = RobotState()
    moveit_robot_state.joint_state = current_joint_state
    move_group.set_start_state(moveit_robot_state)

    move_group.set_joint_value_target(destination_configuration)
    plan = move_group.plan()

    if not plan:
        exception_str = """
            Trajectory could not be planned for a destination of {} with starting joint angles {}.
            Please make sure target and destination are reachable by the robot.
        """.format(destination_configuration, destination_configuration)
        raise Exception(exception_str)
    return planCompat(plan)

def plan_cartesian(move_group, destination_pose, start_joint_angles) :
    current_joint_state = JointState()
    current_joint_state.name = joint_names
    current_joint_state.position = start_joint_angles

    moveit_robot_state = RobotState()
    moveit_robot_state.joint_state = current_joint_state
    move_group.set_start_state(moveit_robot_state)

    waypoints=[]
    waypoints.append(destination_pose)

    fraction=0.0
    matrixes=100
    attempts=0

    while fraction<1.0 and attempts<matrixes:
        (plan,fraction)=move_group.compute_cartesian_path(waypoints,0.1,0.0,True)
        attempts+=1

        if attempts%10==0:
            rospy.loginfo(" still trying after {} attempts".format(attempts))

    # if not plan:
    #     exception_str = """
    #          {} with starting joint angles {}.
    #         ----------------------------------cartesian planning not successful---------------------
    #     """.format(destination_pose, destination_pose)
    #     raise Exception(exception_str)

    if fraction>0.5:
        rospy.loginfo(" ----------------------------------cartesian planning successful----------------------")
    else:
        rospy.loginfo("not successful cartesian {} portion".format(fraction))

    return planCompat(plan)

def read_csv(file_path):
    info=[]
    with open(file_path,'r') as file:
        reader=csv.reader(file)
        
        for row in reader:
            info.append(row)

    print(info)
    return info

def store_dict(dict_path,entire_pose_list,gripper_s, imagelist,depthmaplist,picturepose,color_intrinsic):
    mydict={"pose_list":entire_pose_list,"gripper":gripper_s,"image":imagelist,"depthmap":depthmaplist,"picturepose":picturepose,"colorinfo":color_intrinsic}

    print("dict_dir at {}".format(dict_path))
    

    with open(dict_path,"wb") as file:
        pickle.dump(mydict,file)
    print("store dict")



def go_home(arm,a=1,v=1):
        arm.set_max_acceleration_scaling_factor(a)
        arm.set_max_velocity_scaling_factor(v)
        # 你可以使用“home”或者其他姿态
        arm.set_named_target('home')
        plan=arm.plan()
        
        if not plan:
            exception_str = """
                Trajectory could not be planned"""
            raise Exception(exception_str)

        return planCompat(plan)
        

def go_close(gripper,a=1,v=1):
        gripper.set_max_acceleration_scaling_factor(a)
        gripper.set_max_velocity_scaling_factor(v)
        # 你可以使用“home”或者其他姿态
        gripper.set_named_target('close')
        plan=gripper.plan()
        if not plan:
            exception_str = """
                gripper could not be planned"""
            raise Exception(exception_str)

        return planCompat(plan)
def go_open(gripper,a=1,v=1):
        gripper.set_max_acceleration_scaling_factor(a)
        gripper.set_max_velocity_scaling_factor(v)
        # 你可以使用“home”或者其他姿态
        gripper.set_named_target('open')
        plan=gripper.plan()
        if not plan:
            exception_str = """
                gripper could not be planned"""
            raise Exception(exception_str)

        return planCompat(plan)

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
    arm.set_planning_time(0.2)
    arm.allow_replanning(True)
    arm.set_planner_id("RRTconnect")

        # 设置允许的最大速度和加速度（范围：0~1）
    arm.set_max_acceleration_scaling_factor(1)
    arm.set_max_velocity_scaling_factor(1)

    rospy.loginfo(req.pick_pose)

    current_robot_joint_configuration = req.joints_input.joints

    # Pre grasp - position gripper directly above target object
    pre_grasp_pose = plan_trajectory(arm, req.pick_pose, current_robot_joint_configuration)
    
    # If the trajectory has no points, planning has failed and we return an empty response
    if not pre_grasp_pose.joint_trajectory.points:
        return response
    
    previous_ending_joint_angles = pre_grasp_pose.joint_trajectory.points[-1].positions

    # Grasp - lower gripper so that fingers are on either side of object
    pick_pose = copy.deepcopy(req.pick_pose)
    pick_pose.position.z -= 0.05  # Static value coming from Unity, TODO: pass along with request
    grasp_pose = plan_trajectory(arm, pick_pose, previous_ending_joint_angles)
    
    if not grasp_pose.joint_trajectory.points:
        return response

    previous_ending_joint_angles = grasp_pose.joint_trajectory.points[-1].positions

    # Pick Up - raise gripper back to the pre grasp position
    pick_up_pose = plan_trajectory(arm, req.pick_pose, previous_ending_joint_angles)
    
    if not pick_up_pose.joint_trajectory.points:
        return response

    previous_ending_joint_angles = pick_up_pose.joint_trajectory.points[-1].positions

    # Place - move gripper to desired placement position
    place_pose = plan_trajectory(arm, req.place_pose, previous_ending_joint_angles)

    if not place_pose.joint_trajectory.points:
        return response

    # arm.execute(pre_grasp_pose)
    # 机械臂初始姿态
    # pre_gohome=go_home(arm)
    # # If the trajectory has no points, planning has failed and we return an empty response
    # if not pre_gohome.joint_trajectory.points:
    #     return response
    
    rospy.loginfo(req.pick_pose)
    rospy.loginfo(req.place_pose)
    response.trajectories.append(pre_grasp_pose)
    response.trajectories.append(grasp_pose)
    response.trajectories.append(pick_up_pose)
    response.trajectories.append(place_pose)

    

    arm.clear_pose_targets()
    rospy.loginfo("trajectories complete")
    return response
    # testRobot()


def joint_space_ik_plan(arm, current_robot_joint_configuration, target_pose_list):
    # plan a set of pose
    response_tra=[]
    previous_ending_joint_angles=current_robot_joint_configuration
    for i in range (len(target_pose_list)):
        plan = plan_trajectory(arm, target_pose_list[i], previous_ending_joint_angles)
        if not plan.joint_trajectory.points:
            rospy.loginfo("Unable to plan trajectory at {} point".format(i))
            continue
        else:
            previous_ending_joint_angles = plan.joint_trajectory.points[-1].positions
            response_tra.append(plan)

    return response_tra

def joint_space_fk_plan(arm,current_robot_joint_configuration,configuration_list):
    # plan a set of angles
    response_tra=[]
    previous_ending_joint_angles=current_robot_joint_configuration
    for i in range(len(configuration_list)):
        plan = plan_trjectory_forward(arm, configuration_list[i],previous_ending_joint_angles)
        if not plan.joint_trajectory.points:
            rospy.loginfo("Unable to plan trajectory at {} point".format(i))
            continue
        else:
            
            previous_ending_joint_angles = plan.joint_trajectory.points[-1].positions
            response_tra.append(plan)
    
    return response_tra

def detect_objects(depth_caminfo,color_caminfo,color_iamge,depthimage,detect_mode,object_name):
    # send request to detection server and use fast sam to segment objects
    detected_number=0
    request=DetectionServiceRequest()
    request.rgb_camera_info=color_caminfo
    request.depth_camera_info=depth_caminfo
    request.rgb=color_iamge
    request.depth=depthimage
    request.detect_mode=detect_mode
    request.pose=pose_o
    request.prompt=object_name
    service_name='detect_sam'
    rospy.wait_for_service(service_name)
    try:

        detection_start=Detection_client(service_name,request)
        detection_start.finish_event.wait()
        response=detection_start.get_response()

        detected_number=response.detect_number
        
    
    except rospy.ServiceException as e:
        print ("Service call failed")


    
    return detected_number
def grasp_objects(depth_caminfo,color_caminfo,color_iamge,depthimage,detect_mode,last_pose,object_name):
    # send request to detection server and use fast sam to segment objects
    detected_number=0
    request=DetectionServiceRequest()
    request.rgb_camera_info=color_caminfo
    request.depth_camera_info=depth_caminfo
    request.rgb=color_iamge
    request.depth=depthimage
    request.detect_mode=detect_mode
    request.pose=last_pose
    request.prompt=object_name
    service_name='detect_sam'
    rospy.wait_for_service(service_name)
    try:

        detection_start=Detection_client(service_name,request)
        detection_start.finish_event.wait()
        response=detection_start.get_response()

        detected_number=response.detect_number
        poselist=response.poses
        print(poselist)
        rect_list=response.rectangle

        
    
    except rospy.ServiceException as e:
        print ("Service call failed")


    
    return detected_number, poselist, rect_list


def calculate_distance(start_pose,end_pose):
    distance=math.sqrt((end_pose.position.x-start_pose.position.x)**2 + (end_pose.position.y-start_pose.position.y)**2 + (end_pose.position.z-start_pose.position.z)**2)
    return distance
def calculate_index(distance):
    return 1/(distance+1)

def find_close_pose(arm, start_configuration, target_pose, candidate_orentation):
    prev_diff=100
    similar_pose=target_pose
    
    for i in range(len(candidate_orentation)):
        similar_pose.orientation.x=candidate_orentation[i].orientation.x
        similar_pose.orientation.y=candidate_orentation[i].orientation.y
        similar_pose.orientation.z=candidate_orentation[i].orientation.z
        similar_pose.orientation.w=candidate_orentation[i].orientation.w
        plan = plan_trajectory(arm,similar_pose,start_configuration)
        if not plan.joint_trajectory.points:
            continue
        diff= (similar_pose.orientation.x-target_pose.orientation.x)**2+(similar_pose.orientation.y-target_pose.orientation.y)**2+(similar_pose.orientation.z-target_pose.orientation.z)**2+(similar_pose.orientation.w-target_pose.orientation.w)**2
        if diff<=prev_diff:
            candidate_pose=similar_pose
            similar_plan=plan
            prev_diff=diff

    if prev_diff>=100:
        return plan, target_pose
    else:
        return similar_plan, candidate_pose


def conditional_plan(arm, current_robot_joint_configuration, pose_list, d0, d1):
    response_tra=[]
    lastpose=pose_list[0]
    pose1=pose_o
    # pose_o= 000 1000
    planned_pose_list=[]


    candidate_orentation=[pose1]
    for i in range (len(pose_list)):

        if i == 0:
            # plan trajectory from current robot joint configuration to the first waypoint
            pose=pose_list[i]
            plan = plan_trajectory(arm, pose, current_robot_joint_configuration)
        else:
            # plan trajectory from the previous waypoint to the current waypoint

            pose=pose_list[i]
            
            plan=plan_trajectory(arm, pose, previous_ending_joint_angles)


            # distance=calculate_distance(pose_list[i-1],pose_list[i])
            
            # if distance< d0:
            #     pose=pose_list[i]
                
            #     plan = plan_trajectory(arm, pose, previous_ending_joint_angles)
            #     # If the trajectory has no points, planning has failed and we return an empty response
            #     if not plan.joint_trajectory.points:
            #         #find the similar pose and plan
            #         plan, pose=find_close_pose(arm, previous_ending_joint_angles,pose,candidate_orentation)

            #     lastpose=pose
            # elif d0<=distance and distance<d1:
            #     index=calculate_index(distance)
            #     pose=lastpose
            #     # pose.orientation.x += (pose_list[i].orientation.x-lastpose.orientation.x)*index
            #     # pose.orientation.y += (pose_list[i].orientation.y-lastpose.orientation.y)*index
            #     # pose.orientation.z += (pose_list[i].orientation.z-lastpose.orientation.z)*index
            #     # pose.orientation.w += (pose_list[i].orientation.w-lastpose.orientation.w)*index

            #     pose.orientation.x = pose_list[i].orientation.x
            #     pose.orientation.y = pose_list[i].orientation.y
            #     pose.orientation.z = pose_list[i].orientation.z
            #     pose.orientation.w = pose_list[i].orientation.w
            #     pose.position.x = pose_list[i].position.x
            #     pose.position.y = pose_list[i].position.y
            #     pose.position.z = pose_list[i].position.z

            #     plan = plan_trajectory(arm, pose, previous_ending_joint_angles)
            #     if not plan.joint_trajectory.points:
            #         #find the similar pose and plan
            #         plan, pose=find_close_pose(arm, previous_ending_joint_angles,pose,candidate_orentation)
            #     lastpose=pose
            # else:
            #     pose=lastpose
            #     pose.position.x = pose_list[i].position.x
            #     pose.position.y = pose_list[i].position.y
            #     pose.position.z = pose_list[i].position.z
            #     plan = plan_trajectory(arm, pose, previous_ending_joint_angles)
            #     if not plan.joint_trajectory.points:
            #         #find the similar pose and plan
            #         plan, pose=find_close_pose(arm, previous_ending_joint_angles,pose,candidate_orentation)
            #     lastpose=pose

            # rospy.loginfo("distance is {}",format(distance))

        if not plan.joint_trajectory.points:
            rospy.loginfo("Unable to plan trajectory at {} point".format(i))
            continue
        # update the previous ending joint angles to the current ending joint angles
        else:
            rospy.loginfo("lastpose is {}".format(lastpose))
            previous_ending_joint_angles = plan.joint_trajectory.points[-1].positions
            planned_pose_list.append(pose)
            response_tra.append(plan)

    return response_tra, planned_pose_list
def plan_grasping(arm,object_number,graspposelist,rect_list,detectionpose,previous_ending_joint_angles,placepose):
    # move and grasp the object
    response_tra=[]
    planning_pose_list=[]
    rospy.loginfo("adding gripper rotation-------")
    if object_number>0:
        object_number==1
    for i in range(object_number):
        pose_ready=copy.deepcopy(graspposelist[i])
        pose_ready.position.z+=0.15
        euler_angle=tf.transformations.euler_from_quaternion([pose_ready.orientation.x,pose_ready.orientation.y,pose_ready.orientation.z,pose_ready.orientation.w])
        rospy.loginfo("euler angle is {}".format(euler_angle))
        neweuler=(euler_angle[0],euler_angle[1],euler_angle[2]+rect_list[i])
        
        orientation=tf.transformations.quaternion_from_euler(neweuler[0],neweuler[1],neweuler[2])
        rospy.loginfo("euler angle is {}".format(neweuler))
        pose_ready.orientation.x=orientation[0]
        pose_ready.orientation.y=orientation[1]
        pose_ready.orientation.z=orientation[2]
        pose_ready.orientation.w=orientation[3]
        planning_pose_list.append(pose_ready)
        graspposelist[i].orientation=pose_ready.orientation
        planning_pose_list.append(graspposelist[i])
        planning_pose_list.append(detectionpose)
        planning_pose_list.append(placepose)
        planning_pose_list.append(detectionpose)

    
    executed_pose_list=[]
    for i in range(len(planning_pose_list)):

        pose=planning_pose_list[i]
            
        plan=plan_trajectory(arm, pose, previous_ending_joint_angles)
        if not plan.joint_trajectory.points:
            rospy.loginfo("Unable to plan trajectory at {} point".format(i))
            continue
        # update the previous ending joint angles to the current ending joint angles
        else:
            
            previous_ending_joint_angles = plan.joint_trajectory.points[-1].positions
            response_tra.append(plan)
            executed_pose_list.append(pose)
            

    return response_tra,executed_pose_list
def plan_demo_grasping(arm,object_number,graspposelist,detectionpose,previous_ending_joint_angles,placepose):
    response_tra=[]
    planning_pose_list=[]
    rospy.loginfo("adding gripper rotation-------")
    if object_number>0:
        object_number==1
    for i in range(object_number):

        for i in range(len(graspposelist)):
            pose_demo=copy.deepcopy(graspposelist[i])
            planning_pose_list.append(pose_demo)
        planning_pose_list.append(detectionpose)
        planning_pose_list.append(placepose)
        planning_pose_list.append(detectionpose)

    
    executed_pose_list=[]
    for i in range(len(planning_pose_list)):

        pose=planning_pose_list[i]
            
        plan=plan_trajectory(arm, pose, previous_ending_joint_angles)
        if not plan.joint_trajectory.points:
            rospy.loginfo("Unable to plan trajectory at {} point".format(i))
            continue
        # update the previous ending joint angles to the current ending joint angles
        else:
            
            previous_ending_joint_angles = plan.joint_trajectory.points[-1].positions
            response_tra.append(plan)
            executed_pose_list.append(pose)
            

    return response_tra,executed_pose_list
def contain_pose(poselist,pose,x_adjust,y_adjust,z_adjust):
    for i in range(len(poselist)):
        targetpose=poselist[i]
        if pose.position.x==targetpose.position.x+x_adjust and pose.position.y==targetpose.position.y+y_adjust and pose.position.z==targetpose.position.z+z_adjust:
            return i
        
    return -1

def plan_waypoints_list(req):
    global prepared_pose_joint_value
    global detection_configuration
    global detection_pose
    global recorded_traj
    global executed_traj
    global entire_pose_list
    global gripper_status
    global current_gripper_status
    global demonstration_number
    global previous_joint
    global image_list
    global depth_map_list
    global picture_pose
    global color_intrinsic
    # will replaced by new defined service
    response = TrajectoryServiceResponse()

    group_name = "xarm6"
    gripper_goup_name="xarm_gripper"
    arm = moveit_commander.MoveGroupCommander(group_name)
    arm.set_goal_joint_tolerance(0.001)
    arm.set_goal_position_tolerance(0.001)
    arm.set_goal_orientation_tolerance(0.01)
    workspace_limit=[1,-0.3,1,-0.5,0.7,-0.1]
    arm.set_workspace(workspace_limit)

    gripper=moveit_commander.MoveGroupCommander(gripper_goup_name)
    gripper.set_goal_position_tolerance(0.01)
    gripper.set_pose_reference_frame("xarm_gripper_base_link")
    gripper.set_planning_time(0.5)

    end_effector_link = arm.get_end_effector_link()
        # 设置机械臂基座的参考系
    reference_frame = 'base_link'
    arm.set_pose_reference_frame(reference_frame)

        # 设置最大规划时间和是否允许重新规划
    arm.set_planning_time(1)
    arm.allow_replanning(True)
    arm.set_planner_id("RRTconnect")

        # 设置允许的最大速度和加速度（范围：0~1）
    arm.set_max_acceleration_scaling_factor(0.4)
    arm.set_max_velocity_scaling_factor(0.4)
    # current robot joint configuration
    current_robot_joint_configuration = req.pose_list.joints

    # start trajectory from current robot joint configuration

    # for each waypoint in the list, plan a trajectory from the previous waypoint to the current waypoint


    plan_mode= req.mode
    pose_list=req.pose_list.pose_list


    # set up distance for rotaion and transition, need to be input in terminal
    d_rotation=0
    d_transition=100

    if plan_mode==0:
        #from any where to prepared pose both virtual and real
        rospy.loginfo("mode 0 move to prepared pose")
        prepared_pose_list=[prepared_pose]
        response.trajectories=joint_space_ik_plan(arm, prepared_pose_joint_value, prepared_pose_list)
        prepared_pose_joint_value=response.trajectories[-1].joint_trajectory.points[-1].positions
        for i in range(len(response.trajectories)):
            execute_plan=response.trajectories[i]
            arm.execute(execute_plan)
        detection_configuration=prepared_pose_joint_value
        previous_joint=prepared_pose_joint_value
        detection_pose=copy.deepcopy(prepared_pose)

        # depth_caminfo_sub=Subscriber(3)
        # depth_caminfo_sub.unregister_event.wait()
        # depth_caminfo=depth_caminfo_sub.get_msg()
        

        # color_caminfo_sub=Subscriber(2)
        # color_caminfo_sub.unregister_event.wait()
        # color_caminfo=color_caminfo_sub.get_msg()
       
        # colorimage_sub=Subscriber(0)
        # colorimage_sub.unregister_event.wait()
        # color_iamge=colorimage_sub.get_msg()
        

        # depthimage_sub=Subscriber(1)
        # depthimage_sub.unregister_event.wait()
        # depthimage=depthimage_sub.get_msg()

        # colorimg=colorimage_sub.image
        # depthimg=depthimage_sub.image

        # image_list.append(colorimg)
        # depth_map_list.append(depthimg)
        
        print(" prepared information at mode 0!!!")
        


    elif plan_mode==1:
        # execute traj update


        depth_caminfo_sub=Subscriber(3)
        depth_caminfo_sub.unregister_event.wait()
        depth_caminfo=depth_caminfo_sub.get_msg()
        

        color_caminfo_sub=Subscriber(2)
        color_caminfo_sub.unregister_event.wait()
        color_caminfo=color_caminfo_sub.get_msg()
       
        colorimage_sub=Subscriber(0)
        colorimage_sub.unregister_event.wait()
        color_iamge=colorimage_sub.get_msg()
        

        depthimage_sub=Subscriber(1)
        depthimage_sub.unregister_event.wait()
        depthimage=depthimage_sub.get_msg()

        colorimg=colorimage_sub.image
        depthimg=depthimage_sub.image
        color_intrinsic=list(color_caminfo.K)

        image_list.append(colorimg)
        depth_map_list.append(depthimg)

        picture_pose.append([detection_pose.position.x,detection_pose.position.y,detection_pose.position.z])

        rospy.loginfo("mode 1 execute")
        executed_length=len(executed_traj)
        
        for i in range(executed_length,len(recorded_traj)):
            execute_plan=recorded_traj[i]
            executed_traj.append(execute_plan)
            arm.execute(execute_plan)
            is_gripper=gripper_status[i]
            if i==0:
                continue
            else:
                if is_gripper==gripper_status[i-1]:
                    continue
                else:
                    if is_gripper==-1:
                        gripper_plan=go_close(gripper,1,1)
                        gripper.execute(gripper_plan)
                        rospy.loginfo("the gripper is closing----executed {}".format(i))
                    elif is_gripper==1:
                        gripper_plan=go_open(gripper,1,1)
                        gripper.execute(gripper_plan)
                        rospy.loginfo("the gripper is opening----executed {}".format(i))

            # need to add gripper execute
            executed_traj.append(execute_plan)


        response.trajectories=joint_space_fk_plan(arm, current_robot_joint_configuration, [current_robot_joint_configuration])

        previous_joint=response.trajectories[-1].joint_trajectory.points[-1].positions
        rospy.loginfo("entire pose number {}".format(len(entire_pose_list)))
        rospy.loginfo("--------------------Final gripper status {}".format(gripper_status))
        


    elif plan_mode==2:

        

        detection_configuration_list=[detection_configuration]
        response.trajectories=joint_space_fk_plan(arm,current_robot_joint_configuration,detection_configuration_list)

        recorded_traj=recorded_traj[:len(executed_traj)]
        gripper_status=gripper_status[:len(executed_traj)]
        entire_pose_list=entire_pose_list[:len(executed_traj)]
        rospy.loginfo("length of executed traj is {}".format(len(executed_traj)))
        rospy.loginfo("--------------------current gripper status {}".format(gripper_status))
        current_gripper_status=1


    elif plan_mode==3:
        #search objects 
        gripper_plan=go_open(gripper,1,1)
        gripper.execute(gripper_plan)
        detect_mode=0

        # current_robot_joint_configuration=detection_configuration

        #search in a square
        prepared_pose_list=[]
        search_d=0.1
        next_pose1=copy.deepcopy(prepared_pose)
        next_pose1.position.y+=search_d
        prepared_pose_list.append(next_pose1)
        next_pose2=copy.deepcopy(next_pose1)
        next_pose2.position.y+=search_d
        prepared_pose_list.append(next_pose2)
        next_pose3=copy.deepcopy(next_pose2)
        next_pose3.position.y+=search_d
        prepared_pose_list.append(next_pose3)
        next_pose4=copy.deepcopy(next_pose3)
        next_pose4.position.y+=search_d
        prepared_pose_list.append(next_pose4)
        next_pose5=copy.deepcopy(next_pose4)
        next_pose5.position.y+=search_d
        prepared_pose_list.append(next_pose5)
        next_pose6=copy.deepcopy(next_pose5)
        next_pose6.position.y+=search_d
        prepared_pose_list.append(next_pose6)
        
            
        trajectory,planned_pose_list=conditional_plan(arm, current_robot_joint_configuration, prepared_pose_list, d_rotation,d_transition)

        
        depth_caminfo_sub=Subscriber(3)
        depth_caminfo_sub.unregister_event.wait()
        depth_caminfo=depth_caminfo_sub.get_msg()
        # print(depth_caminfo)

        color_caminfo_sub=Subscriber(2)
        color_caminfo_sub.unregister_event.wait()
        color_caminfo=color_caminfo_sub.get_msg()
        # print(color_caminfo)

        # execute trajectory and detect objects, select  the configuration that detects most objects
        detected_objects_number=0
        
        candidatelist=[]
        
        print("start camera!!!")
        for i in range(len(trajectory)):
            execute_plan=trajectory[i]
            candidate_configuration=execute_plan.joint_trajectory.points[-1].positions
            candidate_pose=planned_pose_list[i]
            arm.execute(execute_plan)
            
            colorimage_sub=Subscriber(0)
            colorimage_sub.unregister_event.wait()
            color_iamge=colorimage_sub.get_msg()
            
            

            depthimage_sub=Subscriber(1)
            depthimage_sub.unregister_event.wait()
            depthimage=depthimage_sub.get_msg()

            colorimg=colorimage_sub.image
            depthimg=depthimage_sub.image

            image_list.append(colorimg)
            depth_map_list.append(depthimg)
            picture_pose.append([candidate_pose.position.x,candidate_pose.position.y,candidate_pose.position.z])


            
            print(" prepared information!!!")
            object_number, poselist, rect_list=grasp_objects(depth_caminfo,color_caminfo,color_iamge,depthimage,detect_mode,candidate_pose,object_name="")

            object_number=0
            rospy.loginfo("object number is {}".format(object_number))
            if object_number>detected_objects_number:
                detection_configuration=candidate_configuration
                detection_pose=candidate_pose
                detected_objects_number=object_number
                candidatelist=pose_list

        # for i in range(len(candidate_list)):
        #     cpose=candidate_list[i]
        #     picture_pose.append([cpose.position.x,cpose.position.y,cpose.position.z,cpose.orientation.x,cpose.orientation.y,cpose.orientation.z,cpose.orientation.w])

        # move real robot to the detection configuration
        execute_configuration=trajectory[-1].joint_trajectory.points[-1].positions
        final_configuration_list=[detection_configuration]
        detection_trajectory=joint_space_fk_plan(arm,execute_configuration, final_configuration_list)
        for i in range(len(detection_trajectory)):
            execute_plan=detection_trajectory[i]
            arm.execute(execute_plan)
        
        # move virtual robot to detection configuration
        virtual_trajectory= joint_space_fk_plan(arm,current_robot_joint_configuration,final_configuration_list)
        response.trajectories=virtual_trajectory


        previous_joint=virtual_trajectory[-1].joint_trajectory.points[-1].positions
        rospy.loginfo("the detection pose is {}".format(detection_pose))
        rospy.loginfo("length of the poselit search is {}".format(len(planned_pose_list)))

    elif plan_mode==4:

        # planning with grasp
        proportion=4
        # target_pose=pose_list[-1]
        # # need to add gripper command
        length_list=len(pose_list)
        # candidate_list=[]
        # if length_list>1 and length_list<proportion:
        #     candidate_list=pose_list[:-1]
        # elif length_list>=proportion:
        #     candidate_list=pose_list[:-int(length_list/proportion)]
        #     rospy.loginfo("candidate length")
        #     rospy.loginfo(int(length_list/proportion))
        # elif length_list==1:
        #     candidate_list=target_pose
        #     # length=1 not completed

        candidate_list=pose_list

        rospy.loginfo(len(candidate_list))
        if length_list>0:
            candidate_trajectories,planned_pose_list=conditional_plan(arm, current_robot_joint_configuration, candidate_list, d_rotation,d_transition)
        else:
            candidate_trajectories=joint_space_fk_plan(arm,current_robot_joint_configuration,[current_robot_joint_configuration])
        for i in range(len(candidate_trajectories)):
            recorded_traj.append(candidate_trajectories[i])
            gripper_status.append(current_gripper_status)
        
        # need pre-target pose and grasp program

        

        response.trajectories=candidate_trajectories
        
        # adjust gripper status
        current_gripper_status=current_gripper_status*-1
        gripper_status[-1]=current_gripper_status
        for planned_pose in planned_pose_list:
            entire_pose_list.append(planned_pose)
    elif plan_mode==5:
        length_list=len(pose_list)
        if length_list>0:
            response.trajectories,planned_pose_list=conditional_plan(arm, current_robot_joint_configuration, pose_list, d_rotation,d_transition)
        else:
            response.trajectories=joint_space_fk_plan(arm,current_robot_joint_configuration,[current_robot_joint_configuration])
        for plan in response.trajectories:
            recorded_traj.append(plan)
            gripper_status.append(current_gripper_status)
        
        for planned_pose in planned_pose_list:
            entire_pose_list.append(planned_pose)

        # lastorientation=pose_list[-1].orientation
        # euler_angle= tf.transformations.euler_from_quaternion([lastorientation.x,lastorientation.y,lastorientation.z,lastorientation.w])
        # lastx=euler_angle[0]/math.pi*180
        # lasty=euler_angle[1]/math.pi*180
        # lastz=euler_angle[2]/math.pi*180

        # rospy.loginfo("last pose is {}".format(pose_list[-1]))
        # rospy.loginfo("euler angle is {}, {}, {}".format(lastx,lasty,lastz))
    elif plan_mode==6:

        #virtual robot move back to detection pose
        traject=[]
        prepared_configuration_list=[detection_configuration]
        
        back_detect_plan=joint_space_fk_plan(arm, current_robot_joint_configuration, prepared_configuration_list)
        traject.append(back_detect_plan[0])

        
        # move virtual robot from detection pose to all recorded pose
        for i in range(len(recorded_traj)):
            plan=recorded_traj[i]
            traject.append(plan)

        
        
        response.trajectories=traject

    elif plan_mode==7:
        demo_path=demonstration_info_path

        # start a new demonstration at detection configuration
        prepared_configuration_list=[detection_configuration]
        back_detect_plan=joint_space_fk_plan(arm, current_robot_joint_configuration, prepared_configuration_list)

        response.trajectories=back_detect_plan

        real_back_plan=joint_space_fk_plan(arm, previous_joint, prepared_configuration_list)
        # execute to detection config
        for i in range(len(real_back_plan)):
            execute_plan=back_detect_plan[i]
            arm.execute(execute_plan)

        # need to open gripper
        gripper_plan=go_open(gripper,1,1)
        gripper.execute(gripper_plan)

        print(demo_path)
        # need to store information
        demo_info=read_csv(demo_path)
        stored_pose_dlist=[]
        for i in range(len(entire_pose_list)):
            stored_pose=entire_pose_list[i]
            stored_pose_d=[stored_pose.position.x,stored_pose.position.y,stored_pose.position.z,stored_pose.orientation.x,stored_pose.orientation.y,stored_pose.orientation.z,stored_pose.orientation.w]
            stored_pose_dlist.append(stored_pose_d)


        current_info=demo_info[demonstration_number]
        dict_path=current_info[0]
        dict_name=os.path.join(dict_path,current_info[1]+'.pkl') 
        store_dict(dict_name,stored_pose_dlist,gripper_status, image_list,depth_map_list,picture_pose,color_intrinsic)

        print("storing No {} th demonstration at {}".format(demonstration_number,dict_name))
        demonstration_number+=1
        


        executed_traj=[]
        recorded_traj=[]
        entire_pose_list=[]
        gripper_status=[]
        image_list=[]
        depth_map_list=[]
        picture_pose=[]
        current_gripper_status=1

    elif plan_mode==8:
        #auto grasp
        detect_mode=1
        
        object_name="box"
        
        plans=joint_space_ik_plan(arm,previous_joint,[detection_pose])
        

        for plan in plans:
            arm.execute(plan)

        gripper_plan=go_open(gripper,1,1)
        gripper.execute(gripper_plan)

        previous_ending_joint_angles=plans[-1].joint_trajectory.points[-1].positions
        previous_joint=previous_ending_joint_angles
        last_pose=detection_pose

        depth_caminfo_sub=Subscriber(3)
        depth_caminfo_sub.unregister_event.wait()
        depth_caminfo=depth_caminfo_sub.get_msg()
        

        color_caminfo_sub=Subscriber(2)
        color_caminfo_sub.unregister_event.wait()
        color_caminfo=color_caminfo_sub.get_msg()
       
        colorimage_sub=Subscriber(0)
        colorimage_sub.unregister_event.wait()
        color_iamge=colorimage_sub.get_msg()
        

        depthimage_sub=Subscriber(1)
        depthimage_sub.unregister_event.wait()
        depthimage=depthimage_sub.get_msg()
        
        print(" prepared information!!!")
        object_number,graspposelist,rect_list=grasp_objects(depth_caminfo,color_caminfo,color_iamge,depthimage,detect_mode,last_pose,object_name)

        
        if object_number>0:


            plan_traj,exe_poselist=plan_grasping(arm,object_number,graspposelist,rect_list,detection_pose,previous_ending_joint_angles,place_pose)
            previous_joint=plan_traj[-1].joint_trajectory.points[-1].positions
            for i in range(len(plan_traj)):
                plan=plan_traj[i]
                pose=exe_poselist[i]
                
                arm.execute(plan)
                indexc=contain_pose(graspposelist,pose,0,0,0)
                print(" ----------------------------pose index is {}--------------------".format(indexc))
                if indexc>=0:
                
                    gripper_plan=go_close(gripper,1,1)
                    gripper.execute(gripper_plan)
                
                elif pose==place_pose:
                    gripper_plan=go_open(gripper,1,1)
                    gripper.execute(gripper_plan)

        
        gripper_plan=go_open(gripper,1,1)
        gripper.execute(gripper_plan)
        # plan=go_home(arm,1,1)
        # arm.execute(plan)
        rospy.loginfo("------------------finish _grasping---------------------")
    elif plan_mode==9:
        #grasp_after demo  
        detect_mode=2
        
        object_name="box"
        gripper_plan=go_open(gripper,1,1)
        gripper.execute(gripper_plan)
        plans=joint_space_ik_plan(arm,previous_joint,pose_list)
        

        for plan in plans:
            arm.execute(plan)

        previous_ending_joint_angles=plans[-1].joint_trajectory.points[-1].positions
        last_pose=pose_list[-1]

        depth_caminfo_sub=Subscriber(3)
        depth_caminfo_sub.unregister_event.wait()
        depth_caminfo=depth_caminfo_sub.get_msg()
        

        color_caminfo_sub=Subscriber(2)
        color_caminfo_sub.unregister_event.wait()
        color_caminfo=color_caminfo_sub.get_msg()
       
        colorimage_sub=Subscriber(0)
        colorimage_sub.unregister_event.wait()
        color_iamge=colorimage_sub.get_msg()
        

        depthimage_sub=Subscriber(1)
        depthimage_sub.unregister_event.wait()
        depthimage=depthimage_sub.get_msg()
        
        print(" prepared information!!!")
        object_number,graspposelist,rect_list=grasp_objects(depth_caminfo,color_caminfo,color_iamge,depthimage,detect_mode,last_pose,object_name)
        print("received demo grasp pose")
        print(graspposelist)
        
        if object_number>0:


            plan_traj,exe_poselist=plan_demo_grasping(arm,object_number,graspposelist,detection_pose,previous_ending_joint_angles,place_pose)
            previous_joint=plan_traj[-1].joint_trajectory.points[-1].positions
            for i in range(len(plan_traj)):
                plan=plan_traj[i]
                pose=exe_poselist[i]
                
                arm.execute(plan)
                indexc=contain_pose(graspposelist,pose,0,0,0)
                print(" ----------------------------pose index is {}--------------------".format(indexc))
                if pose==graspposelist[-1]:
                
                    gripper_plan=go_close(gripper,1,1)
                    gripper.execute(gripper_plan)
                
                elif pose==place_pose:
                    gripper_plan=go_open(gripper,1,1)
                    gripper.execute(gripper_plan)

        
        gripper_plan=go_open(gripper,1,1)
        gripper.execute(gripper_plan)
        # plan=go_home(arm,1,1)
        # arm.execute(plan)
        rospy.loginfo("------------------finish _grasping---------------------")

    elif plan_mode==14:
        # go home
        home_joint=[0.0,0.0,0.0,0.0,0.0,0.0]
        return_pose=copy.deepcopy(prepared_pose)
        return_pose.position.z+=0.05
        trajectories=joint_space_ik_plan(arm,previous_joint,[return_pose])
        for traj in trajectories:
            arm.execute(traj)
        previous_joint=trajectories[-1].joint_trajectory.points[-1].positions
        trajectories=joint_space_fk_plan(arm,previous_joint,[home_joint])
        
        for traj in trajectories:
            arm.execute(traj)
        previous_joint=trajectories[-1].joint_trajectory.points[-1].positions
        response.trajectories=trajectories
    elif plan_mode==15:
        

        trajectories,planned_pose_list=conditional_plan(arm, previous_joint, pose_list, d_rotation,d_transition)
        for traj in trajectories:
            arm.execute(traj)
        previous_joint=trajectories[-1].joint_trajectory.points[-1].positions
        gripperplan=go_close(gripper,1,1)
        gripper.execute(gripperplan)
        gripperplan=go_open(gripper,1,1)
        gripper.execute(gripperplan)

        response.trajectories=trajectories


    



        







    
    #recode pose list
    response.poses= req.pose_list.pose_list
    response.responseindex=0

    arm.clear_pose_targets()
    rospy.loginfo("trajectories complete . mode is {}".format(plan_mode))
    return response


def moveit_xarm():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('mover_server')
    rospy.loginfo("Starting xarm_moveit_server!!!!")
    s = rospy.Service('xarm_mover', TrajectoryService, plan_waypoints_list)
    rospy.loginfo("list error here")
    
    rospy.spin()


if __name__ == "__main__":
    moveit_xarm()