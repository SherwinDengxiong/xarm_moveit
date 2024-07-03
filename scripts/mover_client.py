# service client

import sys
import copy
import rospy
import pickle
from geometry_msgs.msg import Pose, Point, Quaternion
import threading
from cv_bridge import CvBridge

from xarm_moveit.msg import XarmMoveitJoints, PoseList
from sensor_msgs.msg import JointState, Image, CameraInfo

from xarm_moveit.srv import MoverService, MoverServiceRequest, MoverServiceResponse, TrajectoryService, TrajectoryServiceRequest, TrajectoryServiceResponse
from xarm_moveit.srv import DetectionService,DetectionServiceRequest,DetectionServiceResponse



current_joint_angles = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

pose0 = Pose()
pose0.position.x = 0.2
pose0.position.y = 0.0
pose0.position.z = 0.0
pose0.orientation.x = 1.0
pose0.orientation.y = 0.0
pose0.orientation.z = 0.0
pose0.orientation.w = 0.0
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
def mode_0():
    global current_joint_angles
    mode=0

    rospy.init_node('mover_client')
    rospy.wait_for_service('xarm_mover')

    pose1=copy.deepcopy(pose0)
    pose1.position.x +=0.2

    pose2=copy.deepcopy(pose1)
    pose2.position.y +=0.2

    pose3=copy.deepcopy(pose2)
    pose3.position.z +=1.0


    poselist=PoseList()
    poselist.pose_list=[pose1,pose2,pose3]
    poselist.joints=current_joint_angles


    try:
        moverinfo = rospy.ServiceProxy('xarm_mover', TrajectoryService)

		# request data
        response = moverinfo(poselist,mode)
        if len(response.trajectories) > 0:
            # global current_joint_angles
            current_joint_angles=response.trajectories[-1].joint_trajectory.points[-1].positions
            print(current_joint_angles)
            
            print ("in mode 0")
            print(response.poses[-1])
    
    except rospy.ServiceException as e:
        print ("Service call failed")

def mode_1():
    # config,pose

    global current_joint_angles
    mode=1

    rospy.init_node('mover_client')
    rospy.wait_for_service('xarm_mover')

    pose1=copy.deepcopy(pose0)
    pose1.position.x +=0.2

    poselist=PoseList()
    poselist.pose_list=[pose1]
    poselist.joints=current_joint_angles


    try:
        moverinfo = rospy.ServiceProxy('xarm_mover', TrajectoryService)

		# 请求服务调用，输入请求数据
        response = moverinfo(poselist,mode)
      
        if len(response.trajectories) > 0:
            # global current_joint_angles
            current_joint_angles=response.trajectories[-1].joint_trajectory.points[-1].positions
            print(current_joint_angles)
            
            print ("in mode 1")
            print(response.poses[-1])
    
    except rospy.ServiceException as e:
        print ("Service call failed")

def mode_2():
    

    global current_joint_angles
    mode=2

    rospy.init_node('mover_client')
    rospy.wait_for_service('xarm_mover')

    pose1=copy.deepcopy(pose0)
    pose1.position.x +=0.2

    poselist=PoseList()
    poselist.pose_list=[pose1]
    poselist.joints=current_joint_angles


    try:
        moverinfo = rospy.ServiceProxy('xarm_mover', TrajectoryService)

		# 请求服务调用，输入请求数据
        response = moverinfo(poselist,mode)
        
        if len(response.trajectories) > 0:
            # global current_joint_angles
            current_joint_angles=response.trajectories[-1].joint_trajectory.points[-1].positions
            print(current_joint_angles)
            
            print ("in mode 2")
            print(response.poses[-1])
    
    except rospy.ServiceException as e:
        print ("Service call failed")
def mode_3():
    

    global current_joint_angles
    mode=3

    rospy.init_node('mover_client')
    rospy.wait_for_service('xarm_mover')

    pose1=copy.deepcopy(pose0)
    pose1.position.x +=0.2

    poselist=PoseList()
    poselist.pose_list=[pose1]
    poselist.joints=current_joint_angles


    try:
        moverinfo = rospy.ServiceProxy('xarm_mover', TrajectoryService)

		# 请求服务调用，输入请求数据
        response = moverinfo(poselist,mode)
        
        if len(response.trajectories) > 0:
            # global current_joint_angles
            current_joint_angles=response.trajectories[-1].joint_trajectory.points[-1].positions
            print(current_joint_angles)
            
            print ("in mode 3")
            print(response.poses[-1])
    
    except rospy.ServiceException as e:
        print ("Service call failed")
def mode_4():
    

    global current_joint_angles
    mode=4

    rospy.init_node('mover_client')
    rospy.wait_for_service('xarm_mover')

    pose1=copy.deepcopy(pose0)
    pose1.position.z +=0.2

    pose2=copy.deepcopy(pose1)
    pose2.position.z +=0.2

    poselist=PoseList()
    poselist.pose_list=[pose1,pose2]
    poselist.joints=current_joint_angles


    try:
        moverinfo = rospy.ServiceProxy('xarm_mover', TrajectoryService)

		# 请求服务调用，输入请求数据
        response = moverinfo(poselist,mode)
        
        if len(response.trajectories) > 0:
            # global current_joint_angles
            current_joint_angles=response.trajectories[-1].joint_trajectory.points[-1].positions
            print(current_joint_angles)
            
            print ("in mode 4")
            print(response.poses[-1])
    
    except rospy.ServiceException as e:
        print ("Service call failed")

def mode_5():
    

    global current_joint_angles
    mode=5

    rospy.init_node('mover_client')
    rospy.wait_for_service('xarm_mover')

    pose1=copy.deepcopy(pose0)
    pose1.position.x =-0.0
    pose1.position.y =-0.3
    pose1.position.z=0.45

    pose2=copy.deepcopy(pose0)
    pose2.position.x =0.04
    pose2.position.y =-0.4
    pose2.position.z=0.0
    pose2.orientation.x=-0.85
    pose2.orientation.y=-0.5
    pose2.orientation.z=0.03
    pose2.orientation.w=0.13
    

    

    poselist=PoseList()
    poselist.pose_list=[pose1,pose2]
    poselist.joints=current_joint_angles


    try:
        moverinfo = rospy.ServiceProxy('xarm_mover', TrajectoryService)

		# 请求服务调用，输入请求数据
        response = moverinfo(poselist,mode)
        
        if len(response.trajectories) > 0:
            # global current_joint_angles
            current_joint_angles=response.trajectories[-1].joint_trajectory.points[-1].positions
            print(current_joint_angles)
            
            print ("in mode 5")
            print(response.poses[-1])
    
    except rospy.ServiceException as e:
        print ("Service call failed")

def mode_6():
    

    global current_joint_angles
    mode=6

    rospy.init_node('mover_client')
    rospy.wait_for_service('xarm_mover')

    pose1=copy.deepcopy(pose0)
    pose1.position.x +=0.2

    pose2=copy.deepcopy(pose1)
    pose2.position.x +=0.2

    poselist=PoseList()
    poselist.pose_list=[pose1,pose2,pose1,pose2,pose1]
    poselist.joints=current_joint_angles


    try:
        moverinfo = rospy.ServiceProxy('xarm_mover', TrajectoryService)

		# 请求服务调用，输入请求数据
        response = moverinfo(poselist,mode)
        
        if len(response.trajectories) > 0:
            # global current_joint_angles
            current_joint_angles=response.trajectories[-1].joint_trajectory.points[-1].positions
            print(current_joint_angles)
            
            print ("in mode 6")
            print(response.poses[-1])
    
    except rospy.ServiceException as e:
        print ("Service call failed")
def mode_7():
    

    global current_joint_angles
    mode=7

    rospy.init_node('mover_client')
    rospy.wait_for_service('xarm_mover')

    pose1=copy.deepcopy(pose0)
    pose1.position.x +=0.2

    pose2=copy.deepcopy(pose1)
    pose2.position.x +=0.2

    poselist=PoseList()
    poselist.pose_list=[pose1,pose2]
    poselist.joints=current_joint_angles


    try:
        moverinfo = rospy.ServiceProxy('xarm_mover', TrajectoryService)

		# 请求服务调用，输入请求数据
        response = moverinfo(poselist,mode)
        
        if len(response.trajectories) > 0:
            # global current_joint_angles
            current_joint_angles=response.trajectories[-1].joint_trajectory.points[-1].positions
            print(current_joint_angles)
            
            print ("in mode 7")
            print(response.poses[-1])
    
    except rospy.ServiceException as e:
        print ("Service call failed")

def mode_8():
    

    global current_joint_angles
    mode=8

    rospy.init_node('mover_client')
    rospy.wait_for_service('xarm_mover')

    pose1=copy.deepcopy(pose0)
    pose1.position.x=0.0
    pose1.position.y=-0.3
    pose1.position.z=0.45

    # pose2=copy.deepcopy(pose0)
    # pose2.position.y +=0.3
    poselist=PoseList()
    poselist.pose_list=[pose1]
    poselist.joints=current_joint_angles


    try:
        moverinfo = rospy.ServiceProxy('xarm_mover', TrajectoryService)

		# 请求服务调用，输入请求数据
        response = moverinfo(poselist,mode)
        
        if len(response.trajectories) > 0:
            # global current_joint_angles
            current_joint_angles=response.trajectories[-1].joint_trajectory.points[-1].positions
            print(current_joint_angles)
            
            print ("in mode 8")
            print(response.poses[-1])
    
    except rospy.ServiceException as e:
        print ("Service call failed")
def mode_9():
    

    global current_joint_angles
    mode=9

    rospy.init_node('mover_client')
    rospy.wait_for_service('xarm_mover')

    pose1=copy.deepcopy(pose0)
    pose1.position.x=0.0
    pose1.position.y=-0.3
    pose1.position.z=0.45

    # pose2=copy.deepcopy(pose0)
    # pose2.position.y +=0.3
    poselist=PoseList()
    poselist.pose_list=[pose1]
    poselist.joints=current_joint_angles


    try:
        moverinfo = rospy.ServiceProxy('xarm_mover', TrajectoryService)

		# 请求服务调用，输入请求数据
        response = moverinfo(poselist,mode)
        
        if len(response.trajectories) > 0:
            # global current_joint_angles
            current_joint_angles=response.trajectories[-1].joint_trajectory.points[-1].positions
            print(current_joint_angles)
            
            print ("in mode 9")
            print(response.poses[-1])
    
    except rospy.ServiceException as e:
        print ("Service call failed")
def mode(mode):
    

    global current_joint_angles
    

    rospy.init_node('mover_client')
    rospy.wait_for_service('xarm_mover')

    # pose1=copy.deepcopy(pose0)
    # pose1.position.x=-0.08
    # pose1.position.y=-0.34
    # pose1.position.z=0.18

    # pose2=copy.deepcopy(pose0)
    # pose2.position.x=-0.08
    # pose2.position.y=-0.34
    # pose2.position.z=-0.016

    pose1=copy.deepcopy(pose0)
    pose1.position.x=0.0
    pose1.position.y=-0.3
    pose1.position.z=0.5

    pose2=copy.deepcopy(pose0)
    pose2.position.x=0.0
    pose2.position.y=0.0
    pose2.position.z=0.0

    # pose2=copy.deepcopy(pose0)
    # pose2.position.y +=0.3
    poselist=PoseList()
    poselist.pose_list=[pose1]
    poselist.joints=current_joint_angles


    try:
        moverinfo = rospy.ServiceProxy('xarm_mover', TrajectoryService)

		# 请求服务调用，输入请求数据
        response = moverinfo(poselist,mode)
        
        if len(response.trajectories) > 0:
            # global current_joint_angles
            current_joint_angles=response.trajectories[-1].joint_trajectory.points[-1].positions
            print(current_joint_angles)
            
            print ("in mode 15")
            print(response.poses[-1])
    
    except rospy.ServiceException as e:
        print ("Service call failed")

def read_global():
    print(current_joint_angles)
def detection_client():
    print("test detection")
    rospy.init_node('mover_client')

    rospy.wait_for_service('detect_sam')
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
    posesend=copy.deepcopy(pose0)
    posesend.position.x=-0.0
    posesend.position.y=-0.3
    posesend.position.z=0.45


    try:
        moverinfo = rospy.ServiceProxy('detect_sam', DetectionService)

        detections=DetectionServiceRequest()
        detections.depth_camera_info=depth_caminfo
        detections.rgb_camera_info=color_caminfo
        detections.depth=depthimage
        detections.rgb=color_iamge
        detections.detect_mode=2
        detections.pose=posesend
        detections.prompt="box"
		# 请求服务调用，输入请求数据
        response = moverinfo(detections)

        print(response.detect_number)
        print(response.poses)
    
    except rospy.ServiceException as e:
        print ("Service call failed")
def opendict(dict_path):
    print("load dict")
    with open(dict_path,"rb") as file:
        loaddict=pickle.load(file)

    return loaddict
def mode_dict():
    

    global current_joint_angles
    mode=5
    
    rospy.init_node('mover_client')
    rospy.wait_for_service('xarm_mover')

    print("start read the dict")
    stored_dict_path="./src/FastSAM/output/demo_grasp.pkl"
    loaddict=opendict(dict_path=stored_dict_path)
    waypoints=loaddict["waypoints"]
    print(waypoints)
    pose1=copy.deepcopy(pose0)
    pose1.position.x =-0.0
    pose1.position.y =-0.3
    pose1.position.z=0.45
    plist=[]
    plist.append(pose1)
    for i in range(len(waypoints)):
    
        point=waypoints[i]

        pose2=copy.deepcopy(pose0)
        pose2.position.x =point[0]
        pose2.position.y =point[1]
        pose2.position.z=point[2]
        pose2.orientation.x=point[3]
        pose2.orientation.y=point[4]
        pose2.orientation.z=point[5]
        pose2.orientation.w=point[6]
        plist.append(pose2)
    

    

    poselist=PoseList()
    poselist.pose_list=plist
    poselist.joints=current_joint_angles


    try:
        moverinfo = rospy.ServiceProxy('xarm_mover', TrajectoryService)

		# 请求服务调用，输入请求数据
        response = moverinfo(poselist,mode)
        
        if len(response.trajectories) > 0:
            # global current_joint_angles
            current_joint_angles=response.trajectories[-1].joint_trajectory.points[-1].positions
            print(current_joint_angles)
            
            print ("in mode 5")
            print(response.poses[-1])
    
    except rospy.ServiceException as e:
        print ("Service call failed")

def mover_client():
    # ROS节点初始化
    rospy.init_node('mover_client')

    # 发现/spawn服务后，创建一个服务客户端，连接名为/spawn的service
    rospy.wait_for_service('xarm_mover')
    mode=0


    pose1 = Pose()
    pose1.position.x = 0.2
    pose1.position.y = 0.0
    pose1.position.z = 0.0
    pose1.orientation.x = 1.0
    pose1.orientation.y = 0.0
    pose1.orientation.z = 0.0
    pose1.orientation.w = 0.0

    pose2=copy.deepcopy(pose1)
    pose2.position.x +=0.1
    pose2.orientation.x = 1.0
    pose2.orientation.y = 0.1
    pose2.orientation.z = 0.0
    pose2.orientation.w = 0.0


    pose3=copy.deepcopy(pose2)
    pose3.position.y +=0.1
    pose3.orientation.x = 1.0
    pose3.orientation.y = 0.0
    pose3.orientation.z = 0.0
    pose3.orientation.w = 0.2

    joint_angles = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]



    poselist=PoseList()
    poselist.pose_list=[pose1,pose2]
    poselist.joints=joint_angles




    try:
        moverinfo = rospy.ServiceProxy('xarm_mover', TrajectoryService)

		# 请求服务调用，输入请求数据
        response = moverinfo(poselist,mode)
    
    except rospy.ServiceException as e:
        print ("Service call failed")




    
if __name__ == "__main__":
	#服务调用并显示调用结果
    #mover_client()
    mode_0()
    for i in range(3):
        

        # mode_3()
        mode_8()
    mode(14)

    # mode_0()
    # mode_dict()
    # mode_1()
    # mode(14)

    
    
    
    # mode_3()
    
    # # mode_2()

    # mode_4()
    # mode_5()
    # mode_6()
    # mode_1()
    # mode_7()

    # mode_3()
    # mode_4()
    # mode_5()
    # mode_4()
    # mode_6()
    # mode_1()
    # mode_7()
    # mode_4()
    # mode_5()
    # mode_4()
    # mode_1()
    # mode_8()

    # detection_client()

    # mode_3()
    # read_global()
    
    
    
