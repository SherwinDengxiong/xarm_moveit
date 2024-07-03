import sys
import copy
import rospy
from geometry_msgs.msg import Pose, Point, Quaternion
import threading
from cv_bridge import CvBridge
import argparse

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
    pose1.position.z=0.45

    # pose2=copy.deepcopy(pose0)
    # pose2.position.x=0.0
    # pose2.position.y=0.3
    # pose2.position.z=0.5

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
            
            print ("in mode {}".format(mode))
            print(response.poses[-1])
    
    except rospy.ServiceException as e:
        print ("Service call failed")
def main(args):
    
    repeat_time=args.repeat
    
    mode(0)
    for i in range(repeat_time):

        
        mode(8)
    mode(14)


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--repeat", type=int, default=10, help="repeat time"
    )
    parser.add_argument(
        "--img_path", type=str, default="./images/image1.jpg", help="path to image file"
    )
   
    return parser.parse_args()
if __name__ == "__main__":
    args = parse_args()
    main(args)
