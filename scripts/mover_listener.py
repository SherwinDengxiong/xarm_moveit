import rospy
import time
import threading

from sensor_msgs.msg import Image,CameraInfo
from xarm_moveit.msg import RGBD,Extrinsics
from cv_bridge import CvBridge



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
            self.subscriber=rospy.Subscriber('/camera/depth/image_rect_raw',Image, self.colorcallback,queue_size=1)
            self.image=None
            
        elif mode==2:
            # color camera info
            self.subscriber=rospy.Subscriber('/camera/color/camera_info',CameraInfo, self.camerainfo_callback,queue_size=1)
            
        elif mode==3:
            # depth camera info
            self.subscriber=rospy.Subscriber('/camera/depth/camera_info',CameraInfo, self.camerainfo_callback,queue_size=1)
            
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
        
    

    
def color_subscriber():
    rospy.init_node('color_subcriber',anonymous=True)

    # sub=rospy.Subscriber('/camera/color/image_raw',Image, colorcallback,queue_size=1)
    # sub.unregister()
    # extrinsic_sub=Subscriber(4)
    # extrinsic_sub.unregister_event.wait()
    # current_rotation=extrinsic_sub.get_rotation()
    # print(current_rotation)

    depth_caminfo_sub=Subscriber(3)
    depth_caminfo_sub.unregister_event.wait()
    current_caminfo=depth_caminfo_sub.get_msg()
    print(current_caminfo)

    color_caminfo_sub=Subscriber(2)
    color_caminfo_sub.unregister_event.wait()
    color_caminfo=color_caminfo_sub.get_msg()
    print(color_caminfo)

    colorimage_sub=Subscriber(0)
    colorimage_sub.unregister_event.wait()
    current_iamge=colorimage_sub.get_msg()
    # print(current_iamge)

    depthimage_sub=Subscriber(1)
    depthimage_sub.unregister_event.wait()
    depthimage=depthimage_sub.get_msg()
    # print(depthimage)

    rospy.spin()
    

if __name__ == '__main__':
    color_subscriber()