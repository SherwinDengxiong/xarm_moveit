import rospy

import sys
import copy
import math
import moveit_commander
import subprocess
import pickle
import os
import csv
import tf



# import pyrealsense2 as rs
import numpy as np
import cv2
from cv_bridge import CvBridge

# from test_pkg import FastSAM, FastSAMPrompt
# import torch 

from sensor_msgs.msg import JointState, Image
from geometry_msgs.msg import Quaternion, Pose

from xarm_moveit.srv import MoverService, MoverServiceRequest, MoverServiceResponse
from xarm_moveit.srv import TrajectoryService, TrajectoryServiceRequest, TrajectoryServiceResponse
from xarm_moveit.srv import DetectionService,DetectionServiceRequest,DetectionServiceResponse

class detection_solver:
    def __init__(self,boxes,masks,height,width) :
        self.boxes=boxes
        self.masks=masks
        self.height=height
        self.width=width
        self.center=[]
        self.maskarray=[]
        self.calculate_geometry_center(self.boxes)
        self.calculate_mask(self.masks)

    def calculate_geometry_center(self,boxes):
        center=[]
        for i in range(len(boxes)):
            info=boxes[i]
            centerpoint=[info[0],info[1]]
            center.append(centerpoint)
        self.center=center
    def calculate_mask(self,masks):
        maskarray=[]

        for mask in masks:
            masklist=[]
            for i in range(len(mask)):
                point=[int(mask[i][0]),int(mask[i][1])]
                masklist.append(point)

            region_points =np.array(masklist, np.int32)
            
            region_points = region_points.reshape((-1, 1, 2))
            image_mask = np.zeros((self.height,self.width),dtype=np.uint8)
            cv2.fillPoly(image_mask, [region_points], color=255)  

            maskarray.append(image_mask) 
             

        self.maskarray=maskarray
    def get_mask_center(self,index):
        onemask=self.maskarray[index]
        white_pixel_indices=cv2.findNonZero(onemask)
        centroid=np.mean(white_pixel_indices,axis=0)

        return centroid
    
    def get_rectangle(self,index):
        onemask=self.maskarray[index]
        contours,_ = cv2.findContours(onemask,cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        rect = cv2.minAreaRect(contours[0])

        return rect

distance_th=0.17
distance_max_th=0.5
height_th=-0.014
height_max_th=0.5
base_limit_distance=0.2
x_adjustment=0.1
y_adjustment=-0.025
z_adjustment=0.119

      
def transform_robot_3D(xyzw,transitxyz,endxyzw,endxyz,camerapoint):
    
    #xyzw
    rotation_matrix = tf.transformations.quaternion_matrix(xyzw)

    # transit
    
    
    rotation_matrix[:3, 3] = transitxyz
    



    camera_point=np.append(camerapoint, 1)

    robot_point = np.dot(np.linalg.inv(rotation_matrix), camera_point)
    

    endping_matrix=tf.transformations.quaternion_matrix(endxyzw)
    endping_matrix[:3,3]=endxyz
    

    world_point= np.dot(np.linalg.inv(endping_matrix), robot_point)

    return world_point
def find_depth_value(x_pixel,y_pixel,height,width,depth_map):
    if 0<y_pixel<height and 0<x_pixel<width:
        depth_value = depth_map[y_pixel][x_pixel]/1000
        
    else:
        depth_value=0
    return depth_value
def transform_camera_3D(x_pixel,y_pixel,colork,depth_value):

    fx=colork[0][0]
    fy=colork[1][1]
    cx=colork[0][2]
    cy=colork[1][2]
    

    
    # print("depth value is {}".format(depth_value))
    
    camera_x = (x_pixel - cx) * depth_value / fx
    camera_y = (y_pixel - cy) * depth_value / fy
    camera_z = depth_value

    camera_point = np.array([camera_x, camera_y, camera_z])

    return camera_point

def prepare_pose_data(pointlist):
    
    poselist=[]
    for point in pointlist:
        
        pose1 = Pose()
        pose1.position.x = point[0]
        pose1.position.y = point[1]
        pose1.position.z = point[2]
        pose1.orientation.x = 1.0
        pose1.orientation.y = 0.0
        pose1.orientation.z = 0.0
        pose1.orientation.w = 0.0
        poselist.append(pose1)


    return poselist

def opendict(dict_path):
    print("load dict")
    with open(dict_path,"rb") as file:
        loaddict=pickle.load(file)

    return loaddict
def save_image(rgb,path):
    if len(rgb)==0:
        rospy.loginfo("image not find , use existing image")

    else:
        cv2.imwrite(path,rgb)
        rospy.loginfo("image saved at{}".format(path))
def read_robot_transform(file_path):
    info=[]
    with open(file_path,'r') as file:
        reader=csv.reader(file)
        
        for row in reader:
            info.append(row[0])

    print(info)
    qw=float(info[0])
    qx=float(info[1])
    qy=float(info[2])
    qz=float(info[3])
    x=float(info[4])
    y=float(info[5])
    z=float(info[6])

    xyzw=np.array([qx,qy,qz,qw])
    xyz=np.array([x,y,z])

    return xyzw,xyz



def call_inference(rgb,script_path,model_path,image_path,image_name,output_path):
    
    model_arg="--model_path"
    image_arg="--img_path"
    dict_arg="--dict_name"
    dictname=image_name[:-4]
    out_arg="--output"
    
    
    saved_image_path=os.path.join(image_path,image_name)
    print(saved_image_path)
    print(dictname)
    save_image(rgb,saved_image_path)

    

    subprocess.run(["python3", script_path, model_arg,model_path,image_arg,saved_image_path,dict_arg,dictname,out_arg,output_path],check=True)

    stored_dict_path=os.path.join(output_path,dictname+".pkl")
    print("opening the dict-----------")
    loaddict=opendict(dict_path=stored_dict_path)
    boxes=loaddict["boxes"]
    masks=loaddict["masks"]
    rectlist=loaddict["rectlist"]
    top5=loaddict["top5"]
    top5conf=loaddict["t5conf"]
    pointlist=loaddict["pointlist"]
    pointconflist=loaddict["pointconflist"]

    return boxes,masks,rectlist,top5,top5conf,pointlist,pointconflist
def demo_inference(script_path,model_path,image_path,image_name,output_path):
    model_arg="--model_path"
    info_arg="--sample_path"
    dict_arg="--dict_name"
    dictname=image_name[:-4]
    out_arg="--output"
    mode_arg="--mode"

    info_path=os.path.join(image_path,dictname+".pkl")
    print("start inference at mode 2")
    print(model_path)
    print(info_path)
    print(output_path)
    print(dictname)

    mode=str(1)

    subprocess.run(["python3", script_path, mode_arg,mode,model_arg,model_path,info_arg,info_path,out_arg,output_path,dict_arg,dictname],check=True)
    stored_dict_path=os.path.join(output_path,dictname+".pkl")
    loaddict=opendict(dict_path=stored_dict_path)
    waypoints=loaddict["waypoints"]
    
    return waypoints


def get_msgs(colorimage,depthimage,colorinfo,depthinfo,pose):

    # print(colorimage.data)
    if len(colorimage.data)==0:
        cv_image=[]
    else:
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(colorimage, desired_encoding='passthrough')
        depth_map=bridge.imgmsg_to_cv2(depthimage, desired_encoding='passthrough')

        
        depth_intrinsic=list(depthinfo.K)
        color_intrinsic=list(colorinfo.K)
        color_extrinsic=list(colorinfo.P)
        depth_intrinsic=np.array(depth_intrinsic).reshape(3,3)
        color_intrinsic=np.array(color_intrinsic).reshape(3,3)
        endpoint_pose=pose
        # (431.45794677734375, 0.0, 423.0338134765625, 0.0, 431.45794677734375, 240.1116485595703, 0.0, 0.0, 1.0)
        # (911.9283447265625, 0.0, 636.184326171875, 0.0, 912.0912475585938, 389.94366455078125, 0.0, 0.0, 1.0)

        print("depth intrinsic is {}".format(depth_intrinsic))
        print("color intrinsic is {}".format(color_intrinsic))
        

    
    return cv_image,depth_map, depth_intrinsic,color_intrinsic, endpoint_pose

def find_grasp_pose(x_pixel,y_pixel,height,width,colorK,depth_map,quaternion,translation,pose):
    
    depth_value=find_depth_value(x_pixel,y_pixel,height,width,depth_map)
    camera_point=transform_camera_3D(x_pixel=x_pixel,y_pixel=y_pixel,colork=colorK,depth_value=depth_value)
    
    
    endpoint_r=np.array([pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w])
    endpoint_t=np.array([pose.position.x,-pose.position.y,pose.position.z])
    robotpoint=transform_robot_3D(quaternion,translation,endpoint_r,endpoint_t,camera_point)[:3]
   
    robotpoint[0]=-robotpoint[0]+x_adjustment
    robotpoint[1]=-robotpoint[1]+y_adjustment
    robotpoint[2]=robotpoint[2]+z_adjustment
    if depth_value==0:
        robotpoint[2]==-100
    
    

    return robotpoint

def select_keypoint(top5,top5conf,points,pointconfs,label_dict):
    keywords=["cup","mug","bowl"]
    label_list=[]
    max_index=np.argmax(pointconfs)
    label=top5[0]
    point=points[max_index]
    for i in range(len(top5)):
        label=label_dict[top5[i]]

        label_list.append(label)
    for label in label_list:
        contain_keywords=any(keyword in label for keyword in keywords)
        if contain_keywords:
            largest_index=0
            second_index=-1
            for i in range(len(pointconfs)):
                if pointconfs[i]>pointconfs[largest_index]:
                    second_index=largest_index
                    largest_index=1
                elif pointconfs[largest_index]>pointconfs[i]>pointconfs[second_index]:
                    second_index=i
            point=points[second_index]
        else:
            continue

    return point,label,label_list

def select_objects(top5,top5conf,points,pointconfs,label_dict,keywords):

    label_list=[]
    max_index=np.argmax(pointconfs)
    label=top5[0]
    point=points[max_index]
    for i in range(len(top5)):
        label=label_dict[top5[i]]

        label_list.append(label)
    
    for label in label_list:
        contain_keywords=any(keyword in label for keyword in keywords)
        if contain_keywords:
            
            return True
    return False
    
def storedict(dict_path,colorimage,depth_map,colorK,current_pose):
    mydict={"image": colorimage,"depthmap":depth_map,"colorinfo":colorK,"pose":current_pose}

    print("dict_dir at {}".format(dict_path))
    

    with open(dict_path,"wb") as file:
        pickle.dump(mydict,file)
    print("store dict")



def detection(req):
    csv_path="./src/FastSAM/weights/robot_matrix.csv"
    label_dict_path="/home/rover/catkin_ws/src/FastSAM/weights/label.pkl"
    script_path = "./src/FastSAM/Inference.py"
    
    model_path="./src/FastSAM/weights/FastSAM-x.pt"
    
    image_path="./src/FastSAM/images"
    image_name="image2.jpg"
    
    
    
    output_path="./src/FastSAM/output/"
    response = DetectionServiceResponse()
    detect_mode=req.detect_mode

    label_dict=opendict(label_dict_path)
    print(label_dict)
    colorimage,depth_map,depthK,colorK,current_pose=get_msgs(req.rgb,req.depth,req.rgb_camera_info,req.depth_camera_info,req.pose)
    keywords=[req.prompt]

    print(colorimage.shape)
    height=colorimage.shape[0]
    width=colorimage.shape[1]
    
    quaternion,translation=read_robot_transform(csv_path)
    print(quaternion)
    print(translation)
    

    
    # mode 0 detect number of objects in image
    # mode 1 auto grasp
    # mode 2 grasp with demo
    
    if detect_mode==0:
        boxes,masks,rectlist,top5,top5conf,pointlist,pointconflist=call_inference(colorimage,script_path,model_path,image_path,image_name,output_path)
        response.detect_number=len(boxes)
        response.rectangle=[0.0,0.0,0.0,0.0,0.0]
        # solver=detection_solver(boxes,masks,height,width)
        # print(pointlist)
        robot_point_list=[]
        for index in range(len(boxes)):
            print(index)
            point,label,t5label=select_keypoint(top5[index],top5conf[index],pointlist[index],pointconflist[index],label_dict)
            x_pixel=point[0]
            y_pixel=point[1]

            
            rec=((rectlist[index][0],rectlist[index][1]),(rectlist[index][2],rectlist[index][3]),rectlist[index][4])
            
            
            
            # correct=select_objects(top5[index],top5conf[index],pointlist[index],pointconflist[index],label_dict,keywords)
            
            robotpoint=find_grasp_pose(x_pixel,y_pixel,height,width,colorK,depth_map,quaternion,translation,current_pose)
            print("robot pose {}".format(robotpoint))
            if height_max_th>robotpoint[2]>height_th:

                distance=math.sqrt(robotpoint[0]**2+robotpoint[1]**2)
                
                if distance_max_th>distance>distance_th and abs(robotpoint[1])>base_limit_distance :
                
                    center=(int(x_pixel),int(y_pixel))
                    rec_points=cv2.boxPoints(rec)
                    rec_points=np.int0(rec_points)
                    print("rectangles are{}".format(rec_points))
                    angle=find_angle(rec_points)
                    color=(0,0,255)
                    cv2.drawContours(colorimage,[rec_points],0,color,2)
                    cv2.circle(colorimage,center,5,color,2)
                    font=cv2.FONT_HERSHEY_SIMPLEX
                    line_length=50
                    x1=int(center[0]-0.5*line_length*np.cos(angle))
                    y1=int(center[1]-0.5*line_length*np.sin(angle))
                    x2=int(center[0]+0.5*line_length*np.cos(angle))
                    y2=int(center[1]+0.5*line_length*np.sin(angle))
                    cv2.line(colorimage,(x1,y1),(x2,y2),(0,255,255),2)
                    
                    
                    # text=
                    # text="angle_"+str(round(math.degrees(angle)))+"_"+str(round(robotpoint[0],2))+"_"+ str(round(robotpoint[1],2))+"_"+ str(round(robotpoint[2],2))
                    # cv2.putText(colorimage,text,center,font,1,color,1)
                    print("objects are {}".format(t5label))
                    print("robot pose is{}".format(robotpoint))
                    robot_point_list.append(robotpoint)
                    rectangle.append(angle)

        
        response.detect_number=len(robot_point_list)
        response.rectangle=rectangle

        

        print("length of robot pose list is {}".format(len(robot_point_list)))
        cv2.imshow("image",colorimage)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
        response.poses=prepare_pose_data(robot_point_list)



    elif detect_mode==1:
        boxes,masks,rectlist,top5,top5conf,pointlist,pointconflist=call_inference(colorimage,script_path,model_path,image_path,image_name,output_path)
        
        
        rectangle=[]

        
        robot_point_list=[]
        for index in range(len(boxes)):
            print(index)
            point,label,t5label=select_keypoint(top5[index],top5conf[index],pointlist[index],pointconflist[index],label_dict)
            x_pixel=point[0]
            y_pixel=point[1]

            
            rec=((rectlist[index][0],rectlist[index][1]),(rectlist[index][2],rectlist[index][3]),rectlist[index][4])
            
            
            
            # correct=select_objects(top5[index],top5conf[index],pointlist[index],pointconflist[index],label_dict,keywords)
            
            robotpoint=find_grasp_pose(x_pixel,y_pixel,height,width,colorK,depth_map,quaternion,translation,current_pose)
            print("robot pose {}".format(robotpoint))
            if height_max_th>robotpoint[2]>height_th:

                distance=math.sqrt(robotpoint[0]**2+robotpoint[1]**2)
                print("distance is {}".format(distance))
                
                if distance_max_th>distance>distance_th and abs(robotpoint[1])>base_limit_distance :
                
                    center=(int(x_pixel),int(y_pixel))
                    rec_points=cv2.boxPoints(rec)
                    rec_points=np.int0(rec_points)
                    print("rectangles are{}".format(rec_points))
                    angle=find_angle(rec_points)
                    color=(0,0,255)
                    cv2.drawContours(colorimage,[rec_points],0,color,2)
                    cv2.circle(colorimage,center,5,color,2)
                    font=cv2.FONT_HERSHEY_SIMPLEX
                    
                    
                    text="angle_"+str(round(math.degrees(angle)))+"_"+str(round(robotpoint[0],2))+"_"+ str(round(robotpoint[1],2))+"_"+ str(round(robotpoint[2],2))
                    cv2.putText(colorimage,text,center,font,1,color,1)
                    print("objects are {}".format(t5label))
                    print("robot pose is{}".format(robotpoint))
                    robot_point_list.append(robotpoint)
                    rectangle.append(angle)

        response.detect_number=len(robot_point_list)
        response.rectangle=rectangle


        print("robot pose list is {}".format(robot_point_list))
        cv2.imshow("image",colorimage)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
        response.poses=prepare_pose_data(robot_point_list)

    elif detect_mode==2:
        # grasp after demo
        image_path="./src/FastSAM/images"
        dict_path=os.path.join(image_path,image_name[:-4]+".pkl")
        print("storing dict path demo grasp.pkl")
        storedict(dict_path,colorimage,depth_map,colorK,current_pose)
        waypoints=demo_inference(script_path,model_path,image_path,image_name,output_path)
        robot_pose=[]
        for i in range(len(waypoints)):
            point=waypoints[i]
            pose=Pose()
            pose.position.x=point[0]
            pose.position.y=point[1]
            pose.position.z=point[2]
            pose.orientation.x=point[3]
            pose.orientation.y=point[4]
            pose.orientation.z=point[5]
            pose.orientation.w=point[6]
            robot_pose.append(pose)
        print(robot_pose)


        rectangle=[]

        
        
        response.detect_number=1
        print(response.detect_number)
        response.rectangle=rectangle
        response.poses=robot_pose


    
    
    response.responseindex=0


    rospy.loginfo("detection complete")
    return response
def find_angle(points):
    point1=points[0]
    point2=points[1]
    point3=points[2]

    length12=math.sqrt((point1[0]-point2[0])**2+(point1[1]-point2[1])**2)
    length23=math.sqrt((point2[0]-point3[0])**2+(point2[1]-point3[1])**2)
    angle=0
    if length12<=length23:
        firstpoint=point1
        secondpoint=point2
        if secondpoint[1]-firstpoint[1]==0:
            angle=math.pi/2
        elif secondpoint[1]-firstpoint[1]==0:
            angle=0
        else:
            slope=(secondpoint[0]-firstpoint[0])/(secondpoint[1]-firstpoint[1])
            angle=math.pi/2+math.atan(slope)
    else:
        firstpoint=point2
        secondpoint=point3
        if secondpoint[1]-firstpoint[1]==0:
            angle=math.pi/2
        elif secondpoint[1]-firstpoint[1]==0:
            angle=0
        else:
            slope=(secondpoint[0]-firstpoint[0])/(secondpoint[1]-firstpoint[1])
            angle=math.pi/2+math.atan(slope)

    return angle


def detect_sam():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('detection_server')
    rospy.loginfo("Starting sam detection_server!!!!")
  

    
    s = rospy.Service('detect_sam', DetectionService, detection)
    
    rospy.spin()


if __name__ == "__main__":
    detect_sam()