#!/usr/bin/env python

import rospy
import tf
import pygame
import math
from xarm_moveit.msg import VehicleControl
from geometry_msgs.msg import Twist, Pose


def UGV_controller(velocity_pub,gimble,rate):
    

    ################################# LOAD UP A BASIC WINDOW #################################
    
    running = True
    # player = pygame.Rect(DISPLAY_W/2, DISPLAY_H/2, 60,60)
    LEFT, RIGHT, UP, DOWN = False, False, False, False

    FORWARD,BACKWARD,TURN_L,TURN_R=False, False, False, False
    pitch=0
    yaw=0

 
    ###########################################################################################

    #Initialize controller
    joysticks = []
    for i in range(pygame.joystick.get_count()):
        joysticks.append(pygame.joystick.Joystick(i))
    for joystick in joysticks:
        joystick.init()
        print(joystick.get_numaxes())

    # with open(os.path.join("ps4_keys.json"), 'r+') as file:
    #     button_keys = json.load(file)
    # 0: Left analog horizonal, 1: Left Analog Vertical, 3: Right Analog Horizontal
    # 4: Right Analog Vertical 2: Left Trigger, 5: Right Trigger
    analog_keys = {0:0, 1:0, 2:-1, 3:0, 4:0, 5: -1 }

    # START OF GAME LOOP
    while running:
        linearx=0
        angularz=0
        ################################# CHECK PLAYER INPUT #################################
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

            if event.type == pygame.KEYDOWN:
                ############### UPDATE SPRITE IF SPACE IS PRESSED #################################
                print(event)
                if event.key ==pygame.K_i:
                    UP=True
                    
                if event.key ==pygame.K_k:
                    DOWN=True
                    
                if event.key ==pygame.K_j:
                    LEFT=True
                    
                if event.key ==pygame.K_l:
                    RIGHT=True

                if event.key ==pygame.K_w:
                    FORWARD=True
                    
                if event.key ==pygame.K_s:
                    BACKWARD=True
                    
                if event.key ==pygame.K_a:
                    TURN_L=True
                    
                if event.key ==pygame.K_d:
                    TURN_R=True
                    
                
            if event.type == pygame.KEYUP:
                
                if event.key ==pygame.K_i:
                    UP=False
                if event.key ==pygame.K_k:
                    DOWN=False
                if event.key ==pygame.K_j:
                    LEFT=False
                if event.key ==pygame.K_l:
                    RIGHT=False
                if event.key ==pygame.K_w:
                    FORWARD=False
                    
                if event.key ==pygame.K_s:
                    BACKWARD=False
                    
                if event.key ==pygame.K_a:
                    TURN_L=False
                    
                if event.key ==pygame.K_d:
                    TURN_R=False

                
                

            

            #HANDLES ANALOG INPUTS
            # if event.type == pygame.JOYAXISMOTION:
            #     analog_keys[event.axis] = event.value
            #     # print("event value is {}".format(analog_keys))
            #     # Horizontal Analog
            #     if abs(analog_keys[0]) > .4:
            #         print(analog_keys)
            #         # print(event)
            #         if analog_keys[0] < -.7:
                        
            #             LEFT=True
                        
            #         else:
            #             LEFT = False
            #         if analog_keys[0] > .7:
            #             RIGHT = True
                        
            #         else:
            #             RIGHT = False
            #     # Vertical Analog
            #     if abs(analog_keys[4]) > .4:
            #         if analog_keys[4] < -.7:
            #             UP=True
                        
            #         else:
            #             UP = False
            #         if analog_keys[4] > .7:
                        
            #             DOWN=True
                        
            #         else:
            #             DOWN = False


            #     if abs(analog_keys[3]) > .1:
            #         # print(analog_keys)
            #         # print(event)
            #         if analog_keys[3] < -.2:
                        
            #             TURN_L=True
                        
            #         else:
            #             TURN_L = False
            #         if analog_keys[3] > .2:
            #             TURN_R = True
                        
            #         else:
            #             TURN_R = False
            #     # Vertical Analog
            #     if abs(analog_keys[1]) > .1:
            #         if analog_keys[1] < -.2:
            #             FORWARD=True
                        
            #         else:
            #             FORWARD= False
            #         if analog_keys[1] > .2:
                        
            #             BACKWARD=True
                        
            #         else:
            #             BACKWARD = False
                #     # Triggers
                # if analog_keys[2] > 0:  # Left trigger
                #     color += 2
                # if analog_keys[5] > 0:  # Right Trigger
                #     color -= 2
        
        # Handle Player movement
        if LEFT:
            # print("running")
            if math.pi*3/2-0.1>=yaw>=-math.pi*3/2:
                yaw-=0.1
                print('left')
                
            else:
                print("cannot left further")
        if RIGHT:
            if math.pi*3/2>=yaw>=-math.pi*3/2+0.1:
                yaw+=0.1
                print('right')
            else:
                print("cannot right further")
        if UP:
            if math.pi/2-0.1>=pitch>=-math.pi/2:
                pitch+=0.1
                print("up")
            else:
                print("cannot up further")
        if DOWN:
            if math.pi/2>=pitch>=-math.pi/2+0.1:
                pitch-=0.1
                print("down")
            else:
                print("cannot down further")
        if FORWARD:
            
            index=1
            if event.type == pygame.JOYAXISMOTION:
                print(event.value)
                analog_keys[event.axis] = event.value
                index=analog_keys[0]

            linearx=1*index
        if BACKWARD:
            index=-1
            if event.type == pygame.JOYAXISMOTION:
                analog_keys[event.axis] = event.value
                index=analog_keys[0]

            linearx=1*index
        if TURN_L:
            index=1
            if event.type == pygame.JOYAXISMOTION:
                analog_keys[event.axis] = event.value
                index=analog_keys[1]

            angularz=1*index
            
        if TURN_R:
            index=-1
            if event.type == pygame.JOYAXISMOTION:
                analog_keys[event.axis] = event.value
                index=analog_keys[1]

            angularz=1*index
        
        # print("moving {}, {}".format(linearx,angularz))

        
        twist_msg = Twist()
        twist_msg.linear.x=linearx*-0.1
        twist_msg.angular.z=angularz*0.5

        rotation=[0.0,pitch,yaw]
        
        rotation_msg=Pose()
        orientation=tf.transformations.quaternion_from_euler(rotation[0],rotation[1],rotation[2])

        rotation_msg.position.x=0
        rotation_msg.position.y=0
        rotation_msg.position.z=0
        rotation_msg.orientation.x = orientation[0]
        rotation_msg.orientation.y = orientation[1]
        rotation_msg.orientation.z = orientation[2]
        rotation_msg.orientation.w = orientation[3]

        gimble.publish(rotation_msg)
        # msg_sed=VehicleControl()
        # msg_sed.twist_value=twist_msg
        # msg_sed.head_pose=rotation_msg


        # Publish the velocity command
        velocity_pub.publish(twist_msg)
        print("send successfull linear {} angular {} pitch {} yaw{}".format(round(linearx,2),round(angularz,2),round(pitch,2),round(yaw,2)))

        rate.sleep()


def robot_waypoint_control():
    rospy.wait_for_service('xarm_mover')

    running = True
    # player = pygame.Rect(DISPLAY_W/2, DISPLAY_H/2, 60,60)
    X_p,X_m, Y_p, Y_m, Z_p, Z_m=False,False,False,False,False,False
    R_p,R_m,P_p,P_m,yaw_p,yaw_m=False,False,False,False,False,False
    roll=0
    pit=0
    Yaw=0
    roll_limit=[-0.5,0.5]
    pit_limit=[-0.5,0.5]
    yaw_limit=[-math.pi,math.pi]
    rpy_iter=0.05

    x=0.2
    y=0
    z=0
    x_limit=[0.2,0.5]
    y_limit=[-0.5,0.5]
    z_limit=[0,0.5]
    xyz_iter=0.01

    while running:
        
        ################################# CHECK PLAYER INPUT #################################
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

            if event.type == pygame.KEYDOWN:
                ############### UPDATE SPRITE IF SPACE IS PRESSED #################################
                if event.key ==pygame.K_t:
                    X_p=True
                    
                if event.key ==pygame.K_g:
                    X_m=True
                    
                if event.key ==pygame.K_h:
                    Y_p=True
                    
                if event.key ==pygame.K_f:
                    Y_m=True

                if event.key ==pygame.K_y:
                    Z_p=True
                    
                if event.key ==pygame.K_r:
                    Z_m=True
                    
                if event.key ==pygame.K_1:
                    R_m=True
                    
                if event.key ==pygame.K_2:
                    R_p=True
                if event.key ==pygame.K_3:
                    P_m=True
                    
                if event.key ==pygame.K_4:
                    P_p=True
                if event.key ==pygame.K_5:
                    yaw_m=True
                    
                if event.key ==pygame.K_6:
                    yaw_p=True
                    
                
            if event.type == pygame.KEYUP:
                
                if event.key ==pygame.K_t:
                    X_p=False
                    
                if event.key ==pygame.K_g:
                    X_m=False
                    
                if event.key ==pygame.K_h:
                    Y_p=False
                    
                if event.key ==pygame.K_f:
                    Y_m=False

                if event.key ==pygame.K_y:
                    Z_p=False
                    
                if event.key ==pygame.K_r:
                    Z_m=False
                    
                if event.key ==pygame.K_1:
                    R_m=False
                    
                if event.key ==pygame.K_2:
                    R_p=False
                if event.key ==pygame.K_3:
                    P_m=False
                    
                if event.key ==pygame.K_4:
                    P_p=False
                if event.key ==pygame.K_5:
                    yaw_m=False
                    
                if event.key ==pygame.K_6:
                    yaw_p=False

            if X_m:
                # print("running")
                if x_limit[1]>=x>=x_limit[0]+xyz_iter:
                    x-=xyz_iter
                    print('x minus {}'.format(xyz_iter))
                    
                else:
                    print("cannot minus x further")
            if X_p:
                if x_limit[1]-xyz_iter>=x>=x_limit[0]:
                    x+=xyz_iter
                    print('x plus {}'.format(xyz_iter))
                    
                else:
                    print("cannot plus x further")
            if Y_m:
                if y_limit[1]>=y>=y_limit[0]+xyz_iter:
                    y-=xyz_iter
                    print('y minus {}'.format(xyz_iter))
                    
                else:
                    print("cannot minus y further")
            if Y_p:
                if y_limit[1]-xyz_iter>=y>=y_limit[0]:
                    y+=xyz_iter
                    print('y plus {}'.format(xyz_iter))
                    
                else:
                    print("cannot plus y further")
            if Z_m:
                
                if z_limit[1]>=z>=z_limit[0]+xyz_iter:
                    z-=xyz_iter
                    print('z minus {}'.format(xyz_iter))
                    
                else:
                    print("cannot minus z further")
            if Z_p:
                if z_limit[1]-xyz_iter>=z>=z_limit[0]:
                    z+=xyz_iter
                    print('z plus {}'.format(xyz_iter))
                    
                else:
                    print("cannot plus z further")

            if R_m:
                # print("running")
                if roll_limit[1]>=roll>=roll_limit[0]+rpy_iter:
                    roll-=rpy_iter
                    print('roll minus {}'.format(rpy_iter))
                    
                else:
                    print("cannot minus roll further")
            if R_p:
                if roll_limit[1]-rpy_iter>=roll>=roll_limit[0]:
                    roll+=rpy_iter
                    print('roll plus {}'.format(rpy_iter))
                    
                else:
                    print("cannot plus roll further")
            if P_m:
                if pit_limit[1]>=pit>=pit_limit[0]+rpy_iter:
                    pit-=rpy_iter
                    print('pitch minus {}'.format(rpy_iter))
                    
                else:
                    print("cannot minus pitch further")
            if P_p:
                if pit_limit[1]-0.05>=pit>=pit_limit[0]:
                    pit+=rpy_iter
                    print('pitch plus {}'.format(rpy_iter))
                    
                else:
                    print("cannot plus pitch further")
            if yaw_m:
                
                if yaw_limit[1]>=Yaw>=yaw_limit[0]+0.05:
                    Yaw-=rpy_iter
                    print('yaw minus {}'.format(rpy_iter))
                    
                else:
                    print("cannot minus yaw further")
            if yaw_p:
                if yaw_limit[1]-0.05>=Yaw>=yaw_limit[0]:
                    Yaw+=rpy_iter
                    print('yaw plus {}'.format(rpy_iter))
                    
                else:
                    print("cannot plus yaw further")
    
def main():
    # Initialize the ROS node
    rospy.init_node('vehicle_msg_publisher')

    # Create a publisher for the turtle's velocity commands
    velocity_pub = rospy.Publisher('/managed/joy', Twist, queue_size=10)
    Gimble_pub = rospy.Publisher('/turtle1/cmd_vel', Pose, queue_size=10)

    # Set the rate at which to publish the velocity commands
    rate = rospy.Rate(20)  # 1 Hz

    pygame.init()
    DISPLAY_W, DISPLAY_H = 100, 100
    # canvas = pygame.Surface((DISPLAY_W,DISPLAY_H))
    window = pygame.display.set_mode(((DISPLAY_W,DISPLAY_H)))

    UGV_controller(velocity_pub,Gimble_pub,rate)

    # # # Create a Twist message object to store the velocity commands
    # twist_msg = Twist()

    # pygame.init()

    # rotation=[0.0,0.0,0.0]

    # while not rospy.is_shutdown():

    #     for event in pygame.event.get():
    #         if event.type ==pygame.QUIT:
                
    #             exit()
    #     # Get the user input
    #     cmd = input("Enter a command (w = forward, s = backward, a = left, d = right, q = quit): ")

        

    #     # Check the user input and set the appropriate velocity command
    #     if cmd == 'w':
    #         twist_msg.linear.x = 1  # Move forward
    #     elif cmd == 's':
    #         twist_msg.linear.x = -1  # Move backward
    #     elif cmd == 'a':
    #         twist_msg.angular.z = 0.9  # Rotate left
    #     elif cmd == 'd':
    #         twist_msg.angular.z = -0.9  # Rotate right

    #     elif cmd == 'j':
    #         rotation[2] += -0.2  # gimble yaw right
    #     elif cmd == 'l':
    #         rotation[2] += 0.2 # gimble yaw left
    #     elif cmd == 'i':
    #         rotation[1] += 0.1  # gimble yaw left
    #     elif cmd == 'k':
    #         rotation[1] += -0.1 # gimble yaw right



    #     elif cmd == 'q':
    #         break  # Quit the teleoperation
    #     else:
    #         print("Invalid command!")

    #     rotation_msg=Pose()
        
     



    #     orientation=tf.transformations.quaternion_from_euler(rotation[0],rotation[1],rotation[2])

    #     rotation_msg.position.x=0
    #     rotation_msg.position.y=0
    #     rotation_msg.position.z=0
    #     rotation_msg.orientation.x = orientation[0]
    #     rotation_msg.orientation.y = orientation[1]
    #     rotation_msg.orientation.z = orientation[2]
    #     rotation_msg.orientation.w = orientation[3]

        

    #     msg_sed=VehicleControl()
    #     msg_sed.twist_value=twist_msg
    #     msg_sed.head_pose=rotation_msg
        


    #     # Publish the velocity command
    #     velocity_pub.publish(msg_sed)

    #     # Reset the velocity commands
    #     # twist_msg.linear.x = 0.0
    #     twist_msg.angular.z = 0.0

    #     # Sleep to maintain the desired publishing rate
    #     rate.sleep()

if __name__ == '__main__':
    # try:
    #     turtle_teleop()
    # except rospy.ROSInterruptException:
    #     pass
    main()