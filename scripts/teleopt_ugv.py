import rospy
import tf
import pygame
import math
from xarm_moveit.msg import VehicleControl
from geometry_msgs.msg import Twist, Pose


def UGV_controller(vehicle_pub,rate):
    

    ################################# LOAD UP A BASIC WINDOW #################################
    pygame.init()
    DISPLAY_W, DISPLAY_H = 100, 100
    # canvas = pygame.Surface((DISPLAY_W,DISPLAY_H))
    window = pygame.display.set_mode(((DISPLAY_W,DISPLAY_H)))
    running = True
    LEFT, RIGHT, UP, DOWN = False, False, False, False
    FORWARD,BACKWARD,TURN_L,TURN_R=False, False, False, False

    #Initialize controller
    joysticks = []
    for i in range(pygame.joystick.get_count()):
        joysticks.append(pygame.joystick.Joystick(i))
    for joystick in joysticks:
        joystick.init()
        print(joystick.get_numaxes())

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
            if event.type == pygame.JOYAXISMOTION:
                analog_keys[event.axis] = event.value
                print("event value is {}".format(analog_keys))
                # Horizontal Analog
                if abs(analog_keys[3]) > .4:
                    print(analog_keys)
                    # print(event)
                    if analog_keys[3] < -.7:
                        
                        LEFT=True
                        
                    else:
                        LEFT = False
                    if analog_keys[3] > .7:
                        RIGHT = True
                        
                    else:
                        RIGHT = False
                # Vertical Analog
                if abs(analog_keys[4]) > .4:
                    if analog_keys[4] < -.7:
                        UP=True
                        
                    else:
                        UP = False
                    if analog_keys[4] > .7:
                        
                        DOWN=True
                        
                    else:
                        DOWN = False


                if abs(analog_keys[0]) > .1:
                    # print(analog_keys)
                    # print(event)
                    if analog_keys[0] < -.2:
                        
                        TURN_L=True
                        
                    else:
                        TURN_L = False
                    if analog_keys[0] > .2:
                        TURN_R = True
                        
                    else:
                        TURN_R = False
                # Vertical Analog
                if abs(analog_keys[1]) > .1:
                    if analog_keys[1] < -.2:
                        FORWARD=True
                        
                    else:
                        FORWARD= False
                    if analog_keys[1] > .2:
                        
                        BACKWARD=True
                        
                    else:
                        BACKWARD = False
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

            linearx=-1*index
        if BACKWARD:
            index=-1
            if event.type == pygame.JOYAXISMOTION:
                analog_keys[event.axis] = event.value
                index=analog_keys[0]

            linearx=-1*index
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

        twist_msg = Twist()
        twist_msg.linear.x=linearx*0.1
        twist_msg.angular.z=angularz*0.5


        vehicle_pub.publish(twist_msg)
        print("send successfull linear {} angular {}".format(linearx,angularz))
        rate.sleep()

def test():
    rospy.init_node('UGV_publisher')
    vehicle_twist=rospy.Publisher('/managed/joy', Twist, queue_size=10)
    rate = rospy.Rate(20)  # 1 Hz

    UGV_controller(vehicle_twist,rate)

if __name__ == '__main__':
    test()