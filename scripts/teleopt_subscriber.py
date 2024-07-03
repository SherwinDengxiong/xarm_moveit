import rospy
import tf
import math
import robomaster
from robomaster import robot
from xarm_moveit.msg import VehicleControl
from geometry_msgs.msg import Twist,Pose

ep_robot = robot.Robot()
# ep_robot.initialize(conn_type="rndis")
# ep_robot.initialize(conn_type="sta", sn="3JKCK1600302WZ")
ep_robot.initialize(conn_type="ap")
print("gimble connected!")

class Information:
    def __init__(self) :
        self.angle=[0.0,0.0,0.0]


last_info=Information()

def send_head_rotation(euler_angle):
    global last_info

    ep_gimbal = ep_robot.gimbal

    pitch_val = 50
    yaw_val = 200
    pitch_a=math.degrees(euler_angle[1])
    yaw_a=math.degrees(euler_angle[2])
    last_pitch=last_info.angle[1]
    last_yaw=last_info.angle[2]
    if last_pitch!=pitch_a or last_yaw!=yaw_a:

        ep_gimbal.moveto(pitch=pitch_a, yaw=yaw_a, pitch_speed=pitch_val, yaw_speed=yaw_val).wait_for_completed()
        last_info.angle=[0.0,pitch_a,yaw_a]
    rospy.loginfo("move to pitch {}, yaw {}".format(int(pitch_a),int(yaw_a)))
# def send_twist(twist,frequence):
#     vehicle_twist=rospy.Publisher('/managed/joy', Twist, queue_size=10)

#     rate = rospy.Rate(frequence)

#     for i in range(frequence):
#         twist.linear.x*=0.1
#         twist.angular.z*=0.5
#         vehicle_twist.publish(twist)
#         rospy.loginfo("publishing twist at time{}.".format(i))
#         rate.sleep()
def twist_callback(msg_sed):
    # This function will be called whenever a new Twist message is received
    # Access the linear and angular velocities from the msg object

    rotation_msg=msg_sed.orientation
    # twist_value=msg_sed.twist_value
    rospy.loginfo("head rotation {}.".format(rotation_msg))
    # send_twist(twist_value,10)


    # linear_velocity = twist_value.linear
    # angular_velocity = twist_value.angular

    # rospy.loginfo("linear velocity {}. angular velocity {}.".format(linear_velocity,angular_velocity))
    
    # Process the velocities as needed
    # ...
    
    rospy.loginfo("x {}. y {}. z {}. w{}. ".format(rotation_msg.x,rotation_msg.y,rotation_msg.z,rotation_msg.w))
    
    euler_angle=tf.transformations.euler_from_quaternion([rotation_msg.x,rotation_msg.y,rotation_msg.z,rotation_msg.w])
    # 
    
    print("euler_angle is {}.".format(euler_angle))
    send_head_rotation(euler_angle)
    
    


def test():
    # Initialize the ROS node
    rospy.init_node('twist_subscriber')
    rospy.loginfo("----------wait for vehicle control message---------")
    
    # Create a subscriber
    rospy.Subscriber('/turtle1/cmd_vel', Pose, twist_callback)

    # Spin the node to receive incoming messages
    rospy.spin()

if __name__ == '__main__':
    ep_gimbal = ep_robot.gimbal

    pitch_val = 15
    yaw_val = 30
    ep_gimbal.moveto(pitch=15, yaw=90, pitch_speed=50, yaw_speed=100).wait_for_completed()
    ep_gimbal.moveto(pitch=15, yaw=-90, pitch_speed=50, yaw_speed=100).wait_for_completed()
    ep_gimbal.moveto(pitch=0, yaw=0,pitch_speed=50, yaw_speed=100).wait_for_completed()
    test()
    ep_robot.close()



