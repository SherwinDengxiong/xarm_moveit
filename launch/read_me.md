run gazebo and xarm6 with gripper

roslaunch xarm_gazebo xarm6_beside_table.launch add_gripper:=true

roslaunch xarm_moveit mover_gazeboexecute.launch 


we include the xarm6_gripper_moveit_gazebo.launch in the launch file



run real arm with gripper


