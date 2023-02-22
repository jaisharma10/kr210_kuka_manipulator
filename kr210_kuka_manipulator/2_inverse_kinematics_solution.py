#!/usr/bin/env python3

'''
Combines the computations from ikpy library and the Joint Trajectory message
Takes in User Input for End Effector goal positions, performs inverse kinematics
and publishes the trajectory path
'''

'''
Test Configurations

# Near to the ground to check grab
    2.1 0 1.94
# full strech right
    3.33 0.05 0.66
# Full strech up
    0.47 0 3.78
# Random
    0.63 -0.148 2.39

'''

import os 

import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory , JointTrajectoryPoint

from ament_index_python.packages import get_package_share_directory
import ikpy.chain
import sys 
import numpy as np


class TrajectoryPublisher(Node):
    
    def __init__(self):
        super().__init__('publish_trajectory_node')      # Node name
        publish_topic_name = "/joint_trajectory_controller/joint_trajectory"
        self.publish_trajectory = self.create_publisher(JointTrajectory, publish_topic_name, 10)
        self.timer = self.create_timer(1, self.timer_callback)
        self.joints = ['joint_1','joint_2','joint_3',
                       'joint_4','joint_5','joint_6', 
                       'left_gripper_finger_joint',
                       'right_gripper_finger_joint']
        
        package_path = get_package_share_directory('kr210_kuka_manipulator')
        urdf_path = os.path.join(package_path, "urdf", 'kr210_gazebo.urdf')  
    
        ## Toolbox interface
        self.robot_initialize(urdf_path)
        
        # uses sys.arg to read command line arguments (user input)
        # first argument is the file_name
        # next 3 arguments are End Effector Goal Position (x,y,z)
        # last argument is gripper position, input 'o' if you want it open
        argv = sys.argv[1:] 
        self.get_ik_solution(float(argv[0]),float(argv[1]),float(argv[2]),argv[3])
    
    def timer_callback(self):
        kuka_trajectory_msg = JointTrajectory()          
        kuka_trajectory_msg.joint_names = self.joints    
        point = JointTrajectoryPoint()                   
        point.positions = self.goal_positions            
        point.time_from_start = Duration(sec=2)          
        kuka_trajectory_msg.points.append(point)             
        self.publish_trajectory.publish(kuka_trajectory_msg)  
        print("\nMotion Trajectory Sent !\n")

    def robot_initialize(self,urdf_path):
        self.kuka_robot = ikpy.chain.Chain.from_urdf_file(urdf_path)
    
    def get_fk_solution(self):
        T_matrix = self.kuka_robot.forward_kinematics([0] * 9)    
        print("\nTransformation Matrix :\n",T_matrix)
    
    def get_ik_solution(self,x,y,z,gripper):
        angles = self.kuka_robot.inverse_kinematics([x,y,z])
        angles = np.delete(angles, [0,7,8])
        # manaually added to open/close the gripper
        if (gripper == "o"):
            print("\Open Gripper\n")
            self.goal_positions = list(np.append(angles ,[-0.01,-0.01]) )
        else:
            print("\nClosed Gripper\n")
            self.goal_positions = list(np.append(angles ,[0.06, 0.06]) ) 
        print("\nInverse Kinematics Solution :\n" ,self.goal_positions)

def main(args=None):
    rclpy.init(args=args)
    joint_trajectory_object = TrajectoryPublisher()
    rclpy.spin(joint_trajectory_object)
    joint_trajectory_object.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()