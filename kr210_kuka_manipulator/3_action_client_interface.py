#! /usr/bin/env python

__author__ = 'jaisharm@umd.edu (jai)'
__maintainer__ = 'jaisharm@umd.edu (jai)'

'''
Creates an Action Client interface that enables the manipulator to
follow a Trajectory that represents a planar SQUARE in 3D-Space
'''

import sys
import rclpy
from rclpy.duration import Duration
from rclpy.action import ActionClient
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from ament_index_python.packages import get_package_share_directory
import ikpy.chain
import numpy as np 
import os 

class TrajectoryActionClient(Node):

    def __init__(self):
        super().__init__('TrajectoryActionClient') # Node name
        
        # create an Action Client
        # action type: FollowJointTrajectory
        # action name: '/joint_trajectory_controller/follow_joint_trajectory'
        self._action_client = ActionClient(self, FollowJointTrajectory, '/joint_trajectory_controller/follow_joint_trajectory')

        package_path = get_package_share_directory('kr210_kuka_manipulator')
        urdf_path = os.path.join(package_path, "urdf", 'kr210_gazebo.urdf')  
    
        self.robot_initialize(urdf_path)

    def send_goal(self):
        goal_msg = FollowJointTrajectory.Goal()
        joint_names = ['joint_1','joint_2','joint_3',
                       'joint_4','joint_5','joint_6', 
                       'left_gripper_finger_joint',
                       'right_gripper_finger_joint']
        
        # use a stack to collect trajectory points
        # define multiple JointTrajectoryPoint positions and append to stack
        # the positions are calculated with IK by processing End Effector Goals

        points = []        
        
        point1 = JointTrajectoryPoint()
        angles=self.kuka_robot.inverse_kinematics([1.87,-0.1,1.64])
        angles=np.delete(angles, [0,7,8])
        point1.positions = list(np.append(angles,[0.0,0.0]) )

        point2 = JointTrajectoryPoint()
        point2.time_from_start = Duration(seconds=1, nanoseconds=0).to_msg()
        angles=self.kuka_robot.inverse_kinematics([1.87,0.6,1.64])
        angles=np.delete(angles, [0,7,8])
        point2.positions = list(np.append(angles,[0.0,0.0]) )

        point3 = JointTrajectoryPoint()
        point3.time_from_start = Duration(seconds=2, nanoseconds=0).to_msg()
        angles=self.kuka_robot.inverse_kinematics([3.0,0.6,1.64])
        angles=np.delete(angles, [0,7,8])
        point3.positions = list(np.append(angles,[0.0,0.0]) )

        point4 = JointTrajectoryPoint()
        point4.time_from_start = Duration(seconds=3, nanoseconds=0).to_msg()
        angles=self.kuka_robot.inverse_kinematics([3.0,-0.1,1.64])
        angles=np.delete(angles, [0,7,8])
        point4.positions = list(np.append(angles,[0.0,0.0]) )

        point5 = JointTrajectoryPoint()
        point5.time_from_start = Duration(seconds=1, nanoseconds=0).to_msg()
        angles=self.kuka_robot.inverse_kinematics([1.87,-0.1,1.64])
        angles=np.delete(angles, [0,7,8])
        point5.positions = list(np.append(angles,[0.0,0.0]) )

        points.append(point1)
        points.append(point2)
        points.append(point3)
        points.append(point4)
        # points.append(point5)

        # feed all the points to the Action Client
        goal_msg.goal_time_tolerance = Duration(seconds=15, nanoseconds=0).to_msg()
        goal_msg.trajectory.joint_names = joint_names
        goal_msg.trajectory.points = points
        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)
    
    def robot_initialize(self,urdf_path):
        self.kuka_robot = ikpy.chain.Chain.from_urdf_file(urdf_path)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('\nGoal rejected :(')
            return
        self.get_logger().info('\n\nGoal accepted :)\n')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
    
    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('\n\nResult: '+str(result))
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback

def main(args=None):
    rclpy.init()
    action_client_interface = TrajectoryActionClient()
    future = action_client_interface.send_goal()
    rclpy.spin(action_client_interface)

if __name__ == '__main__':
    main()