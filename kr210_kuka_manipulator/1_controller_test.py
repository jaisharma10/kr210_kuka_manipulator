__author__ = 'jaisharm@umd.edu (jai)'
__maintainer__ = 'jaisharm@umd.edu (jai)'


import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class TrajectoryPublisher(Node):
    
    def __init__(self):
        super().__init__('publish_trajectory_node')
        publish_topic_name = "/joint_trajectory_controller/joint_trajectory"
        # node publishes messages of type JointTrajectory, to Topic = 'publish_topic_name', queue size is 10.
        self.publish_trajectory = self.create_publisher(JointTrajectory, publish_topic_name, 10)
        # timer is created to execute timer_callback every 1 second
        self.timer = self.create_timer(1, self.timer_callback)
        # declare joint names and goal positions
        self.joints = ['joint_1','joint_2','joint_3',
                       'joint_4','joint_5','joint_6', 
                       'left_gripper_finger_joint',
                       'right_gripper_finger_joint']
        self.goal_positions = [180.5,-1.5,+2.5,0.5,-0.3,0.1,0.0,0.0]
        
    def timer_callback(self):
        kuka_trajectory_msg = JointTrajectory()          # initialize message type
        kuka_trajectory_msg.joint_names = self.joints    # initialize joint names
        
        point = JointTrajectoryPoint()                   # initialize message type
        point.positions = self.goal_positions            # initilize goal positions
        point.time_from_start = Duration(sec=2)          # initialize start time
        
        kuka_trajectory_msg.points.append(point)              # feed in array of trajectory points into the message 
        self.publish_trajectory.publish(kuka_trajectory_msg)  # publish message
    
def main(args=None):
    rclpy.init(args=args)
    joint_trajectory_object = TrajectoryPublisher()
    rclpy.spin(joint_trajectory_object)
    joint_trajectory_object.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()