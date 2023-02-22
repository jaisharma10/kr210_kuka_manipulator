#!/usr/bin/env python3

__author__ = 'jaisharm@umd.edu (jai)'
__maintainer__ = 'jaisharm@umd.edu (jai)'

'''
Just to test the implementation of ipky python library
Read the URDF file and execute Forward and Inverse Kinematics
for a test Input and publish the resulting message
'''

import ikpy.chain
import numpy as np
import os

urdf_path = os.path.join("urdf","kr210_gazebo.urdf")           # get path to URDF file

# reads the joints and links from URDF
# generates the required Transforms and DH Tables
kuka_robot = ikpy.chain.Chain.from_urdf_file(urdf_path)       

 # use FK to compute Transformation Matrix 
 # produced for when all joint angles are 0 
T_matrix = kuka_robot.forward_kinematics([0] * 9)             

# use IK to compute Joint Angles
# End Effector is at [x=2.2, y=1.2, z=1.2]
angles = kuka_robot.inverse_kinematics([2.2,1.2,1.2])     

# we don't need joint angles for gripper claws (left and right)     
angles_corrected = np.delete(angles, [0,7,8])

# Print Results
print("\nTransformation Matrix :\n",T_matrix)
print("\nAngles Computed\n",angles)
print("\nCorrected Angles \n",list(angles_corrected))