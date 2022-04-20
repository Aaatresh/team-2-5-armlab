#!/usr/bin/python
"""!
Test kinematics

TODO: Use this file and modify as you see fit to test kinematics.py
"""
import argparse
import sys
import os
script_path = os.path.dirname(os.path.realpath(__file__))
os.sys.path.append(os.path.realpath(script_path + '/../'))
from kinematics import *
from config_parse import *
from copy import deepcopy
from configparser import ConfigParser

if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    #ap.add_argument("-c", "--dhconfig", required=True, help="path to DH parameters csv file")

    args=vars(ap.parse_args())

    passed = True
    vclamp = np.vectorize(clamp)

    #dh_params = parse_dh_param_file(args['dhconfig'])
    xi1 = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 1.0])
    xi2 = np.array([0.0, -103.91, 0.0, -1.0, 0.0 , 0.0])
    xi3 = np.array([0.0, 303.91, -50.0, 1.0, 0.0, 0.0])
    xi4 = np.array([0.0, 303.91, -250.0, 1.0, 0.0, 0.0])
    xi5 = np.array([-303.91, 0.0, 0.0, 0.0, 1.0, 0.0])
    screws = np.array([xi1, xi2, xi3, xi4, xi5])
    gst0 = np.array([[1.0, 0.0, 0.0, 0.0],[0.0, 1.0, 0.0, 424.15],[0.0, 0.0, 1.0, 303.91],[0.0, 0.0, 0.0, 1]])
    ### Add arm configurations to test here
    fk_angles = [[-1.03907226,  0.91926288,  0.03077512,  0.87621956,  0.6158568 ]]

    print('Test FK')
    fk_poses = []
    for joint_angles in fk_angles:
        print('Joint angles:', joint_angles)
        pose = FK_pox(joint_angles)
        print('pose = ',pose)
        fk_poses.append(pose)

    # print('Test IK')
    # th_d = IK_pox(fk_poses[0],gst0,screws)
    # print('th_d = ')
    # print(th_d)

    print('Test IK 2')
    pos = np.array([347, 331, 31, 0.0, 0, -0.8])
    th_d = IK_pox(pose)
    print('th_d = ')
    print(th_d)

    print('back through FK')
    pose = FK_pox(th_d)
    