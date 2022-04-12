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
    fk_angles = [[-1.43, -0.13, -0.39, -1.31, -1.4]]

    print('Test FK')
    fk_poses = []
    for joint_angles in fk_angles:
        print('Joint angles:', joint_angles)
        pose = get_pose_from_T(FK_pox(joint_angles, gst0, screws))
        print('pose = ',pose)
        fk_poses.append(pose)

    # print('Test IK')
    # th_d = IK_pox(fk_poses[0],gst0,screws)
    # print('th_d = ')
    # print(th_d)

    print('Test IK 2')
    pos = np.array([215, 30, 83, -np.pi/2, 0, 0])
    th_d = IK_pox(pos)
    print('th_d = ')
    print(th_d)
   
    print("FK:")
    print(get_pose_from_T(FK_pox(th_d, gst0, screws)))

    # print('Test FK')
    # fk_poses = []
    # for joint_angles in fk_angles:
    #     print('Joint angles:', joint_angles)
    #     for i, _ in enumerate(joint_angles):
    #         pose = get_pose_from_T(FK_pox(deepcopy(dh_params), joint_angles, i))
    #         print('Link {} pose: {}'.format(i, pose))
    #         if i == len(joint_angles) - 1:
    #             fk_poses.append(pose)
    #     print()

    # print('Test IK')
    # for pose, angles in zip(fk_poses, fk_angles):
    #     matching_angles = False
    #     print('Pose: {}'.format(pose))
    #     options = IK_geometric(deepcopy(dh_params), pose)
    #     for i, joint_angles in enumerate(options):
    #         print('Option {}: {}'.format(i, joint_angles))
    #         compare = vclamp(joint_angles - angles)
    #         if np.allclose(compare, np.zeros_like(compare), rtol=1e-3, atol=1e-4):
    #             print('Option {} matches angles used in FK'.format(i))
    #             matching_angles = True
    #     if not matching_angles:
    #         print('No match to the FK angles found!')
    #         passed = False
    #     print()
