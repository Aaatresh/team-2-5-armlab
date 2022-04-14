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
    pos = np.array([400, 400, 0, -np.pi/2, 0, 0])
    th_d = IK_pox(pos)
    print('th_d = ')
    print(th_d)

    # config_object["waist"] = {
    #     "P": 640,
    #     "I": 0,
    #     "D": 3600
    # }

    # config_object["shoulder"] = {
    #     "P": 800,
    #     "I": 0,
    #     "D": 0
    # }

    # config_object["elbow"] = {
    #     "P": 800,
    #     "I": 0,
    #     "D": 0
    # }

    # config_object["wrist_angle"] = {
    #     "P": 800,
    #     "I": 0,
    #     "D": 0
    # }

    # config_object["wrist_rotate"] = {
    #     "P": 640,
    #     "I": 0,
    #     "D": 3600
    # }

    # with open('pid.ini', 'w') as conf:
    #     config_object.write(conf)
    config_object = ConfigParser()
    config_object.read("pid.ini")
    waist = config_object["waist"]
    shoulder = config_object["shoulder"]
    elbow = config_object["elbow"]
    wrist_angle = config_object["wrist_angle"]
    wrist_rotate = config_object["wrist_rotate"]

    print("Waist PID Params:")
    print("P: ", waist["p"])
    print("I: ", waist["i"])
    print("D: ", waist["d"])

    print("shoulder PID Params:")
    print("P: ", shoulder["p"])
    print("I: ", shoulder["i"])
    print("D: ", shoulder["d"])

    print("elbow PID Params:")
    print("P: ", elbow["p"])
    print("I: ", elbow["i"])
    print("D: ", elbow["d"])

    print("wrist_angle PID Params:")
    print("P: ", wrist_angle["p"])
    print("I: ", wrist_angle["i"])
    print("D: ", wrist_angle["d"])

    print("wrist_rotate PID Params:")
    print("P: ", wrist_rotate["p"])
    print("I: ", wrist_rotate["i"])
    print("D: ", wrist_rotate["d"])
