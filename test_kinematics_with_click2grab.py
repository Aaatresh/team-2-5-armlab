#!/usr/bin/python
"""!
Test kinematics

TODO: Use this file and modify as you see fit to test kinematics.py
"""
import argparse
import sys
import os

from state_machine import StateMachine

script_path = os.path.dirname(os.path.realpath(__file__))
os.sys.path.append(os.path.realpath(script_path + '/../'))
from kinematics import *
from config_parse import *
from copy import deepcopy

if __name__ == '__main__':
    # ap = argparse.ArgumentParser()
    #ap.add_argument("-c", "--dhconfig", required=True, help="path to DH parameters csv file")

    # args=vars(ap.parse_args())

    rxarm = None
    camera = None
    sm = StateMachine(rxarm, camera)

    sm.click2GrabNPlace()

