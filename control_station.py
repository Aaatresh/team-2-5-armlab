#!/usr/bin/python
"""!
Main GUI for Arm lab
"""
# from operator import matmul
import numpy.matlib 

import os
script_path = os.path.dirname(os.path.realpath(__file__))

import argparse
import sys
import cv2
import numpy as np
import rospy
import time
from functools import partial

from PyQt4.QtCore import (QThread, Qt, pyqtSignal, pyqtSlot, QTimer)
from PyQt4.QtGui import (QPixmap, QImage, QApplication, QWidget, QLabel,
                         QMainWindow, QCursor, QFileDialog)

from ui import Ui_MainWindow
from rxarm import RXArm, RXArmThread
from camera import Camera, VideoThread
from state_machine import StateMachine, StateMachineThread, ExtMtx
""" Radians to/from  Degrees conversions """
D2R = np.pi / 180.0
R2D = 180.0 / np.pi


class Gui(QMainWindow):
    """!
    Main GUI Class

    Contains the main function and interfaces between the GUI and functions.
    """
    def __init__(self, parent=None, dh_config_file=None):
        QWidget.__init__(self, parent)
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
        """ Groups of ui commonents """
        self.joint_readouts = [
            self.ui.rdoutBaseJC,
            self.ui.rdoutShoulderJC,
            self.ui.rdoutElbowJC,
            self.ui.rdoutWristAJC,
            self.ui.rdoutWristRJC,
        ]
        self.joint_slider_rdouts = [
            self.ui.rdoutBase,
            self.ui.rdoutShoulder,
            self.ui.rdoutElbow,
            self.ui.rdoutWristA,
            self.ui.rdoutWristR,
        ]
        self.joint_sliders = [
            self.ui.sldrBase,
            self.ui.sldrShoulder,
            self.ui.sldrElbow,
            self.ui.sldrWristA,
            self.ui.sldrWristR,
        ]
        """Objects Using Other Classes"""
        self.camera = Camera()
        print("Creating rx arm...")
        if (dh_config_file is not None):
            self.rxarm = RXArm(dh_config_file=dh_config_file)
        else:
            self.rxarm = RXArm()
        print("Done creating rx arm instance.")
        self.sm = StateMachine(self.rxarm, self.camera)
        """
        Attach Functions to Buttons & Sliders
        TODO: NAME AND CONNECT BUTTONS AS NEEDED
        """
        # Video
        self.ui.videoDisplay.setMouseTracking(True)
        self.ui.videoDisplay.mouseMoveEvent = self.trackMouse
        self.ui.videoDisplay.mousePressEvent = self.calibrateMousePress

        # Buttons
        # Handy lambda function falsethat can be used with Partial to only set the new state if the rxarm is initialized
        #nxt_if_arm_init = lambda next_state: self.sm.set_next_state(next_state if self.rxarm.initialized else None)
        nxt_if_arm_init = lambda next_state: self.sm.set_next_state(next_state)
        self.ui.btn_estop.clicked.connect(self.estop)
        self.ui.btn_init_arm.clicked.connect(self.initRxarm)
        self.ui.btn_torq_off.clicked.connect(
            lambda: self.rxarm.disable_torque())
        self.ui.btn_torq_on.clicked.connect(lambda: self.rxarm.enable_torque())
        self.ui.btn_sleep_arm.clicked.connect(lambda: self.rxarm.sleep())

        #User Buttons
        self.ui.btnUser1.setText("Calibrate")
        self.ui.btnUser1.clicked.connect(partial(nxt_if_arm_init, 'calibrate'))
        self.ui.btnUser2.setText('Open Gripper')
        self.ui.btnUser2.clicked.connect(lambda: self.rxarm.open_gripper())
        self.ui.btnUser3.setText('Close Gripper')
        self.ui.btnUser3.clicked.connect(lambda: self.rxarm.close_gripper())
        self.ui.btnUser4.setText('Execute')
        self.ui.btnUser4.clicked.connect(partial(nxt_if_arm_init, 'execute'))

        #CN: When ready to teach a string of poses
        self.ui.btnUser5.setText('Start Teach')
        self.ui.btnUser5.clicked.connect(partial(nxt_if_arm_init, 'initteachmode'))

        #CN: When ready to record current pose to teaching path
        self.ui.btnUser6.setText('Record Pose')
        self.ui.btnUser6.clicked.connect(partial(nxt_if_arm_init, 'recpose'))

        #CN: Set current pose grip state and append teaching path
        self.ui.btnUser7.setText('Pose Grip: Open')
        self.ui.btnUser7.clicked.connect(partial(nxt_if_arm_init, 'gripstateO'))

        #CN: Set current pose grip state and append teaching path
        self.ui.btnUser8.setText('Pose Grip: Closed')
        self.ui.btnUser8.clicked.connect(partial(nxt_if_arm_init, 'gripstateC'))

        #CN: When ready to teach a string of poses
        self.ui.btnUser9.setText('End Teach')
        self.ui.btnUser9.clicked.connect(partial(nxt_if_arm_init, 'endteach'))

        #CN: When ready to replay a string of poses
        self.ui.btnUser10.setText('Recital')
        self.ui.btnUser10.clicked.connect(partial(nxt_if_arm_init, 'recital'))


        #CN: Print out blocks detected
        self.ui.btnUser13.setText('ID Blocks')
        self.ui.btnUser13.clicked.connect(partial(nxt_if_arm_init, 'IDblocks'))
        #click to grab
        self.ui.btnUser11.setText('Grab Click')
        self.ui.btnUser11.clicked.connect(partial(nxt_if_arm_init, 'grabclick'))

        #PID tuning
        self.ui.btnUser12.setText('PID Tune')
        self.ui.btnUser12.clicked.connect(partial(nxt_if_arm_init, 'tunePID'))

        #competition tasks
        self.ui.btnUser14.setText('Comp 1')
        self.ui.btnUser14.clicked.connect(partial(nxt_if_arm_init, 'comp1'))

        self.ui.btnUser15.setText('Comp 2')
        self.ui.btnUser15.clicked.connect(partial(nxt_if_arm_init, 'comp2'))
        
        self.ui.btnUser16.setText('Comp 3')
        self.ui.btnUser16.clicked.connect(partial(nxt_if_arm_init, 'comp3'))

        self.ui.btnUser17.setText('Comp 4')
        self.ui.btnUser17.clicked.connect(partial(nxt_if_arm_init, 'comp4'))

        # Sliders
        for sldr in self.joint_sliders:
            sldr.valueChanged.connect(self.sliderChange)
        self.ui.sldrMoveTime.valueChanged.connect(self.sliderChange)
        self.ui.sldrAccelTime.valueChanged.connect(self.sliderChange)
        # Direct Control
        self.ui.chk_directcontrol.stateChanged.connect(self.directControlChk)
        # Status
        self.ui.rdoutStatus.setText("Waiting for input")
        """initalize manual control off"""
        self.ui.SliderFrame.setEnabled(False)
        """Setup Threads"""

        # State machine
        self.StateMachineThread = StateMachineThread(self.sm)
        self.StateMachineThread.updateStatusMessage.connect(
            self.updateStatusMessage)
        self.StateMachineThread.start()
        self.VideoThread = VideoThread(self.camera)
        self.VideoThread.updateFrame.connect(self.setImage)
        self.VideoThread.start()
        self.ArmThread = RXArmThread(self.rxarm)
        self.ArmThread.updateJointReadout.connect(self.updateJointReadout)
        self.ArmThread.updateEndEffectorReadout.connect(
            self.updateEndEffectorReadout)
        self.ArmThread.start()

    """ Slots attach callback functions to signals emitted from threads"""

    @pyqtSlot(str)
    def updateStatusMessage(self, msg):
        self.ui.rdoutStatus.setText(msg)

    @pyqtSlot(list)
    def updateJointReadout(self, joints):
        for rdout, joint in zip(self.joint_readouts, joints):
            rdout.setText(str('%+.2f' % (joint * R2D)))

    ### TODO: output the rest of the orientation according to the convention chosen
    @pyqtSlot(list)
    def updateEndEffectorReadout(self, pos):
        self.ui.rdoutX.setText(str("%+.2f mm" % (1000 * pos[0])))
        self.ui.rdoutY.setText(str("%+.2f mm" % (1000 * pos[1])))
        self.ui.rdoutZ.setText(str("%+.2f mm" % (1000 * pos[2])))
        self.ui.rdoutPhi.setText(str("%+.2f rad" % (pos[3])))
        #self.ui.rdoutTheta.setText(str("%+.2f" % (pos[4])))
        #self.ui.rdoutPsi.setText(str("%+.2f" % (pos[5])))

    @pyqtSlot(QImage, QImage, QImage, QImage)
    def setImage(self, rgb_image, depth_image, tag_image, blockdetect_image):
        """!
        @brief      Display the images from the camera.

        @param      rgb_image    The rgb image
        @param      depth_image  The depth image
        """
        if (self.ui.radioVideo.isChecked()):
            self.ui.videoDisplay.setPixmap(QPixmap.fromImage(rgb_image))
        if (self.ui.radioDepth.isChecked()):
            self.ui.videoDisplay.setPixmap(QPixmap.fromImage(depth_image))
        if (self.ui.radioUsr1.isChecked()):
            self.ui.videoDisplay.setPixmap(QPixmap.fromImage(tag_image))
        if (self.ui.radioUsr2.isChecked()):
            self.ui.videoDisplay.setPixmap(QPixmap.fromImage(blockdetect_image))    

    """ Other callback functions attached to GUI elements"""

    def estop(self):
        self.rxarm.disable_torque()
        self.sm.set_next_state('estop')

    def sliderChange(self):
        """!
        @brief Slider changed

        Function to change the slider labels when sliders are moved and to command the arm to the given position
        """
        for rdout, sldr in zip(self.joint_slider_rdouts, self.joint_sliders):
            rdout.setText(str(sldr.value()))

        self.ui.rdoutMoveTime.setText(
            str(self.ui.sldrMoveTime.value() / 10.0) + "s")
        self.ui.rdoutAccelTime.setText(
            str(self.ui.sldrAccelTime.value() / 20.0) + "s")
        self.rxarm.set_moving_time(self.ui.sldrMoveTime.value() / 10.0)
        self.rxarm.set_accel_time(self.ui.sldrAccelTime.value() / 20.0)

        # Do nothing if the rxarm is not initialized
        if self.rxarm.initialized:
            joint_positions = np.array(
                [sldr.value() * D2R for sldr in self.joint_sliders])
            # Only send the joints that the rxarm has
            self.rxarm.set_positions(joint_positions[0:self.rxarm.num_joints])

    def directControlChk(self, state):
        """!
        @brief      Changes to direct control mode

                    Will only work if the rxarm is initialized.

        @param      state  State of the checkbox
        """
        if state == Qt.Checked and self.rxarm.initialized:
            # Go to manual and enable sliders
            self.sm.set_next_state("manual")
            self.ui.SliderFrame.setEnabled(True)
        else:
            # Lock sliders and go to idle
            self.sm.set_next_state("idle")
            self.ui.SliderFrame.setEnabled(False)
            self.ui.chk_directcontrol.setChecked(False)

    def trackMouse(self, mouse_event):
        """!
        @brief      Show the mouse position in GUI

                    TODO: after implementing workspace calibration display the world coordinates the mouse points to in the RGB
                    video image.

        @param      mouse_event  QtMouseEvent containing the pose of the mouse at the time of the event not current time
        """

        pt = mouse_event.pos()

        "CN: adding code to convert pixel coords to world coors"
        # extMtx = np.array([[0,-1,0,175],[-1,0,0,0],[0,0,-1,976],[0,0,0,1]]) #OLD
        # extMtx = np.array([[1,0,0,-14.1429],[0,-1,0,194.4616],[0,0,-1,978],[0,0,0,1]])
        
            # OLD METHOD (CHECKPT 1)
        #extMtx = self.status_message = "RXArm Initialized!"

        extMtx = self.camera.extrinsic_matrix
        # print("ExtMtx After\n")
        # print(extMtx)
        extMtxR = np.array([extMtx[0,0:3],extMtx[1,0:3],extMtx[2,0:3]])
        extMtxt = np.array([[extMtx[0,3]],[extMtx[1,3]],[extMtx[2,3]]])

        extMtxRinv = np.linalg.inv(extMtxR)
        negextMtxRinv_t = np.matmul(-extMtxRinv,extMtxt)
        # invExtMtx = np.array([[extMtxRinv[0,0:3], negextMtxRinv_t[0]],[extMtxRinv[1,0:3], negextMtxRinv_t[1]],[extMtxRinv[2,0:3], negextMtxRinv_t[2]],[0,0,0,1]])
        # print extMtxt

    
        invExtMtx = np.block([
            [extMtxRinv,negextMtxRinv_t],
            [np.zeros((1,3)),np.ones((1,1))]
        ])

        #invExtMtx = np.linalg.inv
        # Kfactory = np.array([904.317626953125, 0.0, 644.0140380859375], [0.0, 904.8245239257812, 360.77752685546875], [0.0, 0.0, 1.0])
        # Kinv = np.linalg.inv(Kfactory)


        # Kteam =   np.array([[949.7594,0,650.1970],[0,949.3147,365.4619],[0,0,1.0000]])
        Kteam =   np.array([[954.6327,0,629.4831],[0,968.4867,386.4730],[0,0,1.0000]])
        Kinv = np.linalg.inv(Kteam)

        # Pteam = np.array([[979.8243,0,653.8207],[0,982.7768,369.4433],[0, 0, 1.0000]])
        Pteam = np.array([[975.5068,0,628.0801],[0,993.6321,386.8233],[0, 0, 1.0000]])
        Pinv = np.linalg.inv(Pteam)

        if self.camera.DepthFrameRaw.any() != 0:
            z = self.camera.DepthFrameRaw[pt.y()][pt.x()]
            self.ui.rdoutMousePixels.setText("(%.0f,%.0f,%.0f)" %
                                             (pt.x(), pt.y(), z))
            # camPt = np.array([[pt.x()],[pt.y()],[z],[1]])
            
            uv1 = np.array([[pt.x()],[pt.y()],[1]])
            # print "uv1 \n", uv1
            # print "pinv \n", Pinv
            
            xyz_c = z*np.matmul(Pinv,uv1)
            # print "xyz_c \n", xyz_c
            # print "Hinv \n", invExtMtx
            # input()
            
            #print(invExtMtx.shape)

            xyz1_w = np.matmul(invExtMtx,np.array([[xyz_c[0,0]],[xyz_c[1,0]],[xyz_c[2,0]],[1]]))
            
            if(xyz1_w[0,0] < 0):
                wpX = xyz1_w[0,0] * 100/95
            else:
                wpX = xyz1_w[0,0] * 100/94 + 2.15
            
            if(xyz1_w[1,0] > 175):
                wpY = xyz1_w[1,0] * 10/9 - 19.4444
            else:
                wpY = xyz1_w[1,0]
                
            wpZ = 976-z #just use depth cam
            self.ui.rdoutMouseWorld.setText("(%.0f,%.0f,%.0f)" %
                                            (wpX,wpY,wpZ))

    def calibrateMousePress(self, mouse_event):
        """!
        @brief Record mouse click positions for calibration

        @param      mouse_event  QtMouseEvent containing the pose of the mouse at the time of the event not current time
        """
        """ Get mouse posiiton """
        pt = mouse_event.pos()
        self.camera.last_click[0] = pt.x()
        self.camera.last_click[1] = pt.y()
        self.camera.new_click = True
        # print(self.camera.last_click)

    def initRxarm(self):
        """!
        @brief      Initializes the rxarm.
        """
        self.ui.SliderFrame.setEnabled(False)
        self.ui.chk_directcontrol.setChecked(False)
        self.rxarm.enable_torque()
        self.sm.set_next_state('initialize_rxarm')



### TODO: Add ability to parse POX config file as well
def main(args=None):
    """!
    @brief      Starts the GUI
    """
    app = QApplication(sys.argv)
    app_window = Gui(dh_config_file=args['dhconfig'])
    app_window.show()
    sys.exit(app.exec_())



# Run main if this file is being run directly
### TODO: Add ability to parse POX config file as well
if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument("-c",
                    "--dhconfig",
                    required=False,
                    help="path to DH parameters csv file")
    main(args=vars(ap.parse_args()))
