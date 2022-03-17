"""!
The state machine that implements the logic.
"""
from numpy import True_
from PyQt4.QtCore import (QThread, Qt, pyqtSignal, pyqtSlot, QTimer)
import time
import numpy as np
import rospy
from datetime import datetime

class StateMachine():
    """!
    @brief      This class describes a state machine.

                TODO: Add states and state functions to this class to implement all of the required logic for the armlab
    """

    def __init__(self, rxarm, camera):
        """!
        @brief      Constructs a new instance.

        @param      rxarm   The rxarm
        @param      planner  The planner
        @param      camera   The camera
        """
        self.rxarm = rxarm
        self.camera = camera
        self.status_message = "State: Idle"
        self.current_state = "idle"
        self.next_state = "idle"
        self.gripcommand = 0
        self.poses = [0,0,0,0,0,0]
        # self.waypoints = [
        #     [-np.pi/2,       -0.5,      -0.3,            0.0,       0.0],
        #     [0.75*-np.pi/2,   0.5,      0.3,      0.0,       np.pi/2],
        #     [0.5*-np.pi/2,   -0.5,     -0.3,     np.pi / 2,     0.0],
        #     [0.25*-np.pi/2,   0.5,     0.3,     0.0,       np.pi/2],
        #     [0.0,             0.0,      0.0,         0.0,     0.0],
        #     [0.25*np.pi/2,   -0.5,      -0.3,      0.0,       np.pi/2],
        #     [0.5*np.pi/2,     0.5,     0.3,     np.pi / 2,     0.0],
        #     [0.75*np.pi/2,   -0.5,     -0.3,     0.0,       np.pi/2],
        #     [np.pi/2,         0.5,     0.3,      0.0,     0.0],
        #     [0.0,             0.0,     0.0,      0.0,     0.0]]

        self.waypoints = [
            [-0.153398081660271,	-0.90965062379837,	-0.633534073829651,	-0.615126311779022,	0.029145635664463],
            [-0.164135947823524,	-0.237767025828361,	1.00015544891357,	-1.28700995445251,	0.032213598489761],
            [-0.855961322784424,	0.329805880784988,	1.0860583782196,	-2.14757323265076,	-0.110446617007256],
            [-0.214757323265076,	0.630466103553772,	1.90060222148895,	-1.4404079914093,	0.107378661632538],
            [-0.003067961661145,	-0.098174773156643,	0.128854393959045,	-1.71038866043091,	0.134990319609642]]


    def set_next_state(self, state):
        """!
        @brief      Sets the next state.

            This is in a different thread than run so we do nothing here and let run handle it on the next iteration.

        @param      state  a string representing the next state.
        """
        self.next_state = state

    def run(self):
        """!
        @brief      Run the logic for the next state

                    This is run in its own thread.

                    TODO: Add states and funcitons as needed.
        """
        if self.next_state == "initialize_rxarm":
            self.initialize_rxarm()

        if self.next_state == "idle":
            self.idle()

        if self.next_state == "estop":
            self.estop()

        if self.next_state == "execute":
            self.execute()

        if self.next_state == "calibrate":
            self.calibrate()

        if self.next_state == "detect":
            self.detect()

        if self.next_state == "manual":
            self.manual()

        #CN: ADDED FOR TEACHING
        if self.next_state == "initteachmode":
            self.initteachmode()

        if self.next_state == "teachmode":
            self.teachmode()

        if self.next_state == "recpose":
            self.recpose()

        if self.next_state == "gripstateO":
            self.recposeGripO()

        if self.next_state == "gripstateC":
            self.recposeGripC()
        
        if self.next_state == "endteach":
            self.endteach()

        if self.next_state == "recital":
            self.recital()

    """Functions run for each state"""

    def manual(self):
        """!
        @brief      Manually control the rxarm
        """
        self.status_message = "State: Manual - Use sliders to control arm"
        self.current_state = "manual"

    def idle(self):
        """!
        @brief      Do nothing
        """
        self.status_message = "State: Idle - Waiting for input"
        self.current_state = "idle"

    def estop(self):
        """!
        @brief      Emergency stop disable torque.
        """
        self.status_message = "EMERGENCY STOP - Check rxarm and restart program"
        self.current_state = "estop"
        self.rxarm.disable_torque()

    def execute(self):
        """!
        @brief      Go through all waypoints
        TODO: Implement this function to execute a waypoint plan
              Make sure you respect estop signal
        """
        numPoses = len(self.waypoints)
        # print(numPoses)

        self.status_message = "State: Execute - Executing motion plan"
        estopPRESSED=0
        for pose in self.waypoints:
            #if estop is pressed, go to estop state...
            if self.next_state == "estop":
                estopPRESSED = 1
                break
            #otherwise go to next pose
            print(pose)
            self.rxarm.set_positions(pose)
            rospy.sleep(2.)
            

        
        if estopPRESSED == 1:
            self.next_state = "estop"
        else:
            self.next_state = "idle"

    def calibrate(self):
        """!
        @brief      Gets the user input to perform the calibration
        """
        self.current_state = "calibrate"
        self.next_state = "idle"

        """TODO Perform camera calibration routine here"""
        self.status_message = "Calibration - Completed Calibration"

    """ TODO """
    def detect(self):
        """!
        @brief      Detect the blocks
        """
        rospy.sleep(1)

    def initialize_rxarm(self):
        """!
        @brief      Initializes the rxarm.
        """
        self.current_state = "initialize_rxarm"
        self.status_message = "RXArm Initialized!"
        print("HERE! \n")
        if not self.rxarm.initialize():
            print('Failed to initialize the rxarm')
            self.status_message = "State: Failed to initialize the rxarm!"
            rospy.sleep(5)
        self.next_state = "idle"

    def initteachmode(self):
        self.status_message = "TEACH MODE - new pose array started, torque off"
        self.current_state = "initteachmode"
        self.rxarm.disable_torque() 
        #create array for poses
        self.poses = np.array([0,0,0,0,0,0]);
        self.gripcommand = 0;
        #format: [BASE ANGLE, SHOULDER ANGLE, ELBOW ANGLE, WRIST 1, WRIST 2, GRIP STATE]
        rospy.sleep(2)
        self.next_state="idleteachmode"

    def teachmode(self):
        self.status_message = "waiting for teach pose"
        self.current_state = "idleteachmode"
        self.rxarm.disable_torque() 
        

        #format: [BASE ANGLE, SHOULDER ANGLE, ELBOW ANGLE, WRIST 1, WRIST 2, GRIP STATE]
    def recpose(self):
        self.status_message = "Recording Current Pose"
        self.current_state = "recpose"
        newpose = self.rxarm.get_positions()
        newpose = np.append(newpose,self.gripcommand)
        self.poses = np.vstack((self.poses,newpose))
        self.next_state="idleteachmode"



    def recposeGripO(self): 
        self.status_message = "Set Gripper State: Open"
        self.current_state = "gripstateO"
        self.gripcommand = 0
        self.next_state="idleteachmode"
    def recposeGripC(self):
        self.status_message = "Set Gripper State: Closed"
        self.current_state = "gripstateC"
        self.gripcommand = 1
        self.next_state="idleteachmode"

    def endteach(self):
        self.current_state = "endteach"
        self.status_message = "Ending Teach! Exporting Pose Path to File"
        now = datetime.now()
        # mdyhms =  now.strftime("%m_%d_%Y__%H_%M_%S")
        # csvname = "Poses_"+mdyhms+".csv"
        csvname = "Poses.csv"
        np.savetxt(csvname, self.poses, delimiter=",")
        self.next_state="idle"

    def recital(self):
        self.current_state = "recital"
        self.status_message = "Replaying waypoints from Teach"
        
        csvname = "Poses.csv"
        fetchedCSV = np.genfromtxt(csvname, delimiter=",")

        self.waypoints = fetchedCSV[1:,0:5]
        self.next_state="idle"
class StateMachineThread(QThread):
    """!
    @brief      Runs the state machine
    """
    updateStatusMessage = pyqtSignal(str)
    
    def __init__(self, state_machine, parent=None):
        """!
        @brief      Constructs a new instance.

        @param      state_machine  The state machine
        @param      parent         The parent
        """
        QThread.__init__(self, parent=parent)
        self.sm=state_machine

    def run(self):
        """!
        @brief      Update the state machine at a set rate
        """
        while True:
            self.sm.run()
            self.updateStatusMessage.emit(self.sm.status_message)
            rospy.sleep(0.05)