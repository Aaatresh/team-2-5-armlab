"""!
The state machine that implements the logic.
"""
from numpy import True_
from PyQt4.QtCore import (QThread, Qt, pyqtSignal, pyqtSlot, QTimer)
import time
import numpy as np
import rospy
from datetime import datetime
import cv2 as cv2

from kinematics import IK_pox

ExtMtx = np.eye(4)
# ExtMtx = np.array([[1,0,0,41],[0,-1,0,175],[0,0,-1,978],[0,0,0,1]]); #BY HAND 
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
        self.camera.extrinsic_matrix = np.eye(4)
        self.status_message = "State: Idle"
        self.current_state = "idle"
        self.next_state = "idle"
        self.gripcommand = 0
        self.poses = [0,0,0,0,0,0]
        self.waypoints = [
            [-np.pi/2,       -0.5,      -0.3,            0.0,       0.0],
            [0.75*-np.pi/2,   0.5,      0.3,      0.0,       np.pi/2],
            [0.5*-np.pi/2,   -0.5,     -0.3,     np.pi / 2,     0.0],
            [0.25*-np.pi/2,   0.5,     0.3,     0.0,       np.pi/2],
            [0.0,             0.0,      0.0,         0.0,     0.0],
            [0.25*np.pi/2,   -0.5,      -0.3,      0.0,       np.pi/2],
            [0.5*np.pi/2,     0.5,     0.3,     np.pi / 2,     0.0],
            [0.75*np.pi/2,   -0.5,     -0.3,     0.0,       np.pi/2],
            [np.pi/2,         0.5,     0.3,      0.0,     0.0],
            [0.0,             0.0,     0.0,      0.0,     0.0]]

        self.waypointGrips = 0 
        

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
            # self.status_message = "State: Calibrate - Waiting for input" 

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

        if self.next_state == "grabclick":
            self.click2GrabNPlace()

        if self.next_state == "tunePID":
            self.tunePID()

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
        for e,pose in enumerate(self.waypoints):
            #if estop is pressed, go to estop state...
            if self.next_state == "estop":
                estopPRESSED = 1
                break
            #otherwise go to next pose
            print(pose)
            self.rxarm.set_positions(pose)
            if self.waypointGrips[e] == 1:
                self.rxarm.close_gripper()
            else:
                self.rxarm.open_gripper()
            rospy.sleep(2.)
            

        
        if estopPRESSED == 1:
            self.next_state = "estop"
        else:
            self.next_state = "idle"

    def execute_click2grab(self):
        """!
        @brief      Go through all waypoints
        TODO: Implement this function to execute a waypoint plan
              Make sure you respect estop signal
        """
        numPoses = len(self.waypoints)
        # print(numPoses)

        self.status_message = "State: Execute - Executing motion plan"
        estopPRESSED=0
        for e,pose in enumerate(self.waypoints):
            #if estop is pressed, go to estop state...
            if self.next_state == "estop":
                estopPRESSED = 1
                break
            #otherwise go to next pose
            # print(pose)
            self.rxarm.set_positions(pose)
            # if self.waypointGrips[e] == 1:
            #     self.rxarm.close_gripper()
            # else:
            #     self.rxarm.open_gripper()
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
        self.next_state = "calibrate"
        self.status_message = "State: Calibrate - Waiting for input" 

        """TODO Perform camera calibration routine here"""
        global ExtMtx
        points3d = np.array([[-250, -25, 0],[250, -25, 0],[250, 275, 0],[-250, 275, 0]],dtype=np.float32)
        points2d = np.array([[431,580],[905,581],[902,295],[434,295]],dtype=np.float32) #camera at home position, hardcode loc of april

        # for calpoints in points2d:
        #     self.status_message = "Calibration - Click AprilTag1"
        #     self.camera.new_click = False
        #     pt = mouse_event.pos()

        points2d = []
        self.status_message = "Calibration - Click on points"
        self.next_state = "idle"
        # self.current_state = "calibrate"
        for pt_num in range(4):

            print("waiting for point ", str(pt_num+1))

            self.status_message = "Calibration - Click AprilTag" + str(pt_num+1)

            self.camera.new_click = False
            while(self.camera.new_click == False):
                pass

            points2d.append([self.camera.last_click[0], self.camera.last_click[1]])

        points2d = np.array(points2d, dtype=np.float32)

        Kteam =   np.array([[954.6327,0,629.4831],[0,968.4867,386.4730],[0,0,1.0000]],dtype=np.float32)
        Kinv = np.linalg.inv(Kteam)
        # self.camera.intrinsic_matrix = np.array([[904.3176, 0, 644.014],[0, 904.8245, 360.7775],[0,0,1]],dtype=np.float32)

        self.camera.intrinsic_matrix = Kteam
        
        distcoeff = np.array([0.1505,-0.2453,0.0002,-0.0014]).reshape((4,1)) # manual
        # distcoeff = np.array([0.5331,-0.4672,0.0003,-0.0011,0.4170])
        success,rot_vec,t_vec = cv2.solvePnP(points3d,points2d,self.camera.intrinsic_matrix,distcoeff,flags=cv2.SOLVEPNP_ITERATIVE)
        print(success)
        # construct extrinsic matrix 
        RotMtx = cv2.Rodrigues(rot_vec)[0]

        #Brute force adjust depth
        # t_vec[2] -= 75

        ExtMtx = np.block([[RotMtx,t_vec],[0,0,0,1]])
        self.camera.extrinsic_matrix = ExtMtx
        # print("ExtMtx SHOULD BE\n")
        # print(ExtMtx)
        self.status_message = "Calibration - Completed Calibration"
        self.next_state = "idle"


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
        self.waypointGrips = fetchedCSV[1:,5]
        self.next_state="idle"

    def plan_and_execute(self, start_joint_position, final_joint_position, xyz_w, final_gripper_state="open"):


        final_joint_position_1 = IK_pox(np.array([xyz_w[0], xyz_w[1], xyz_w[2] + 50.0,
                                                              -np.pi/2.0, 0.0, 0.0], dtype=np.float32))
        final_joint_position_1[2] = final_joint_position_1[2]*-1
        final_joint_position_1[3] = final_joint_position_1[3]*-1
        final_joint_position_2 = IK_pox(np.array([xyz_w[0], xyz_w[1], xyz_w[2] + 70.0,
                                                              -np.pi/2.0, 0.0, 0.0], dtype=np.float32))
        final_joint_position_3 = IK_pox(np.array([xyz_w[0], xyz_w[1], xyz_w[2] + 100.0,
                                                              -np.pi/2.0, 0.0, 0.0], dtype=np.float32))

        # Go from start to final joint state
        self.waypoints = [final_joint_position_1, final_joint_position]
        print("waypoints: ", self.waypoints)
        hold = input()
        self.execute_click2grab()

        print("first execute!")

        # Gripper state to grab/place block
        if(final_gripper_state == "closed"):
            self.rxarm.close_gripper()
        elif(final_gripper_state == "open"):
            self.rxarm.open_gripper()
        else:
            print("ERROR! Incorrect gripper state given...")

        # Go from final to start joint state
        self.waypoints = [final_joint_position_1,
                          start_joint_position]
        self.execute_click2grab()

    def click2GrabNPlace(self):

        Pteam = np.array([[975.5068, 0, 628.0801], [0, 993.6321, 386.8233], [0, 0, 1.0000]])
        Pinv = np.linalg.inv(Pteam)

        # extMtx = np.array([[1,0,0,-14.1429],[0,-1,0,194.4616],[0,0,-1,978],[0,0,0,1]])
        extMtx = self.camera.extrinsic_matrix
        extMtxR = np.array([extMtx[0, 0:3], extMtx[1, 0:3], extMtx[2, 0:3]])
        extMtxt = np.array([[extMtx[0, 3]], [extMtx[1, 3]], [extMtx[2, 3]]])

        extMtxRinv = np.linalg.inv(extMtxR)
        negextMtxRinv_t = np.matmul(-extMtxRinv, extMtxt)

        invExtMtx = np.block([
            [extMtxRinv, negextMtxRinv_t],
            [np.zeros((1, 3)), np.ones((1, 1))]
        ])

        """ Click to grab """
        print("Click point on screen to perform grabbing operation...")

        # Wait for click
        self.camera.new_click = False
        while(self.camera.new_click == False):
            pass

        pixel_point = np.array([self.camera.last_click[0], self.camera.last_click[1], 1]).reshape(-1, 1)
        z = self.camera.DepthFrameRaw[pixel_point[1, 0]][pixel_point[0, 0]]
        print("pixel_point: ", pixel_point)
        print("z = ", z)
        xyz_c = z * np.matmul(Pinv, pixel_point)
        xyz_w = np.matmul(invExtMtx, np.array([[xyz_c[0, 0]], [xyz_c[1, 0]], [xyz_c[2,0]], [1]]))
        xyz_w[2,0] = 976 - z

        position = np.reshape(xyz_w[:3], (3,))

        # pose = [position, orientation]
        pose = np.append(position, [[-np.pi/2, 0, 0]])

        print("pose: ", pose)

        final_joint_state = IK_pox(pose)
        final_joint_state[2] = final_joint_state[2]*-1
        final_joint_state[3] = final_joint_state[3]*-1

        print("final joint state: ", final_joint_state)

        self.initialize_rxarm()

        # Make sure gripper is open
        self.rxarm.open_gripper()
        time.sleep(2)
        start_joint_state = self.rxarm.get_positions()

        print("joint positions: ", start_joint_state)
        print("final joint position: ", final_joint_state)

        # plan path to point, close gripper and plan a path back to its starting position
        self.plan_and_execute(start_joint_state, final_joint_state, xyz_w, final_gripper_state="closed")

        """Click to place"""

        # Wait for click
        self.camera.new_click = False
        while (self.camera.new_click == False):
            pass

        pixel_point = np.array([self.camera.last_click[0], self.camera.last_click[1], 1]).reshape(-1, 1)
        z = self.camera.DepthFrameRaw[pixel_point[1, 0]][pixel_point[0, 0]]

        xyz_c = z * np.matmul(Pinv, pixel_point)
        xyz_w = np.matmul(invExtMtx, np.array([[xyz_c[0, 0]], [xyz_c[1, 0]], [xyz_c[2, 0]], [1]]))

        position = np.reshape(xyz_w[:3], (3, ))

        pose = np.append(position, [[-np.pi/2, 0, 0]])

        print("xyz world: ", xyz_w)
        print("pose: ", pose)
        print("pose shape: ", pose.shape)

        final_joint_state = IK_pox(pose)
        final_joint_state[2] = final_joint_state[2]*-1
        final_joint_state[3] = final_joint_state[3]*-1

        start_joint_state = self.rxarm.get_joint_positions()

        # plan path to point, open gripper and plan a path back to its starting position
        self.plan_and_execute(start_joint_state, final_joint_state, xyz_w,
                                               final_gripper_state="open")

        self.next_state="idle"

    def tunePID(self):
        print("Current PID Params:")
        for name in self.rxarm.joint_names:
            print("{0} gains: {1}".format(name, self.rxarm.get_motor_pid_params(name)))

        print("Enter name of joint to tune")
        name = input()
        print("Enter P, I, and D values for selected joint")
        print("P parametr")
        p_param = input()
        print("I parameter")
        i_param = input()
        print("D parameter")
        d_param = input()

        values = [p_param, i_param, d_param]

        self.rxarm.set_motor_pid_params(name, values)
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
        # self.sm.stm = self.updateStatusMessage

    def run(self):
        """!
        @brief      Update the state machine at a set rate
        """
        while True:
            self.sm.run()
            self.updateStatusMessage.emit(self.sm.status_message)
            rospy.sleep(0.05)