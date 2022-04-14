
def moveBlock(x, y, z, gripper_state):

    start_joint_state = self.rxarm.get_positions()

    final_pose = np.array([x, y, z, -np.pi/2, 0, 0])
    final_joint_state = IK_pox(final_pose)

    intermediate_pose = np.array([x, y, z-30, -np.pi/2, 0, 0])
    intermediate_joint_state = IK_pox(intermediate_pose)

    self.waypoints = [intermediate_joint_state, final_joint_state]
    self.execute_click2grab()

    if(gripper_state == "grab" or gripper_state == "close"):
        self.rxarm.close_gripper()
    elif(gripper_state == "drop" or gripper_state == "open"):
        self.rxarm.open_gripper()
    else:
        print("Error! Incorret gripper state given ...")

    self.waypoints = [intermediate_joint_state, start_joint_state]
    self.execute_click2grab()

