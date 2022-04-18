import matplotlib.pyplot as plt
import numpy as np
from kinematics import FK_pox
from mpl_toolkits import mplot3d


# true_joint_states = np.genfromtxt("Poses.csv", delimiter=", ")
# obs_joint_states = np.genfromtxt("observed_joint_states.csv", delimiter=", ")

obs_joint_states = np.random.randn(10, 6)


xi1 = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 1.0])
xi2 = np.array([0.0, -103.91, 0.0, -1.0, 0.0 , 0.0])
xi3 = np.array([0.0, 303.91, -50.0, 1.0, 0.0, 0.0])
xi4 = np.array([0.0, 303.91, -250.0, 1.0, 0.0, 0.0])
xi5 = np.array([-303.91, 0.0, 0.0, 0.0, 1.0, 0.0])
screws = np.array([xi1, xi2, xi3, xi4, xi5])
gst0 = np.array([[1.0, 0.0, 0.0, 0.0],[0.0, 1.0, 0.0, 424.15],[0.0, 0.0, 1.0, 303.91],[0.0, 0.0, 0.0, 1]])

all_positions = []
for joint_angles in obs_joint_states:
    joint_angles = joint_angles.squeeze()
    position = FK_pox(joint_angles, gst0, screws)

    all_positions.append(position)

all_positions = np.array(all_positions)

fig = plt.figure()
ax = plt.axes(projection="3d")
xline = all_positions[:, 0]
yline = all_positions[:, 1]
zline = all_positions[:, 2]

ax.plot3D(xline, yline, zline, 'gray')

