import matplotlib.pyplot as plt
import numpy as np

# true_joint_states = np.genfromtxt("Poses.csv", delimiter=", ")
# obs_joint_states = np.genfromtxt("observed_joint_states.csv", delimiter=", ")

true_joint_states = np.random.randn(10, 6)
obs_joint_states = np.random.randn(10, 6)

x = range(true_joint_states.shape[0])

# Iterate through each joint
for j in range(true_joint_states.shape[1]):
    plt.figure()
    plt.title("Plot joint " + str(j+1) + " angles")
    plt.ylabel("Angle (Radians)")
    plt.xlabel("Waypoint")
    plt.plot(true_joint_states[:, j], label="Commanded state of joint " + str(j+1))
    plt.plot(obs_joint_states[:, j], label="Observed state of joint " + str(j+1))
    plt.legend()
    plt.xticks(x)
    plt.show()