import rospy
from std_msgs.msg import String
from sensor_msgs.msg import JointState


posdata = []
veldata = []
def callback_fn(data):

    posdata.append(data.position[0:5])
    veldata.append(data.velocity[0:5])
    
def joint_angle_subscriber():

    rospy.init_node("joint_angle_listener", anonymous=True)
    rospy.Subscriber("/rx200/joint_states", JointState, callback_fn)

    rospy.spin()


if __name__ == "__main__":

    joint_angle_subscriber()