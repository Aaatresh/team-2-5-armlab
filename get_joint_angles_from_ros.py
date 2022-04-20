import rospy
from std_msgs.msg import String
from sensor_mgs.msg import JointState

def callback_fn(data):

    print("I see: ", data)

def joint_angle_subscriber():

    rospy.init_node("joint_angle_listener", anonymous=True)
    rospy.subscriber("/rx200/joint_states", JointState, callback_fn)

    rospy.spin()


if __name__ == "__main__":

    joint_angle_subscriber()