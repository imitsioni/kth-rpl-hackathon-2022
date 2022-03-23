from wrapping import BaxterWrapping
import moveit_commander
import time
from geometry_msgs.msg import Vector3
import rospy



if __name__ == '__main__':
    # remap again
    joint_state_topic = ['joint_states:=/robot/joint_states']
    moveit_commander.roscpp_initialize(joint_state_topic)

    rospy.init_node('baxter_wrapping', anonymous=False)

    # init class obj
    baxter = BaxterWrapping()

    ############### INITIALIZATION ##############
    # Initialize right and left arm
    baxter.rabp_go_base_pose()
    baxter.labp_go_base_pose()

    # Get Box estimations from vision
    A = Vector3()
    A.x = 0.72
    A.y = -0.17
    A.z = -0.14

    B = Vector3()
    B.x = 0.72
    B.y = -0.17
    B.z = -0.14

    C = Vector3()
    C.x = 0.72
    C.y = -0.17
    C.z = -0.14

    D = Vector3()
    D.x = 0.72
    D.y = -0.17
    D.z = -0.14

    closest_corners = [A, B, C, D]
    final_corners = [C, D, A, B]
    edge = 0.08
    ############### EXECUTION OF ONE FOLDING ##############

    for corner in range(4):
        # Grasp corner
        baxter.grasp(corner)       # int
        time.sleep(5)

        # Close
        baxter.close_gripper()

        # Stretch
        baxter.pull()

        # Place
        baxter.place(closest_corners[corner], final_corners[corner], edge)    # Vector3, Vector3, float

        # Open gripper
        baxter.open_gripper()

        # Initialize position
        baxter.initialize()
        time.sleep(5)