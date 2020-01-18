from trajectory import testcase_trajectory, testcase_trajectory2
import rospy
from std_msgs.msg import String
import time



if __name__ == '__main__':

    rospy.init_node("offboard_node")

    desired_state_pub = rospy.Publisher('gi/desired_state/string', String, queue_size=10)

    t1 = time.time()
    rate = rospy.Rate(5)
    height = 0
    land = False
    while True:

        t2 = time.time()
        t = t2 - t1
        t = t/3
        print ("Time elapsed: ", t)
        rate.sleep()

        # replace this testcase trajectory with your own
        state = testcase_trajectory2(t)

        height = state[2]
        if state[2] >= 5:
            height = state[2]
            height = True

        if land is True:
            height -= 0.3

        state_string = String()
        state_string.data = "position:" + str(state[0]) + "," + str(state[1]) + "," + str(height) + "." + \
                            "velocity:" + str(state[3]) + "," + str(state[4]) + "," + str(state[5]) + "." + \
                            "acceleration:" + str(state[6]) + "," + str(state[7]) + "," + str(state[8]) + "." + \
                            "jerk:" + str(state[9]) + "," + str(state[10]) + "," + str(state[11]) + "." + \
                            "snap:" + str(state[12]) + "," + str(state[13]) + "," + str(state[14])

        desired_state_pub.publish(state_string)

