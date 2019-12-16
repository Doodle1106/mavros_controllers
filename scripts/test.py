import rospy
from mavros_msgs.msg import GlobalPositionTarget, State, PositionTarget
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode
from geometry_msgs.msg import PoseStamped, Twist, TwistStamped
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import Float32, Float64, String
from nav_msgs.msg import Odometry
import time
from pyquaternion import Quaternion
import math
import threading
from trajectory import testcase_trajectory
from minimum_snap_controller import controller, K_param, param
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib import style

import numpy as np

class Px4Controller:

    def __init__(self):

        self.imu = None
        self.gps = None
        self.local_pose = None
        self.local_speed = None

        self.current_state = None
        self.desired_state = None
        self.model_param = param()
        self.K = K_param()

        self.current_heading = None
        self.takeoff_height = -0.1
        self.local_enu_position = None

        self.cur_target_pose = None
        self.global_target = None

        self.received_new_task = False
        self.arm_state = False
        self.offboard_state = False
        self.received_imu = False
        self.frame = "BODY"

        self.state = None

        self.max_acc = 1

        '''
        ros subscribers
        '''
        self.local_pose_sub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.local_pose_callback)
        self.local_speed_sub = rospy.Subscriber("/mavros/local_position/velocity_local", TwistStamped, self.local_speed_callback)
        self.mavros_sub = rospy.Subscriber("/mavros/state", State, self.mavros_state_callback)
        self.gps_sub = rospy.Subscriber("/mavros/global_position/global", NavSatFix, self.gps_callback)
        self.imu_sub = rospy.Subscriber("/mavros/imu/data", Imu, self.imu_callback)

        self.set_target_position_sub = rospy.Subscriber("gi/set_pose/position", PoseStamped, self.set_target_position_callback)
        self.set_target_yaw_sub = rospy.Subscriber("gi/set_pose/orientation", Float32, self.set_target_yaw_callback)
        self.custom_activity_sub = rospy.Subscriber("gi/set_activity/type", String, self.custom_activity_callback)


        '''
        ros publishers
        '''
        self.local_target_pub = rospy.Publisher('mavros/setpoint_raw/local', PositionTarget, queue_size=100)
        self.trajectory_pub = rospy.Publisher('gi/desired_trajectory', Odometry, queue_size=100)

        '''
        ros services
        '''
        self.armService = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.flightModeService = rospy.ServiceProxy('/mavros/set_mode', SetMode)


        print("Px4 Controller Initialized!")


    def start(self):
        rospy.init_node("offboard_node")

        self.cur_target_pose = self.construct_target(0, 0, self.takeoff_height, self.current_heading)

        #print ("self.cur_target_pose:", self.cur_target_pose, type(self.cur_target_pose))

        for i in range(10):
            self.local_target_pub.publish(self.cur_target_pose)
            self.arm_state = self.arm()
            self.offboard_state = self.offboard()
            time.sleep(0.2)

        '''
        main ROS thread
        '''

        t1 = time.time()
        while self.arm_state and self.offboard_state and (rospy.is_shutdown() is False):

            self.local_target_pub.publish(self.cur_target_pose)

            t2 = time.time()

            # test
            self.update_state()
            self.update_desired_state(t2-t1)
            _, _, _, _, ad = self.solve_controller()

            if (np.linalg.norm(ad) >= self.max_acc):
                ad = (ad/np.linalg.norm(ad)) * self.max_acc
                print("clipped ad: ", ad)

            target_accel = self.construct_accel(ad[0], ad[1], ad[2])

            self.local_target_pub.publish(target_accel)

            # print ("current state: ", self.current_state[0], self.current_state[1], self.current_state[2])
            # print ("desired state: ", self.desired_state[0], self.desired_state[1], self.desired_state[2])
            # print ("target_accel: ", ad)

            trajectory = Odometry()
            trajectory.header.stamp = rospy.Time.now()
            trajectory.header.frame_id = "map"
            trajectory.pose.pose.position.x = self.desired_state[0]
            trajectory.pose.pose.position.y = self.desired_state[1]
            trajectory.pose.pose.position.z = self.desired_state[2]
            self.trajectory_pub.publish(trajectory)


            # print("current state: ", self.current_state)
            # print("desired state: ", self.desired_state)
            # print("solved controller result: ", controller_result)

            time.sleep(0.01)

    # omega is used for current drone state, while yaw and yaw_dot is used for desired state
    #         0  1  2  3   4   5   6    7    8    9     10    11    12       13       14       15   16
    # state: [x, y, z, x', y', z', x'', y'', z'', x''', y''', z''', omega.x, omega.y, omega.z, yaw, yaw_dot]
    # only position, velocity and angular speed is required from drone current state
    def update_state(self):

        self.current_state = [self.local_pose.pose.position.x, self.local_pose.pose.position.y, self.local_pose.pose.position.z,
                              self.local_speed.twist.linear.x, self.local_speed.twist.linear.y, self.local_speed.twist.linear.z,
                              None, None, None,
                              None, None, None,
                              self.local_speed.twist.angular.x, self.local_speed.twist.angular.y, self.local_speed.twist.angular.z,
                              None, None]

    def update_desired_state(self, t):

        self.desired_state = testcase_trajectory(t)

    def solve_controller(self):

        result = controller(self.current_state, self.desired_state, self.model_param, self.K, self.local_pose.pose.orientation)
        return result

    def construct_accel(self, acc_x, acc_y, acc_z):
        target_accel = PositionTarget()
        target_accel.header.stamp = rospy.Time.now()

        target_accel.coordinate_frame = 1

        target_accel.acceleration_or_force.x = acc_x
        target_accel.acceleration_or_force.y = acc_y
        target_accel.acceleration_or_force.z = acc_z

        # target_accel.position.z = 3.2

        target_accel.type_mask = PositionTarget.IGNORE_PX + PositionTarget.IGNORE_PY + PositionTarget.IGNORE_PZ \
                                + PositionTarget.IGNORE_VX + PositionTarget.IGNORE_VY + PositionTarget.IGNORE_VZ \
                                + PositionTarget.IGNORE_YAW + PositionTarget.IGNORE_YAW_RATE

        return target_accel
        # target_raw_pose.yaw = yaw
        # target_raw_pose.yaw_rate = yaw_rate

    def construct_target(self, x, y, z, yaw, yaw_rate = 1):
        target_raw_pose = PositionTarget()
        target_raw_pose.header.stamp = rospy.Time.now()

        target_raw_pose.coordinate_frame = 9

        target_raw_pose.position.x = x
        target_raw_pose.position.y = y
        target_raw_pose.position.z = z

        target_raw_pose.type_mask = PositionTarget.IGNORE_VX + PositionTarget.IGNORE_VY + PositionTarget.IGNORE_VZ \
                                    + PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ \
                                    + PositionTarget.FORCE

        target_raw_pose.yaw = yaw
        target_raw_pose.yaw_rate = yaw_rate

        return target_raw_pose



    '''
    cur_p : poseStamped
    target_p: positionTarget
    '''
    def position_distance(self, cur_p, target_p, threshold=0.1):
        delta_x = math.fabs(cur_p.pose.position.x - target_p.position.x)
        delta_y = math.fabs(cur_p.pose.position.y - target_p.position.y)
        delta_z = math.fabs(cur_p.pose.position.z - target_p.position.z)

        if (delta_x + delta_y + delta_z < threshold):
            return True
        else:
            return False

    def local_pose_callback(self, msg):
        self.local_pose = msg
        self.local_enu_position = msg

    def local_speed_callback(self, msg):
        self.local_speed = msg

    def mavros_state_callback(self, msg):
        self.mavros_state = msg.mode


    def imu_callback(self, msg):
        global global_imu, current_heading
        self.imu = msg

        self.current_heading = self.q2yaw(self.imu.orientation)

        self.received_imu = True


    def gps_callback(self, msg):
        self.gps = msg

    def FLU2ENU(self, msg):

        FLU_x = msg.pose.position.x * math.cos(self.current_heading) - msg.pose.position.y * math.sin(self.current_heading)
        FLU_y = msg.pose.position.x * math.sin(self.current_heading) + msg.pose.position.y * math.cos(self.current_heading)
        FLU_z = msg.pose.position.z

        return FLU_x, FLU_y, FLU_z


    def set_target_position_callback(self, msg):
        print("Received New Position Task!")

        if msg.header.frame_id == 'base_link':
            '''
            BODY_FLU
            '''
            # For Body frame, we will use FLU (Forward, Left and Up)
            #           +Z     +X
            #            ^    ^
            #            |  /
            #            |/
            #  +Y <------body

            self.frame = "BODY"

            print("body FLU frame")

            ENU_X, ENU_Y, ENU_Z = self.FLU2ENU(msg)

            ENU_X = ENU_X + self.local_pose.pose.position.x
            ENU_Y = ENU_Y + self.local_pose.pose.position.y
            ENU_Z = ENU_Z + self.local_pose.pose.position.z

            self.cur_target_pose = self.construct_target(ENU_X,
                                                         ENU_Y,
                                                         ENU_Z,
                                                         self.current_heading)


        else:
            '''
            LOCAL_ENU
            '''
            # For world frame, we will use ENU (EAST, NORTH and UP)
            #     +Z     +Y
            #      ^    ^
            #      |  /
            #      |/
            #    world------> +X

            self.frame = "LOCAL_ENU"
            print("local ENU frame")

            self.cur_target_pose = self.construct_target(msg.pose.position.x,
                                                         msg.pose.position.y,
                                                         msg.pose.position.z,
                                                         self.current_heading)

    '''
     Receive A Custom Activity
     '''

    def custom_activity_callback(self, msg):

        print("Received Custom Activity:", msg.data)

        if msg.data == "LAND":
            print("LANDING!")
            self.state = "LAND"
            self.cur_target_pose = self.construct_target(self.local_pose.pose.position.x,
                                                         self.local_pose.pose.position.y,
                                                         0.1,
                                                         self.current_heading)

        if msg.data == "HOVER":
            print("HOVERING!")
            self.state = "HOVER"
            self.hover()

        else:
            print("Received Custom Activity:", msg.data, "not supported yet!")


    def set_target_yaw_callback(self, msg):
        print("Received New Yaw Task!")

        yaw_deg = msg.data * math.pi / 180.0
        self.cur_target_pose = self.construct_target(self.local_pose.pose.position.x,
                                                     self.local_pose.pose.position.y,
                                                     self.local_pose.pose.position.z,
                                                     yaw_deg)

    '''
    return yaw from current IMU
    '''
    def q2yaw(self, q):
        if isinstance(q, Quaternion):
            rotate_z_rad = q.yaw_pitch_roll[0]
        else:
            q_ = Quaternion(q.w, q.x, q.y, q.z)
            rotate_z_rad = q_.yaw_pitch_roll[0]

        return rotate_z_rad


    def arm(self):
        if self.armService(True):
            return True
        else:
            print("Vehicle arming failed!")
            return False

    def disarm(self):
        if self.armService(False):
            return True
        else:
            print("Vehicle disarming failed!")
            return False


    def offboard(self):
        if self.flightModeService(custom_mode='OFFBOARD'):
            return True
        else:
            print("Vechile Offboard failed")
            return False


    def hover(self):

        self.cur_target_pose = self.construct_target(self.local_pose.pose.position.x,
                                                     self.local_pose.pose.position.y,
                                                     self.local_pose.pose.position.z,
                                                     self.current_heading)

    def takeoff_detection(self):
        if self.local_pose.pose.position.z > 0.05 and self.offboard_state and self.arm_state:
            return True
        else:
            return False


if __name__ == '__main__':

    con = Px4Controller()
    con.start()
    rospy.spin()

