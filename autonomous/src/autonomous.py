#! /usr/bin/env python

import rospy
from std_msgs.msg import Float32, Float64, Bool
from geometry_msgs.msg import Twist
import tf
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from nav_msgs.msg import Odometry
import time

# Creates proxy node
rospy.init_node('autonomous')

class Autonomous:
    def __init__(self):
        # Subscriber data
        is_auto_sub = rospy.Subscriber("auto/state", Bool, self.is_auto_callback)
        self.is_auto_data = Bool()
        auto_selector_sub = rospy.Subscriber("auto/select", Float32, self.auto_selector_callback)
        self.auto_selector_data = Float32()

        odom_sub = rospy.Subscriber("/base_pose_ground_truth", Odometry, self.odom_callback)
        self.odom_data = Odometry()

        # Publishing data
        self.turret_state_pub = rospy.Publisher("auto/turret/state", Float32, queue_size=50)
        self.intake_state_pub = rospy.Publisher("auto/intake/state", Float32, queue_size=50)
        self.move_state_pub = rospy.Publisher("auto/move/state", Float32, queue_size=50)

        # For controling robot position
        self.set_pose_pub = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size=50)
        self.set_goal_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=50)

        self.left_rpm_pub = rospy.Publisher("cmd_vel/left", Float64, queue_size=50)
        self.right_rpm_pub = rospy.Publisher("cmd_vel/right", Float64, queue_size=50)

        self.listener = tf.TransformListener()

        # Used in the custom state machine
        self.state = 0
        self.set_goal_count = 0
        self.last_time = 0

    # Callbacks
    def is_auto_callback(self, msg):
        self.is_auto_data.data = msg.data

    def auto_selector_callback(self, msg):
        self.auto_selector_data.data = msg.data

    def odom_callback(self, msg):
        self.odom_data = msg

    # Main loop
    def main(self):
        """ This is the main loop for autonomous """
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            if (self.is_auto_select() == 1):
                self.auto_1()
            elif (self.is_auto_select() == 2):
                self.auto_2()
            elif (self.is_auto_select() == 3):
                self.auto_3()
            else:
                print("Waiting for selector")
            rate.sleep()
    
    # Autonomous
    def auto_1(self):
        """ This is the autonomous for slot 1 """

        if (self.state == 0):
            self.last_time = time.time()
            self.state = 1

        if (self.state == 1):
            print("State 1: Turning Hood to face front and moving to goal")
            self.intake_state(0)
            self.turret_state(6)
            self.move_state(0)
            if (time.time() - self.last_time > 2.0):
                self.set_goal_count = 0
                self.last_time = time.time()
                self.state = 2
        if (self.state == 2):
            self.intake_state(0)
            self.turret_state(1)
            self.move_state(0)
            if (time.time() - self.last_time > 4.0):
                self.set_goal_count = 0
                self.last_time = time.time()
                self.state = 3
        if(self.state == 3):
            self.intake_state(0)
            self.turret_state(3)
            self.move_state(0)
            if (time.time() - self.last_time > 3.0):
                self.set_goal_count = 0
                self.last_time = time.time()
                self.state = 4
        if(self.state == 4):
            self.intake_state(0)
            self.turret_state(0)
            self.move_state(1)
            right_rpm = Float64()
            left_rpm = Float64()
            right_rpm.data = -30
            left_rpm.data = -30
            self.right_rpm_pub.publish(right_rpm)
            self.left_rpm_pub.publish(left_rpm)
            if (time.time() - self.last_time > 3.0):
                self.set_goal_count = 0
                self.last_time = time.time()
                self.state = 5
        if (self.state == 5):
            self.move_state(0)    

    def auto_2(self):
        """ This is the autonomous for slot 2 """
        print("Running Auton 2")

        self.set_pose(Pose(0.508, -0.70485 ,0 ,0 ,0 , 0))

        # Goals that the robot will follow
        goal_1 = Pose(4.3068, -0.70482, 0, 0, 0, 0)
        goal_2 = Pose(6.648, -0.70482, 0, 0, 0, 0)
        goal_3 = Pose(3.0068, -0.70482, 0, 0, 0, 0)

        pass_goal = Pose(3.8, -0.70482, 0, 0, 0, 0)

        if (self.state == 0):
            self.last_time = time.time()
            self.state = 1

        if (self.state == 1):
            print("State 1: Turning Hood to face front and moving to goal")
            self.intake_state(0)
            self.turret_state(0)
            self.move_state(1)

    def auto_3(self):
        """ This is the autonomous for slot 3 """
        print("Running Auton 2")

        self.set_pose(Pose(0.508, -0.70485 ,0 ,0 ,0 , 0))

        # Goals that the robot will follow
        goal_1 = Pose(4.3068, -0.70482, 0, 0, 0, 0)
        goal_2 = Pose(6.648, -0.70482, 0, 0, 0, 0)
        goal_3 = Pose(3.0068, -0.70482, 0, 0, 0, 0)

        pass_goal = Pose(3.8, -0.70482, 0, 0, 0, 0)

        if (self.state == 0):
            self.last_time = time.time()
            self.state = 1

        if (self.state == 1):
            print("State 1: Turning Hood to face front and moving to goal")
            self.intake_state(1)
            self.turret_state(5.5)
            self.move_state(1)

            if (self.set_goal_count < 20):
                if (self.set_goal(goal_1)):
                    self.set_goal_count += 1

            if (time.time() - self.last_time > 2.2):
                self.set_goal_count = 0
                self.last_time = time.time()
                self.state = 2

        if (self.state == 2):
            print("State 2: Prime and move to goal")
            self.intake_state(1)
            self.turret_state(2)
            self.move_state(1)

            if (self.is_goal_met(goal_1, 1.0)):
                self.set_goal_count = 0
                self.last_time = time.time()
                self.state = 3
        
        if (self.state == 3):
            print("State 3: Wait for prime")
            self.intake_state(1)
            self.turret_state(2)
            self.move_state(0)

            if (time.time() - self.last_time > 2.2):
                self.set_goal_count = 0
                self.last_time = time.time()
                self.state = 4

        if (self.state == 4):
            print("State 3: Shoot balls and plan next path")
            self.intake_state(1)
            self.turret_state(4)
            self.move_state(0)

            if (self.set_goal_count < 20):
                if (self.set_goal(goal_2)):
                    self.set_goal_count += 1

            if (time.time() - self.last_time > 2.2):
                self.set_goal_count = 0
                self.last_time = time.time()
                self.state = 5

        if (self.state == 5):
            print("State 4: Move and pick up balls to next goal with hood DOWN")
            self.intake_state(1)
            self.turret_state(10)
            self.move_state(1)

            if (self.is_goal_met(goal_2, 1.0)):
                self.set_goal_count = 0
                self.last_time = time.time()
                self.state = 6
        
        if (self.state == 6):
            print("State 5: Plan new path back")
            self.intake_state(1)
            self.turret_state(10)
            self.move_state(0)

            if (self.set_goal_count < 20):
                if (self.set_goal(goal_3)):
                    self.set_goal_count += 1
            
            if (self.set_goal_count == 19):
                self.set_goal_count = 0
                self.last_time = time.time()
                self.state = 7
        
        if (self.state == 7):
            print("State 6: Move back with hood DOWN")
            self.intake_state(1)
            self.turret_state(10)
            self.move_state(1)

            if (self.is_goal_met(pass_goal, 1.0)):
                self.set_goal_count = 0
                self.last_time = time.time()
                self.state = 8
        
        if (self.state == 8):
            print("State 7: Move back with hood up tracking")
            self.intake_state(1)
            self.turret_state(2)
            self.move_state(1)

            if (self.is_goal_met(goal_3, 1.0)):
                self.set_goal_count = 0
                self.last_time = time.time()
                self.state = 9
        
        if (self.state == 9):
            print("State 8: Wait to shoot")
            self.intake_state(1)
            self.turret_state(2)
            self.move_state(0)
            if (time.time() - self.last_time > 0.5):
                self.set_goal_count = 0
                self.last_time = time.time()
                self.state = 10

        if (self.state == 10):
            print("State 8: Shoot three Balls and end auto")
            self.intake_state(1)
            self.turret_state(4)
            self.move_state(0)
            if (time.time() - self.last_time > 1.0):
                self.set_goal_count = 0
                self.last_time = time.time()
                self.state = 11
        
        if (self.state == 11):
            self.intake_state(0)
            self.turret_state(4)
            self.move_state(0)

    # Set robot data
    def set_pose(self, wanted_pose):
        """ Set a new pose of the robot """
        data = PoseWithCovarianceStamped()
        data.pose.pose.position.x = wanted_pose.pos_x
        data.pose.pose.position.y = wanted_pose.pos_y

        data.pose.pose.orientation.z = wanted_pose.ori_z
        data.pose.pose.orientation.w = wanted_pose.ori_w


        self.set_pose_pub.publish(data)

    def set_goal(self, wanted_pose):
        """ Set a goal for the move_base path planner """
        data = PoseStamped()

        now  = rospy.get_rostime()
        data.header.stamp.secs = now.secs
        data.header.stamp.nsecs = now.nsecs
        data.header.frame_id = "map"

        try:
            (trans,rot) = self.listener.lookupTransform('/map', '/odom', rospy.Time(0))
            data.pose.position.x = wanted_pose.pos_x + trans[0]
            data.pose.position.y = wanted_pose.pos_y + trans[1]
            data.pose.position.z = 0

            data.pose.orientation.x = wanted_pose.ori_x
            data.pose.orientation.y = wanted_pose.ori_y
            data.pose.orientation.z = wanted_pose.ori_z
            data.pose.orientation.w = wanted_pose.ori_w

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return False

        self.set_goal_pub.publish(data)
        return True
    
    def is_auto_select(self):
        """ Checks and returns which auto to run """
        if (self.is_auto_data.data):
            return self.auto_selector_data.data
        else:
            return 0

    def is_goal_met(self, wanted_pose, area_length):
        """ Checks to see if you are within a certain distance of the goal """
        x = self.odom_data.pose.pose.position.x
        y = self.odom_data.pose.pose.position.y
        # Checks robot pose in a square
        if (x < wanted_pose.pos_x + area_length and x > wanted_pose.pos_x - area_length):
            if (y < wanted_pose.pos_y + area_length and y > wanted_pose.pos_y - area_length):
                return True
            else:
                return False
        else:
            return False

    def is_ball_in_range(self, area_length):
        """ Checks to see if a ball is within distance to pick up """
        x = self.odom_data.pose.pose.position.x
        y = self.odom_data.pose.pose.position.y

        balls = [Pose(0,0,0,0,0,0), Pose(0,0,0,0,0,0), Pose(0,0,0,0,0,0), Pose(0,0,0,0,0,0), Pose(0,0,0,0,0,0)]
        # Checks robot pose in a square
        for ball in balls:
            if (x < ball.pos_x + area_length and x > ball.pos_x - area_length):
                if (y < ball.pos_y + area_length and y > ball.pos_y - area_length):
                    return True
        
        return False


    def is_shoot_in_range(self, wanted_pose, area_length):
        """ Checks to see if it is within range to shoot"""
        x = self.odom_data.pose.pose.position.x
        y = self.odom_data.pose.pose.position.y
        # Checks robot pose in a square
        if (x < wanted_pose.pos_x + area_length and x > wanted_pose.pos_x - area_length):
            if (y < wanted_pose.pos_y + area_length and y > wanted_pose.pos_y - area_length):
                return True
            else:
                return False
        else:
            return False
    
    # States of the intake and shooter
    def intake_state(self, cond):
        """ 0 is idle, 1 is action. Publishes data to proxy """
        data = Float32()

        data.data = cond
        
        self.intake_state_pub.publish(data)

    def turret_state(self, cond):
        """ 0 is idle, 1 is prime, 2 is shoot, and 3 is turret rotation out of frame perimeter. Publishes data to proxy """
        data = Float32()
        data.data = cond

        self.turret_state_pub.publish(data)   

    def move_state(self, cond):
        """ 0 is stopped, 1 is active. Publishes data to pathfinding """
        data = Float32()

        data.data = cond

        self.move_state_pub.publish(data) 


class Pose:
    def __init__(self, pos_x, pos_y, pos_z, ori_r, ori_p, ori_y):
        self.pos_x = pos_x
        self.pos_y = pos_y
        self.pos_z = pos_z

        # Converts euler angles to quaternian
        quaternion = tf.transformations.quaternion_from_euler(ori_r, ori_p, ori_y)

        self.ori_x = quaternion[0]
        self.ori_y = quaternion[1]
        self.ori_z = quaternion[2]
        self.ori_w = quaternion[3]

if __name__ == '__main__':
    auto = Autonomous()
    auto.main()
