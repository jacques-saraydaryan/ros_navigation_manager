#!/usr/bin/env python
# coding: utf8

import math
from copy import deepcopy

from AbstractNavStrategy import AbstractNavStrategy, rospy
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range, LaserScan
from tf.transformations import euler_from_quaternion


class GoThroughDoor(AbstractNavStrategy):
    def __init__(self, actMove_base):
        AbstractNavStrategy.__init__(self, actMove_base)

        print("Initializing GoThroughDoor strategy")
        self._odom_sub = rospy.Subscriber("/pepper_robot/odom", Odometry, self.odom_callback)
        self._back_sonar_sub = rospy.Subscriber("/pepper_robot/sonar/back", Range, self.back_sonar_callback)
        self._front_sonar_sub = rospy.Subscriber("/pepper_robot/sonar/front", Range, self.front_sonar_callback)
        self._laser_sub = rospy.Subscriber("/pepper_robot/laser", LaserScan, self.laser_callback)
        self._twist_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

        self.odom_pose = Pose()
        self.yaw = 0.0
        self.back_range = 0.0
        self.front_range = 0.0
        self.right_laser_range = []
        self.left_laser_range = []

        self.twist = Twist()
        self.linear_speed_y_left = 0.1
        self.linear_speed_y_right = -0.1

        # When goto() is called, it saves the current_pose parameter into initial_pose so the robot can go back to its
        # initial pose if there's an obstacle
        self.initial_pose = Pose()

        self.obstacle_detected = False
        self.obstacle_detected_time = 0.0
        self.obstacle_distance = 100000.0
        self.max_wait_time = 3.0

    def odom_callback(self, msg):
        """
        Update odometry pose and yaw
        """
        self.odom_pose = msg.pose.pose
        orientation_q = self.odom_pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (_, _, self.yaw) = euler_from_quaternion(orientation_list)

    def back_sonar_callback(self, msg):
        """
        Update the data from back sonar
        """
        self.back_range = msg.range

    def front_sonar_callback(self, msg):
        """
        Update the data from front sonar
        """
        self.front_range = msg.range

    def laser_callback(self, msg):
        """
        Update the data from laser front, right and left
        """
        self.left_laser_range = msg.ranges[3:12]  # [:10]
        self.front_laser_range = msg.ranges[23:-23]
        self.right_laser_range = msg.ranges[-15:]  # [-10:] to avoid the detection of the door ?

    def goto(self, current_pose, target_pose):
        """
        current_pose : geometry_msgs/Pose, the current pose of the robot
        target_pose : geometry_msgs/Pose representing the MapNode on the other side of the door

         Strategy to cross the door :
         - A first rotation so the robot is almost in the direction of the other side of the door
         - Then a rotation on the right of 90° to be parallel
         - Then it moves to the left as long as there is no obstacle and it tries to stay far from the walls
         - If there is an obstacle it moves on the right until the way is clear
         - Once the targetPose is reached, it rotates on the left by 90°
         - It sends the result to the NavigationManager that will take the lead and call the original navigation strategy used to reach the real goal
        """
        self.subscribe()
        self.initial_pose = current_pose
        no_obstacle = True

        # Rotates in the direction of the target_pose, so the other side of the door (next MapNode to reach)
        print("Rotate in direction of door")
        angle = math.atan2(target_pose.position.y - current_pose.position.y, target_pose.position.x - current_pose.position.x)
        print("Angle to reach : {} rad".format(angle))
        """
        self.twist.angular.z = -0.2
        while abs(angle - self.yaw) > 0.01:
            self._twist_pub.publish(self.twist)
        """
        self.adaptive_rotation(angle, 0.2, self.degree_to_rad(1))
        print("Facing the door")

        """
        # First rotation on the right (90°)
        print("First rotation")
        self.twist.angular.z = -0.2
        target_rad = self.yaw - math.pi / 2
        while abs(target_rad - self.yaw) > 0.01:
            self._twist_pub.publish(self.twist)
        self.twist.angular.z = 0

        # Translation to the left to reach the other side of the door
        print("Translation")
        self.twist.linear.y = self.linear_speed_y_left
        target_y = target_pose.position.y
        rotation_time = rospy.Time.now() + rospy.Duration.from_sec(0.5)
        while abs(self.odom_pose.position.y - target_y) > 0.1 and no_obstacle:
            no_obstacle = self.check_obstacle_on_side()
            self.check_obstacle_front_and_back(rotation_time)

            self.twist.linear.y = self.linear_speed_y_left
            self._twist_pub.publish(self.twist)

        if self.obstacle_detected:
            print("Can t cross door, obstacle detected, moved back to initial position, returning failure to navigation manager")
            self.safe_stop()
            self.reset()
            self.unsubscribe()
            return False

        self.twist.linear.y = 0
        self._twist_pub.publish(self.twist)
        print("Reached other side of door")

        # Second rotation on the left (90°)
        print("Second rotation")
        self.twist.angular.z = 0.2
        target_rad = self.yaw + math.pi / 2
        while abs(target_rad - self.yaw) > 0.1:
            self._twist_pub.publish(self.twist)

        self.safe_stop()
        self.reset()
        self.unsubscribe()
        return True
        """

    def check_obstacle_front_and_back(self, rotation_time):
        """
        Check if the robot is too close to the sides of the door with the sonars
        """
        back_range = deepcopy(self.back_range)
        front_range = deepcopy(self.front_range)
        # Rotate if we are too close to a wall
        if back_range - front_range > 0.1:
            self.twist.angular.z = 0.1
        elif front_range - back_range > 0.1:
            self.twist.angular.z = -0.1
        else:
            if rospy.Time.now() >= rotation_time and self.twist.angular.z != 0:
                # Correct orientation
                self.twist.angular.z = -self.twist.angular.z
                rotation_time = rospy.Time.now() + rospy.Duration.from_sec(0.5)
            else:
                self.twist.angular.z = 0

    def check_obstacle_on_side(self):
        """
        Check if there's an obstacle on the way to the other side of the door.
        If so, the robot stops, if after a while the obstacle is still there the robot goes back to its initial pose.

        """
        laser_range = deepcopy(self.left_laser_range)
        if len([r for r in laser_range if r < 0.7]) > 2:
            self.obstacle_detected_time = rospy.Time.now()
            self.obstacle_distance = min(laser_range)
            self.obstacle_detected = True
            print("Obstacle detected by laser at distance : {}".format(self.obstacle_distance))

            self.twist.linear.y = 0
            self.twist.angular.z = 0
            self._twist_pub.publish(self.twist)

            while len([r for r in laser_range if r <= self.obstacle_distance + 0.1]) > 2:
                if (rospy.Time.now().to_sec() - self.obstacle_detected_time.to_sec()) > self.max_wait_time:
                    print("Obstacle still there, moving back")
                    while abs(self.initial_pose.position.y - self.odom_pose.position.y) > 0.05:
                        self.twist.linear.y = self.linear_speed_y_right
                        self._twist_pub.publish(self.twist)
                    return False

                # Update data from left laser
                laser_range = deepcopy(self.left_laser_range)

            # Obstacle no longer there, so we exit the loop and reset the obstacle parameters
            print("No more obstacle, resetting parameters and resumption of the strategy")
            self.reset()

        return True

    def adaptive_rotation(self, angle_to_reach, speed, max_error):
        """
        angle_to_reach : the angle in radians to reach
        speed : the initial rotation speed
        max_error : the tolerance on the angle in radians
        """
        while abs(self.yaw - angle_to_reach) > max_error:
            if 0 < self.yaw < angle_to_reach or 0 > angle_to_reach > self.yaw:
                rotation_sign = 1
            else:
                rotation_sign = -1
            speed = rotation_sign * abs(speed * (self.yaw - angle_to_reach))
            print("Rotation speed : {}".format(speed))
            self.twist.angular.z = speed
            self._twist_pub.publish(self.twist)

    def stopAll(self):
        pass

    @staticmethod
    def degree_to_rad(angle):
        return angle * math.pi / 180

    def reset(self):
        self.obstacle_detected = False
        self.obstacle_distance = 100000.0
        self.obstacle_detected_time = 0.0

    def safe_stop(self):
        self.twist.linear = [0.0, 0.0, 0.0]
        self.twist.angular = [0.0, 0.0, 0.0]
        safety_time = rospy.Time.now() + rospy.Duration.from_sec(1)
        while rospy.Time.now() < safety_time:
            self._twist_pub.publish(self.twist)

    def subscribe(self):
        self._odom_sub = rospy.Subscriber("/pepper_robot/odom", Odometry, self.odom_callback)
        self._back_sonar_sub = rospy.Subscriber("/pepper_robot/sonar/back", Range, self.back_sonar_callback)
        self._front_sonar_sub = rospy.Subscriber("/pepper_robot/sonar/front", Range, self.front_sonar_callback)
        self._laser_sub = rospy.Subscriber("/pepper_robot/laser", LaserScan, self.laser_callback)
        self._twist_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

    def unsubscribe(self):
        self._odom_sub.unregister()
        self._back_sonar_sub.unregister()
        self._front_sonar_sub.unregister()
        self._twist_pub.unregister()
        self._laser_sub.unregister()
