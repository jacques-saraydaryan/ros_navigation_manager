#!/usr/bin/env python
# coding: utf8

import threading
import rospy
import math
import tf
from copy import deepcopy

from AbstractNavStrategy import AbstractNavStrategy
from geometry_msgs.msg import Twist, Pose, PoseStamped, Point
from sensor_msgs.msg import Range, LaserScan
from tf.transformations import euler_from_quaternion
from pepper_pose_for_nav.srv import MoveHeadAtPosition


class GoThroughDoor(AbstractNavStrategy):
    def __init__(self, actMove_base):
        print("Initializing GoThroughDoor strategy")
        AbstractNavStrategy.__init__(self, actMove_base)
        self._tflistener = tf.TransformListener()

        #self.odom_pose = Pose()
        self.robot_pose = PoseStamped()
        self.initial_pose = Pose()
        self.target_pose = Pose()
        self.angle_between_points = 0.0  # The angle between the first point (initial pose) and the second point (target pose)
        self.yaw = 0.0

        self.twist = Twist()
        self.linear_speed_y_left = 0.1
        self.linear_speed_y_right = -0.1

        # Sonars
        self.front_sonar = Range()
        self.back_sonar = Range()

        # Lasers
        # Subscribe to initialize the angle_increment attribute of the lasers
        self.init_laser = rospy.Subscriber("/pepper_robot/laser", LaserScan, self.init_laser_cb)
        self.front_laser_range = []
        self.right_laser_range = []
        self.left_laser_range = []
        # Angle maximum (so half of the field of view of a laser) for each laser of the LaserScan message which is of the form :
        # [15 values for right laser; 8 blank values; 15 values for front laser; 8 blank values; 15 values for left laser]
        self.left_laser_angle = 30
        self.front_laser_angle = 30
        self.right_laser_angle = 30

        self.obstacle_detected = False
        self.obstacle_detected_time = 0.0
        self.obstacle_distance = 100000.0
        self.max_wait_time = 5.0

        self.start_time = 0.0
        self.stop = False

        # The four points of the rectangle
        self.first_point = Point()
        self.second_point = Point()
        self.third_point = Point()
        self.fourth_point = Point()

        try:
            rospy.wait_for_service("/move_head_pose_srv", 5)
            self.move_head_service = rospy.ServiceProxy("/move_head_pose_srv", MoveHeadAtPosition)
        except Exception as e:
            rospy.loginfo("Service move_head_pose call failed: %s" % e)

        rospy.sleep(0.2)    # HACK : let some time to initialize angle_increment before unsubscribe
        self.init_laser.unregister()

    def odom_callback(self, msg):
        """
        Update odometry pose and yaw
        """
        self.odom_pose = msg.pose.pose
        orientation_q = self.odom_pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (_, _, self.yaw) = euler_from_quaternion(orientation_list)

    def get_robot_pose_loop(self):
        while not self.stop:
            self.get_robot_pose()
            rospy.sleep(0.2)

    def get_robot_pose(self):
        self._tflistener.waitForTransform("/map", "/base_link", rospy.Time(0), rospy.Duration(2))
        (trans, rot) = self._tflistener.lookupTransform("/map", "/base_link", rospy.Time(0))
        self.robot_pose.header.frame_id = "map"
        self.robot_pose.header.stamp = rospy.Time.now()
        self.robot_pose.pose.position.x = trans[0]
        self.robot_pose.pose.position.y = trans[1]
        self.robot_pose.pose.position.z = trans[2]
        self.robot_pose.pose.orientation.x = rot[0]
        self.robot_pose.pose.orientation.y = rot[1]
        self.robot_pose.pose.orientation.z = rot[2]
        self.robot_pose.pose.orientation.w = rot[3]

        orientation_q = self.robot_pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (_, _, self.yaw) = euler_from_quaternion(orientation_list)

    def back_sonar_callback(self, msg):
        """
        Update the data from back sonar
        """
        self.back_sonar = msg

    def front_sonar_callback(self, msg):
        """
        Update the data from front sonar
        """
        self.front_sonar = msg

    def init_laser_cb(self, msg):
        self.angle_increment = msg.angle_increment
        print("Init angle increment")

    def laser_callback(self, msg):
        """
        Update the data from laser front, right and left. Overview of the LaserScan message :
        [15 values for right laser; 8 blank values; 15 values for front laser; 8 blank values; 15 values for left laser]

        msg.ranges[0] : -120°, first beam of the right laser
        msg.ranges[7] : -90°, middle beam of the right laser
        msg.ranges[14] : -60°, last beam of the right laser
        msg.ranges[23] : -30°, first beam of the front laser
        msg.ranges[30] : 0°, middle beam of the front laser
        msg.ranges[37] : 30°, last beam of the front laser
        msg.ranges[46] : 60°, first beam of the left laser
        msg.ranges[53] : 90°, middle beam of the left laser
        msg.ranges[60] : 120°, last beam of the left laser
        """
        self.right_laser_range = msg.ranges[7 - self.right_laser_index: 7 + self.right_laser_index + 1]   # +1 on the right limit because the last one is excluded
        self.front_laser_range = msg.ranges[30 - self.front_laser_index: 30 + self.front_laser_index + 1]
        self.left_laser_range = msg.ranges[53 - self.left_laser_index: 53 + self.left_laser_index + 1]

    def goto(self, current_pose, target_pose):
        """
        current_pose : geometry_msgs/Pose, the current pose of the robot
        target_pose : geometry_msgs/Pose representing the MapNode on the other side of the door

         Strategy to cross the door :
         - A first rotation so the robot is almost in the direction of the other side of the door
         - Then a rotation on the right of 90° to be parallel
         - Then it moves to the left as long as there is no obstacle and it tries to stay in the middle of the way
         - If there is an obstacle it moves on the right
         - Once the targetPose is reached, it rotates on the left by 90°
         - It sends the result to the NavigationManager that will take the lead and call the original navigation strategy used to reach the real goal
        """
        self.reset()
        self.subscribe()
        self.move_head_service(pitch_value=0.2, yaw_value=0.0, continuous_fix=False)
        self.robot_pose_thread = threading.Thread(target=self.get_robot_pose_loop)
        self.robot_pose_thread.start()
        self.initial_pose = current_pose

        while not self.stop and not self.obstacle_detected:
            # Rotates in the direction of the target_pose, so the other side of the door (next MapNode to reach)
            print("Rotate in direction of door")
            self.angle_between_points = math.atan2(target_pose.position.y - current_pose.position.y, target_pose.position.x - current_pose.position.x)
            self.adaptive_rotation(self.angle_between_points, self.degree_to_rad(1))
            print("Facing the door")

            self.check_door_open()
            if self.obstacle_detected:
                break

            # First rotation on the right (-90°)
            print("First rotation")
            rotation_angle = self.yaw - math.pi / 2
            self.adaptive_rotation(rotation_angle, self.degree_to_rad(1))

            # Translation to the left to reach the other side of the door
            print("Translation")
            self.change_laser_angle("left_laser_angle", 20)
            target_x = target_pose.position.x
            target_y = target_pose.position.y
            initial_orientation = self.yaw
            self.start_time = rospy.Time.now()
            rotation_time = rospy.Time.now() + rospy.Duration.from_sec(1.0)

            while (abs(self.robot_pose.pose.position.y - target_y) > 0.05 or abs(self.robot_pose.pose.position.x - target_x) > 0.05) \
                    and not self.stop and not self.obstacle_detected:
                self.check_obstacle_on_side()
                self.correct_orientation(rotation_time, initial_orientation)

                self.twist.linear.y = self.linear_speed_y_left
                self._twist_pub.publish(self.twist)

            if self.stop or self.obstacle_detected:
                break

            self.twist.linear.y = 0
            self._twist_pub.publish(self.twist)
            print("Reached other side of door")
            print("Robot pose : {}".format(self.robot_pose.pose.position))

            # Second rotation on the left (90°)
            print("Second rotation")
            self.adaptive_rotation(self.angle_between_points, self.degree_to_rad(2))

            self.stopAll()
            return True

        if self.obstacle_detected:
            print("Obstacle detected, moved back to initial position, returning failure to navigation manager")
        elif self.stop:
            print("The strategy has been stopped")
        self.stopAll()
        return False

    def check_door_open(self):
        """
        When facing the door, check if it's opened.
        Robot is supposed to be between 60 and 80cm from the door, and the door has a width between 70 and 95cm
        """
        self.change_laser_angle("front_laser_angle", 14)
        laser_range = deepcopy(self.front_laser_range)

        if len([r for r in laser_range if r < 1.5]) >= 3:
            self.obstacle_detected = True
            self.obstacle_distance = min(laser_range)
            self.obstacle_detected_time = rospy.Time.now()
            print("Door blocked : {} m".format(self.obstacle_distance))
            print("Waiting 10 sec before cancelling strategy")

            while len([r for r in laser_range if r < 1.5]) >= 3 and (rospy.Time.now() - self.obstacle_detected_time) < rospy.Duration(10):
                laser_range = deepcopy(self.front_laser_range)

            if len([r for r in laser_range if r < 1.5]) <= 2:
                print("Door opened")
                self.reset_obstacle()

        return

    def correct_orientation(self, rotation_time, orientation, sign=1):
        """
        Check if the robot is too close to the sides of the door with the sonars and if its orientation is towards the goal.
        Sign is used when the robot is going back to its initial pose because he detected an obstacle, so we need to change the sign of rotations
        """
        back_sonar_ = deepcopy(self.back_sonar)
        back_range = back_sonar_.range
        front_sonar_ = deepcopy(self.front_sonar)
        front_range = front_sonar_.range

        # If range of sonars is close enough, we correct the orientation if needed
        if back_range + front_range <= 1.0 and orientation - 0.15 <= self.yaw <= orientation + 0.15:
            if back_range - front_range > 0.1:
                #print("Back > front")
                self.twist.angular.z = sign*0.1
            elif front_range - back_range > 0.1:
                #print("Front > back")
                self.twist.angular.z = sign*-0.1
            else:
                if rospy.Time.now() >= rotation_time and self.twist.angular.z != 0:
                    # Correct orientation
                    #print("Reducing angular velocity to orient robot in direction of goal")
                    self.twist.angular.z -= math.copysign(0.05, self.twist.angular.z)
                    rotation_time = rospy.Time.now() + rospy.Duration.from_sec(1.0)
                else:
                    self.twist.angular.z = 0
            return

        # If the difference between the initial and the current orientations is more than 0.15 radians (8,6°), we send
        # an angular velocity to correct it
        if self.yaw < orientation - 0.15:
            #print("Current yaw < initial yaw, correcting")
            self.twist.angular.z = sign*0.1
            return
        elif self.yaw > orientation + 0.15:
            #print("Current yaw > initial yaw, correcting")
            self.twist.angular.z = sign*-0.1
            return

        # Now if the angle between the two points (the MapNodes) and the one between the first MapNode (the initial pose)
        # and the current pose of the robot are too different we correct the orientation.
        # The robot can drift from the path and still have a good orientation because of the wheels or the ground not
        # being perfect.
        # This correction is applied only after a few seconds so the robot has moved
        if (rospy.Time.now() - self.start_time) > rospy.Duration(5):
            angle_init_pose_robot_pose = math.atan2(self.robot_pose.pose.position.y - self.initial_pose.position.y,
                                                    self.robot_pose.pose.position.x - self.initial_pose.position.x)

            if angle_init_pose_robot_pose < self.angle_between_points - 0.2:
                #print("Robot drift on right")
                #print("Angle to reach : {}".format(self.angle_between_points*180/math.pi))
                #print("Actual angle : {}".format(angle_init_pose_robot_pose*180/math.pi))
                self.twist.angular.z = sign*0.1
            elif angle_init_pose_robot_pose > self.angle_between_points + 0.2:
                #print("Robot drift on left")
                #print("Angle to reach : {}".format(self.angle_between_points * 180 / math.pi))
                #print("Actual angle : {}".format(angle_init_pose_robot_pose * 180 / math.pi))
                self.twist.angular.z = sign*-0.1
            else:
                if rospy.Time.now() >= rotation_time and self.twist.angular.z != 0:
                    # Correct orientation
                    #print("Reducing angular velocity to orient robot in direction of goal")
                    self.twist.angular.z -= math.copysign(0.05, self.twist.angular.z)
                    rotation_time = rospy.Time.now() + rospy.Duration.from_sec(1.0)
                else:
                    self.twist.angular.z = 0

    def check_obstacle_on_side(self):
        """
        Check if there's an obstacle on the way to the other side of the door.
        If so, the robot stops, if after a while the obstacle is still there the robot goes back to its initial pose.
        """
        laser_range = deepcopy(self.left_laser_range)
        if len([r for r in laser_range if r <= 0.7]) >= 2:
            self.obstacle_detected_time = rospy.Time.now()
            self.obstacle_distance = min(laser_range)
            self.obstacle_detected = True
            print("Obstacle detected by laser at distance : {} m".format(self.obstacle_distance))

            self.twist.linear.y = 0
            self.twist.angular.z = 0
            self._twist_pub.publish(self.twist)

            while len([r for r in laser_range if r <= self.obstacle_distance + 0.1]) >= 1:
                if (rospy.Time.now() - self.obstacle_detected_time) > rospy.Duration.from_sec(self.max_wait_time):
                    print("Obstacle still there, moving back")
                    while abs(self.initial_pose.position.y - self.robot_pose.pose.position.y) > 0.05 \
                            or abs(self.initial_pose.position.x - self.robot_pose.pose.position.x) > 0.05:
                        right_laser_range = deepcopy(self.right_laser_range)
                        if len([r for r in right_laser_range if r <= 0.7]) >= 2:
                            print("Obstacle on the way back, can't go back to initial position")
                            break
                        self.correct_orientation(rospy.Time.now() + rospy.Duration.from_sec(1.0), self.yaw, -1)
                        self.twist.linear.y = self.linear_speed_y_right
                        self._twist_pub.publish(self.twist)
                    return

                # Update data from left laser
                laser_range = deepcopy(self.left_laser_range)

            # Obstacle no longer there, so we exit the loop and reset the obstacle parameters
            print("No more obstacle, resetting parameters and resumption of the strategy")
            self.reset_obstacle()

        return

    def adaptive_rotation(self, angle_to_reach, max_error):
        """
        angle_to_reach : the angle in radians to reach
        max_error : the tolerance on the angle in radians
        """
        print("Angle to reach : {} deg".format(angle_to_reach*180/math.pi))
        start_time = rospy.Time.now()
        while abs(self.yaw - angle_to_reach) > max_error and (rospy.Time.now() - start_time) < rospy.Duration(30):
            speed = (angle_to_reach - self.yaw) / 10  # The diff in radians is between -3.14 and 3.14 so the speed is between -0.3 and 0.3
            if 0.2 <= abs(speed):
                speed = math.copysign(0.3, speed)
            elif 0.05 < abs(speed) < 0.2:
                speed = math.copysign(0.2, speed)
            elif 0 <= abs(speed) <= 0.05:
                speed = math.copysign(0.1, speed)
            self.twist.angular.z = speed
            self._twist_pub.publish(self.twist)

        self.safe_stop()
        print("Yaw : {} deg".format(self.yaw*180/math.pi))

    def change_laser_angle(self, laser_name, angle):
        """
        Changes the max angle of the laser in parameter, so we only take account of the values inside the angle.
        Check the laser_callback function for more information
        """
        if laser_name == "left_laser_angle":
            self.left_laser_angle = angle
            self.left_laser_index = int(math.floor(self.degree_to_rad(self.left_laser_angle) / self.angle_increment))
            #print("Left laser index : {}".format(self.left_laser_index))
        elif laser_name == "right_laser_angle":
            self.right_laser_angle = angle
            self.right_laser_index = int(math.floor(self.degree_to_rad(self.right_laser_angle) / self.angle_increment))
            #print("Right laser index : {}".format(self.right_laser_index))
        elif laser_name == "front_laser_angle":
            self.front_laser_angle = angle
            self.front_laser_index = int(math.floor(self.degree_to_rad(self.front_laser_angle) / self.angle_increment))
            #print("Front laser index : {}".format(self.front_laser_index))
        else:
            rospy.logwarn("Wrong name of laser, possible ones are : front_laser_angle, right_laser_angle, left_laser_angle")
            return False

        rospy.sleep(0.5)    # HACK : let some time for the change to take effect

    @staticmethod
    def degree_to_rad(angle):
        return angle * math.pi / 180

    def subscribe(self):
        """
        Start the subscribers and publishers
        """
        self.publishers = True
        #self._odom_sub = rospy.Subscriber("/pepper_robot/odom", Odometry, self.odom_callback)
        self._back_sonar_sub = rospy.Subscriber("/pepper_robot/sonar/back", Range, self.back_sonar_callback)
        self._front_sonar_sub = rospy.Subscriber("/pepper_robot/sonar/front", Range, self.front_sonar_callback)
        self._laser_sub = rospy.Subscriber("/pepper_robot/laser", LaserScan, self.laser_callback)
        self._twist_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

    def unsubscribe(self):
        """
        Stop the subscribers and publishers
        """
        self.publishers = False
        #self._odom_sub.unregister()
        self._back_sonar_sub.unregister()
        self._front_sonar_sub.unregister()
        self._twist_pub.unregister()
        self._laser_sub.unregister()

    def reset_obstacle(self):
        """
        Reset only obstacle parameters
        """
        self.obstacle_detected = False
        self.obstacle_distance = 100000.0
        self.obstacle_detected_time = 0.0

    def reset(self):
        """
        Reset the lasers angle to 30° (60° for the FOV) so we take all the data from them, and the obstacle parameters
        """
        self.change_laser_angle("left_laser_angle", 30)
        self.change_laser_angle("front_laser_angle", 30)
        self.change_laser_angle("right_laser_angle", 30)
        self.obstacle_detected = False
        self.obstacle_distance = 100000.0
        self.obstacle_detected_time = 0.0
        self.stop = False

    def safe_stop(self):
        """
        Publishes during a few seconds so we are sure the robot is stopped
        """
        self.twist.linear.x = self.twist.linear.y = self.twist.linear.z = 0.0
        self.twist.angular.x = self.twist.angular.y = self.twist.angular.z = 0.0
        safety_time = rospy.Time.now() + rospy.Duration.from_sec(1)
        while rospy.Time.now() < safety_time:
            self._twist_pub.publish(self.twist)
        print("Safe stop done")

    def stopAll(self):
        """
        Stop everything, moves the head to look forward
        """
        print("Stop GTD strategy")
        self.stop = True
        if self.robot_pose_thread.is_alive():
            self.robot_pose_thread.join(2.0)
        if self.publishers:
            self.safe_stop()
            self.unsubscribe()
        self.move_head_service(pitch_value=0.6, yaw_value=0.0, continuous_fix=True)
