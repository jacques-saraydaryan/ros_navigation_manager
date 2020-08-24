#!/usr/bin/env python
# coding: utf8

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped


class SetRobotPosition:
    def __init__(self):
        self.pose_sub = rospy.Subscriber("/initialpose", PoseWithCovarianceStamped, self.set_position)
        self.pose_pub = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size=5)

    def set_position(self, data):
        print("\nReceived a message on topic /initialpose, sending the same message with a smaller covariance...")
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = data.header.frame_id
        msg.pose.pose = data.pose.pose

        tmp_covariance = list(data.pose.covariance)
        tmp_covariance[0] = 0.05
        tmp_covariance[7] = 0.05
        tmp_covariance[-1] = 0.2
        msg.pose.covariance = tuple(tmp_covariance)

        self.pose_sub.unregister()
        self.pose_pub.publish(msg)
        print("Message sent, the robot has more confidence in its pose")
        self.subscribe()

    def subscribe(self):
        self.pose_sub = rospy.Subscriber("/initialpose", PoseWithCovarianceStamped, self.set_position)


if __name__ == '__main__':
    rospy.init_node("pepper_set_position")

    SetRobotPosition()

    rospy.spin()
