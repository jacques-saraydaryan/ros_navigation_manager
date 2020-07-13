#!/usr/bin/env python
# coding: utf8

import unittest
import rospy
import actionlib
import tf

from move_base_msgs.msg import MoveBaseAction
from geometry_msgs.msg import PoseStamped
from common.navstrategy.go_through_door import GoThroughDoor
from map_manager.srv import GetNode


class TestNavMng(unittest.TestCase):
    def setUp(self):
        rospy.init_node("pepper_navigation_test")
        self._actMove_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self._tflistener = tf.TransformListener()
        self.gtd_strategy = GoThroughDoor(self._actMove_base)

        rospy.wait_for_service("/pepper/get_node", 5)
        self._get_node_service = rospy.ServiceProxy("/pepper/get_node", GetNode)


    def test_go_through_door(self):
        now = rospy.Time.now()

        self._tflistener.waitForTransform("/map", "/base_link", now, rospy.Duration(2))
        (trans, rot) = self._tflistener.lookupTransform("/map", "/base_link", now)
        robotPose = PoseStamped()
        robotPose.header.frame_id = "map"
        robotPose.header.stamp = rospy.Time.now()
        robotPose.pose.position.x = trans[0]
        robotPose.pose.position.y = trans[1]
        robotPose.pose.position.z = trans[2]
        robotPose.pose.orientation.x = rot[0]
        robotPose.pose.orientation.y = rot[1]
        robotPose.pose.orientation.z = rot[2]
        robotPose.pose.orientation.w = rot[3]

        targetPose = self._get_node_service("Node_1").node

        self.gtd_strategy.goto(robotPose.pose, targetPose.pose)

        rospy.spin()


if __name__ == '__main__':
    unittest.main()
