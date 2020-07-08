#!/usr/bin/env python
# coding: utf8

import rospy
from AbstractNavStrategy import AbstractNavStrategy
from std_srvs.srv import Empty
from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import GetMap


class ItMCleanRetryNav(AbstractNavStrategy):
    def __init__(self, actMove_base):
        AbstractNavStrategy.__init__(self, actMove_base)
        try:
            rospy.wait_for_service('/static_map', 5)
            rospy.loginfo("end service static_map wait time")
            self._getMap = rospy.ServiceProxy('static_map', GetMap)
        except Exception as e:
            rospy.loginfo("Service static_map call failed: %s" % e)

        try:
            rospy.wait_for_service('/move_base/clear_costmaps', 5)
            rospy.loginfo("end service all wait time")
            self._reset_costmap = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
        except Exception as e:
            rospy.loginfo("Service clear cost maps call failed: %s" % e)

        self._map_pub = rospy.Publisher('map', OccupancyGrid, queue_size=1)

    def goto(self, current_pose, target_pose):
        self.set_current_goal(target_pose)

        while self._retry_nb < self._retry_max_nb:
            self._actMove_base.send_goal(self.current_goal)
            isActionResultSuccess = self._actMove_base.wait_for_result(rospy.Duration.from_sec(self._maxWaitTimePerGoal))
            current_action_state = self._actMove_base.get_state()

            if isActionResultSuccess and current_action_state == 3:
                rospy.loginfo("Goal Successfully achieved at position : \n{} \n".format(target_pose))
                self.reset()
                return True
            else:
                rospy.logwarn('Goal FAILURE')
                rospy.logwarn('Retrying, ' + str(self._retry_max_nb - self._retry_nb - 1) + ' attempts remaining')
                rospy.loginfo('Clear all costmaps')
                self.resetCostMaps()
                self._retry_nb += 1
        rospy.logwarn('Goal FAILURE after retrying and clearing')
        self.reset()
        return False

    def resetCostMaps(self):
        try:
            # call clear all costmap before perfoming navigation
            self._reset_costmap()
        except Exception as e:
            rospy.loginfo("Service clear costmap call failed: %s" % e)
        #CAUTION Resending a map could cause robot localisation failure
        #Get map and republish to be sure that costmap are uptades again
        #try:
        #    current_map=self._getMap()
        #except Exception as e:
        #    rospy.loginfo("Service static map call failed: %s" % e)
        #
        #self._map_pub.publish(current_map.map)
        #rospy.sleep(5)

    def stopAll(self):
        pass
