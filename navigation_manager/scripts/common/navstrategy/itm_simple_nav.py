#!/usr/bin/env python
# coding: utf8

from AbstractNavStrategy import AbstractNavStrategy, rospy


class ItMSimpleNav(AbstractNavStrategy):
    """
    This class provides the simple navigation strategy for the robot using the Interactive Markers of the map_manager
    package (in the robocup_pepper_world_manager folder) to reach a point.
    """
    def __init__(self, actMove_base):
        AbstractNavStrategy.__init__(self, actMove_base)

    def goto(self, targetPose):
        """
        Abstract method that sends a simple navigation goal to reach `targetPose`
        """
        self.set_current_goal(targetPose)
        self._actMove_base.send_goal(self.current_goal)
        isActionResultSuccess = self._actMove_base.wait_for_result(rospy.Duration.from_sec(self._maxWaitTimePerGoal))
        current_action_state = self._actMove_base.get_state()

        if current_action_state == 4 or current_action_state == 5:
            rospy.logwarn("Goal lost or aborted : {}, {}".format(current_action_state, self._actMove_base.get_goal_status_text()))
            return False
        if not isActionResultSuccess:
            rospy.logwarn("Goal FAILURE")
            return False
        else:
            rospy.loginfo("Goal Successfully achieved at position : \n{} \n".format(targetPose))
            return True

    def stopAll(self):
        pass
