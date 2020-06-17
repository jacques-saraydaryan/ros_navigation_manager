#!/usr/bin/env python
# coding: utf8

from AbstractNavStrategy import *


class ItMRetryNav(AbstractNavStrategy):
    def __init__(self, actMove_base):
        AbstractNavStrategy.__init__(self, actMove_base)

    def goto(self, targetPose):
        self.set_current_goal(targetPose)

        while self._retry_nb < self._retry_max_nb:
            self._actMove_base.send_goal(self.current_goal)
            isActionResultSuccess = self._actMove_base.wait_for_result(rospy.Duration.from_sec(self._maxWaitTimePerGoal))
            current_action_state = self._actMove_base.get_state()
            if isActionResultSuccess and current_action_state == 3:
                rospy.loginfo("Goal Successfully achieved at position : \n{} \n".format(targetPose))
                self.reset()
                return True
            else:
                rospy.logwarn('Goal FAILURE (waiting ' + str(self._maxWaitTimePerGoal) + ')')
                rospy.logwarn('Retrying , only ' + str(self._retry_max_nb - self._retry_nb) + ' attempts remaining')
                self._retry_nb += 1


        self.reset()
        return False

    def stopAll(self):
        pass
