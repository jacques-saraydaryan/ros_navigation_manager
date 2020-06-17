__author__ = 'Jacques Saraydaryan'

import rospy
from abc import abstractmethod
from threading import Timer
from move_base_msgs.msg import MoveBaseGoal


class AbstractNavStrategy:
    _actMove_base=''
    _retry_nb=0
    _retry_max_nb = 3
    _maxWaitTimePerGoal=60*1
    _maxTimeElapsed = 60*10  # in second
    _timeout_checker=False
    _t_timer=''
    current_goal = MoveBaseGoal()

    def __init__(self,actMove_base):
        self._actMove_base=actMove_base

    @abstractmethod
    def goto(self, targetPose): pass

    @abstractmethod
    def stopAll(self): pass

    def startTimeWatch(self):
        self.startTimeWatchWithTimeOut(self._maxTimeElapsed)

    def startTimeWatchWithTimeOut(self, tOut):
        self._t_timer = Timer(tOut, self.timeout_reached)
        self._t_timer.start()

    def timeout_reached(self):
        self._timeout_checker = True

    def reset(self):
        self._retry_nb = 0
        self._timeout_checker = False
        try:
            self._t_timer.cancel()
        except Exception as e:
            rospy.loginfo("Unable to reset timer: %s" % e)

    def setMaxNbRetry(self, maxNbRetry):
        self._retry_max_nb = maxNbRetry

    def setMaxTimeElapsed(self, maxElapsedTime):
        self._maxTimeElapsed = maxElapsedTime

    def setMaxTimeElapsedPerGoal(self, maxElapsedTimePerGoal):
        self._maxWaitTimePerGoal = maxElapsedTimePerGoal

    def set_timeout_checker(self, value):
        self._timeout_checker = value

    def set_current_goal(self, goal):
        """
        Update the current_goal with the geometry_msgs/Pose goal in parameter
        """
        self.current_goal.target_pose.header.frame_id = "map"
        self.current_goal.target_pose.header.stamp = rospy.Time.now()
        self.current_goal.target_pose.pose.position = goal.position
        self.current_goal.target_pose.pose.orientation = goal.orientation
