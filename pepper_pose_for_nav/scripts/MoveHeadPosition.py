#!/usr/bin/env python

import qi
#import argparse
import sys
import time
import rospy

from naoqi import ALProxy
from pepper_pose_for_nav.srv import MoveHeadAtPosition
from geometry_msgs.msg import Twist


class HeadFix:
    _continuous_fix_activated = True
    HEAD_PITCH_OBSTACLE = 0.6
    HEAD_YAW_OBSTACLE_LEFT = 0.7
    HEAD_YAW_OBSTACLE_RIGHT = -0.7

    def __init__(self, session, ip_, port_):
        """
        This example uses the setExternalCollisionProtectionEnabled method.
        """
        # Get the services.
        self._motion_service = session.service("ALMotion")
        self._memory_service = session.service("ALMemory")
        self._tracker_service = session.service("ALTracker")
        self._autolife_service = session.service("ALAutonomousLife")
        self._basic_awareness_service = session.service("ALBasicAwareness")
        self._posture_service = session.service("ALRobotPosture")
        self._dcm_proxy_service = ALProxy("DCM", ip_, port_)

        # Disabling all behaviours
        if self._autolife_service.getState() != 'disabled':
            self._autolife_service.setState('disabled')
        if self._autolife_service.getAutonomousAbilityEnabled("BasicAwareness") != 'disabled':
            self._autolife_service.setAutonomousAbilityEnabled("BasicAwareness", False)
        self._tracker_service.stopTracker()

        # Set posture and some variables
        self._posture_service.goToPosture("Stand", 0.3)

        self._motion_service.setStiffnesses("HEAD", 0.8)
        self._motion_service.setStiffnesses("TORSO", 0.8)
        self._error = 0.1
        self._pitch_value = 0.3
        self._yaw_value = 0.0
        self._fractionMaxSpeed = 0.2

        # declare ros service
        self.setHeadPositionSrv = rospy.Service('move_head_pose_srv', MoveHeadAtPosition, self.setHeadPositionSrvCallback)
        self.cmdSub = rospy.Subscriber("cmd_vel", Twist, self.cmdVelCallBack)


    def fixHead(self):
        """
        Loop to fix head on default position if continuous_fix_activated is true
        """
        while not rospy.is_shutdown():
            if self._continuous_fix_activated:
                headYawPos = self._memory_service.getData("Device/SubDeviceList/HeadYaw/Position/Sensor/Value")
                headPitchPos = self._memory_service.getData("Device/SubDeviceList/HeadPitch/Position/Sensor/Value")
                if self.needToFixHead(headPitchPos, headYawPos):
                    # FIXME: problem reactivation tracker when setAngles("HeadPitch") is sent
                    # self._motion_service.setAngles("HeadYaw", self._yaw_value, self._fractionMaxSpeed)
                    # self._motion_service.setAngles("HeadPitch", self._pitch_value, self._fractionMaxSpeed)

                    # So we tried DCM commands, but the problem persists
                    self.dcm_srv(self._pitch_value, self._yaw_value)

            self.stop_tracker()
            time.sleep(0.1)

    def needToFixHead(self, headPitchPos, headYawPos):
        """
        Check if the head is too far from the default position
        """
        result = False
        if headPitchPos > (self._pitch_value + self._error) or headPitchPos < (self._pitch_value - self._error):
            result = True
        if headYawPos > (self._yaw_value + self._error) or headYawPos < (self._yaw_value - self._error):
            result = True
        return result

    def stop_tracker(self):
        """
        Disable tracker if needed
        """
        if self._tracker_service.isActive():
            print("Tracker is active")
            self._tracker_service.stopTracker()

    def setHeadPositionSrvCallback(self, req):
        """
        Service with a head pitch and yaw positions, and a boolean if you want to change the continuous_fix_activated.
        The robot will move the head to the given position and enable/disable the autonomous fix of the head on the default position
        """
        self._pitch_value = req.pitch_value
        self._yaw_value = req.yaw_value
        self._continuous_fix_activated = req.continuous_fix

        if not self._continuous_fix_activated:
            # self._motion_service.setAngles("HeadYaw", self._yaw_value, self._fractionMaxSpeed)
            # self._motion_service.setAngles("HeadPitch", self._pitch_value, self._fractionMaxSpeed)
            self.dcm_srv(self._pitch_value, self._yaw_value)

            self._autolife_service.setAutonomousAbilityEnabled("BasicAwareness", True)
            self._basic_awareness_service.setStimulusDetectionEnabled("Sound", False)
        else:
            self._autolife_service.setAutonomousAbilityEnabled("BasicAwareness", False)
        return True

    def cmdVelCallBack(self, data):
        """
        When the robot turns on the right, it moves the head to the right, and same for left and forward
        """
        self._pitch_value = self.HEAD_PITCH_OBSTACLE

        # if the cmd turn on the right
        if data.angular.z < 0:
            self._yaw_value = self.HEAD_YAW_OBSTACLE_RIGHT

        # if the cmd turn on the left
        elif data.angular.z > 0:
            self._yaw_value = self.HEAD_YAW_OBSTACLE_LEFT

        elif data.angular.z == 0:
            self._yaw_value = 0.0

    def dcm_srv(self, pitch, yaw):
        """
        Sends positions of head pitch and head yaw with DCM commands
        """
        # FIXME: even the DCM command on HeadPitch reactivates the tracker
        self._dcm_proxy_service.set(["Device/SubDeviceList/HeadPitch/Position/Actuator/Value", "ClearAll", [[pitch, self._dcm_proxy_service.getTime(500)]]])
        self._dcm_proxy_service.set(["Device/SubDeviceList/HeadYaw/Position/Actuator/Value", "ClearAll", [[yaw, self._dcm_proxy_service.getTime(500)]]])


if __name__ == "__main__":
    rospy.init_node('pepper_head_pose_fix')
    ip = rospy.get_param('~ip', "192.168.1.189")
    port = rospy.get_param('~port', 9559)

    s = qi.Session()
    try:
        s.connect("tcp://" + ip + ":" + str(port))
    except RuntimeError:
        print ("Can't connect to Naoqi at ip \"" + ip + "\" on port " + str(port) + ".\n"
               "Please check your script arguments. Run with -h option for help.")
        sys.exit(1)
    headFix = HeadFix(s, ip, port)
    headFix.fixHead()
    rospy.spin()
