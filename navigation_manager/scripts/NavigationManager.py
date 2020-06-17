#!/usr/bin/env python
__author__ = 'Kathrin Evers'
__author__ ='Jacques Saraydaryan'

import json
import math

import actionlib
import rospy
import tf
import tf.transformations
from geometry_msgs.msg import Pose, PoseStamped
from move_base_msgs.msg import MoveBaseAction
import dynamic_reconfigure.client

from map_manager.srv import *
# from nav_pos_gateway_client.msg import NavGoalToGateway
# from follow_me_mng.msg import FollowMeCmd
from navigation_manager.msg import NavMngAction
from robocup_msgs.msg import gm_bus_msg

from common.navstrategy.GoAndRetryNavStrategy import GoAndRetryNavStrategy
from common.navstrategy.GoCRRCloseToGoal import GoCRRCloseToGoal
from common.navstrategy.GoCleanRetryNavStrategy import GoCleanRetryNavStrategy
from common.navstrategy.GoCleanRetryReplayLastNavStrategy import GoCleanRetryReplayLastNavStrategy
from common.navstrategy.SimplyGoNavStrategy import SimplyGoNavStrategy
from common.navstrategy.itm_clean_retry_nav import ItMCleanRetryNav
from common.navstrategy.itm_retry_nav import ItMRetryNav
from common.navstrategy.itm_simple_nav import ItMSimpleNav


class Nm:
    # Initialize global variables
    _gm_bus_pub = ""
    _gm_bus_sub = ""
    _getPoint_service = ""
    _pub_goal = ""
    _sub_goal = ""
    #_pub_follow_me_order = ""
    #_sub_follow_me_answer = ""
    _tflistener = ""
    _current_itm = ""
    _current_action_id = ""
    _current_action_result = 0
    _current_payload = ""
    _current_message_id = 0
    _current_message_action = ""
    _current_gateway_id = ""
    _repetitions = 0
    #_current_follow_id = 0
    #_last_body_time = 0
    #_current_body_id = ""
    _actMove_base=""
    _actionToServiceMap={}
    _navigationStrategyMap={}
    _path = []

    def __init__(self):
        rospy.init_node('navigation_management_server')

        #initiate function on action
        self._actionToServiceMap={"NP": self.npAction,
                                  "Goto": self.gotoAction,
                                  "NF": self.nfAction,
                                  "NFS": self.nfsAction,
                                  "NT": self.ntAction
                                  }
        
        # initialize services and topics as well as function calls
        self._gm_bus_pub = rospy.Publisher("gm_bus_answer", gm_bus_msg, queue_size=1)
        self._gm_bus_sub = rospy.Subscriber("gm_bus_command", gm_bus_msg, self.gmBusListener)

        rospy.wait_for_service("/pepper/get_itm", 5)
        self._get_itm_service = rospy.ServiceProxy("/pepper/get_itm", GetItM)
        rospy.wait_for_service("/pepper/modify_itm", 5)
        self._modify_itm_service = rospy.ServiceProxy("/pepper/modify_itm", ModifyItM)
        rospy.wait_for_service("/pepper/make_path", 5)
        self._make_path_service = rospy.ServiceProxy("/pepper/make_path", MakePath)
        rospy.wait_for_service("/pepper/update_graph", 5)
        self.update_graph_service = rospy.ServiceProxy("/pepper/update_graph", UpdateGraph)

        # Subscribe to the move_base action server
        self._actMove_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        self._actMove_base.wait_for_server(rospy.Duration(60))
        #FIXME What happen if action server is not available ?
        rospy.loginfo("Connected to move base server")
        self.dyn_rec_client = dynamic_reconfigure.client.Client("/move_base/DWAPlannerROS")


        self._navigationStrategyMaps={"Simple": SimplyGoNavStrategy(self._actMove_base),
                                      "Retry": GoAndRetryNavStrategy(self._actMove_base),
                                      "CleanAndRetry": GoCleanRetryNavStrategy(self._actMove_base),
                                      "CleanRetryReplay": GoCleanRetryReplayLastNavStrategy(self._actMove_base),
                                      "CRRCloseToGoal": GoCRRCloseToGoal(self._actMove_base),
                                      "ItMSimple": ItMSimpleNav(self._actMove_base),
                                      "ItMRetry": ItMRetryNav(self._actMove_base),
                                      "ItMCR": ItMCleanRetryNav(self._actMove_base)
                                      }
        self._tflistener = tf.TransformListener()

        self._actionServer = actionlib.SimpleActionServer('navigation_manager', NavMngAction, self.executeActionServer, False)
        self._actionServer.start()

        rospy.spin()

    def gmBusListener(self, data):
        if data.action in self._actionToServiceMap.keys():
            try:
                #call the process associating to the action
                self._actionToServiceMap[data.action](data)
            except Exception as e:
                rospy.logwarn("unable to find or launch function corresponding to the action:, error:[%s]:%s",str(data.action), str(e))


    def executeActionServer(self, goal):
        isActionSucceed=False
        try:
            # call the process associating to the action
            #isActionSucceed=self._actionToServiceMap[data.action](data)
            # FIXME need to rework all _actionToServiceMap call...
            if goal.action == "NP":
                current_navigationStrategy = self._navigationStrategyMaps[goal.navstrategy]
                isActionSucceed = self.navigateToGoal("None", goal.action, current_navigationStrategy, goal.goal_pose, goal.make_plan_mode)
            elif goal.action == "NT":
                self.turnAround(float(math.pi))
                isActionSucceed = True
        except Exception as e:
            rospy.logwarn("unable to find or launch function corresponding to the action %s:, error:[%s]",str(goal.action), str(e))
        if isActionSucceed:
            self._actionServer.set_succeeded()
            print("Navigation succeeded")
        else:
            print("Navigation aborted")
            self._actionServer.set_aborted()
        return

    def get_current_pose(self):
        """
        Returns the pose stamped of the robot
        """
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

        return robotPose

    @staticmethod
    def convert_to_pose_stamped(pose):
        tmp = PoseStamped()
        tmp.header.frame_id = "map"
        tmp.header.stamp = rospy.Time.now()
        tmp.pose = pose

        return tmp

    def navigateToGoal(self, current_message_id, current_message_action, navigationStrategy, pose, planning_mode):
        result = False
        self.create_path(pose, planning_mode)
        self.reconfigure_param('yaw_goal_tolerance', 3.5)
        # Call the navigation strategy to reach the goal
        navigationStrategy.startTimeWatch()
        for pt in self._path:
            if pt[0] != "goal":
                result = navigationStrategy.goto(pt[1])
                if not result:
                    self.update_graph("blocked", pt[0])
                    self._modify_itm_service(pt[0], "color", "RED")
                else:
                    self.update_graph("free", pt[0])
                    self._modify_itm_service(pt[0], "color", "GREEN")
            else:
                self.reconfigure_param('yaw_goal_tolerance', 0.1)
                result = navigationStrategy.goto(pt[1])

        # Step 3: Send result to the general Manager
        resultId=4  # Failure
        if result:
            rospy.loginfo("Goal reached")
            resultId=3  # Success

        gm_result = gm_bus_msg()
        gm_result.action = current_message_action
        gm_result.action_id = current_message_id
        gm_result.payload = ""
        gm_result.result = resultId
        self._gm_bus_pub.publish(gm_result)
        return result

    def create_path(self, goal_pose, mode):
        # Clean the list of points
        self.clean_path()
        # Get the two poses as geometry_msgs/PoseStamped messages
        current_pose = self.get_current_pose()
        tmp_pose_stamped = self.convert_to_pose_stamped(goal_pose)
        # Call the make_path service, add ItMs and the goal
        for itm in self._make_path_service(mode, current_pose, tmp_pose_stamped).list_of_itms:
            self.add_itm_on_path(itm)
        self._path.append(["goal", goal_pose])

    def add_itm_on_path(self, itm):
        self._path.append([itm.name, itm.pose])
        self._modify_itm_service(itm.name, "color", "BLUE")

    def remove_itm_on_path(self, itm_name):
        for obj in self._path:
            if obj[0] == itm_name:
                self._path.remove(obj)

    def clean_path(self):
        self._path = []

    def reconfigure_param(self, param, value):
        new_config = {param: value}
        self.dyn_rec_client.update_configuration(new_config)

    def update_graph(self, action, node):
        if action == "blocked":
            self.update_graph_service("", node, 30.0)
        elif action == "free":
            self.update_graph_service("", node, 1.0)
        elif action == "reset":
            self.update_graph_service("reset", node, 1.0)
        return

    #
    # navigate by following a person
    #
    def follow(self):
        ## publish a follow me order in the corresponding topic
        #self._current_follow_id = str(uuid.uuid1())
        #follow_order = FollowMeCmd()
        #follow_order.safe_zone_radius = 0
        #follow_order.id_people = "ALL"
        #follow_order.follow = True
        #follow_order.id = ""
        #follow_order.result = -1
        ##self._pub_follow_me_order.publish(follow_order)
        pass

    def followStop(self):
        ## publish a follow me order in the corresponding topic
        #follow_order = FollowMeCmd()
        #follow_order.safe_zone_radius = 0
        #follow_order.id_people = self._current_payload
        #follow_order.follow = False
        #follow_order.id = self._current_follow_id
        #follow_order.result = -1
        ##self._pub_follow_me_order.publish(follow_order)
        pass

    def followAnswer(self, data):
        ## answer of the following
        #if data.id == self._current_follow_id:
        #    if data.result == 4:  # failure
        #        # FIXME: alternative procedure if following didn't succeed, e.g. go to exit door, announce the loss
        #        print "I lost my operator :("
        #    elif data.result == 3:  # success
        #        # FIXME: Does this ever happen?
        #        print "I successfully followed my operator"
        pass

    #
    # turn around
    #
    def turnAround(self, data):

        # 1: load current orientation
        rospy.loginfo("In turn around function")
        now = rospy.Time.now()
        self._tflistener.waitForTransform("/map", "base_footprint", now, rospy.Duration(2))
        (trans, rot) = self._tflistener.lookupTransform("/map", "base_footprint", now)
        robotPose = Pose()
        robotPose.position.x = trans[0]
        robotPose.position.y = trans[1]
        robotPose.position.z = trans[2]

        quaternion = (rot[0], rot[1], rot[2], rot[3])
        euler = tf.transformations.euler_from_quaternion(quaternion)
        roll = euler[0]
        pitch = euler[1]
        yaw = euler[2] + data
        q = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
        robotPose.orientation.x = q[0]
        robotPose.orientation.y = q[1]
        robotPose.orientation.z = q[2]
        robotPose.orientation.w = q[3]

        #FIXME TO BE TESTED
        #strategy to apply
        current_navigationStrategy=self._navigationStrategyMaps["CRRCloseToGoal"]
        #Use a navigation strategy
        rospy.loginfo(robotPose)
        result= current_navigationStrategy.goto(None,robotPose)
        return result
        
        #self._current_gateway_id = str(uuid.uuid1())
        #goal_message = NavGoalToGateway()
        #goal_message.uuid = self._current_gateway_id
        #goal_message.pose = robotPose
        #goal_message.result = -1
        #self._pub_goal.publish(goal_message)

        # gm_result = gm_bus_msg()
        # gm_result.action = self._current_message_action
        # gm_result.action_id = self._current_message_id
        # gm_result.payload = ""
        # gm_result.result = 3
        # self._gm_bus_pub.publish(gm_result)
    def npAction(self,data):
        # navigate to point
        self._current_itm = self._get_itm_service(data.payload)
        rospy.loginfo("navigation: interest point = %s", self._current_itm)
        self._current_message_action = data.action
        self._current_message_id = data.action_id
        self._repetitions = 0
        #strategy to apply
        #current_navigationStrategy=self._navigationStrategyMaps["Simple"]
        #current_navigationStrategy=self._navigationStrategyMaps["CleanAndRetry"]  
        current_navigationStrategy=self._navigationStrategyMaps["CleanRetryReplay"]  
             
        self.navigateToGoal(self._current_message_id, self._current_message_action, current_navigationStrategy, self._current_itm.itm.pose)

    def gotoAction(self,data):
        # navigate to point
        self._current_message_action = data.action
        self._current_message_id = data.action_id
        rad = float()
        try:
            rospy.loginfo("trying to execute the Goto action")
            payload = json.loads(data.payload)
            current_angle = payload["rotation"]
            rospy.loginfo("received this angle: %s", current_angle)
            if float(current_angle) == 0:
                rad = 1
            else:
                rad = float(current_angle) / 180 * float(math.pi)
            rospy.loginfo("angle in rad: %f", rad)
        except Exception as e:
            rospy.logwarn("unable to load payload content [%s]", str(e))
        self.turnAround(rad)

    def nfAction(self,data):
        # navigate while following a human
        # data.payload
        self._current_message_action = data.action
        self._current_message_id = data.action_id
        self._current_payload= data.payload
        self.follow()
        
    def nfsAction(self,data):
        # stop navigating while following a human
        # data.payload
        self._current_message_action = data.action
        self._current_message_id = data.action_id
        self.followStop()
         
    def ntAction(self,data):
        # turn around (180 degrees)
        self._current_message_action = data.action
        self._current_message_id = data.action_id
        self.turnAround(float(math.pi))


if __name__ == '__main__':
    nm = Nm()
