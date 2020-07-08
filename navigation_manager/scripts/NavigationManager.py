#!/usr/bin/env python
__author__ = 'Kathrin Evers'
__author__ ='Jacques Saraydaryan'

import json
import math

import actionlib
import rospy
import tf
import tf.transformations
from geometry_msgs.msg import PoseStamped
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
from common.navstrategy.go_through_door import GoThroughDoor


class Nm:
    #_pub_follow_me_order = ""
    #_sub_follow_me_answer = ""
    _current_node = ""
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

    def __init__(self):
        rospy.init_node('navigation_management_server')

        self._nodes_path = []
        self._edges_path = []
        self._current_pose = PoseStamped()
        self._tflistener = tf.TransformListener()
        
        # initialize services and topics as well as function calls
        self._gm_bus_pub = rospy.Publisher("gm_bus_answer", gm_bus_msg, queue_size=1)
        self._gm_bus_sub = rospy.Subscriber("gm_bus_command", gm_bus_msg, self.gmBusListener)

        rospy.wait_for_service("/pepper/get_node", 5)
        self._get_node_service = rospy.ServiceProxy("/pepper/get_node", GetNode)
        rospy.wait_for_service("/pepper/modify_node", 5)
        self._modify_node_service = rospy.ServiceProxy("/pepper/modify_node", ModifyNode)
        rospy.wait_for_service("/pepper/make_path", 5)
        self._make_path_service = rospy.ServiceProxy("/pepper/make_path", MakePath)
        rospy.wait_for_service("/pepper/update_graph", 5)
        self._update_graph_service = rospy.ServiceProxy("/pepper/update_graph", UpdateGraph)

        # Subscribe to the move_base action server
        self._actMove_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        if not self._actMove_base.wait_for_server(rospy.Duration(60)):
            #FIXME What happen if action server is not available ?
            rospy.logerr("Can't connect to move_base Action Server")
            return

        rospy.loginfo("Connected to move base server")
        self._dyn_rec_client = dynamic_reconfigure.client.Client("/move_base/DWAPlannerROS")

        self._navigationStrategyMaps={"Simple": SimplyGoNavStrategy(self._actMove_base),
                                      "Retry": GoAndRetryNavStrategy(self._actMove_base),
                                      "CleanAndRetry": GoCleanRetryNavStrategy(self._actMove_base),
                                      "CleanRetryReplay": GoCleanRetryReplayLastNavStrategy(self._actMove_base),
                                      "CRRCloseToGoal": GoCRRCloseToGoal(self._actMove_base),
                                      "ItMSimple": ItMSimpleNav(self._actMove_base),
                                      "ItMRetry": ItMRetryNav(self._actMove_base),
                                      "ItMCR": ItMCleanRetryNav(self._actMove_base),
                                      "GTD": GoThroughDoor(self._actMove_base)
                                      }
        self.gtd_strategy = self._navigationStrategyMaps["GTD"]

        # initiate function on action
        self._actionToServiceMap = {"NP": self.npAction,
                                    "Goto": self.gotoAction,
                                    "NF": self.nfAction,
                                    "NFS": self.nfsAction,
                                    "NT": self.ntAction
                                    }

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
                if goal.node != "":
                    print("Navigation goal : node {}".format(goal.node))
                    node_obj = self._get_node_service(goal.node).node
                    if node_obj.name == "":
                        rospy.logerr("Node of name {} doesn't exist, cancelling navigation".format(goal.node))
                        self._actionServer.set_aborted()
                        return
                    pose = node_obj.pose
                else:
                    pose = goal.goal_pose
                isActionSucceed = self.navigateToGoal("None", goal.action, current_navigationStrategy,
                                                      goal.make_plan_mode, goal.side_door_crossing, pose)

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

    def navigateToGoal(self, current_message_id, current_message_action, navigationStrategy, planning_mode, side_door_crossing, goal_pose):
        """
        Navigate to the goal_pose using the navigation strategy and the planning mode corresponding to the parameters
        """
        result = False
        self.create_path(goal_pose, planning_mode)
        self.reconfigure_param('yaw_goal_tolerance', 3.5)

        # Call the navigation strategy to reach all nodes
        navigationStrategy.startTimeWatch()
        for i in range(len(self._nodes_path)):
            pt = self._nodes_path[i]
            result = navigationStrategy.goto(self._current_pose, pt[1])
            if not result:
                self.update_graph("blocked", pt[0])
                self._modify_node_service(pt[0], "color", "RED")
            else:
                self.update_graph("free", pt[0])
                self._modify_node_service(pt[0], "color", "GREEN")
                if pt != self._nodes_path[-1]:
                    # Check if the edge to the next node is crossing a door :
                    edge = self._edges_path[i]
                    if side_door_crossing and edge.is_crossing_door and self.check_edge(edge, pt[0], self._nodes_path[i+1][0]):
                        print("Executing GoThroughDoor strategy to reach node : {}".format(self._nodes_path[i+1]))
                        crossing_result = self.gtd_strategy.goto(self._current_pose.pose, self._nodes_path[i+1][1])

            self._current_pose = self.get_current_pose()

        # Now we reach the goal :
        result = navigationStrategy.goto(self._current_pose, goal_pose)

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
        self._current_pose = self.get_current_pose()
        tmp_pose_stamped = self.convert_to_pose_stamped(goal_pose)

        # Call the make_path service, add the nodes and edges
        tmp_path = self._make_path_service(mode, self._current_pose, tmp_pose_stamped).path
        for node in tmp_path.nodes:
            self.add_node_on_path(node)
        for edge in tmp_path.edges:
            self.add_edge_on_path(edge)

    def add_node_on_path(self, node):
        self._nodes_path.append([node.name, node.pose])
        self._modify_node_service(node.name, "color", "BLUE")

    def add_edge_on_path(self, edge):
        self._edges_path.append(edge)

    def clean_path(self):
        self._nodes_path = []
        self._edges_path = []

    @staticmethod
    def check_edge(edge, node_1, node_2):
        if (edge.first_node == node_1 and edge.second_node == node_2) \
                or (edge.first_node == node_2 and edge.second_node == node_1):
            return True
        return False

    def reconfigure_param(self, param, value):
        new_config = {param: value}
        self._dyn_rec_client.update_configuration(new_config)

    def update_graph(self, action, node):
        if action == "blocked":
            self._update_graph_service("", node, 30.0)
        elif action == "free":
            self._update_graph_service("", node, 1.0)
        elif action == "reset":
            self._update_graph_service("reset", node, 1.0)
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
        robotPose = self.get_current_pose()

        euler = tf.transformations.euler_from_quaternion(robotPose.pose.orientation)
        roll = euler[0]
        pitch = euler[1]
        yaw = euler[2] + data
        q = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
        robotPose.pose.orientation.x = q[0]
        robotPose.pose.orientation.y = q[1]
        robotPose.pose.orientation.z = q[2]
        robotPose.pose.orientation.w = q[3]

        #FIXME TO BE TESTED
        #strategy to apply
        current_navigationStrategy=self._navigationStrategyMaps["CRRCloseToGoal"]
        #Use a navigation strategy
        rospy.loginfo(robotPose)
        result= current_navigationStrategy.goto(self._current_pose, robotPose)
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
        _current_node = self._get_node_service(data.payload).node
        if _current_node.name == "":
            rospy.logerr("Node of name {} doesn't exist, cancelling navigation".format(data.payload))
            return
        rospy.loginfo("navigation to node = %s", _current_node)
        self._current_message_action = data.action
        self._current_message_id = data.action_id
        self._repetitions = 0
        #strategy to apply
        #current_navigationStrategy=self._navigationStrategyMaps["Simple"]
        #current_navigationStrategy=self._navigationStrategyMaps["CleanAndRetry"]  
        current_navigationStrategy=self._navigationStrategyMaps["ItMCR"]

        self.navigateToGoal(self._current_message_id, self._current_message_action, current_navigationStrategy, "euclidian", False, _current_node.pose)

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
