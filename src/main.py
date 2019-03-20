#!/usr/bin/env python

import rospy
import cv2
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import cv_bridge
from geometry_msgs.msg import Twist, Pose, PoseStamped, PointStamped
from smach import State, StateMachine
import smach_ros
from dynamic_reconfigure.server import Server
from nav_msgs.msg import Odometry
from ar_track_alvar_msgs.msg import AlvarMarkers
from kobuki_msgs.msg import BumperEvent, Sound, Led
from tf.transformations import decompose_matrix, compose_matrix
from ros_numpy import numpify
from sensor_msgs.msg import Joy, LaserScan, Image
import numpy as np
import angles as angles_lib
import math
import random
from std_msgs.msg import Bool, String, Int32
from ar_track_alvar_msgs.msg import AlvarMarkers
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import tf2_ros
import tf2_geometry_msgs
import tf

TAGS_FOUND = []
START_POSE = None
TAG_POSE = None
CURRENT_POSE = None
client = None
TAGS_IN_TOTAL = 3
CURRENT_STATE = None


class Turn(State):
    def __init__(self):
        State.__init__(self, outcomes=["found"],
                       output_keys=["goal", "current_marker"])
        self.rate = rospy.Rate(10)
        self.cmd_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        self.marker_sub = rospy.Subscriber(
            'ar_pose_marker_base', AlvarMarkers, self.marker_callback)
        self.detected_marker = None
        self.rate = rospy.Rate(30)
        self.distance_from_marker = 0.8

        self.listener = tf.TransformListener()

    def execute(self, userdata):
        global CURRENT_STATE
        CURRENT_STATE = "turn"

        self.detected_marker = None
        while self.detected_marker is None:
            msg = Twist()
            msg.linear.x = 0.0
            msg.angular.z = -0.4
            self.cmd_pub.publish(msg)
            self.rate.sleep()

        print "detected marker " + str(self.detected_marker)
        userdata.current_marker = self.detected_marker

        pose = PointStamped()
        pose.header.frame_id = "ar_marker_" + str(self.detected_marker)
        pose.header.stamp = rospy.Time(0)
        pose.point.z = -self.distance_from_marker

        self.listener.waitForTransform("odom", pose.header.frame_id, rospy.Time(0),rospy.Duration(4))
        
        pose_transformed = self.listener.transformPoint("odom", pose)

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "odom"
        goal.target_pose.pose.position.x = pose_transformed.point.x
        goal.target_pose.pose.position.y = pose_transformed.point.y
        goal.target_pose.pose.orientation = START_POSE.orientation
        userdata.goal = goal

        self.detected_marker = None
        self.cmd_pub.publish(Twist())

        return "found"

    def marker_callback(self, msg):
        global TAG_POSE
        global CURRENT_STATE

        if CURRENT_STATE == "turn" and len(msg.markers) > 0:
            msg = msg.markers[0]

            TAG_POSE = msg.pose.pose
            self.detected_marker = msg.id

class MoveBehind(State):
    def __init__(self):
        State.__init__(self, outcomes=["failed", "done"],
                            input_keys=["goal", "current_marker"],
                            output_keys=["current_marker"])
        self.rate = rospy.Rate(10)

        self.move_base_client = actionlib.SimpleActionClient(
            "move_base", MoveBaseAction)

    def execute(self, userdata):
        global client, TAGS_FOUND, START_POSE, TAGS_IN_TOTAL, CURRENT_POSE
        global CURRENT_STATE
        CURRENT_STATE = "navigate"

        
        userdata.goal.target_pose.header.stamp = rospy.Time.now()
        result = self.move_base_client.send_goal_and_wait(userdata.goal)

        if result != 3:
            pass #return "failed"

        return "done"

class FixAlignment(State):
    def __init__(self):
        State.__init__(self, outcomes=["failed", "done"],
                            input_keys=["current_marker"],
                            output_keys=["current_marker"])
        self.rate = rospy.Rate(10)

        self.move_base_client = actionlib.SimpleActionClient(
            "move_base", MoveBaseAction)

    def execute(self, userdata):
        # global client, TAGS_FOUND, START_POSE, TAGS_IN_TOTAL, CURRENT_POSE
        # global CURRENT_STATE
        # CURRENT_STATE = "navigate"

        
        # userdata.goal.target_pose.header.stamp = rospy.Time.now()
        # result = self.move_base_client.send_goal_and_wait(userdata.goal)

        # if result != 3:
        #     pass #return "failed"

        return "done"

class PushBox(State):
    def __init__(self):
        State.__init__(self, outcomes=["failed", "done"],
                            input_keys=["current_marker"],
                            output_keys=["current_marker"])
        self.rate = rospy.Rate(10)

        self.move_base_client = actionlib.SimpleActionClient(
            "move_base", MoveBaseAction)
        self.cmd_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)

    def execute(self, userdata):
        while not rospy.is_shutdown():
            msg = Twist()
            msg.linear.x = 0.3
            msg.angular.z = 0.0
            self.cmd_pub.publish(msg)
            self.rate.sleep()

        return "done"


def odom_callback(msg):
    global CURRENT_POSE, START_POSE

    CURRENT_POSE = msg.pose.pose
    if START_POSE == None:
        START_POSE = CURRENT_POSE


if __name__ == "__main__":
    rospy.init_node('demo6')

    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()

    sm = StateMachine(outcomes=['success', 'failure'])
    sm.userdata.goal = None

    rospy.Subscriber("odom", Odometry, callback=odom_callback)

    with sm:

        StateMachine.add("FindAR", Turn(), transitions={
                         "found": "MoveBehind"})

        StateMachine.add("MoveBehind", MoveBehind(), transitions={
                         "done": "PushBox", "failed": "MoveBehind"})

        StateMachine.add("PushBox", PushBox(), transitions={
                         "done": "success", "failed": "failure"})

    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()
    outcome = sm.execute()
    sis.stop()
