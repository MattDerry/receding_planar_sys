#!/usr/bin/env python
"""
Matt Derry
Jan 2015

This node defines an experimental task that mimics crane usage where obstacle regions
are defined and target goals pop up to move the suspended mass to.

SUBSCRIPTIONS:
    - /operating_condition (OperatingCondition)
    - /joy (sensor_msgs/Joy)

PUBLISHERS:
    - mass_ref_point (PointStamped)
    - mass_ref_frame (tf) ... not really a topic
    - visualization_markers (VisualizationMarkerArray)
"""
import rospy
import copy
import visualization_msgs.msg as VM
from visualization_msgs.msg import Marker
import std_srvs.srv as SS
from geometry_msgs.msg import Pose as P
from geometry_msgs.msg import Point
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Twist
from puppeteer_msgs.msg import OperatingCondition
from puppeteer_msgs.msg import PlanarSystemState
from sensor_msgs.msg import Joy
import system_definition as sd
import tf
import numpy as np
import random
import math

# global constants
DT = 1/100.
MARKERWF = 'optimization_frame'
MARKERFRAME = 'mass_ref_frame'
XLIM = None
YLIM = None
obstacle_markers = []



def makeObstacleMarker(obst, oid):
    marker = Marker()
    marker.type = Marker.CUBE
    marker.scale.x = obst[1]
    marker.scale.y = obst[0]
    marker.scale.z = 0.01
    marker.color.a = 1.0
    marker.id = oid
    marker.pose.position.x = obst[2]
    marker.pose.position.y = YLIM[0] + (obst[0]/2)
    marker.lifetime = rospy.Duration()
    marker.action = Marker.ADD;
    marker.header.frame_id = MARKERWF
    # Lower left
    #rospy.loginfo("OBST2 0: %f, 1: %f, 2: %f", obst[0], obst[1], obst[2])
    # x1 = obst[2] + (obst[1]/2)
    # x2 = obst[2] - (obst[1]/2)
    # #rospy.loginfo("OBST2 x1: %f, x2: %f", x1, x2)
    # marker.points.append(Point(x1, YLIM[0], 0.0))
    # # Upper left
    # marker.points.append(Point(x1, YLIM[0] + obst[0], 0.0))
    # # Upper right
    # marker.points.append(Point(x2, YLIM[0] + obst[0], 0.0))
    # # Lower right
    # marker.points.append(Point(x2, YLIM[0], 0.0))
    # # repeat lower left
    # marker.points.append(Point(x1, YLIM[0], 0.0))
    rospy.loginfo("[CRANE] Made Obstacle Marker")
    return marker

def makeTargetMarker():
        marker = Marker()
        marker.type = Marker.SPHERE
        marker.scale.x = 0.25
        marker.scale.y = 0.25
        marker.scale.z = 0.01
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.color.a = 0.6
        marker.lifetime = rospy.Duration()
        marker.header.frame_id = MARKERWF
        rospy.loginfo("[CRANE] Made Target Marker")
        return marker

class Targets:
    def __init__(self, x, y, r):
        self.x = x
        self.y = y
        self.radius = r

class States:
    def __init__(self):
        self.READY = 0
        self.TRACKING_TO_TARGET = 1
        self.COMPLETED_TARGET = 2
        self.COMPLETED_TASK = 3


class TaskStateMachine:
    def __init__(self, targets):
        self.states = States()
        self.all_targets = []
        self.available_targets = []
        self.used_targets = []
        self.current_target = None
        self.current_target_marker = makeTargetMarker()
        self.consecutive_in_targets = 0
        self.IN_TARGETS_THRESHOLD = 20
        for t in targets:
            self.all_targets.append(Targets(t[0], t[1], t[2]))
        self.current_state = self.states.READY
        # create current target marker publisher
        self.target_pub = rospy.Publisher("current_target", Marker, queue_size=1)

    def updateState(self, mass_pos):
        if self.current_state == self.states.READY or self.current_state == self.states.COMPLETED_TARGET:
            if not self.available_targets:
                rospy.loginfo("[CRANE]Task Completed")
                self.current_state = self.states.COMPLETED_TASK
                self.available_targets = self.all_targets
                self.used_targets = []
                self.current_target = None
                self.current_state = self.states.READY
                self.consecutive_in_targets = 0
            else:
                rospy.loginfo("[CRANE]Published Target")
                index = random.randint(0, len(self.available_targets)-1)
                self.current_target = self.available_targets[index]
                self.current_target_marker.header.stamp = rospy.Time.now()
                self.current_target_marker.pose.position.x = self.available_targets[index].x
                self.current_target_marker.pose.position.y = YLIM[0] + self.available_targets[index].y
                self.current_target_marker.scale.x = self.available_targets[index].radius
                self.current_target_marker.scale.y = self.available_targets[index].radius
                self.used_targets.append(self.available_targets[index])
                del self.available_targets[index]
                #publish current_target
                self.target_pub.publish(self.current_target_marker)
                self.consecutive_in_targets = 0
                self.current_state = self.states.TRACKING_TO_TARGET
        elif self.current_state == self.states.TRACKING_TO_TARGET:
            if self.inCurrentTarget(mass_pos):
                rospy.loginfo("[CRANE]In Target")
                self.consecutive_in_targets += 1
                if self.consecutive_in_targets > self.IN_TARGETS_THRESHOLD:
                    rospy.loginfo("[CRANE]Completed Target")
                    self.current_state = self.states.COMPLETED_TARGET
            else:
                rospy.loginfo("[CRANE]Reset Current Target")
                self.consecutive_in_targets = 0

    def distance(self, p1x, p1y, p2x, p2y):
        return math.sqrt(math.pow(p2x-p1x,2) + math.pow(p2y - p1y, 2))

    def inCurrentTarget(self, mass_pos):
        dist = self.distance(mass_pos.xm, mass_pos.ym, self.current_target.x, YLIM[0] + self.current_target.y)
        rospy.loginfo("[CRANE]P1 (%f, %f), P2 (%f, %f)", mass_pos.xm, mass_pos.ym, self.current_target.x, YLIM[0] + self.current_target.y)
        rospy.loginfo("[CRANE] Distance: %f, Radius: %f", dist, self.current_target.radius)
        if dist > self.current_target.radius:
            return False
        else:
            return True

class CraneTaskCoordinator:
    def __init__(self, targets):
        # create marker server:
        # self.server = InteractiveMarkerServer("mass_reference_control", q_size=1)
        # create listener and broadcaster
        self.br = tf.TransformBroadcaster()
        self.listener = tf.TransformListener()
        self.tsm = TaskStateMachine(targets)
        # create subscriber for operating condition
        self.op_cond_sub = rospy.Subscriber("/operating_condition", OperatingCondition, self.op_cb)
        self.operating_condition = OperatingCondition.IDLE
        # create subscriber for current mass position
        self.filt_state_sub = rospy.Subscriber("filt_state", PlanarSystemState, self.state_cb)
        # create publisher for obstacle markers if any exist
        self.obstacle_pub = rospy.Publisher("obstacle_markers", Marker, queue_size=5)

        return

    def state_cb(self, msg):
        if self.operating_condition == OperatingCondition.RUN:
            self.tsm.updateState(msg)

    def op_cb(self, msg):
        if msg.state == OperatingCondition.RUN and self.operating_condition != OperatingCondition.RUN:
            for obm in obstacle_markers:
                obm.header.stamp = rospy.Time.now()
                self.obstacle_pub.publish(obm)
        self.operating_condition = msg.state

def main():
    rospy.init_node('crane_task_coordinator')

    # are we respecting limits?
    if rospy.has_param("xlim") and rospy.has_param("ylim"):
        global XLIM
        XLIM = rospy.get_param("xlim")
        XLIM = np.sort(np.array(XLIM))
        tmp = rospy.get_param("ylim")
        global YLIM
        YLIM = np.sort(np.array(tmp))
        if rospy.has_param("obstacles"):
            rospy.loginfo("[CRANE]Has obstacles")
            obstacles = rospy.get_param("obstacles")
            global obstacle_markers
            o_id = 0
            for obstacle in obstacles:
                rospy.loginfo("OBST 0: %f, 1: %f, 2: %f", obstacle[0], obstacle[1], obstacle[2])
                obstacle_markers.append(makeObstacleMarker(obstacle, o_id))
                o_id += 1
        if rospy.has_param("targets"):
            rospy.loginfo("[CRANE]Has targets")
            targets = rospy.get_param("targets")

    try:
        sim = CraneTaskCoordinator(targets)
    except rospy.ROSInterruptException: pass

    rospy.spin()


if __name__=='__main__':
    main()
