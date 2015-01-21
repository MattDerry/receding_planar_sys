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
import roslib
import copy
import visualization_msgs.msg as VM
from visualization_msgs.msg import Marker
import std_srvs.srv as SS
from geometry_msgs.msg import Pose as P
from geometry_msgs.msg import Point
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Twist
from puppeteer_msgs.msg import PlanarSystemState
from puppeteer_msgs.msg import OperatingCondition
from puppeteer_msgs.srv import OperatingConditionChange
from puppeteer_msgs.srv import OperatingConditionChangeRequest
from puppeteer_msgs.srv import InputTrajectory
from puppeteer_msgs.srv import InputTrajectoryError
from sensor_msgs.msg import Joy
import system_definition as sd
import tf
import numpy as np
import random
import math
import yaml
import os
import time
import csv

# global constants
DT = 1/100.
MARKER_DT = 1/20.
MARKERWF = 'optimization_frame'
MARKERFRAME = 'mass_ref_frame'
XLIM = None
YLIM = None
limit_marker = None
obstacle_markers = []

TASK_COLLISION_TIME_PENALTY = 7.0
ERROR_SCALING = 450.0

def makeLimitMarker():
    print XLIM
    print YLIM
    marker = Marker()
    marker.type = Marker.LINE_STRIP
    marker.scale.x = 0.05
    marker.color.a = 1.0
    marker.points = []
    marker.lifetime = rospy.Duration()
    marker.header.frame_id = MARKERWF
    # Lower left
    marker.points.append(Point(XLIM[0], YLIM[0], 0.0))
    # Upper left
    marker.points.append(Point(XLIM[0], YLIM[1], 0.0))
    # Upper right
    marker.points.append(Point(XLIM[1], YLIM[1], 0.0))
    # Lower right
    marker.points.append(Point(XLIM[1], YLIM[0], 0.0))
    # repeat lower left
    marker.points.append(Point(XLIM[0], YLIM[0], 0.0))
    rospy.loginfo("[JOY] Made Limit Marker")
    return marker

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

def makeTimingMarker():
        marker = Marker()
        marker.type = Marker.TEXT_VIEW_FACING
        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 0.08
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.2
        marker.color.a = 0.6
        marker.pose.position.x = -0.55
        marker.pose.position.y = 0.67
        marker.pose.position.z = 0.0
        marker.text = "Task Duration: 0.0 seconds"
        marker.lifetime = rospy.Duration()
        marker.id = 1
        marker.header.frame_id = MARKERWF
        rospy.loginfo("[CRANE] Made Timing Marker")
        return marker

def makePenaltyMarker():
        marker = Marker()
        marker.type = Marker.TEXT_VIEW_FACING
        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 0.08
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 0.8
        marker.pose.position.x = -0.62
        marker.pose.position.y = 0.6
        marker.pose.position.z = 0.0
        marker.text = "Penalty: +0.0 seconds"
        marker.lifetime = rospy.Duration()
        marker.id = 1
        marker.header.frame_id = MARKERWF
        rospy.loginfo("[CRANE] Made Timing Marker")
        return marker

def makeCollisionMarker():
    marker = Marker()
    marker.type = Marker.CUBE
    marker.scale.z = 0.01
    marker.color.r = 1.0
    marker.color.a = 1.0
    marker.lifetime = rospy.Duration(1.0)
    marker.header.frame_id = MARKERWF
    marker.action = Marker.ADD;
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

class ObstacleStates:
    def __init__(self):
        self.INACTIVE = 0
        self.PREACTIVE = 1
        self.ACTIVE = 2

class Obstacle:
    def __init__(self, obstacle, id, state):
        self.marker = makeObstacleMarker(obstacle, id)
        self.possible_states = ObstacleStates()
        self.state = self.possible_states.INACTIVE
        self.activation_time = None
        self.preactivation_time = None


    def within_limits(self):
        if (self.marker.pose.position.x + self.marker.scale.x/2) <= XLIM[1] and (self.marker.pose.position.x - self.marker.scale.x/2) >= XLIM[0]:
            if (self.marker.pose.position.y + self.marker.scale.y/2) <= YLIM[1] and (self.marker.pose.position.y - self.marker.scale.y/2) >= YLIM[0]:
                return True
            else:
                return False
        else:
            return False

    def target_intersection(self, target):
        if target.pose.position.x <= (self.marker.pose.position.x + self.marker.scale.x/2) and target.pose.position.x >= (self.marker.pose.position.x - self.marker.scale.x/2):
            if target.pose.position.y <= (self.marker.pose.position.y + self.marker.scale.y/2) and target.pose.position.y >= (self.marker.pose.position.y - self.marker.scale.y/2):
                return True
            else:
                return False
        else:
            return False

    def randomize(self, width_limits, height_limits, xpos_limits):
        self.marker.scale.x = random.random() * (width_limits[1] - width_limits[0]) + width_limits[0]
        self.marker.scale.y = random.random() * (height_limits[1] - height_limits[0]) + height_limits[0]
        self.marker.pose.position.x = random.random() * (xpos_limits[1] - xpos_limits[0]) + xpos_limits[0]
        self.marker.pose.position.y = YLIM[0] + (self.marker.scale.y/2)

    def activate(self, lifetime):
        self.marker.color.a = 1.0
        self.marker.lifetime = rospy.Duration(lifetime)
        self.state = self.possible_states.ACTIVE
        self.activation_time = rospy.Time.now()

    def preactivate(self):
        self.marker.color.a = 0.25
        self.state = self.possible_states.PREACTIVE
        self.preactivation_time = rospy.Time.now()

    def deactivate(self):
        self.state = self.possible_states.INACTIVE
        self.activation_time = None
        self.preactivation_time  = None


class TaskStateMachine:
    def __init__(self, targets, left_targets, right_targets):
        self.states = States()
        self.all_targets = []
        self.all_left_targets = []
        self.all_right_targets = []
        self.available_targets = []
        self.used_targets = []
        self.collision_marker = makeCollisionMarker()
        self.current_target = None
        self.current_target_marker = makeTargetMarker()
        self.timing_marker = makeTimingMarker()
        self.penalty_marker = makePenaltyMarker()
        self.task_start_time = rospy.Time.now()
        self.task_duration = 0.0
        self.task_final_time = 0.0
        self.task_penalties = 0.0
        self.consecutive_in_targets = 0
        self.in_obst = False
        self.IN_TARGETS_THRESHOLD = 20
        for t in left_targets:
            self.all_left_targets.append(Targets(t[0], t[1], t[2]))
        for t in right_targets:
            self.all_right_targets.append(Targets(t[0], t[1], t[2]))
        index = random.randint(0, len(self.all_left_targets)-1)
        self.all_targets.append(self.all_left_targets[index])
        index = random.randint(0, len(self.all_right_targets)-1)
        self.all_targets.append(self.all_right_targets[index])
        rospy.loginfo("[CRANE] Has %d targets", len(targets))
        self.available_targets = self.all_targets
        self.current_state = self.states.READY
        # create current target marker publisher
        self.target_pub = rospy.Publisher("current_target", Marker, queue_size=1)
        self.task_timer_pub = rospy.Publisher("task_duration", Marker, queue_size=2)
        self.task_pen_pub = rospy.Publisher("task_penalties", Marker, queue_size=1)
        self.collision_pub = rospy.Publisher("collision", Marker, queue_size=1)
        rospy.wait_for_service("/operating_condition_change")
        self.op_change_client = rospy.ServiceProxy("/operating_condition_change", OperatingConditionChange)
        rospy.wait_for_service("get_trajectory_error")
        self.trajectory_error_client = rospy.ServiceProxy("get_trajectory_error", InputTrajectoryError)
        rospy.wait_for_service("get_trajectory")
        self.trajectory_client = rospy.ServiceProxy("get_trajectory", InputTrajectory)

    def updateState(self, mass_pos):
        if self.current_state == self.states.READY:
            if not self.available_targets:
                self.available_targets = self.all_targets
                self.used_targets = []
                self.current_target = None
                self.consecutive_in_targets = 0
            else:
                rospy.loginfo("[CRANE]Published Target")
                self.start_timer()
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
                self.timing_marker.header.stamp = rospy.Time.now()
                self.penalty_marker.header.stamp = rospy.Time.now()
                self.task_timer_pub.publish(self.timing_marker)
                self.task_pen_pub.publish(self.penalty_marker)
                self.consecutive_in_targets = 0
                self.current_state = self.states.TRACKING_TO_TARGET
        elif self.current_state == self.states.COMPLETED_TARGET:
                if not self.available_targets:
                    rospy.loginfo("[CRANE]Task Completed")
                    self.stop_timer()
                    pkg_dir = roslib.packages.get_pkg_dir("receding_planar_sys")
                    file_name = pkg_dir + '/data/user_task_time_log.csv'
                    with open(file_name, 'a') as timelogfile:
                        wr = csv.writer(timelogfile, quoting=csv.QUOTE_ALL)
                        wr.writerow([self.task_duration, self.task_penalties, self.task_final_time])
                        rospy.loginfo("Wrote time log entry")
                    self.current_state = self.states.COMPLETED_TASK
                    try:
                        self.op_change_client(OperatingCondition(OperatingCondition.STOP))
                        time.sleep(2.0)
                        res = self.trajectory_client()
                        error_response = self.trajectory_error_client(res.ref_trajectory, res.times)
                        rospy.loginfo("[CRANE] Error for use in trust calculation: %f", error_response.rms)
                        trial_trust = self.calc_trust(error_response.rms)
                        rospy.loginfo("[CRANE] Trust for last trial: %.3f", trial_trust)
                        if rospy.has_param("user_trust_history"):
                            trust_history = rospy.get_param("user_trust_history")
                            trust_history.append(trial_trust)
                        else:
                            trust_history = [1.0, trial_trust]
                        rospy.loginfo("Write history file")
                        file_name = pkg_dir + '/launch/user_trust_history.yaml'
                        with open(file_name, 'w') as yaml_file:
                            data = {'user_trust_history' : trust_history}
                            yaml_file.write( yaml.dump(data, default_flow_style=False))
                            rospy.loginfo("Wrote history file")
                        rospy.loginfo("Done")
                        rospy.loginfo(os.getcwd())
                    except rospy.ServiceException, e:
                        rospy.loginfo("Task Coordinator client: Service did not process stop request: %s"%str(e))
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
            if self.in_current_target(mass_pos):
                rospy.logdebug("[CRANE]In Target")
                self.consecutive_in_targets += 1
                if self.consecutive_in_targets > self.IN_TARGETS_THRESHOLD:
                    rospy.loginfo("[CRANE]Completed Target")
                    self.current_state = self.states.COMPLETED_TARGET
            elif self.in_obstacle(mass_pos):
                rospy.logdebug("[CRANE]In Obstacle")
                if not self.in_obst:
                    self.task_penalties += TASK_COLLISION_TIME_PENALTY
                    self.penalty_marker.text = "Penalty: +%.1f seconds" % self.task_penalties
                    self.penalty_marker.header.stamp = rospy.Time.now()
                    self.task_pen_pub.publish(self.penalty_marker)
                    self.collision_marker.header.stamp = rospy.Time.now()
                    self.collision_pub.publish(self.collision_marker)
                self.in_obst = True
            else:
                rospy.logdebug("[CRANE]Reset Current Target")
                self.consecutive_in_targets = 0
                self.in_obst = False


    def distance(self, p1x, p1y, p2x, p2y):
        return math.sqrt(math.pow(p2x-p1x,2) + math.pow(p2y - p1y, 2))


    def in_current_target(self, mass_pos):
        dist = self.distance(mass_pos.xm, mass_pos.ym, self.current_target.x, YLIM[0] + self.current_target.y)
        if dist > self.current_target.radius:
            return False
        else:
            return True


    def in_obstacle(self, mass_pos):
        for obstacle in obstacle_markers:
            xmin = obstacle.pose.position.x - (obstacle.scale.x/2 + 0.025)
            xmax = xmin + obstacle.scale.x + 0.05
            ymin = obstacle.pose.position.y - (obstacle.scale.y/2 + 0.025)
            ymax = ymin + obstacle.scale.y + 0.05
            if self.in_range(mass_pos, xmin, xmax, ymin, ymax):
                self.collision_marker.pose.position.x = obstacle.pose.position.x
                self.collision_marker.pose.position.y = obstacle.pose.position.y
                self.collision_marker.scale.x = obstacle.scale.x
                self.collision_marker.scale.y = obstacle.scale.y
                self.collision_marker.points = []
                return True
        return False


    def in_range(self, mass_pos, xmin, xmax, ymin, ymax):
        if mass_pos.xm >= xmin and mass_pos.xm <= xmax and mass_pos.ym >= ymin and mass_pos.ym <= ymax:
            return True
        else:
            return False

    def start_timer(self):
        self.task_start_time = rospy.Time.now()
        self.task_timer_timer = rospy.Timer(rospy.Duration(MARKER_DT), self.task_timer_cb)
        return

    def stop_timer(self):
        self.task_duration = (rospy.Time.now() - self.task_start_time).to_sec()
        self.task_final_time = self.task_duration + self.task_penalties
        rospy.loginfo("[CRANE] Final task time: %.2f", self.task_final_time)
        self.task_timer_timer.shutdown()

    def task_timer_cb(self, msg):
        self.task_duration = (rospy.Time.now() - self.task_start_time).to_sec()
        self.timing_marker.text = "Task Duration: %.1f seconds" % self.task_duration
        self.timing_marker.header.stamp = rospy.Time.now()
        self.task_timer_pub.publish(self.timing_marker)

    def calc_trust(self, rms, success=1):
        trust = 0.0
        trust = success*math.exp(-1*ERROR_SCALING*rms)
        return trust

class CraneTaskCoordinator:
    def __init__(self, targets, left_targets, right_targets):
        # create marker server:
        # self.server = InteractiveMarkerServer("mass_reference_control", q_size=1)
        # create listener and broadcaster
        self.br = tf.TransformBroadcaster()
        self.listener = tf.TransformListener()
        self.tsm = TaskStateMachine(targets, left_targets, right_targets)
        # create subscriber for operating condition
        self.op_cond_sub = rospy.Subscriber("/operating_condition", OperatingCondition, self.op_cb)
        self.operating_condition = OperatingCondition.IDLE
        # create subscriber for current mass position
        self.filt_state_sub = rospy.Subscriber("filt_state", PlanarSystemState, self.state_cb)
        # create publisher for obstacle markers if any exist
        self.obstacle_pub = rospy.Publisher("obstacle_markers", Marker, queue_size=5)
        # publish limit marker if one exists
        self.limit_pub = rospy.Publisher("limit_markers", Marker, queue_size=1)
        return

    def state_cb(self, msg):
        if self.operating_condition == OperatingCondition.RUN:
            self.tsm.updateState(msg)

    def op_cb(self, msg):
        if msg.state == OperatingCondition.RUN and self.operating_condition != OperatingCondition.RUN:
            if limit_marker is not None:
                self.limit_pub.publish(limit_marker)
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
        global limit_marker
        limit_marker = makeLimitMarker()
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
        if rospy.has_param("left_targets"):
            rospy.loginfo("[CRANE]Has left targets")
            left_targets = rospy.get_param("left_targets")
        if rospy.has_param("right_targets"):
            rospy.loginfo("[CRANE]Has right targets")
            right_targets = rospy.get_param("right_targets")
    try:
        sim = CraneTaskCoordinator(targets, left_targets, right_targets)
    except rospy.ROSInterruptException: pass

    rospy.spin()


if __name__=='__main__':
    main()
