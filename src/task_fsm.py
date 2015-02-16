#!/usr/bin/env python
"""
Matt Derry
Jan 2015

This TaskStateMachine class defines a base class for an experimental task that
mimics crane usage where obstacle regions are defined and target goals pop up to
 move the suspended mass to.

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
from puppeteer_msgs.srv import UserControlParams
from sensor_msgs.msg import Joy
import system_definition as sd
from obstacle_factory import Obstacle
from obstacle_factory import ObstacleFactory
from obstacle_factory import ObstacleStates
from obstacle_factory import ObstacleSnapshot
from target_factory import TargetStates
from target_factory import Target
from target_factory import TargetFactory
import tf
import numpy as np
import random
import math
import yaml
import os
import time
import csv

class TaskStates:
    def __init__(self):
        self.READY = 0
        self.TRACKING_TO_TARGET = 1
        self.COMPLETED_TARGET = 2
        self.COMPLETED_TASK = 3


class TaskStateMachine:
    def __init__(self, x_world_limits, y_world_limits):
        self.frame_id = 'optimization_frame'
        self.sim_dt = 1/100.
        self.marker_dt = 1/20.
        self.states = TaskStates()
        self.current_target = None
        self.timing_marker = self.make_timing_marker()
        self.score_marker = self.make_score_marker()
        self.task_start_time = rospy.Time.now()
        self.task_duration = 0.0
        self.task_final_time = 0.0
        self.task_penalties = 0.0
        self.task_collisions = 0
        self.consecutive_in_targets = 0
        self.IN_TARGETS_THRESHOLD = 20
        self.COLLISION_TIME_PENALTY = 10
        self.mass_radius = 0.05
        self.in_obstacle = False

        self.obstacles = []
        self.targets = []

        self.xlim = x_world_limits
        self.ylim = y_world_limits

        self.obstacle_factory = ObstacleFactory(self.frame_id, self.xlim, self.ylim)
        self.target_factory = TargetFactory(self.frame_id, self.xlim, self.ylim)

        self.current_state = self.states.READY

        # create current target marker publisher
        self.target_pub = rospy.Publisher("current_target", Marker, queue_size=1)
        self.task_timer_pub = rospy.Publisher("task_duration", Marker, queue_size=2)
        self.task_pen_pub = rospy.Publisher("task_penalties", Marker, queue_size=1)
        self.collision_pub = rospy.Publisher("collision", Marker, queue_size=1)
        self.obstacle_pub = rospy.Publisher("obstacle_markers", Marker, queue_size=5)
        self.score_pub = rospy.Publisher("user_score", Marker, queue_size=2)

        rospy.wait_for_service("/operating_condition_change")
        self.op_change_client = rospy.ServiceProxy("/operating_condition_change", OperatingConditionChange)
        rospy.wait_for_service("get_trajectory_error")
        self.trajectory_error_client = rospy.ServiceProxy("get_trajectory_error", InputTrajectoryError)
        rospy.wait_for_service("get_trajectory")
        self.trajectory_client = rospy.ServiceProxy("get_trajectory", InputTrajectory)
        rospy.wait_for_service("get_user_control_params")
        self.control_params_client = rospy.ServiceProxy("get_user_control_params", UserControlParams)

    def update_state(self, mass_pos):
        return

    def distance(self, p1x, p1y, p2x, p2y):
        return math.sqrt(math.pow(p2x-p1x,2) + math.pow(p2y - p1y, 2))

    def target_in_obstacle(self, target):
        for obstacle in obstacle_markers:
            if obstacle.target_intersection(target):
                return True
        return False

    def start_timer(self):
        self.task_start_time = rospy.Time.now()
        self.task_timer_timer = rospy.Timer(rospy.Duration(self.marker_dt), self.task_timer_cb)
        return

    def stop_timer(self):
        self.task_duration = (rospy.Time.now() - self.task_start_time).to_sec()
        self.task_penalties = self.task_collisions * self.COLLISION_TIME_PENALTY
        self.task_final_time = self.task_duration + self.task_penalties
        rospy.loginfo("Final task time: %.2f", self.task_final_time)
        self.task_timer_timer.shutdown()

    def task_timer_cb(self, msg):
        self.task_duration = (rospy.Time.now() - self.task_start_time).to_sec()
        self.timing_marker.text = "Task Duration: %.1f seconds" % self.task_duration
        self.timing_marker.header.stamp = rospy.Time.now()
        self.task_timer_pub.publish(self.timing_marker)

    def calc_trust(self, error_scaling, rms, success=1):
        trust = 0.0
        trust = success * math.exp(-1 * error_scaling * rms)
        return trust

    def mass_in_obstacle(self, mass_pos):
        # rospy.loginfo("[FSM] Number of Obstacles: %d, mass_radius: %f", len(self.obstacles), self.mass_radius)
        for obstacle in self.obstacles:
            if obstacle.mass_in_collision(mass_pos, self.mass_radius):
                return True
        return False

    def publish_collisions(self, mass_pos):
        for obstacle in self.obstacles:
            if obstacle.mass_in_collision(mass_pos, self.mass_radius):
                obstacle.collision_marker.header.stamp = rospy.Time.now()
                self.collision_pub.publish(obstacle.collision_marker)

    def in_range(self, mass_pos, xmin, xmax, ymin, ymax):
        if mass_pos.xm >= xmin and mass_pos.xm <= xmax and mass_pos.ym >= ymin and mass_pos.ym <= ymax:
            return True
        else:
            return False

    def in_current_target(self, mass_pos):
        dist = self.distance(mass_pos.xm, mass_pos.ym, self.current_target.marker.pose.position.x, self.current_target.marker.pose.position.y)
        if dist > self.current_target.marker.scale.x:
            return False
        else:
            return True

    def make_timing_marker(self):
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
            marker.pose.position.z = -0.1
            marker.text = "Task Duration: 0.0 seconds"
            marker.lifetime = rospy.Duration()
            marker.id = 1
            marker.header.frame_id = self.frame_id
            rospy.loginfo("[CRANE] Made Timing Marker")
            return marker

    def make_penalty_marker(self):
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
            marker.header.frame_id = self.frame_id
            rospy.loginfo("[CRANE] Made Timing Marker")
            return marker

    def make_score_marker(self):
            marker = Marker()
            marker.type = Marker.TEXT_VIEW_FACING
            marker.scale.x = 1.0
            marker.scale.y = 1.0
            marker.scale.z = 0.2
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 0.2
            marker.color.a = 0.8
            marker.pose.position.x = 0.0
            marker.pose.position.y = 0.0
            marker.pose.position.z = -0.1
            marker.text = "Score: 0.0"
            marker.lifetime = rospy.Duration()
            marker.id = 1
            marker.header.frame_id = self.frame_id
            rospy.loginfo("[CRANE] Made Score Marker")
            return marker
