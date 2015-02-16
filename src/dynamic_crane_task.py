#!/usr/bin/env python
"""
Matt Derry
Jan 2015

This DynamicCraneTask class defines a subclass for an experimental task that
mimics crane usage where obstacle regions are defined and target goals pop up to
 move the suspended mass to.

"""
import rospy
import roslib
import tf
import visualization_msgs.msg as VM
from visualization_msgs.msg import Marker
import std_srvs.srv as SS
from sensor_msgs.msg import Joy
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
import system_definition as sd
from task_fsm import TaskStateMachine
from task_fsm import TaskStates
from obstacle_factory import Obstacle
from obstacle_factory import ObstacleFactory
from obstacle_factory import ObstacleStates
from obstacle_factory import ObstacleSnapshot
from target_factory import TargetStates
from target_factory import Target
from target_factory import TargetFactory
from random import Random
import random
import numpy as np
import math
import yaml
import os
import time
import csv
import pickle

DT = 1/100.
MARKER_DT = 1/20.
ERROR_SCALING = 450.0

MAX_SCORE = 100
TIME_WEIGHT = 1
ERROR_WEIGHT = 10000
COLLISION_WEIGHT = 20

class DynamicTaskStateMachine(TaskStateMachine):
    def __init__(self, x_world_limits, y_world_limits):
        TaskStateMachine.__init__(self, x_world_limits, y_world_limits)
        self.sim_dt = DT
        self.marker_dt = MARKER_DT
        pkg_dir = roslib.packages.get_pkg_dir("receding_planar_sys")
        file_name = pkg_dir + '/trial_num.pkl'
        if os.path.isfile(file_name):
            pkl_file = open(file_name, 'rb')
            self.trial_num = pickle.load(pkl_file)
            self.trial_num += 1
            if self.trial_num > 15:
                self.trial_num = 0
            pkl_file.close()
        else:
            self.trial_num = 0
        pkl_file = open(file_name, 'w+')
        pickle.dump(self.trial_num, pkl_file)
        pkl_file.close()
        self.get_obstacles(self.trial_num)
        rospy.loginfo("[DCRANE] Has %d obstacles after get_obstacles", len(self.obstacles))
        self.all_targets = []
        self.available_targets = []
        self.used_targets = []
        self.all_targets.append(self.target_factory.make_random_target_in_range(0.5, 0.9, -0.4, 0.65, 0.1))
        self.all_targets.append(self.target_factory.make_random_target_in_range(-0.9, -0.5, -0.4, 0.65, 0.1))
        self.available_targets = self.all_targets
        self.target_count = 0
        self.previous_time = rospy.Time.now()
        self.mass_radius = 0.1
        return

    def update_state(self, mass_pos):
        if self.current_state == self.states.READY:
            if self.target_count > 1:
                self.available_targets = self.all_targets
                self.used_targets = []
                self.current_target = None
                self.consecutive_in_targets = 0
                self.user_score = MAX_SCORE
            else:
                rospy.loginfo("[CRANE]Published Target")
                self.start_timer()
                self.previous_time = rospy.Time.now()
                index = random.randint(0, len(self.available_targets)-1)
                self.current_target = self.available_targets[index]
                self.current_target.update_target_state(self.current_target.possible_states.ACTIVE)
                self.used_targets.append(self.available_targets[index])
                del self.available_targets[index]
                #publish current_target
                self.current_target.marker.header.stamp = rospy.Time.now()
                self.target_pub.publish(self.current_target.marker)
                self.timing_marker.header.stamp = rospy.Time.now()
                self.task_timer_pub.publish(self.timing_marker)
                self.consecutive_in_targets = 0
                self.current_state = self.states.TRACKING_TO_TARGET
        elif self.current_state == self.states.COMPLETED_TARGET:
                if self.target_count > 1:
                    rospy.loginfo("[CRANE]Task Completed")
                    self.stop_timer()
                    self.current_state = self.states.COMPLETED_TASK
                    try:
                        self.op_change_client(OperatingCondition(OperatingCondition.STOP))
                        res = self.control_params_client()
                        time.sleep(2.0)
                        traj_res = self.trajectory_client()
                        error_response = self.trajectory_error_client(traj_res.ref_trajectory, traj_res.times)
                        rospy.loginfo("[CRANE] Error for use in trust calculation: %f", error_response.rms)
                        trial_trust = self.calc_trust(ERROR_SCALING, error_response.rms)
                        score = self.calc_score(error_response.rms, self.task_collisions, self.task_duration)
                        rospy.loginfo("Score: %f, error: %f, collisions: %d, duration: %f", score, error_response.rms, self.task_collisions, self.task_duration)
                        self.score_marker.text = "Score: %.1f" % score
                        self.score_marker.header.stamp = rospy.Time.now()
                        self.score_pub.publish(self.score_marker)
                        rospy.loginfo("[CRANE] Trust for last trial: %.3f", trial_trust)
                        pkg_dir = roslib.packages.get_pkg_dir("receding_planar_sys")
                        file_name = pkg_dir + '/data/user_task_time_log.csv'
                        with open(file_name, 'a') as timelogfile:
                            wr = csv.writer(timelogfile, quoting=csv.QUOTE_ALL)
                            wr.writerow([self.task_duration, self.task_penalties, self.task_final_time, res.trust_estimate, res.cutoff_frequency, res.alpha, error_response.rms, trial_trust])
                            rospy.loginfo("Wrote time log entry")
                        if rospy.has_param("user_trust_history"):
                            trust_history = rospy.get_param("user_trust_history")
                            trust_history.append(trial_trust)
                        else:
                            trust_history = [1.0, trial_trust]
                        rospy.loginfo("Write history file")
                        file_name = pkg_dir + '/data/user_trust_history.yaml'
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
                    self.current_target.update_target_state(self.current_target.possible_states.ACTIVE)
                    self.used_targets.append(self.available_targets[index])
                    del self.available_targets[index]
                    #publish current_target
                    self.current_target.marker.header.stamp = rospy.Time.now()
                    self.target_pub.publish(self.current_target.marker)
                    self.consecutive_in_targets = 0
                    self.current_state = self.states.TRACKING_TO_TARGET
        elif self.current_state == self.states.TRACKING_TO_TARGET:
            self.update_obstacles()
            if self.in_current_target(mass_pos):
                rospy.logdebug("[CRANE]In Target")
                self.consecutive_in_targets += 1
                if self.consecutive_in_targets > self.IN_TARGETS_THRESHOLD:
                    rospy.loginfo("[CRANE]Completed Target")
                    self.target_count += 1
                    self.current_state = self.states.COMPLETED_TARGET
            else:
                rospy.logdebug("[CRANE]Reset Current Target")
                self.consecutive_in_targets = 0
                self.in_obstacle = False
            if self.mass_in_obstacle(mass_pos):
                rospy.logdebug("[CRANE]In Obstacle")
                if not self.in_obstacle:
                    self.task_collisions += 1
                    self.publish_collisions(mass_pos)
                self.in_obstacle = True

    def update_obstacles(self):
        self.current_time = rospy.Time.now()
        dt = self.current_time - self.previous_time
        self.previous_time = self.current_time
        if self.obstacles is not None:
            for obstacle in self.obstacles:
                obstacle.update_position(dt)
                self.obstacle_pub.publish(obstacle.marker)
        return

    def calc_score(self, error, collisions, duration):
        score = MAX_SCORE
        score = score - (error * ERROR_WEIGHT)
        score = score - (collisions * COLLISION_WEIGHT)
        score = score - (duration * TIME_WEIGHT)
        if score < 0:
            score = 0
        return score

    def get_obstacles(self, seed):
        if rospy.has_param("obstacles"):
            rospy.loginfo("[CRANE]Has obstacles")
            obstacle_params = rospy.get_param("obstacles")
            random = Random(seed)
            for obs in obstacle_params:
                # yaml obstacle: [height, width, xpos, ypos, heading, velocity, clockwise(0,1), sphere(0,1)]
                rospy.loginfo("OBST (height: %f, width: %f, xpos: %f, ypos: %f, heading: %f, velocity: %f, clockwise: %f, sphere: %f, respond_to_limits: %f", obs[0], obs[1], obs[2], obs[3], obs[4], obs[5], obs[6], obs[7], obs[8])
                # make_obstacle: frame_id, shape, state, color, x_position, y_position, x_scale, y_scale, heading, velocity, trajectory=[]):
                if obs[7] == 0:
                    o = None
                    if obs[4] < 0:
                        o = self.obstacle_factory.make_obstacle_with_random_heading(Marker.CUBE, obs[2], obs[3], obs[0], obs[1], obs[5], random)
                    else:
                        o = self.obstacle_factory.make_obstacle(Marker.CUBE, obs[2], obs[3], obs[0], obs[1], obs[4], obs[5])
                    o.bounce_on_limit_collision = obs[8]
                    o.clockwise = obs[6]
                    self.obstacles.append(o)
                else:
                    o = None
                    if obs[4] < 0:
                        o = self.obstacle_factory.make_obstacle_with_random_heading(Marker.SPHERE, obs[2], obs[3], obs[0], obs[1], obs[5], random)
                    else:
                        o = self.obstacle_factory.make_obstacle(Marker.SPHERE, obs[2], obs[3], obs[0], obs[1], obs[4], obs[5])
                    o.bounce_on_limit_collision = obs[8]
                    o.clockwise = obs[6]
                    self.obstacles.append(o)
            rospy.loginfo("[DCRANE] Has %d obstacles in get_obstacles", len(self.obstacles))

class CraneTaskCoordinator:
    def __init__(self):
        self.br = tf.TransformBroadcaster()
        self.listener = tf.TransformListener()
        self.frame_id = 'optimization_frame'
        self.xlim = None
        self.ylim = None
        self.get_world_limits()
        self.limit_marker = None
        self.limit_marker = self.make_limit_marker()
        self.limit_pub = rospy.Publisher("limit_markers", Marker, queue_size=1)
        self.tsm = DynamicTaskStateMachine(self.xlim, self.ylim)
        self.operating_condition = OperatingCondition.IDLE
        # create subscriber for operating condition
        self.op_cond_sub = rospy.Subscriber("/operating_condition", OperatingCondition, self.op_cb)
        # create subscriber for current mass position
        self.filt_state_sub = rospy.Subscriber("filt_state", PlanarSystemState, self.state_cb)
        # publish limit marker if one exists
        return

    def state_cb(self, msg):
        if self.operating_condition == OperatingCondition.RUN:
            self.tsm.update_state(msg)

    def op_cb(self, msg):
        if msg.state == OperatingCondition.RUN and self.operating_condition != OperatingCondition.RUN:
            rospy.loginfo("[DCRANE] Check Limit Marker")
            if self.limit_marker is not None:
                rospy.loginfo("[DCRANE] Publish Limit Marker")
                self.limit_pub.publish(self.limit_marker)
        self.operating_condition = msg.state

    def get_world_limits(self):
        if rospy.has_param("xlim") and rospy.has_param("ylim"):
            self.xlim = rospy.get_param("xlim")
            self.xlim = np.sort(np.array(self.xlim))
            self.ylim = rospy.get_param("ylim")
            self.ylim = np.sort(np.array(self.ylim))

    def make_limit_marker(self):
        if self.xlim is None or self.xlim is None:
            return None
        else:
            marker = Marker()
            marker.type = Marker.LINE_STRIP
            marker.scale.x = 0.05
            marker.color.a = 1.0
            marker.points = []
            marker.lifetime = rospy.Duration()
            marker.header.frame_id = self.frame_id
            # Lower left
            marker.points.append(Point(self.xlim[0], self.ylim[0], 0.0))
            # Upper left
            marker.points.append(Point(self.xlim[0], self.ylim[1], 0.0))
            # Upper right
            marker.points.append(Point(self.xlim[1], self.ylim[1], 0.0))
            # Lower right
            marker.points.append(Point(self.xlim[1], self.ylim[0], 0.0))
            # repeat lower left
            marker.points.append(Point(self.xlim[0], self.ylim[0], 0.0))
            return marker


def main():
    rospy.init_node('dynamic_crane_task_coordinator')
    try:
        sim = CraneTaskCoordinator()
    except rospy.ROSInterruptException: pass
    rospy.spin()


if __name__=='__main__':
    main()
