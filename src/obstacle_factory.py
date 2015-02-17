#!/usr/bin/env python
"""
Matt Derry
Jan 2015

The obstacle factory class defines a notion of obstacle and provides a number of
 methods for creating obstacles with different parameters and states.

"""
import rospy
import roslib
import visualization_msgs.msg as VM
from visualization_msgs.msg import Marker
from puppeteer_msgs.msg import PlanarSystemState
import system_definition as sd
import numpy as np
import math
import random


class ObstacleStates:
    def __init__(self):
        self.INACTIVE = 0
        self.PREACTIVE = 1
        self.ACTIVE = 2

class ObstacleSnapshot:
    def __init__(self, state, position, time):
        self.state = state
        self.position = position
        self.time = time

class Obstacle:
    def __init__(self, id, velocity, heading, xlim, ylim, trajectory=[]):
        self.marker = Marker()
        self.collision_marker = Marker()
        self.possible_states = ObstacleStates()
        self.state = self.possible_states.ACTIVE
        self.activation_time = None
        self.preactivation_time = None
        self.velocity = velocity
        self.heading = heading
        self.xlim = xlim
        self.ylim = ylim
        self.pose_history = []
        self.desired_trajectory = trajectory
        self.bounce_on_limit_collision = False
        self.previous_limit_collision = False
        self.clockwise = True

    def within_limits(self):
        if xlim is None or ylim is None:
            return True
        if (self.marker.pose.position.x + self.marker.scale.x/2) <= self.xlim[1] and (self.marker.pose.position.x - self.marker.scale.x/2) >= self.xlim[0]:
            if (self.marker.pose.position.y + self.marker.scale.y/2) <= self.ylim[1] and (self.marker.pose.position.y - self.marker.scale.y/2) >= self.ylim[0]:
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

    def randomize(self, width_limits, height_limits, xpos_limits, floor_position):
        self.marker.scale.x = random.random() * (width_limits[1] - width_limits[0]) + width_limits[0]
        self.marker.scale.y = random.random() * (height_limits[1] - height_limits[0]) + height_limits[0]
        self.marker.pose.position.x = random.random() * (xpos_limits[1] - xpos_limits[0]) + xpos_limits[0]
        self.marker.pose.position.y = floor_position + (self.marker.scale.y/2)
        self.marker.header.stamp = rospy.Time.now()

    def activate(self, lifetime=None):
        self.marker.color.a = 1.0
        if lifetime is not None:
            self.marker.lifetime = rospy.Duration(lifetime)
        self.state = self.possible_states.ACTIVE
        self.activation_time = rospy.Time.now()
        self.marker.header.stamp = rospy.Time.now()

    def mass_in_collision(self, mass_pos, mass_radius=0.03):
        if self.state == self.possible_states.ACTIVE:
            if self.marker.type == Marker.CUBE:
                xmin = self.marker.pose.position.x - (self.marker.scale.x/2. + mass_radius)
                xmax = xmin + self.marker.scale.x + (2.*mass_radius)
                ymin = self.marker.pose.position.y - (self.marker.scale.y/2. + mass_radius)
                ymax = ymin + self.marker.scale.y + (2.*mass_radius)
                if self.in_range(mass_pos, xmin, xmax, ymin, ymax):
                    #rospy.loginfo("[OBSTACLE] Collision in range: x:(%f, %f), y:(%f, %f), state:(%f, %f), radius: (%f, %f)", xmin, xmax, ymin, ymax, mass_pos.xm, mass_pos.ym, mass_radius, mass_radius/2.0)
                    self.collision_marker.pose.position.x = self.marker.pose.position.x
                    self.collision_marker.pose.position.y = self.marker.pose.position.y
                    self.collision_marker.scale.x = self.marker.scale.x
                    self.collision_marker.scale.y = self.marker.scale.y
                    self.collision_marker.points = []
                    return True
            else:
                if self.distance(self.marker.pose.position.x, self.marker.pose.position.y, mass_pos.xm, mass_pos.ym) < ((self.marker.scale.x/2.) + (mass_radius/2.0)):
                    self.collision_marker.pose.position.x = self.marker.pose.position.x
                    self.collision_marker.pose.position.y = self.marker.pose.position.y
                    return True
        return False

    def preactivate(self):
        self.marker.color.a = 0.25
        self.state = self.possible_states.PREACTIVE
        self.preactivation_time = rospy.Time.now()
        self.marker.header.stamp = rospy.Time.now()

    def deactivate(self):
        self.state = self.possible_states.INACTIVE
        self.activation_time = None
        self.preactivation_time  = None
        self.marker.header.stamp = rospy.Time.now()

    def update_position(self, dt):
        self.marker.pose.position.x = self.marker.pose.position.x + (self.velocity * dt.to_sec() * math.cos(self.heading))
        self.marker.pose.position.y = self.marker.pose.position.y + (self.velocity * dt.to_sec() * math.sin(self.heading))

        limit_collision = False
        if self.marker.pose.position.x + (self.marker.scale.x/2.0) > self.xlim[1]:
            limit_collision = True
        if self.marker.pose.position.x - (self.marker.scale.x/2.0) < self.xlim[0]:
            limit_collision = True
        if self.marker.pose.position.y + (self.marker.scale.y/2.0) > self.ylim[1]:
            limit_collision = True
        if self.marker.pose.position.y - (self.marker.scale.y/2.0) < self.ylim[0]:
            limit_collision = True

        self.collision_marker.pose.position.x = self.marker.pose.position.x
        self.collision_marker.pose.position.y = self.marker.pose.position.y

        if limit_collision:
            if self.bounce_on_limit_collision:
                if self.clockwise:
                    self.heading = self.heading - (1.0*np.pi/2.)
                else:
                    self.heading = self.heading + (1.0*np.pi/2.)
                self.heading = ( self.heading + np.pi) % (2.0 * np.pi ) - np.pi

        self.previous_limit_collision = limit_collision

        self.marker.header.stamp = rospy.Time.now()
        self.collision_marker.header.stamp = rospy.Time.now()
        self.pose_history.append(ObstacleSnapshot(self.state, self.marker.pose.position, self.marker.header.stamp))

    def distance(self, p1x, p1y, p2x, p2y):
        return math.sqrt(math.pow(p2x-p1x,2) + math.pow(p2y - p1y, 2))

    def in_range(self, mass_pos, xmin, xmax, ymin, ymax):
        if mass_pos.xm > xmin and mass_pos.xm < xmax and mass_pos.ym > ymin and mass_pos.ym < ymax:
            return True
        else:
            return False

class ObstacleFactory:
    def __init__(self, frame, world_xlim, world_ylim):
        self.frame_id = frame
        self.xlim = world_xlim
        self.ylim = world_ylim
        self.possible_states = ObstacleStates()
        self.next_id = 0

    def make_obstacle_with_random_heading(self, shape, x_position, y_position, x_scale, y_scale, velocity, rand=None):
        if rand is None:
            heading = -np.pi + 2.0*np.pi*random.random()
        else:
            heading = -np.pi + 2.0*np.pi*rand.random()
        return self.make_obstacle(shape, x_position, y_position, x_scale, y_scale, heading, velocity)

    def make_obstacle_with_random_heading_and_velocity(self, shape, x_position, y_position, x_scale, y_scale, min_vel, max_vel):
        heading = -np.pi + 2.0*np.pi*random.random()
        velocity = min_vel + ((max_vel - min_vel) * random.random())
        return self.make_obstacle(shape, x_position, y_position, x_scale, y_scale, heading, velocity)

    def make_obstacle(self, shape, x_position, y_position, x_scale, y_scale, heading, velocity, trajectory=[]):
        obstacle = Obstacle(self.next_id, velocity, heading, self.xlim, self.ylim, trajectory)
        obstacle.marker = self.make_obstacle_marker(x_position, y_position, x_scale, y_scale, shape, self.next_id)
        obstacle.collision_marker = self.make_collision_marker(obstacle.marker)
        self.next_id = self.next_id + 1
        return obstacle

    def make_obstacle_marker(self, x_position, y_position, x_scale, y_scale, shape, oid):
        marker = Marker()
        marker.type = shape
        marker.scale.x = x_scale
        marker.scale.y = y_scale
        marker.scale.z = 0.01
        marker.lifetime = rospy.Duration()
        marker.action = Marker.ADD
        marker.color.a = 1.0
        marker.id = oid
        marker.pose.position.x = x_position
        marker.pose.position.y = y_position
        marker.header.frame_id = self.frame_id
        marker.header.stamp = rospy.Time.now()
        return marker

    def make_collision_marker(self, o_marker):
        marker = Marker()
        marker.type = o_marker.type
        marker.scale.x = o_marker.scale.x
        marker.scale.y = o_marker.scale.y
        marker.scale.z = o_marker.scale.z
        marker.lifetime = rospy.Duration(1.0)
        marker.action = Marker.ADD
        marker.id = o_marker.id
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 0.8
        marker.pose.position.x = o_marker.pose.position.x
        marker.pose.position.y = o_marker.pose.position.y
        marker.pose.position.z = -0.01
        marker.header.frame_id = o_marker.header.frame_id
        marker.header.stamp = rospy.Time.now()
        return marker
