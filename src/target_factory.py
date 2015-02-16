#!/usr/bin/env python
"""
Matt Derry
Jan 2015

The target factory class defines a notion of a task target and provides a number
 of methods for creating task targets with different parameters and states.

"""
import rospy
import roslib
import visualization_msgs.msg as VM
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA
from puppeteer_msgs.msg import PlanarSystemState
import random
import system_definition as sd

class TargetStates:
    def __init__(self):
        self.INACTIVE = 0
        self.ACTIVE = 1
        self.ACHIEVED = 2

class Target:
    def __init__(self, x, y, r, id, state):
        self.possible_states = TargetStates()
        self.id = id
        self.x = x
        self.y = y
        self.radius = r
        self.state = state
        self.marker = Marker()

    def within_limits(self, xlim, ylim):
        if xlim is None or ylim is None:
            return True
        if (self.marker.pose.position.x + self.radius) <= xlim[1] and (self.marker.pose.position.x - self.radius) >= xlim[0]:
            if (self.marker.pose.position.y + self.radius) <= ylim[1] and (self.marker.pose.position.y - self.radius) >= ylim[0]:
                return True
            else:
                return False
        else:
            return False

    def in_target(self, mass_pos):
        dist = self.distance(mass_pos.xm, mass_pos.ym, self.marker.pose.position.x, self.marker.pose.position.y)
        if dist > self.radius:
            return False
        else:
            return True

    def update_target_state(self, state):
        if state == self.possible_states.INACTIVE:
            self.state = state
            col = ColorRGBA(1.0, 1.0, 1.0, 0.3)
        elif state == self.possible_states.ACTIVE:
            self.state = state
            col = ColorRGBA(0.0, 1.0, 0.0, 0.6)
        elif state == self.possible_states.ACHIEVED:
            self.state = state
            col = ColorRGBA(0.0, 0.0, 1.0, 0.6)
        else:
            rospy.logerror("Setting target to unknown state")
            return

        self.marker.color = col
        self.marker.header.stamp = rospy.Time.now()
        return

    def distance(self, p1x, p1y, p2x, p2y):
        return math.sqrt(math.pow(p2x-p1x,2) + math.pow(p2y - p1y, 2))

class TargetFactory:
    def __init__(self, frame, world_xlim, world_ylim):
        self.frame_id = frame
        self.xlim = world_xlim
        self.ylim = world_ylim
        self.possible_states = TargetStates()
        self.next_id = 0

    def make_random_target_in_range(self, x_min, x_max, y_min, y_max, radius):
        x_position = (x_max - x_min) * random.random() + x_min
        y_position = (y_max - y_min) * random.random() + y_min
        return self.make_target(x_position, y_position, radius)

    def make_target(self, x_position, y_position, radius, state=None):
        if state is None:
            state = self.possible_states.INACTIVE
        target = Target(x_position, y_position, radius, self.next_id, state)
        target.marker = self.make_target_marker(x_position, y_position, radius)
        target.update_target_state(state)
        self.next_id = self.next_id + 1
        return target

    def make_target_marker(self, x_position, y_position, radius):
        marker = Marker()
        marker.type = Marker.SPHERE
        marker.scale.x = radius
        marker.scale.y = radius
        marker.pose.position.x = x_position
        marker.pose.position.y = y_position
        marker.scale.z = 0.01
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.color.a = 0.6
        marker.lifetime = rospy.Duration()
        marker.header.frame_id = self.frame_id
        return marker
