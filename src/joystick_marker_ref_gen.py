#!/usr/bin/env python
"""
Matt Derry
Jan 2015

This node uses joystick commands to provide reference data for the receding
horizon optimal controller. If we are in global RUN state, this node sends a tf
and a point message at some frequency publishing the ineractive marker is. A
different node looks at this data and handles interpolation/time offseting for
the controller.

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
from sensor_msgs.msg import Joy
import system_definition as sd
import tf
import numpy as np

# global constants
DT = 1/100.
MARKERWF = 'optimization_frame'
MARKERFRAME = 'mass_ref_frame'
X_GAIN = 1.0
Y_GAIN = 1.0
XLIM = None
YLIM = None
limit_marker = None

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

def makeMarker( color ):
    marker = Marker()
    marker.type = Marker.SPHERE
    marker.lifetime = rospy.Duration()
    marker.scale.x = 0.25
    marker.scale.y = 0.25
    marker.scale.z = 0.25
    marker.color.r = 0.1
    marker.color.g = 0.1
    marker.color.b = 0.1
    marker.color.a = 0.75
    if color == 'red':
        marker.color.r += 0.4
    elif color == 'blue':
        marker.color.b += 0.4
    elif color == 'green':
        marker.color.g += 0.4
    else:
        rospy.warn("Marker color not recognized!")
    return marker


class SingleController:
    def __init__(self, conframe, simframe, simpos=[0.0,]*3,
                 simquat=[0.0,]*4, simpose=None, color='green'):
        """
        conframe ~ the frame that we should publish to control the kinematic
              input
        simframe ~ the frame that the simpos and simquat point to
        simpos ~ nominal location of the kinematic config variable in trep
              simulation... used for determining offset
        simquat ~ nominal orientation of the kinematic config var in the trep
              simulation... used for offset
        simpose ~ a pose message to define the pose
        color ~ color of the marker attached to the frame
        """
        self.conframe = conframe
        self.simframe = simframe
        self.int_marker = makeMarker(color)
        self.int_marker.header.frame_id = MARKERWF
        if simpose != None:
            self.int_marker.pose = simpose
        else:
            # build pose from pos and quat
            simquat[-1] = 1.0
            self.set_pose(pos=simpos, quat=simquat)

        self.command = Twist()
        self.pose_update = rospy.Time.now()


    def set_pose(self, pose=None, pos=[0.0,]*3, quat=[0.0,]*4):
        if pose != None:
            self.int_marker.header.frame_id = pose.header.frame_id
            self.int_marker.pose = pose
            self.simpose = copy.deepcopy(self.int_marker.pose)
            self.simpos = tuple([pose.position.__getattribute__(x)
                                 for x in pose.position.__slots__])
            self.simquat = tuple([pose.orientation.__getattribute__(x)
                                  for x in pose.orientation.__slots__])
        else:
            quat[-1] = 1
            self.simpos = pos
            self.simquat = quat
            self.int_marker.pose = P(position=Point(*pos),
                                     orientation=Quaternion(*quat))
            self.simpose = P(position=Point(*pos),
                                     orientation=Quaternion(*quat))


    def update_pose(self):
        current_time = rospy.Time.now()
        dt = current_time - self.int_marker.header.stamp
        # rospy.loginfo("Old X: %f, Old Y: %f", self.int_marker.pose.position.x, self.int_marker.pose.position.y)
        self.int_marker.pose.position.x = self.int_marker.pose.position.x + self.command.linear.x * dt.to_sec()
        self.int_marker.pose.position.y = self.int_marker.pose.position.y + self.command.linear.y * dt.to_sec()
        # rospy.loginfo("New X: %f, New Y: %f", self.int_marker.pose.position.x, self.int_marker.pose.position.y)
        if XLIM is not None:
            self.int_marker.pose.position.x = np.clip(self.int_marker.pose.position.x, *XLIM)
        if YLIM is not None:
            self.int_marker.pose.position.y = np.clip(self.int_marker.pose.position.y, *YLIM)

        self.int_marker.header.stamp = current_time

        self.simpose = copy.deepcopy(self.int_marker.pose)
        self.simpos = tuple([self.int_marker.pose.position.__getattribute__(x)
                             for x in self.int_marker.pose.position.__slots__])
        self.simquat = tuple([self.int_marker.pose.orientation.__getattribute__(x)
                             for x in self.int_marker.pose.orientation.__slots__])


class MarkerControls:
    def __init__(self):
        # create marker server:
        # self.server = InteractiveMarkerServer("mass_reference_control", q_size=1)
        # create listener and broadcaster
        self.br = tf.TransformBroadcaster()
        self.listener = tf.TransformListener()

        # let's build all of the controllers
        self.controllers = []
        self.controllers.append(SingleController(MARKERFRAME,
                                                 MARKERWF,
                                                 color='green'))

        # create subscriber for operating condition
        self.op_cond_sub = rospy.Subscriber("/operating_condition",
                                            OperatingCondition, self.opcb)
        self.operating_condition = OperatingCondition.IDLE
        # create publisher
        self.marker_pub = rospy.Publisher("mass_ref_point", PointStamped, queue_size=3)
        # publish markers for drawing paths
        self.con_pub = rospy.Publisher("visualization_markers", VM.MarkerArray, queue_size=3)
        # publish limit marker if one exists
        self.limit_pub = rospy.Publisher("limit_markers", Marker, queue_size=1)
        # create subscriber for joy stick message
        self.joy_sub = rospy.Subscriber("/joy", Joy, self.joy_cb)
        # create current position marker publisher
        self.current_pose_pub = rospy.Publisher("current_pose", Marker, queue_size=1)
        # setup timer to publish transforms and messages:
        rospy.Timer(rospy.Duration(DT), self.timercb)
        return


    def opcb(self, data):
        if data.state < self.operating_condition:
            rospy.loginfo("Resetting marker!")
            # reset marker:
            for con in self.controllers:
                con.set_pose()
        self.operating_condition = data.state
        if self.operating_condition == OperatingCondition.RUN:
            self.controllers.append(SingleController(MARKERFRAME, MARKERWF, color='green'))
            # if limit_marker is not None:
                # rospy.loginfo("[JOY] Publish limits...")
                # self.limit_pub.publish(limit_marker)
        elif self.operating_condition == OperatingCondition.IDLE:
            rospy.loginfo("Removing marker from server")
            self.controllers = []
        return


    def joy_cb(self, msg):
        if self.operating_condition == OperatingCondition.RUN:
            # rospy.loginfo("joy_cb called...")
            for con in self.controllers:
                con.command.linear.x = msg.axes[0] * X_GAIN
                con.command.linear.y = msg.axes[1] * Y_GAIN
                # rospy.loginfo("CMD X: %f, Y: %f", con.command.linear.x, con.command.linear.y)
        return


    def send_transforms(self):
        tnow = rospy.Time.now()
        mlist = []
        for con in self.controllers:
            con.update_pose()
            pos = con.int_marker.pose.position
            quat = con.int_marker.pose.orientation
            self.br.sendTransform((pos.x, pos.y, pos.z),
                                  (quat.x, quat.y, quat.z, quat.w),
                                  tnow,
                                  con.conframe, con.simframe)
            pt = PointStamped()
            pt.header.stamp = tnow
            pt.header.frame_id = MARKERWF
            pt.point.x = pos.x
            pt.point.y = pos.y
            pt.point.z = pos.z
            self.marker_pub.publish(pt)
            self.current_pose_pub.publish(con.int_marker)
            m = con.int_marker
            m.header = con.int_marker.header
            m.pose = con.int_marker.pose
            mlist.append(m)
        ma = VM.MarkerArray()
        ma.markers = mlist
        self.con_pub.publish(ma)
        return


    def timercb(self, event):
        # check operating condition:
        if self.operating_condition == OperatingCondition.RUN:
            self.send_transforms()
        return


def main():
    rospy.init_node('joy_marker_controls')
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

    try:
        sim = MarkerControls()
    except rospy.ROSInterruptException: pass

    rospy.spin()


if __name__=='__main__':
    main()
