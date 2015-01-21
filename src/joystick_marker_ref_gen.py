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
from puppeteer_msgs.msg import OperatingCondition
from puppeteer_msgs.msg import PlanarSystemState
from puppeteer_msgs.msg import PlanarSystemConfig
from puppeteer_msgs.srv import InputTrajectoryError
from puppeteer_msgs.srv import InputTrajectory
from sensor_msgs.msg import Joy
import system_definition as sd
import tf
import numpy as np
import math
from scipy.signal import lfilter, lfilter_zi, lfiltic, butter, filtfilt
from scipy.fftpack import fft, ifft
import matplotlib.pyplot as mpl
import csv

# global constants
DT = 1/100.
MARKERWF = 'optimization_frame'
MARKERFRAME = 'mass_ref_frame'
X_GAIN = 0.5
Y_GAIN = 0.5

# Signal Filtering Constants
NYQUIST = (1/DT)/2
CUTOFF = 25.
MIN_CF = 0.1
FILTER_ORDER = 4
FILTER_TYPE = 'low'
ANALOG_FILTER = False

# Trust Constants
STARTING_TRUST = 1.0
MOVING_AVERAGE_WINDOW = 5

# Global Variables
XLIM = None
YLIM = None
limit_marker = None
trust_history = []
trial_trust = 0.0

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
    marker.scale.x = 0.05
    marker.scale.y = 0.05
    marker.scale.z = 0.05
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

def makeTrustMarker(trust):
        marker = Marker()
        marker.type = Marker.TEXT_VIEW_FACING
        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 0.08
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.2
        marker.color.a = 0.6
        marker.pose.position.x = 0.75
        marker.pose.position.y = 0.67
        marker.pose.position.z = 0.0
        marker.text = "Trust: %.2f" % trust
        marker.lifetime = rospy.Duration()
        marker.id = 1
        marker.header.frame_id = MARKERWF
        rospy.loginfo("Made Trust Marker")
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
        self.int_marker = makeMarker('green')
        self.int_marker.header.frame_id = MARKERWF
        self.int_marker_filt = makeMarker(color)
        self.int_marker_filt.header.frame_id = MARKERWF

        self.pose_times = []
        self.raw_pose_x = []
        self.raw_pose_y = []
        self.filt_pose_x = []
        self.filt_pose_y = []
        self.raw_ref_trajectory = []

        self.cutoff = CUTOFF
        self.norm_cutoff = self.cutoff/NYQUIST

        self.filt_b, self.filt_a = butter(FILTER_ORDER, self.norm_cutoff, FILTER_TYPE, analog=ANALOG_FILTER)
        self.xzi = lfilter_zi(self.filt_b, self.filt_a)
        self.yzi = lfilter_zi(self.filt_b, self.filt_a)

        if simpose != None:
            self.int_marker.pose = simpose
            self.int_marker_file.pose = simpose
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
            self.int_marker_filt.header.frame_id = pose.header.frame_id
            self.int_marker_filt.pose = pose
            self.simpose = copy.deepcopy(self.int_marker.pose)
            self.pose_times.append(0.0)
            self.raw_pose_x.append(self.simpose.position.x)
            self.raw_pose_y.append(self.simpose.position.y)
            p = Point()
            p.x = self.simpose.position.x
            p.y = self.simpose.position.y
            self.raw_ref_trajectory.append(p)
            self.filt_pose_x.append(self.simpose.position.x)
            self.filt_pose_y.append(self.simpose.position.y)
            self.simpos = tuple([pose.position.__getattribute__(x)
                                 for x in pose.position.__slots__])
            self.simquat = tuple([pose.orientation.__getattribute__(x)
                                  for x in pose.orientation.__slots__])
        else:
            quat[-1] = 1
            self.simpos = pos
            self.simquat = quat
            self.int_marker.pose = P(position=Point(*pos), orientation=Quaternion(*quat))
            self.int_marker_filt.pose = P(position=Point(*pos), orientation=Quaternion(*quat))
            self.simpose = P(position=Point(*pos), orientation=Quaternion(*quat))
            self.pose_times.append(0.0)
            self.raw_pose_x.append(self.simpose.position.x)
            self.raw_pose_y.append(self.simpose.position.y)
            p = Point()
            p.x = self.simpose.position.x
            p.y = self.simpose.position.y
            self.raw_ref_trajectory.append(p)
            self.filt_pose_x.append(self.simpose.position.x)
            self.filt_pose_y.append(self.simpose.position.y)


    def update_pose(self):
        current_time = rospy.Time.now()
        dt = current_time - self.int_marker.header.stamp
        # rospy.loginfo("Old X: %f, Old Y: %f", self.int_marker.pose.position.x, self.int_marker.pose.position.y)
        self.int_marker.pose.position.x = self.int_marker.pose.position.x + self.command.linear.x * dt.to_sec()
        self.int_marker.pose.position.y = self.int_marker.pose.position.y + self.command.linear.y * dt.to_sec()
        # rospy.loginfo("New X: %f, New Y: %f", self.int_marker.pose.position.x, self.int_marker.pose.position.y)
        # if XLIM is not None:
        #  self.int_marker.pose.position.x = np.clip(self.int_marker.pose.position.x, *XLIM)
        # if YLIM is not None:
        #  self.int_marker.pose.position.y = np.clip(self.int_marker.pose.position.y, *YLIM)

        self.pose_times.append(dt.to_sec())
        self.raw_pose_x.append(self.int_marker.pose.position.x)
        self.raw_pose_y.append(self.int_marker.pose.position.y)
        p = Point()
        p.x = self.int_marker.pose.position.x
        p.y = self.int_marker.pose.position.y
        self.raw_ref_trajectory.append(p)

        filt_x = filtfilt(self.filt_b, self.filt_a, self.raw_pose_x, padtype=None, padlen=None)
        filt_y = filtfilt(self.filt_b, self.filt_a, self.raw_pose_y, padtype=None, padlen=None)

        # rospy.loginfo("[JOY] unfiltered pos: (%f, %f), filtered pos: (%f, %f)", self.int_marker.pose.position.x, self.int_marker.pose.position.y, filt_x[len(filt_x)-1], filt_y[len(filt_y)-1])

        # K = 300.0
        self.int_marker_filt.pose.position.x = filt_x[len(filt_x)-1] # [0][0]
        self.int_marker_filt.pose.position.y = filt_y[len(filt_x)-1] # [0][0]
        # self.int_marker_filt.pose.position.x = self.int_marker.pose.position.x
        # self.int_marker_filt.pose.position.y = self.int_marker.pose.position.y

        if XLIM is not None:
            self.int_marker_filt.pose.position.x = np.clip(self.int_marker_filt.pose.position.x, *XLIM)
        if YLIM is not None:
            self.int_marker_filt.pose.position.y = np.clip(self.int_marker_filt.pose.position.y, *YLIM)

        self.filt_pose_x.append(self.int_marker_filt.pose.position.x)
        self.filt_pose_y.append(self.int_marker_filt.pose.position.y)

        self.int_marker.header.stamp = current_time
        self.int_marker_filt.header.stamp = current_time

        self.simpose = copy.deepcopy(self.int_marker_filt.pose)
        self.simpos = tuple([self.int_marker_filt.pose.position.__getattribute__(x)
                             for x in self.int_marker_filt.pose.position.__slots__])
        self.simquat = tuple([self.int_marker_filt.pose.orientation.__getattribute__(x)
                             for x in self.int_marker_filt.pose.orientation.__slots__])


    def calculate_error(self, mass_state, ins_idx):
        i = 0  # timestep index
        self.raw_mse_total = 0 # accumulating error
        self.raw_mse_x = 0
        self.raw_mse_y = 0
        self.filt_mse_total = 0 # accumulating error
        self.filt_mse_x = 0
        self.filt_mse_y = 0
        rospy.loginfo("Input length: %d, state length: %d", len(self.raw_pose_x), len(mass_state))
        for ms in mass_state:
            scaled_idx = ins_idx[i]
            self.raw_mse_x = self.raw_mse_x + pow(ms.xm - self.raw_pose_x[scaled_idx], 2)
            self.raw_mse_y = self.raw_mse_y + pow(ms.ym - self.raw_pose_y[scaled_idx], 2)
            self.filt_mse_x = self.filt_mse_x + pow(ms.xm - self.filt_pose_x[scaled_idx], 2)
            self.filt_mse_y = self.filt_mse_y + pow(ms.ym - self.filt_pose_y[scaled_idx], 2)
            i += 1
        self.raw_mse_x = self.raw_mse_x/i
        self.raw_mse_y = self.raw_mse_y/i
        self.raw_mse_total = self.raw_mse_x + self.raw_mse_y
        self.filt_mse_x = self.filt_mse_x/i
        self.filt_mse_y = self.filt_mse_y/i
        self.filt_mse_total = self.filt_mse_x + self.filt_mse_y
        self.raw_rms_x = math.sqrt(self.raw_mse_x)
        self.raw_rms_y = math.sqrt(self.raw_mse_y)
        self.raw_rms_total = math.sqrt(self.raw_mse_total)
        self.filt_rms_x = math.sqrt(self.filt_mse_x)
        self.filt_rms_y = math.sqrt(self.filt_mse_y)
        self.filt_rms_total = math.sqrt(self.filt_mse_total)
        rospy.loginfo("Filtered total RMS: %f", self.filt_rms_total)
        return

class MarkerControls:
    def __init__(self):
        # create marker server:
        # self.server = InteractiveMarkerServer("mass_reference_control", q_size=1)
        # create listener and broadcaster
        self.br = tf.TransformBroadcaster()
        self.listener = tf.TransformListener()

        self.mass_state = []
        self.input_state_index = []

        # let's build all of the controllers
        self.controllers = []
        self.controllers.append(SingleController(MARKERFRAME,
                                                 MARKERWF,
                                                 color='green'))

        self.get_trajectory_serv = rospy.Service("get_trajectory", InputTrajectory, self.get_trajectory_handler)
        # create subscriber for current mass position
        #self.filt_state_sub = rospy.Subscriber("filt_state", PlanarSystemState, self.state_cb)
        self.filt_state_sub = rospy.Subscriber("meas_config", PlanarSystemConfig, self.state_cb)
        # create subscriber for operating condition
        self.op_cond_sub = rospy.Subscriber("/operating_condition", OperatingCondition, self.opcb)
        self.operating_condition = OperatingCondition.IDLE
        # create publisher
        self.marker_pub = rospy.Publisher("mass_ref_point", PointStamped, queue_size=3)
        # publish markers for drawing paths
        self.con_pub = rospy.Publisher("visualization_markers", VM.MarkerArray, queue_size=3)
        # publish markers for drawing paths
        self.filt_con_pub = rospy.Publisher("filtered_visualization_markers", VM.MarkerArray, queue_size=3)
        # publish limit marker if one exists
        self.limit_pub = rospy.Publisher("limit_markers", Marker, queue_size=1)
        # create subscriber for joy stick message
        self.joy_sub = rospy.Subscriber("/joy", Joy, self.joy_cb)
        # create current position marker publisher
        self.current_pose_pub = rospy.Publisher("current_pose", Marker, queue_size=1)
        self.trust_pub = rospy.Publisher("current_trust", Marker, queue_size=2)
        self.trust_marker = makeTrustMarker(trial_trust)
        # wait for service servers:
        rospy.loginfo("Waiting for get_trajectory_error service")
        rospy.wait_for_service("get_trajectory_error")
        rospy.loginfo("get_trajectory_error service now available")
        self.trajectory_error_client = rospy.ServiceProxy("get_trajectory_error", InputTrajectoryError)
        # setup timer to publish transforms and messages:
        rospy.Timer(rospy.Duration(DT), self.timercb)
        return

    def get_trajectory_handler(self, req):
        for con in self.controllers:
            return {'ref_trajectory' : con.raw_ref_trajectory, 'times' : con.pose_times}

    def opcb(self, data):
        if data.state < self.operating_condition:
            rospy.loginfo("Resetting marker!")
            # reset marker:
            for con in self.controllers:
                con.set_pose()
        #self.operating_condition = data.state
        if data.state == OperatingCondition.RUN and self.operating_condition != OperatingCondition.RUN:
            self.controllers.append(SingleController(MARKERFRAME, MARKERWF, color='green'))
            self.trust_pub.publish(self.trust_marker)
            # create subscriber for current mass position
            #self.filt_state_sub = rospy.Subscriber("filt_state", PlanarSystemState, self.state_cb)
            self.filt_state_sub = rospy.Subscriber("meas_config", PlanarSystemConfig, self.state_cb)
            self.start_time = rospy.Time.now()
            # if limit_marker is not None:
                # rospy.loginfo("[JOY] Publish limits...")
                # self.limit_pub.publish(limit_marker)
        elif data.state == OperatingCondition.IDLE:
            rospy.loginfo("Removing marker from server")
            self.controllers = []
            self.mass_state = []
            self.input_state_index = []
            self.filt_state_sub.unregister()
            self.rx_state = []
            self.mx_state = []
            self.time_history = []
        elif data.state == OperatingCondition.STOP and self.operating_condition != OperatingCondition.STOP:
            for con in self.controllers:
                con.calculate_error(self.mass_state, self.input_state_index)
                # fx = fft(con.raw_pose_x)
                # xl = np.linspace(0.0, 1.0/(2.0*DT), len(con.raw_pose_x)/2)
                # fx_t = 2.0/len(con.raw_pose_x)*np.abs(fx[0:len(con.raw_pose_x)/2])
                # print fx.shape
                # print xl.shape
                # print fx_t.shape
                # with open('/home/mderry/rss_data/xl.csv', 'wb') as myfile:
                #     wr = csv.writer(myfile, quoting=csv.QUOTE_ALL)
                #     wr.writerow(xl)
                # with open('/home/mderry/rss_data/fx_t.csv', 'wb') as myfile:
                #     wr = csv.writer(myfile, quoting=csv.QUOTE_ALL)
                #     wr.writerow(fx_t)
                # fy = fft(con.raw_pose_y)
                # yl = np.linspace(0.0, 1.0/(2.0*DT), len(con.raw_pose_y)/2)
                # fy_t = 2.0/len(con.raw_pose_y)*np.abs(fy[0:len(con.raw_pose_y)/2])
                # print fy.shape
                # print yl.shape
                # print fy_t.shape
                # with open('/home/mderry/rss_data/yl.csv', 'wb') as myfile:
                #     wr = csv.writer(myfile, quoting=csv.QUOTE_ALL)
                #     wr.writerow(yl)
                # with open('/home/mderry/rss_data/fy_t.csv', 'wb') as myfile:
                #     wr = csv.writer(myfile, quoting=csv.QUOTE_ALL)
                #     wr.writerow(fy_t)
                # mpl.figure()
                # mpl.plot(yl, fy_t, color='r', linewidth=7.0)
                # mpl.hold(True)
                # mpl.plot(xl, fx_t, color='b', linewidth=7.0)
                # mpl.grid()
                # mpl.show()
        # elif data.state == OperatingCondition.EMERGENCY and self.operating_condition != OperatingCondition.EMERGENCY:
                # with open('/home/mderry/rss_data/rx.csv', 'wb') as myfile:
                #     wr = csv.writer(myfile, quoting=csv.QUOTE_ALL)
                #     for val in self.rx_state:
                #         wr.writerow([val])
                # with open('/home/mderry/rss_data/mx.csv', 'wb') as myfile:
                #     wr = csv.writer(myfile, quoting=csv.QUOTE_ALL)
                #     for val in self.mx_state:
                #         wr.writerow([val])
                # with open('/home/mderry/rss_data/time.csv', 'wb') as myfile:
                #     wr = csv.writer(myfile, quoting=csv.QUOTE_ALL)
                #     for val in self.time_history:
                #         wr.writerow([val])
        self.operating_condition = data.state
        return


    def state_cb(self, msg):
        self.mass_state.append(msg)
        self.rx_state.append(msg.xr)
        self.mx_state.append(msg.xm)
        self.time_history.append((rospy.Time.now() - self.start_time).to_sec())
        for con in self.controllers:
            if len(con.raw_pose_x) > 120:
                self.input_state_index.append(len(con.raw_pose_x)-115)
            else:
                self.input_state_index.append(0)
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
        fmlist = []
        for con in self.controllers:
            con.update_pose()
            pos = con.int_marker_filt.pose.position
            quat = con.int_marker_filt.pose.orientation
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
            fm = con.int_marker_filt
            fm.header = con.int_marker_filt.header
            fm.pose = con.int_marker_filt.pose
            fmlist.append(fm)
        ma = VM.MarkerArray()
        ma.markers = mlist
        fma = VM.MarkerArray()
        fma.markers = fmlist
        self.con_pub.publish(ma)
        self.filt_con_pub.publish(fma)
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

    global trust_history
    global trial_trust
    if rospy.has_param("user_trust_history"):
        trust_history = rospy.get_param("user_trust_history")
        count = 0.
        trust_sum = 0.0
        for trust in reversed(trust_history):
            trust_sum += trust
            count += 1.
            if count > MOVING_AVERAGE_WINDOW:
                break
        trial_trust = trust_sum/count
    else:
        trust_history = [STARTING_TRUST]
        trial_trust = 1.0

    # Calculate cutoff frequency from trust
    global CUTOFF
    CUTOFF = (NYQUIST/2-MIN_CF)*pow(trial_trust, 5) + MIN_CF

    rospy.loginfo("[Joy] Current Trust: %.3f, Cutoff Frequency: %.4f", trial_trust, CUTOFF)

    try:
        sim = MarkerControls()
    except rospy.ROSInterruptException: pass

    rospy.spin()


if __name__=='__main__':
    main()
