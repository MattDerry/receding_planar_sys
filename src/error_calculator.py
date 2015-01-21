# ROS imports
import roslib; roslib.load_manifest('receding_planar_sys')
import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import QuaternionStamped
from geometry_msgs.msg import Point
from puppeteer_msgs.msg import FullRobotState
from puppeteer_msgs.msg import PlanarSystemConfig
from puppeteer_msgs.msg import OperatingCondition
from puppeteer_msgs.srv import OperatingConditionChange
from puppeteer_msgs.srv import OperatingConditionChangeRequest

# OTHER imports
import trep
from trep import tx, ty, tz, rx, ry, rz
from math import sin, cos
from math import pi as mpi
import numpy as np
import sys
import scipy as sp

## define some global constants:
BALL_MASS = 0.1244 ## kg
g = 9.81 ## m/s^2
h0 = 1 ## default height of robot in m
DT = 1/30.0 ## nominal dt for the system

## define some miscellaneous geometry helper functions
def to_se3(R,p):
    """
    This function takes in a vector and a rotation matrix, and returns
    an element of SE(3) as a 4x4 numpy array
    """
    pt = np.ravel(p)
    if np.size(pt) != 3:
        print "Wrong size vector when converting to SE(3)"
        return 1
    elif R.shape != (3,3):
        print "Wrong size rotation matrix when converting to SE(3)"
        return 1
    else:
        return np.vstack((np.hstack((R,np.array([pt]).T)),np.array([0,0,0,1])))


def hat(w):
    """
    This function implements the 3D 'hat' operator
    """
    wt = np.ravel(w)
    return np.array([[0,-wt[2],wt[1]],
            [wt[2],0,-wt[0]],
            [-wt[1],wt[0],0]])

def quat_to_rot(quat):
    """
    converts a quaternion of form [w,x,y,z] to a 3x3 element of
    SO(3) (represented as a numpy array)
    """
    quat = np.array(quat)
    q = np.ravel(quat)
    if np.size(q) != 4:
        print "Invalid quaternion passed to quat_to_rot()"
        return 1
    th = 2*np.arccos(q[0])
    if th == 0:
        w = np.array([0,0,0])
    else:
        w = q[1:]/np.sin(th/2.0)
    R = np.eye(3)+hat(w)*sin(th)+np.dot(hat(w),hat(w))*(1-cos(th))
    return R



## define a class for simulating the system.
class MassSystem3D:
    """
    Class for integrating system.  Includes many helper functions.
    """
    def __init__(self, mass=BALL_MASS, q0=None):
        self.mass = mass
        self.q0 = q0
        self.sys = self.create_system()
        ## set initial configuration variables
        if q0:
            if len(q0) < sys.nQ:
                print "Invalid number of initial conditions"
                sys.exit()
            self.sys.q = {
                'xm' : q0[0],
                'ym' : q0[1],
                'zm' : q0[2],
                'xr' : q0[3],
                'zr' : q0[4],
                'r' : q0[5],
                }

        self.sys.satisfy_constraints()
        self.mvi = trep.MidpointVI(self.sys)
        self.mvi.initialize_from_configs(0,self.sys.q,DT,self.sys.q)

        return

    def get_current_configuration(self):
        return self.mvi.q2

    def get_current_time(self):
        return self.mvi.t2

    def reset_integration(self, state=None):
        if state:
            self.q0 = state
        if self.q0:
            self.sys.q = {
                'xm' : self.q0[0],
                'ym' : self.q0[1],
                'zm' : self.q0[2],
                'xr' : self.q0[3],
                'zr' : self.q0[4],
                'r' : self.q0[5],
                }
        self.sys.satisfy_constraints()
        self.mvi.initialize_from_configs(0,self.q0,DT,self.q0)

        return

    def create_system(self):
        # define system:
        system = trep.System()

        frames = [
            tx('xm', name='x-mass'), [
                ty('ym', name='y-mass'), [
                    tz('zm', name='z-mass', mass=self.mass) ]],
            ty(h0, name='robot_plane'), [
                tx('xr', name='x-robot', kinematic=True), [
                    tz('zr', name='z-robot', kinematic=True) ]]]
        system.import_frames(frames)
        trep.potentials.Gravity(system, (0, -g, 0))
        trep.forces.Damping(system, 0.05)

        # add string constraint as a kinematic configuration var
        trep.constraints.Distance(system, 'z-mass', 'z-robot','r')

        return system


    def take_step(self, dt=DT, rho=()):
        self.mvi.step(self.mvi.t2+dt, (), rho)
        return

## Now let's define a class for ros to use.  We will define all
## subscriptions, publishers, and callbacks
class FastSimulator:
    def __init__(self):
        rospy.loginfo("Starting fast_simulation node!")

        ## define a transform listener for publishing the transforms
        ## to the location of the mass
        self.listener = tf.TransformListener()
        self.br = tf.TransformBroadcaster()
        ## wait for transform:
        try:
            rospy.loginfo("Waiting for transform from /map to /optimization_frame")
            self.listener.waitForTransform("/optimization_frame", "/map",
                                           rospy.Time(), rospy.Duration(3.0))
        except (tf.Exception):
            rospy.logwarn("Could not find transform to optimization_frame after waiting!")

        self.sys = MassSystem3D()

def main():
    """
    Run the main loop, by instatiating a MassSimulator, and then
    calling ros.spin
    """
    rospy.init_node('fast_simulator', log_level=rospy.INFO)

    try:
        sim = FastSimulator()
    except rospy.ROSInterruptException: pass

    rospy.spin()


if __name__=='__main__':
    main()
