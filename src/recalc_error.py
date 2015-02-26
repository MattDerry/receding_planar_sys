#!/usr/bin/env python
import rospy
import roslib
from puppeteer_msgs.srv import InputTrajectoryError
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Point

import glob
import os
import sys
import fnmatch
import numpy as np
import re
import math

BASEDIR = "data/user_matt/"
EXT3 = "_unfiltered_mass_ref_point.csv"
UPDATEDERROR = "updated_error.csv"
HEADER = "trial,rms_error,angle_error,trust\r\n"
ERROR_SCALING = 450.0

def check_for_csv_files(f):
    """
    Take in a filename for a bag file, and ensure that there are necessary CSV
    files to go along with the bag file. If they are missing, generate them.
    """
    if not os.path.exists(os.path.splitext(f)[0]+EXT3):
        print "Missing at least one CSV file!"
        # Generate CSV file:
        pkg_dir = roslib.packages.get_pkg_dir("receding_planar_sys")
        os.system(os.path.join(pkg_dir,"./bag_to_csv.sh {0:s}".format(f)))
    return

def calc_trust(error_scaling, rms, success=1):
    trust = 0.0
    trust = success * math.exp(-1 * error_scaling * rms)
    return trust

def calculate_and_append(bagname, fname, sumname, service_client):
    """
    Take in [fname] of a CSV file dumped from a bag file. Calculate stats about
    how the optimization proceeded, and append to sumfile
    """
    basedir = os.path.dirname(fname)
    outname = os.path.join(basedir, sumname)
    dat = np.genfromtxt(fname, names=True, delimiter=",")
    if not os.path.exists(outname):
        f = open(outname, "w")
        f.write(HEADER)
        f.close()

    out = []
    # trial number:
    match = re.search(r'(?<=trial)\d+', fname)
    out.append(int(match.group(0)))
    times = []
    points = []
    for p in dat:
        pt = Point()
        pt.x = p[4]
        pt.y = p[5]
        points.append(pt)
        times.append(p[0])
    print "========================================"
    print "Calling Error Service"
    print "========================================"
    error_response = service_client(points, times, bagname)
    # cost mean, median, and stdev:
    out.append(error_response.rms)
    out.append(error_response.angle_rms)
    trial_trust = calc_trust(ERROR_SCALING, error_response.rms)
    out.append(trial_trust)
    f = open(outname, "a")
    f.write(','.join(map(str,out)) + "\r\n")
    f.close()
    return

def main():
    """
    Run the main loop, by instantiating a System class, and then
    calling ros.spin
    """
    rospy.init_node('recalc_error')

    rospy.loginfo("Wait for trajectory service")
    rospy.wait_for_service("/robot_1/get_trajectory_error")
    trajectory_error_client = rospy.ServiceProxy("/robot_1/get_trajectory_error", InputTrajectoryError)
    rospy.loginfo("Trajectory service available")

    # recursively iterate through BASEDIR and operate on all bag files that are not
    # backups
    global BASEDIR
    pkg_dir = roslib.packages.get_pkg_dir("receding_planar_sys")
    TMPDIR = os.path.join(pkg_dir, BASEDIR)
    BASEDIR = TMPDIR

    rospy.loginfo("Setting data directory to: %s", BASEDIR)

    for root, dirnames, filenames in os.walk(BASEDIR):
        matches = []
        for filename in fnmatch.filter(filenames, '*.bag'):
            # don't add backups to array:
            if "backup" not in filename:
                matches.append(os.path.join(root, filename))
        if matches:
            print "\r\n"
            print "========================================"
            print "Working on a new directory:"
            print "========================================"
            # sort bag files by trial number:
            matches = sorted(matches, key=lambda x: int(re.findall(r'\d+', x)[-1]))
            # remove stats summary files in this dir:
            removelist = glob.glob(os.path.dirname(matches[0])+"/*updated_error.csv")
            for r in removelist:
                os.remove(r)
            for m in matches:
                print "parsing file:",m
                check_for_csv_files(m)
                errfile = os.path.splitext(m)[0]+EXT3
                calculate_and_append(m, errfile, UPDATEDERROR, trajectory_error_client)



if __name__=='__main__':
    main()
