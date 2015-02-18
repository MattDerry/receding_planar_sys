#!/usr/bin/env python

"""
This script calls the record.sh script using the correct filenames. It also
shuts everything down correctly.
"""

import os
import sys
import pickle
import rospkg
import argparse
import glob

# constants:
SUBDIR = "data/"
PKLNAME = "trial_num.pkl"
EXT = ".bag"

# get package base path
rospack = rospkg.RosPack()
pkgpath = rospack.get_path('receding_planar_sys')
DIR = os.path.join(pkgpath, SUBDIR)

# get the user's name
parser = argparse.ArgumentParser()
parser.add_argument("--user", "-u", help="User name or identifier for naming files",
                    required=True)
arguments, unknown = parser.parse_known_args()
NAME = arguments.user

# get the trial number:
PKLNAME = os.path.join(pkgpath, PKLNAME)
if os.path.exists(PKLNAME):
    NUM = pickle.load(open(PKLNAME, "rb"))
else:
    print "[WARN] No pickle file found!"
    print "name =",PKLNAME
    NUM = 0
    print "Record script Assuming trial ",NUM

# recording file name:
fname = "{0:s}_trial{1:d}{2:s}".format(NAME, NUM, EXT)
fname = os.path.join(DIR, fname)
print "Recording data under name:",fname

# check for collision and backup collisions:
if os.path.exists(fname):
    # get number of collisions
    N = len(glob.glob(os.path.join(DIR,"{0:s}_trial{1:d}_backup*{2:s}".format(NAME, NUM, EXT))))
    # get new name:
    print "Target bag file exists!"
    newname = os.path.join(DIR,"{0:s}_trial{1:d}_backup{3:d}{2:s}".format(NAME, NUM, EXT, N+1))
    print "newname =",newname
    if not os.path.exists(newname):
        os.rename(fname, newname)
    else:
        print "[ERROR] could not make proper backup file"
        sys.exit(1)

# now we are ready to record:
cmd = os.path.join(pkgpath, "record.sh {0:s}".format(fname))
os.system(cmd)


# import subprocess
# cmd = [cmd, ]
# p = subprocess.Popen(cmd, stdout=subprocess.PIPE)
# for line in p.stdout:
#     print line
# p.wait()
# print p.returncode



