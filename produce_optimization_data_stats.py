"""
This script descends recursively into all directories in BASEDIR. It then
grabs all CSV files that contain '/robot_1/optimization_data' and
'/robot_1/sim_optimization_data' dumped from bag files. It then creates new CSV
files in each of the directories containing a set of statistics for the CSV
files.
"""


import glob
import os
import sys
import fnmatch
import numpy as np
import re

BASEDIR = "data/"
EXT1 = "_opt_data.csv"
EXT2 = "_sim_opt_data.csv"
SUMNAMEOPT = "optimization_summary.csv"
SUMNAMESIM = "sim_optimization_summary.csv"
HEADER = "trial,cost_mean,cost_median,cost_stdev,failed_to_convege_count\r\n"


def check_for_csv_files(f):
    """
    Take in a filename for a bag file, and ensure that there are necessary CSV
    files to go along with the bag file. If they are missing, generate them.
    """
    if not os.path.exists(os.path.splitext(f)[0]+EXT1) or \
       not os.path.exists(os.path.splitext(f)[0]+EXT2):
        print "Missing at least one CSV file!"
        # Generate CSV file:
        os.system("./bag_to_csv.sh {0:s}".format(f))
    return




def calculate_stats_and_append(fname, sumname):
    """
    Take in [fname] of a CSV file dumped from a bag file. Calculate stats about
    how the optimization proceeded, and append to sumfile
    """
    basedir = os.path.dirname(fname)
    outname = os.path.join(basedir, sumname)
    # dat = np.loadtxt(optfile, skiprows=1, delimiter=",")
    dat = np.genfromtxt(fname, names=True, delimiter=",")
    if not os.path.exists(outname):
        f = open(outname, "w")
        f.write(HEADER)
        f.close()
    else:
        out = []
        # trial number:
        match = re.search(r'(?<=trial)\d+', fname)
        out.append(int(match.group(0)))
        # cost mean, median, and stdev:
        out.append(np.mean(dat['fieldcost']))
        out.append(np.median(dat['fieldcost']))
        out.append(np.std(dat['fieldcost']))
        # convergence failed count:
        out.append(dat['fielddone'].astype(np.int16).tolist().count(0))
        f = open(outname, "a")
        f.write(','.join(map(str,out)) + "\r\n")
        f.close()
    return
    
    

# recursively iterate through BASEDIR and operate on all bag files that are not
# backups
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
        removelist = glob.glob(os.path.dirname(matches[0])+"/*optimization_summary.csv")
        for r in removelist:
            os.remove(r)
        for m in matches:
            print "parsing file:",m
            check_for_csv_files(m)
            optfile = os.path.splitext(m)[0]+EXT1
            simfile = os.path.splitext(m)[0]+EXT2
            calculate_stats_and_append(optfile, SUMNAMEOPT)
            calculate_stats_and_append(simfile, SUMNAMESIM)
