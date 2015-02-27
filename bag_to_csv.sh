#!/bin/bash

# Define bag and csv filename:
filename=default.bag
input=$1
if [ -n "$input" ]
then
    filename=$1
fi

if [ -a ${filename} ]
then
    # info=`rosbag info ${filename}`
    # meas=`echo $info | grep -e '\ \/[^ ]*' -o |grep -e 'meas_config'`
    # filt=`echo $info | grep -e '\ \/[^ ]*' -o |grep -e 'filt_config'`
    # ref=`echo $info | grep -e '\ \/[^ ]*' -o |grep -e 'ref_config'`
    # ser=`echo $info | grep -e '\ \/[^ ]*' -o |grep -e 'serial_commands'`
    # filtstate=`echo $info | grep -e '\ \/[^ ]*' -o |grep -e 'filt_state'`
    # rostopic echo -p -b ${filename} $filt > ${filename%.*}_filt.txt
    # rostopic echo -p -b ${filename} $meas > ${filename%.*}_meas.txt
    # rostopic echo -p -b ${filename} $ref > ${filename%.*}_ref.txt
    # rostopic echo -p -b ${filename} $ser > ${filename%.*}_ser.txt
    # rostopic echo -p -b ${filename} $filtstate > ${filename%.*}_filtstate.txt
	rostopic echo -p -b ${filename} /robot_1/optimization_data > ${filename%.*}_opt_data.csv
	rostopic echo -p -b ${filename} /robot_1/sim_optimization_data > ${filename%.*}_sim_opt_data.csv
	rostopic echo -p -b ${filename} /robot_1/fixed_sim_optimization_data > ${filename%.*}_fixed_sim_opt_data.csv
	rostopic echo -p -b ${filename} /robot_1/unfiltered_mass_ref_point > ${filename%.*}_unfiltered_mass_ref_point.csv
else
    echo "[ERROR] bag file not found: ${filename}"
fi
