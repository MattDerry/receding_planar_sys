#!/bin/bash

# Define bag and csv filename:
filename=
input=$1
if [ -n "$input" ]
then
    filename=$1
else
	exit 1
fi
echo "Using '${filename}' as filename"


# Wait for a user to press a button:
# echo "Press any button to start recording data..."
# read -n 1 -s
# echo "Beginning recording..."
# Start recording bag file:
rosbag record -O ${filename} -e "(.*)meas_config" "(.*)ref_config" \
    "(.*)filt_state" "(.*)filt_config" "(.*)serial_commands" "(.*)serviced_values" \
    "(.*)post_covariance" "(.*)object1_position" "(.*)robot_kinect_position" \
    "(.*)start_time" "(.*)optimization_data" "(.*)mass_ref_point" \
    "(.*)current_target" "(.*)collision" "(.*)limit_collision" "(.*)obstacle_markers" \
	"(.*)user_score"  "(.*)current_pose" "(.*)limit_markers" "(.*)operating_condition" \
	"(.*)unfiltered_mass_ref_point" "(.*)sim_optimization_data"

# sleep 1
# echo "Now press any button to stop recording..."
# read -n 1 -s
# echo "Stopping recording, now killing recording process..."
# Stop recording:
# killall -2 record
