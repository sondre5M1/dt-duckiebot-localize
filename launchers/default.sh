#!/bin/bash

source /environment.sh

# initialize launch file
dt-launchfile-init

# OUR COMMANDS BELOW THIS LINE
# ----------------------------------------------------------------------------

dt-set-module-healthy
# NOTE: Use the variable DT_REPO_PATH to know the absolute path to your code
# NOTE: Use `dt-exec COMMAND` to run the main process (blocking process)

# launching app
dt-exec echo "This code is intended to work on a duckiebot model DB21M."
roslaunch dt_duckie_odometry odometry_node.launch veh:=$VEHICLE_NAME

# ----------------------------------------------------------------------------
# OUR CODE ABOVE THIS LINE

# wait for app to end
dt-launchfile-join
