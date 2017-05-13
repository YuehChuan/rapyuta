#!/bin/bash
echo "Activating ROS..."
source /opt/ros/kinetic/setup.bash
echo "...done."


echo "Setup ROS_HOSTNAME."
export ROS_HOSTNAME=$HOSTNAME.local
export RAPYUTA_ROOT=$HOME/rapyuta

echo "Building machines file..."
make -C  $RAPYUTA_ROOT
echo "...done"
echo "Activating development."
source $RAPYUTA_ROOT/rapyuta_ws/devel/setup.bash
export PATH=$PATH:$RAPYUTA_ROOT/rapyuta_ws/src/ubiquity_launches/bin


# TODO: check that the time is >= 2015

# TODO: run a python script that checks all libraries are installed

exec "$@" #Passes arguments. Need this for ROS remote launching to work.


echo "done!!!"
