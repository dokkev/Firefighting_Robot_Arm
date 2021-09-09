#!/bin/bash

checkstring='True'
while true; do
  returnstring=$(rostopic echo /manipulator_connected -n1)
  #echo $returnstring
  if grep -q "$checkstring" <<< "$returnstring"; then
    echo "manipulator controller ready"
    break
  else
    echo "waiting for robot controller to become ready"
  fi
  sleep 1
done

# start second part of bring up script
roslaunch hdt_6dof_a24_pincer_bringup hdt_arm_bringup_2.launch controller_type:=$1

