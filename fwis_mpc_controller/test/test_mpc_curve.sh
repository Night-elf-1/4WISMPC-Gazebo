#!/bin/bash
set -e
source /opt/ros/noetic/setup.bash
source /home/hamster/test2_ws/devel/setup.bash

roscore &
ROSCORE_PID=$!
sleep 2

python3 /home/hamster/test2_ws/src/fwis_mpc_controller/test/mpc_dummy_curve.py &
PUB_PID=$!
sleep 2

timeout 15 stdbuf -oL rosrun fwis_mpc_controller fwis_mpc_controller_node _use_tf_for_state:=false _control_rate:=5.0 _rear_wheel_velocity_sign:=1.0 || true

kill $PUB_PID $ROSCORE_PID 2>/dev/null || true
