#!/bin/bash
set -e
source /opt/ros/noetic/setup.bash
source /home/hamster/test2_ws/devel/setup.bash

# Start roscore in background
roscore &
ROSCORE_PID=$!
sleep 2

# Start dummy data publisher in background
python3 /home/hamster/test2_ws/src/fwis_mpc_controller/test/mpc_dummy_data.py &
PUB_PID=$!
sleep 2

# Run MPC node with a timeout
timeout 20 stdbuf -oL rosrun fwis_mpc_controller fwis_mpc_controller_node _use_tf_for_state:=false _control_rate:=5.0 _rear_wheel_velocity_sign:=1.0 || true

# Cleanup
kill $PUB_PID $ROSCORE_PID 2>/dev/null || true
