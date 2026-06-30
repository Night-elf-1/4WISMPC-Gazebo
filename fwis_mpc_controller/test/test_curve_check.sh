#!/bin/bash
source /opt/ros/noetic/setup.bash
source /home/hamster/test2_ws/devel/setup.bash

roscore &
ROSCORE_PID=$!
sleep 2

python3 /home/hamster/test2_ws/src/fwis_mpc_controller/test/mpc_dummy_curve.py &
PUB_PID=$!
sleep 2

echo "=== path size ==="
rostopic echo -n 1 /reference_path/poses | grep -c "position:"

echo "=== curvature ==="
rostopic echo -n 1 /reference_curvature

echo "=== speeds ==="
rostopic echo -n 1 /reference_speeds

kill $PUB_PID $ROSCORE_PID 2>/dev/null || true
