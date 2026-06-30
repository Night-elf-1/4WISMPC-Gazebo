#!/bin/bash
source /opt/ros/noetic/setup.bash
source /home/hamster/test2_ws/devel/setup.bash

roscore &
ROSCORE_PID=$!
sleep 2

python3 /home/hamster/test2_ws/src/fwis_mpc_controller/test/mpc_dummy_curve.py &
PUB_PID=$!
sleep 3

echo "Starting node..."
# 使用 stdbuf 行缓冲，避免日志文件在进程退出前为空
stdbuf -oL rosrun fwis_mpc_controller fwis_mpc_controller_node _use_tf_for_state:=false _control_rate:=5.0 _rear_wheel_velocity_sign:=1.0 > /tmp/mpc_curve.log 2>&1 &
NODE_PID=$!
sleep 12

kill $NODE_PID $PUB_PID $ROSCORE_PID 2>/dev/null || true
wait $NODE_PID 2>/dev/null || true
sleep 1
cat /tmp/mpc_curve.log
