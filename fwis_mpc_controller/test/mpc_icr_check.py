#!/usr/bin/env python3
"""
监听 MPC 下发的 8 个车轮指令，验证是否满足 4WIS 公共 ICR 约束，
并检查后轮滑移速度符号是否与 car_model 4WIS 约定一致（后轮取负）。

约定：
  0: front-left   xw=+L_f, yw=+W
  1: rear-left    xw=-L_r, yw=+W
  2: rear-right   xw=-L_r, yw=-W
  3: front-right  xw=+L_f, yw=-W
"""
import sys
import math
import rospy
from std_msgs.msg import Float64

# 与 car_model / MPC 一致的几何参数
L_f = 0.4615
L_r = 0.4725
W = 0.4
WHEEL_R = 0.15

xw = [L_f, -L_r, -L_r, L_f]
yw = [W, W, -W, -W]

vel_topics = [
    '/smart/front_left_svelocity_controller/command',
    '/smart/rear_left_velocity_controller/command',
    '/smart/rear_right_velocity_controller/command',
    '/smart/front_right_velocity_controller/command',
]
steer_topics = [
    '/smart/front_left_str_controller/command',
    '/smart/rear_left_str_controller/command',
    '/smart/rear_right_str_controller/command',
    '/smart/front_right_str_controller/command',
]


class IcrChecker:
    def __init__(self):
        self.vel = [None] * 4      # 线速度幅值（>0）
        self.raw_cmd = [None] * 4   # 原始角速度指令（带符号）
        self.steer = [None] * 4
        for i, t in enumerate(vel_topics):
            rospy.Subscriber(t, Float64, lambda msg, idx=i: self.cb_vel(msg, idx), queue_size=1)
        for i, t in enumerate(steer_topics):
            rospy.Subscriber(t, Float64, lambda msg, idx=i: self.cb_steer(msg, idx), queue_size=1)

    def cb_vel(self, msg, idx):
        # 保存原始带符号的指令（用于符号检查）和纵向速度幅值（用于 ICR 几何）
        self.raw_cmd[idx] = msg.data
        self.vel[idx] = abs(msg.data) * WHEEL_R  # 线速度幅值

    def cb_steer(self, msg, idx):
        self.steer[idx] = msg.data

    def ready(self):
        return all(v is not None for v in self.vel) and all(s is not None for s in self.steer)

    def check(self):
        # 用四个车轮的横向/纵向速度分量分别估计车体 v_body 和 omega，再取平均
        omega_est = []
        vbody_est = []
        for i in range(4):
            v = self.vel[i]
            d = self.steer[i]
            if abs(xw[i]) > 1e-6:
                omega_est.append(v * math.sin(d) / xw[i])
            vbody_est.append(v * math.cos(d) + yw[i] * omega_est[-1])

        omega = sum(omega_est) / len(omega_est)
        v_body = sum(vbody_est) / len(vbody_est)

        print('\n=== ICR check ===')
        print('v_body = %.4f m/s, omega = %.4f rad/s, R = %.3f m, k = %.3f 1/m' %
              (v_body, omega, v_body / omega if abs(omega) > 1e-6 else float('inf'), omega / v_body if abs(v_body) > 1e-6 else 0.0))
        print('wheel |  v_lin  |  delta  | residual h_i')
        max_res = 0.0
        for i in range(4):
            h = (v_body - yw[i] * omega) * math.sin(self.steer[i]) - xw[i] * omega * math.cos(self.steer[i])
            print('  %d   | %7.3f | %7.3f | %+.4f' % (i, self.vel[i], self.steer[i], h))
            max_res = max(max_res, abs(h))

        # 符号检查：该 Gazebo 模型四个轮子转动轴同向，前进/后退时四个指令应同号。
        # 这里只要求同侧前后轮同号即可（避免后轮被错误取反）。
        sign_ok = True
        if abs(self.raw_cmd[0]) > 1e-3 and abs(self.raw_cmd[1]) > 1e-3:
            if self.raw_cmd[0] * self.raw_cmd[1] < 0:
                sign_ok = False
                print('FAIL: front-left and rear-left velocity commands have opposite signs')
        if abs(self.raw_cmd[3]) > 1e-3 and abs(self.raw_cmd[2]) > 1e-3:
            if self.raw_cmd[3] * self.raw_cmd[2] < 0:
                sign_ok = False
                print('FAIL: front-right and rear-right velocity commands have opposite signs')

        print('max |residual| = %.4f' % max_res)
        if max_res < 0.01 and sign_ok:
            print('PASS: ICR constraints satisfied and rear velocity sign matches car_model convention')
            return 0
        else:
            print('FAIL')
            return 1


def main():
    rospy.init_node('mpc_icr_check', anonymous=True)
    checker = IcrChecker()
    timeout = rospy.Duration(8.0)
    start = rospy.Time.now()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        if checker.ready():
            sys.exit(checker.check())
        if rospy.Time.now() - start > timeout:
            print('TIMEOUT: did not receive all 8 wheel commands')
            sys.exit(2)
        rate.sleep()


if __name__ == '__main__':
    main()
