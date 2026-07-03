# 4WIS MPC四轮独立求解路径跟踪

## 项目依赖：

```
ubuntu20.04
matplotlibcpp
cmake
Eigen
osqp
osqpeigen
IPOPT
CPPAD
```

- cmake安装：

```bash
sudo apt install cmake
```

- Eigen安装：

```bash
sudo apt-get install libeigen3-dev
```

- Ipopt和Cppad安装：

参考链接：https://blog.csdn.net/weixin_42301220/article/details/127946528

- Osqp和OsqpEigen安装

参考链接：https://blog.csdn.net/qq_36497771/article/details/139980479



## 如何编译：

1.拉取代码：

```bash
git clone https://github.com/Night-elf-1/4WISMPC-Gazebo.git
```

2.在根目录下编译代码：

```bash
catkin_make
```



## 使用方法：

### main分支：

1.线性MPC启动：

```bash
roslaunch fwis_mpc_controller fwis_mpc_tracking.launch
```

2.非线性MPC启动：

```bash
roslaunch fwis_nonlinear_mpc_controller fwis_nonlinear_mpc_tracking.launch
```

3.8 维 NMPC（四轮独立轮速/转角直接优化）启动：

```bash
roslaunch nmpc_4wisnew2 nmpc_4wisnew2_tracking.launch
```

