#ifndef FWIS_MPC_HPP
#define FWIS_MPC_HPP

#include <iostream>
#include <vector>
#include <tuple>
#include <Eigen/Dense>
#include <cmath>
#include "cubic_spline.hpp"
#include "OsqpEigen/OsqpEigen.h"
#include "eigen3/Eigen/Core"

static inline bool finish = true;

struct parameters {
    double L = 0.4615;  // 质心到前后轴的距离 (Half wheelbase)，与 smart.xacro 中 front_tyre_x 一致
    double W = 0.4;   // 质心到左右轮的距离 (Half track width)
    int NX = 3;       // 状态量: x, y, yaw
    int NU = 8;       // 控制量: v1, v2, v3, v4, delta1, delta2, delta3, delta4
    int NP = 30;      // 预测步长
    int NC = 10;       // 控制步长
    double dt = 0.1;  // 采样时间
    double row = 10;
};

class KinematicModel_MPC {
public:
    double x, y, yaw, v;
    double L, W, dt;
public:
    KinematicModel_MPC(double x, double y, double psi, double v, double L, double W, double dt) 
        : x(x), y(y), yaw(psi), v(v), L(L), W(W), dt(dt) {};
    ~KinematicModel_MPC(){};
    
    // 更新AGV状态 (接收8个控制量)
    void updatestate(const Eigen::VectorXd& U_real);
    // 获取AGV状态
    std::tuple<double, double, double, double> getstate();
};

class fwisMpcController {
public:
    int NX, NU, NP, NC;
    Eigen::VectorXd U;
    Eigen::MatrixXd R;
    Eigen::MatrixXd RB;
    Eigen::MatrixXd Q;
    Eigen::MatrixXd QB;

public:
    fwisMpcController(int nx, int nu, int np, int nc) 
        : NX(nx), NU(nu), NP(np), NC(nc), 
          R(Eigen::MatrixXd::Identity(nu, nu)), 
          RB(Eigen::MatrixXd::Identity(nc * nu, nc * nu)), 
          Q(Eigen::MatrixXd::Identity(nx, nx)), 
          QB(100 * Eigen::MatrixXd::Identity(np * nx, np * nx)), 
          U(Eigen::VectorXd::Zero(nu)) {
              // 初始化一个基础控制量，避免初始状态矩阵奇异
              U.head(4).setConstant(0.01); 
          };
    ~fwisMpcController(){};

    std::vector<double> calculateReferenceSpeeds(const std::vector<double>& curvatures, const double& max_speed);
    void smooth_yaw(std::vector<double>& cyaw);
    std::tuple<int, double> calc_ref_trajectory(double current_x, double current_y, std::vector<double> cx, std::vector<double> cy, std::vector<double> cyaw);
    
    // MPC求解器
    Eigen::VectorXd mpc_solve(std::vector<double>& cx, std::vector<double>& cy, std::vector<double>& cyaw, std::vector<double>& ck, std::vector<double>& speed, Eigen::Vector3d inital_x, int min_index, double min_errors, parameters params_);
};

#endif