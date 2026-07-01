//
// 非线性MPC (NMPC) 版本 —— 四轮独立转向/驱动 AGV 路径跟踪
// 采用“虚拟控制量 [vx, omega] + 四轮逆运动学映射”的写法，避免 8 个独立轮速/转角
// 优化时出现的病态解（原地打转、四轮各打各的）。
//
#ifndef FWIS_NONLINEAR_MPC_HPP
#define FWIS_NONLINEAR_MPC_HPP

#include <iostream>
#include <vector>
#include <tuple>
#include <Eigen/Dense>
#include <cmath>
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "cubic_spline.hpp"

using CppAD::AD;
using namespace std;

// ============== 可调参数 ==============
#define NMPC_NX 3        // 状态量维度 [x, y, yaw]
#define NMPC_NU 2        // 优化控制量维度 [vx, omega]（车体纵向速度、横摆角速度）
#define NMPC_T 10        // 预测步长（horizon）
#define NMPC_DT 0.1      // 采样时间

#define NMPC_V_MAX 5.0
#define NMPC_V_MIN -5.0
#define NMPC_OMEGA_MAX 1.0    // 横摆角速度上限 [rad/s]
#define NMPC_OMEGA_MIN -1.0   // 横摆角速度下限 [rad/s]
#define NMPC_DV_MAX 0.5       // 单步 vx 最大变化 [m/s]
#define NMPC_DOMEGA_MAX 0.2   // 单步 omega 最大变化 [rad]

// 四轮转向角上限（逆运动学映射时使用）
#define NMPC_DELTA_MAX (70.0/180.0*M_PI)
#define NMPC_DELTA_MIN (-70.0/180.0*M_PI)

struct parameters {
    double L = 0.4615;       // 质心到前后轴的距离 (Half wheelbase)
    double W = 0.4;          // 质心到左右轮的距离 (Half track width)
    int NX = NMPC_NX;
    int NU = NMPC_NU;
    int NP = NMPC_T;
    int NC = NMPC_T - 1;
    double dt = NMPC_DT;
};

// 参考轨迹矩阵：行 0=x,1=y,2=yaw,3=v_ref，列为预测时域上的各时刻参考点
using M_XREF_DIFF = Eigen::Matrix<double, 4, NMPC_T>;

// 优化变量在 vars 数组中的起始下标
static const int x_start_    = 0;
static const int y_start_    = x_start_    + NMPC_T;
static const int yaw_start_  = y_start_    + NMPC_T;
static const int vx_start_   = yaw_start_  + NMPC_T;
static const int omega_start_= vx_start_   + (NMPC_T - 1);

class KinematicModel_MPC {
public:
    double x, y, yaw, v, L, W, dt;
    Eigen::VectorXd U;      // 上一时刻实际控制量(8维)

    KinematicModel_MPC(double x, double y, double yaw, double v, double L, double W, double dt)
        : x(x), y(y), yaw(yaw), v(v), L(L), W(W), dt(dt) {
        U = Eigen::VectorXd::Zero(8);
    };
    ~KinematicModel_MPC() {};

    void updatestate(Eigen::VectorXd U_cmd);
    std::tuple<double, double, double, double> getstate();
};

// ============== 虚拟控制 -> 四轮命令 ==============
// 根据车体纵向速度 vx 和横摆角速度 omega，计算四个轮子的线速度和转角。
// 假设无侧向滑移（vy=0），即标准 4WIS 运动学模型。
Eigen::VectorXd virtualControlToWheel(double vx, double omega, double L, double W);

// ============== NMPC 代价/约束函数对象 ==============
class FG_EVAL_DIFF {
public:
    M_XREF_DIFF traj_ref;
    Eigen::VectorXd U_prev;   // 上一时刻虚拟控制 [vx, omega]
    double L, W;

    FG_EVAL_DIFF(const M_XREF_DIFF &trajRef, const Eigen::VectorXd &uPrev, double L_, double W_);

    typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
    void operator()(ADvector &fg, const ADvector &vars);
};

class fwisNonlinearMpcController {
public:
    int NX, NU, NP, NC;
    Eigen::VectorXd U;      // 上一时刻虚拟控制 [vx, omega]

    fwisNonlinearMpcController(int nx, int nu, int np, int nc) : NX(nx), NU(nu), NP(np), NC(nc) {
        U = Eigen::VectorXd::Zero(nu);
    };
    ~fwisNonlinearMpcController() {};

    std::vector<double> calculateReferenceSpeeds(const std::vector<double>& curvatures, const double& max_speed);
    void smooth_yaw(std::vector<double>& cyaw);
    std::tuple<int, double> calc_ref_trajectory(double current_x, double current_y, std::vector<double> cx, std::vector<double> cy, std::vector<double> cyaw);
    M_XREF_DIFF build_horizon_reference(int min_index, std::vector<double>& cx, std::vector<double>& cy,
                                         std::vector<double>& cyaw, std::vector<double>& speed, double dl,
                                         double current_v, int &target_ind);

    // NMPC 求解器：返回本周期实际下发的 8 维轮速/转角命令 [v1,v2,v3,v4,d1,d2,d3,d4]
    Eigen::VectorXd mpc_solve(std::vector<double>& cx, std::vector<double>& cy, std::vector<double>& cyaw,
                               std::vector<double>& ck, std::vector<double>& speed, Eigen::Vector3d inital_x,
                               int min_index, double min_errors, KinematicModel_MPC agv_model, parameters params_);
};

#endif
