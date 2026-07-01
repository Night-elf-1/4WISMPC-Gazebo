#include "fwis_nonlinear_mpc_controller/fwis_nonlinear_mpc.hpp"

// ===================== 工具函数 =====================

std::vector<double> fwisNonlinearMpcController::calculateReferenceSpeeds(const std::vector<double>& curvatures, const double& max_speed) {
    std::vector<double> referenceSpeeds;
    for (double k : curvatures) {
        double speed = max_speed * (1 - 3 * std::abs(k));
        speed = std::max(1.0, std::min(max_speed, speed));
        referenceSpeeds.push_back(speed);
    }
    return referenceSpeeds;
}

void fwisNonlinearMpcController::smooth_yaw(std::vector<double>& cyaw) {
    for (int i = 0; i < (int)cyaw.size() - 1; i++) {
        double dyaw = cyaw[i + 1] - cyaw[i];
        while (dyaw > M_PI / 2.0) {
            cyaw[i + 1] -= M_PI * 2.0;
            dyaw = cyaw[i + 1] - cyaw[i];
        }
        while (dyaw < -M_PI / 2.0) {
            cyaw[i + 1] += M_PI * 2.0;
            dyaw = cyaw[i + 1] - cyaw[i];
        }
    }
}

static std::tuple<int, double> calc_nearest_index_diff(double current_x, double current_y,
                                                         std::vector<double> cx, std::vector<double> cy,
                                                         std::vector<double> cyaw) {
    double mind = std::numeric_limits<double>::max();
    int ind = 0;
    for (int i = 0; i < (int)cx.size(); i++) {
        double idx = current_x - cx[i];
        double idy = current_y - cy[i];
        double d_e = std::sqrt(idx * idx + idy * idy);
        if (d_e < mind) {
            mind = d_e;
            ind = i;
        }
    }
    return std::make_tuple(ind, mind);
}

std::tuple<int, double> fwisNonlinearMpcController::calc_ref_trajectory(double current_x, double current_y,
                                                                 std::vector<double> cx, std::vector<double> cy,
                                                                 std::vector<double> cyaw) {
    auto [ind, d_e] = calc_nearest_index_diff(current_x, current_y, cx, cy, cyaw);
    return std::make_tuple(ind, d_e);
}

M_XREF_DIFF fwisNonlinearMpcController::build_horizon_reference(int min_index, std::vector<double>& cx, std::vector<double>& cy,
                                                        std::vector<double>& cyaw, std::vector<double>& speed, double dl,
                                                        double current_v, int &target_ind) {
    M_XREF_DIFF xref = M_XREF_DIFF::Zero();
    int ncourse = (int)cx.size();

    int ind = min_index;
    if (target_ind >= ind) ind = target_ind;

    xref(0, 0) = cx[ind];
    xref(1, 0) = cy[ind];
    xref(2, 0) = cyaw[ind];
    xref(3, 0) = speed[ind];

    double travel = 0.0;
    for (int i = 0; i < NMPC_T; i++) {
        travel += std::abs(current_v) * NMPC_DT;
        int dind = (int)std::round(travel / dl);

        int use_ind = (ind + dind < ncourse) ? (ind + dind) : (ncourse - 1);
        xref(0, i) = cx[use_ind];
        xref(1, i) = cy[use_ind];
        xref(2, i) = cyaw[use_ind];
        xref(3, i) = speed[use_ind];
    }

    target_ind = ind;
    return xref;
}

// ===================== 虚拟控制 -> 四轮命令 =====================

Eigen::VectorXd virtualControlToWheel(double vx, double omega, double L, double W) {
    Eigen::VectorXd U_out(8);
    double xw[4] = { L, -L, -L, L };
    double yw[4] = { W,  W, -W, -W };

    for (int i = 0; i < 4; i++) {
        double vx_i = vx - yw[i] * omega;
        double vy_i = xw[i] * omega;
        U_out(i) = std::sqrt(vx_i * vx_i + vy_i * vy_i) * (vx_i >= 0 ? 1.0 : -1.0);
        U_out(i + 4) = std::atan2(vy_i, vx_i);
        // 限幅保护
        U_out(i + 4) = std::max(NMPC_DELTA_MIN, std::min(NMPC_DELTA_MAX, U_out(i + 4)));
    }
    return U_out;
}

// ===================== NMPC 代价 / 约束函数 =====================

FG_EVAL_DIFF::FG_EVAL_DIFF(const M_XREF_DIFF &trajRef, const Eigen::VectorXd &uPrev, double L_, double W_)
    : traj_ref(trajRef), U_prev(uPrev), L(L_), W(W_) {}

void FG_EVAL_DIFF::operator()(FG_EVAL_DIFF::ADvector &fg, const FG_EVAL_DIFF::ADvector &vars) {
    fg[0] = 0;

    const double w_pos = 2.0;     // 位置跟踪权重（降低，避免高速时过度修正导致画龙）
    const double w_yaw = 1.0;     // 航向跟踪权重
    const double w_v   = 0.5;     // 纵向速度跟踪权重（降低，速度跟踪让位于平滑性）
    const double w_u   = 0.01;    // 控制量大小权重
    const double w_du  = 2.0;     // 控制量变化率权重（提高，抑制 vx/omega 高频振荡）

    // ---- 控制量代价（大小 + 相邻步变化率 + 速度跟踪） ----
    for (int i = 0; i < NMPC_T - 1; i++) {
        AD<double> vx    = vars[vx_start_ + i];
        AD<double> omega = vars[omega_start_ + i];

        fg[0] += w_u * (CppAD::pow(vx, 2) + CppAD::pow(omega, 2));
        fg[0] += w_v * CppAD::pow(traj_ref(3, i + 1) - vx, 2);

        AD<double> pvx    = (i == 0) ? AD<double>(U_prev(0)) : vars[vx_start_ + i - 1];
        AD<double> pomega = (i == 0) ? AD<double>(U_prev(1)) : vars[omega_start_ + i - 1];

        fg[0] += w_du * (CppAD::pow(vx - pvx, 2) + CppAD::pow(omega - pomega, 2));
    }

    // ---- 初始状态约束 ----
    fg[1 + x_start_]   = vars[x_start_];
    fg[1 + y_start_]   = vars[y_start_];
    fg[1 + yaw_start_] = vars[yaw_start_];

    // ---- 动力学约束 + 轨迹跟踪代价 ----
    for (int i = 0; i < NMPC_T - 1; i++) {
        AD<double> x0   = vars[x_start_ + i];
        AD<double> y0   = vars[y_start_ + i];
        AD<double> yaw0 = vars[yaw_start_ + i];

        AD<double> x1   = vars[x_start_ + i + 1];
        AD<double> y1   = vars[y_start_ + i + 1];
        AD<double> yaw1 = vars[yaw_start_ + i + 1];

        AD<double> vx    = vars[vx_start_ + i];
        AD<double> omega = vars[omega_start_ + i];

        // 标准运动学模型（vy=0，仅由 vx 和 omega 驱动）
        AD<double> x_pred   = x0 + vx * CppAD::cos(yaw0) * NMPC_DT;
        AD<double> y_pred   = y0 + vx * CppAD::sin(yaw0) * NMPC_DT;
        AD<double> yaw_pred = yaw0 + omega * NMPC_DT;

        fg[2 + x_start_ + i]   = x1 - x_pred;
        fg[2 + y_start_ + i]   = y1 - y_pred;
        fg[2 + yaw_start_ + i] = yaw1 - yaw_pred;

        // 轨迹跟踪代价
        fg[0] += w_pos * CppAD::pow(traj_ref(0, i + 1) - x_pred, 2);
        fg[0] += w_pos * CppAD::pow(traj_ref(1, i + 1) - y_pred, 2);
        AD<double> dyaw = yaw_pred - traj_ref(2, i + 1);
        fg[0] += w_yaw * CppAD::pow(CppAD::atan2(CppAD::sin(dyaw), CppAD::cos(dyaw)), 2);
    }

    // ---- 控制量变化率硬约束 ----
    int idx = 1 + 3 * NMPC_T;
    for (int i = 1; i < NMPC_T - 1; i++) {
        fg[idx++] = vars[vx_start_ + i] - vars[vx_start_ + i - 1];
        fg[idx++] = vars[omega_start_ + i] - vars[omega_start_ + i - 1];
    }
}

// ===================== NMPC 求解 =====================

Eigen::VectorXd fwisNonlinearMpcController::mpc_solve(std::vector<double>& cx, std::vector<double>& cy, std::vector<double>& cyaw,
                                              std::vector<double>& ck, std::vector<double>& speed, Eigen::Vector3d inital_x,
                                              int min_index, double min_errors, KinematicModel_MPC agv_model, parameters params_) {
    typedef CPPAD_TESTVECTOR(double) Dvector;

    double x   = inital_x(0);
    double y   = inital_x(1);
    double yaw = inital_x(2);

    static int target_ind_state = min_index;
    if (target_ind_state < min_index) target_ind_state = min_index;
    M_XREF_DIFF traj_ref = build_horizon_reference(min_index, cx, cy, cyaw, speed, 1.0,
                                                    agv_model.v, target_ind_state);

    size_t n_vars = NMPC_T * 3 + (NMPC_T - 1) * 2;
    size_t n_constraints = NMPC_T * 3 + 2 * (NMPC_T - 2);

    Dvector vars(n_vars);
    for (size_t i = 0; i < n_vars; i++) vars[i] = 0.0;

    vars[x_start_]   = x;
    vars[y_start_]   = y;
    vars[yaw_start_] = yaw;

    // 初始猜测：参考速度 + 0 角速度，沿参考方向直行
    double v_ref_init = speed[min_index];
    for (int i = 0; i < NMPC_T - 1; i++) {
        vars[vx_start_ + i]    = U.norm() < 1e-3 ? v_ref_init : U(0);
        vars[omega_start_ + i] = U.norm() < 1e-3 ? 0.0 : U(1);
    }
    // 前向仿真填充状态初值
    {
        double sx = x, sy = y, syaw = yaw;
        for (int i = 0; i < NMPC_T - 1; i++) {
            double vx    = vars[vx_start_ + i];
            double omega = vars[omega_start_ + i];
            sx   += vx * std::cos(syaw) * NMPC_DT;
            sy   += vx * std::sin(syaw) * NMPC_DT;
            syaw += omega * NMPC_DT;
            vars[x_start_ + i + 1]   = sx;
            vars[y_start_ + i + 1]   = sy;
            vars[yaw_start_ + i + 1] = syaw;
        }
    }

    Dvector vars_lowerbound(n_vars), vars_upperbound(n_vars);
    for (size_t i = 0; i < n_vars; i++) {
        vars_lowerbound[i] = -1.0e8;
        vars_upperbound[i] =  1.0e8;
    }
    for (int i = vx_start_; i < vx_start_ + NMPC_T - 1; i++) {
        vars_lowerbound[i] = NMPC_V_MIN;
        vars_upperbound[i] = NMPC_V_MAX;
    }
    for (int i = omega_start_; i < omega_start_ + NMPC_T - 1; i++) {
        vars_lowerbound[i] = NMPC_OMEGA_MIN;
        vars_upperbound[i] = NMPC_OMEGA_MAX;
    }

    Dvector constraints_lowerbound(n_constraints), constraints_upperbound(n_constraints);
    for (size_t i = 0; i < n_constraints; i++) {
        constraints_lowerbound[i] = 0;
        constraints_upperbound[i] = 0;
    }
    constraints_lowerbound[x_start_]   = x;
    constraints_upperbound[x_start_]   = x;
    constraints_lowerbound[y_start_]   = y;
    constraints_upperbound[y_start_]   = y;
    constraints_lowerbound[yaw_start_] = yaw;
    constraints_upperbound[yaw_start_] = yaw;

    // 控制量变化率硬约束
    {
        int off = NMPC_T * 3;
        for (int i = 1; i < NMPC_T - 1; i++) {
            constraints_lowerbound[off] = -NMPC_DV_MAX;
            constraints_upperbound[off] =  NMPC_DV_MAX;
            off++;
            constraints_lowerbound[off] = -NMPC_DOMEGA_MAX;
            constraints_upperbound[off] =  NMPC_DOMEGA_MAX;
            off++;
        }
    }

    FG_EVAL_DIFF fg_eval(traj_ref, U, params_.L, params_.W);

    std::string options;
    options += "Integer print_level  0\n";
    options += "Sparse  true        reverse\n";
    options += "Integer max_iter      150\n";
    options += "Numeric max_cpu_time          0.3\n";

    CppAD::ipopt::solve_result<Dvector> solution;
    CppAD::ipopt::solve<Dvector, FG_EVAL_DIFF>(
        options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
        constraints_upperbound, fg_eval, solution);

    bool ok = (solution.status == CppAD::ipopt::solve_result<Dvector>::success);
    if (!ok) {
        std::cout << "[fwisNonlinearMpcController] IPOPT 未能成功收敛, status=" << solution.status
                   << "，本周期沿用上一时刻控制量" << std::endl;
        return virtualControlToWheel(U(0), U(1), params_.L, params_.W);
    }

    double vx_out    = solution.x[vx_start_];
    double omega_out = solution.x[omega_start_];

    U(0) = vx_out;
    U(1) = omega_out;

    Eigen::VectorXd U_out = virtualControlToWheel(vx_out, omega_out, params_.L, params_.W);
    std::cout << "U_out: " << U_out.transpose() << std::endl;
    return U_out;
}

// ===================== 车辆运动学更新（与原版一致，用于仿真/状态估计） =====================

void KinematicModel_MPC::updatestate(Eigen::VectorXd U_cmd) {
    U = U_cmd;
    double v1 = U(0), v2 = U(1), v3 = U(2), v4 = U(3);
    double d1 = U(4), d2 = U(5), d3 = U(6), d4 = U(7);

    double dx = 0.25 * (v1 * std::cos(d1 + yaw) + v2 * std::cos(d2 + yaw) +
                        v3 * std::cos(d3 + yaw) + v4 * std::cos(d4 + yaw));
    double dy = 0.25 * (v1 * std::sin(d1 + yaw) + v2 * std::sin(d2 + yaw) +
                        v3 * std::sin(d3 + yaw) + v4 * std::sin(d4 + yaw));

    double denom = 4.0 * (L * L + W * W);
    double xw1 = L, yw1 = W;
    double xw2 = -L, yw2 = W;
    double xw3 = -L, yw3 = -W;
    double xw4 = L, yw4 = -W;

    double d_theta = ( (-yw1*std::cos(d1) + xw1*std::sin(d1))*v1 +
                       (-yw2*std::cos(d2) + xw2*std::sin(d2))*v2 +
                       (-yw3*std::cos(d3) + xw3*std::sin(d3))*v3 +
                       (-yw4*std::cos(d4) + xw4*std::sin(d4))*v4 ) / denom;

    x += dx * dt;
    y += dy * dt;
    yaw += d_theta * dt;
    v = 0.25 * (v1 + v2 + v3 + v4);
}

std::tuple<double, double, double, double> KinematicModel_MPC::getstate() {
    return std::make_tuple(x, y, yaw, v);
}
