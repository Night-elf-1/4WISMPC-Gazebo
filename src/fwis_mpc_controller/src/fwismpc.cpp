#include "fwis_mpc_controller/fwismpc.hpp"

using namespace std;

std::vector<double> fwisMpcController::calculateReferenceSpeeds(const std::vector<double>& curvatures, const double& max_speed){
    std::vector<double> referenceSpeeds;
    for (double k : curvatures) {
        double speed = max_speed * (1 - 3 * abs(k)); 
        speed = std::max(0.5, std::min(max_speed, speed)); 
        referenceSpeeds.push_back(speed);
    }
    return referenceSpeeds;
}

void fwisMpcController::smooth_yaw(vector<double>& cyaw){
    for(int i=0; i<cyaw.size()-1; i++){
        double dyaw = cyaw[i+1] - cyaw[i];
        while (dyaw > M_PI/2.0){
            cyaw[i+1] -= M_PI*2.0;
            dyaw = cyaw[i+1] - cyaw[i];
        }
        while (dyaw < -M_PI/2.0){
            cyaw[i+1] += M_PI*2.0;
            dyaw = cyaw[i+1] - cyaw[i];
        }
    }
}

std::tuple<int, double> calc_nearest_index(double current_x, double current_y, vector<double> cx, vector<double> cy, vector<double> cyaw){
    double mind = numeric_limits<double>::max();        
    double ind = 0;                                     
    for (int i = 0; i < cx.size(); i++){
        double idx = current_x - cx[i];
        double idy = current_y - cy[i];
        double d_e = std::sqrt(idx*idx + idy*idy);   // 欧氏距离，避免垂距导致最近点回退

        if (d_e < mind){
            mind = d_e;
            ind = i;
        }
    }
    return std::make_tuple(ind, mind);
}

std::tuple<int, double> fwisMpcController::calc_ref_trajectory(double current_x, double current_y, vector<double> cx, vector<double> cy, vector<double> cyaw){
    auto [ind, d_e] = calc_nearest_index(current_x, current_y, cx, cy, cyaw);
    return std::make_tuple(ind, d_e);
}

Eigen::VectorXd fwisMpcController::mpc_solve(vector<double>& cx, vector<double>& cy, vector<double>& cyaw, vector<double>& ck, vector<double>& speed, Eigen::Vector3d inital_x, int min_index, double min_errors, parameters params_){
    
    // // 控制量物理约束 (4个轮速，4个转角)
    // Eigen::VectorXd u_min(NU), u_max(NU), delta_umin(NU), delta_umax(NU);
    // // 4WIS 小车可原地转向，转向范围接近 ±90°，轮速约束再放宽
    // u_min << -3.0, -3.0, -3.0, -3.0, -1.5, -1.5, -1.5, -1.5;
    // u_max <<  3.0,  3.0,  3.0,  3.0,  1.5,  1.5,  1.5,  1.5;
    // delta_umin << -0.5, -0.5, -0.5, -0.5, -0.1, -0.1, -0.1, -0.1;
    // delta_umax <<  0.5,  0.5,  0.5,  0.5,  0.1,  0.1,  0.1,  0.1;

    // double yaw_r = cyaw[min_index];
    // double v_r = speed[min_index];
    // double k_r = ck[min_index];
    // double dt = params_.dt;
    // double L = params_.L;
    // double W = params_.W;

    // double xw[4] = { L, -L, -L,  L};
    // double yw[4] = { W,  W, -W, -W};

    // // 1. 计算 4WIS 的完美动力学参考控制量 U_r
    // Eigen::VectorXd U_r(NU);
    // double omega_r = v_r * k_r; 
    // for(int i = 0; i < 4; i++) {
    //     double vx_i = v_r - yw[i] * omega_r;
    //     double vy_i = xw[i] * omega_r;
    //     U_r(i) = std::sqrt(vx_i*vx_i + vy_i*vy_i) * (vx_i >= 0 ? 1.0 : -1.0);
    //     U_r(i + 4) = std::atan2(vy_i, vx_i);
    // }

    // Eigen::Matrix3d Ad = Eigen::Matrix3d::Identity();
    // Eigen::MatrixXd Bd = Eigen::MatrixXd::Zero(NX, NU);

    // double sum_D = 0, sum_E = 0;
    // for (int i = 0; i < 4; ++i) {
    //     double v_ir = U_r(i);
    //     double delta_ir = U_r(i + 4);

    //     double Di = -0.25 * v_ir * sin(delta_ir + yaw_r) * dt;
    //     double Ei =  0.25 * v_ir * cos(delta_ir + yaw_r) * dt;
    //     double Fi =  0.25 * cos(delta_ir + yaw_r) * dt;
    //     double Gi =  0.25 * sin(delta_ir + yaw_r) * dt;
        
    //     double denom = 4.0 * (L*L + W*W);
    //     double Ji = (xw[i] * sin(delta_ir) - yw[i] * cos(delta_ir)) / denom * dt;
    //     double Ki = v_ir * (xw[i] * cos(delta_ir) + yw[i] * sin(delta_ir)) / denom * dt;

    //     sum_D += Di;
    //     sum_E += Ei;

    //     Bd(0, i) = Fi;     Bd(0, i + 4) = Di;
    //     Bd(1, i) = Gi;     Bd(1, i + 4) = Ei;
    //     Bd(2, i) = Ji;     Bd(2, i + 4) = Ki;
    // }

    // Ad(0, 2) = sum_D;
    // Ad(1, 2) = sum_E;

    // Eigen::VectorXd kesi(NX + NU);             
    // Eigen::Vector3d x_r(cx[min_index], cy[min_index], yaw_r);
    // kesi.head(NX) = inital_x - x_r;
    // kesi.tail(NU) = U - U_r;  


    // 控制量物理约束 (4个轮速，4个转角)
    Eigen::VectorXd u_min(NU), u_max(NU), delta_umin(NU), delta_umax(NU);
    u_min << -3.0, -3.0, -3.0, -3.0, -1.5, -1.5, -1.5, -1.5;
    u_max <<  3.0,  3.0,  3.0,  3.0,  1.5,  1.5,  1.5,  1.5;
    delta_umin << -0.5, -0.5, -0.5, -0.5, -0.1, -0.1, -0.1, -0.1;
    delta_umax <<  0.5,  0.5,  0.5,  0.5,  0.1,  0.1,  0.1,  0.1;

    double yaw_r = cyaw[min_index];
    double v_r = speed[min_index];
    // 【修改点 1】：限制曲率的绝对峰值，防止由于样条曲线抖动引发奇点爆算
    double k_r = std::max(-2.5, std::min(2.5, ck[min_index])); 
    
    double dt = params_.dt;
    double L = params_.L;
    double W = params_.W;

    double xw[4] = { L, -L, -L,  L};
    double yw[4] = { W,  W, -W, -W};

    // ==========================================
    // 【修改点 2】: 计算 4WIS 参考控制量 U_r (解决转向角越界崩溃Bug)
    // ==========================================
    Eigen::VectorXd U_r(NU);
    double omega_r = v_r * k_r; 
    for(int i = 0; i < 4; i++) {
        double vx_i = v_r - yw[i] * omega_r;
        double vy_i = xw[i] * omega_r;
        
        double v_mag = std::sqrt(vx_i * vx_i + vy_i * vy_i);
        double steer_angle = std::atan2(vy_i, vx_i);

        // 如果转向角超过 90 度，则翻转车轮速度方向，同时让转向角反向，保证其符合硬件极限
        if (steer_angle > M_PI / 2.0) {
            steer_angle -= M_PI;
            v_mag = -v_mag;
        } else if (steer_angle < -M_PI / 2.0) {
            steer_angle += M_PI;
            v_mag = -v_mag;
        }
        
        U_r(i) = v_mag;
        U_r(i + 4) = steer_angle;
    }

    Eigen::Matrix3d Ad = Eigen::Matrix3d::Identity();
    Eigen::MatrixXd Bd = Eigen::MatrixXd::Zero(NX, NU);

    double sum_D = 0, sum_E = 0;
    for (int i = 0; i < 4; ++i) {
        double v_ir = U_r(i);
        double delta_ir = U_r(i + 4);

        double Di = -0.25 * v_ir * sin(delta_ir + yaw_r) * dt;
        double Ei =  0.25 * v_ir * cos(delta_ir + yaw_r) * dt;
        double Fi =  0.25 * cos(delta_ir + yaw_r) * dt;
        double Gi =  0.25 * sin(delta_ir + yaw_r) * dt;
        
        double denom = 4.0 * (L*L + W*W);
        double Ji = (xw[i] * sin(delta_ir) - yw[i] * cos(delta_ir)) / denom * dt;
        double Ki = v_ir * (xw[i] * cos(delta_ir) + yw[i] * sin(delta_ir)) / denom * dt;

        sum_D += Di;
        sum_E += Ei;

        Bd(0, i) = Fi;     Bd(0, i + 4) = Di;
        Bd(1, i) = Gi;     Bd(1, i + 4) = Ei;
        Bd(2, i) = Ji;     Bd(2, i + 4) = Ki;
    }

    Ad(0, 2) = sum_D;
    Ad(1, 2) = sum_E;

    // ==========================================
    // 【修改点 3】: 强制约束航向角误差，防止 2PI 跳变引起死亡翻滚
    // ==========================================
    Eigen::VectorXd kesi(NX + NU);             
    Eigen::Vector3d x_r(cx[min_index], cy[min_index], yaw_r);
    
    kesi.head(NX) = inital_x - x_r;
    // 将 yaw 误差 (kesi(2)) 规范化到 [-pi, pi] 范围内
    while (kesi(2) > M_PI)  kesi(2) -= 2.0 * M_PI;
    while (kesi(2) < -M_PI) kesi(2) += 2.0 * M_PI;
    
    kesi.tail(NU) = U - U_r;


    Eigen::MatrixXd A_3 = Eigen::MatrixXd::Zero(NX + NU, NX + NU);              
    A_3.topLeftCorner(NX, NX) = Ad;
    A_3.topRightCorner(NX, NU) = Bd;
    A_3.bottomRightCorner(NU, NU) = Eigen::MatrixXd::Identity(NU, NU);       

    Eigen::MatrixXd B_3 = Eigen::MatrixXd::Zero(NX + NU, NU);                       
    B_3.topLeftCorner(NX, NU) = Bd;                                    
    B_3.bottomRightCorner(NU, NU) = Eigen::MatrixXd::Identity(NU, NU);

    int NY = NX + NU; 
    Eigen::MatrixXd C = Eigen::MatrixXd::Identity(NY, NY);          

    // ==========================================
    // 💡 核心优化点 1: 消除 W_mat 计算的冗余矩阵乘法
    // ==========================================
    Eigen::MatrixXd W_mat = Eigen::MatrixXd::Zero(NP * NY, NX + NU);       
    Eigen::MatrixXd A_power = A_3; // 初始为 A_3^1
    for (int i = 0; i < NP; ++i) {
        W_mat.middleRows(i * NY, NY) = C * A_power;
        A_power *= A_3; // 递推下一个次方，每次只做一次乘法
    }

    // ==========================================
    // 💡 核心优化点 2: 消除 Z 矩阵的 O(N^3) 嵌套爆炸
    // ==========================================
    Eigen::MatrixXd Z = Eigen::MatrixXd::Zero(NP * NY, NC * NU);   
    
    // 先预计算基底数组 CAB[k] = C * A_3^k * B_3
    std::vector<Eigen::MatrixXd> CAB(NP);
    Eigen::MatrixXd A_k = Eigen::MatrixXd::Identity(NX + NU, NX + NU);
    for(int k = 0; k < NP; ++k) {
        CAB[k] = C * A_k * B_3;
        A_k *= A_3; // 同样使用递推
    }
    
    // 直接填入 Z 矩阵，彻底告别最内层循环
    for (int i = 0; i < NP; ++i) {
        for (int j = 0; j < NC; ++j) {
            if (j <= i) {
                Z.block(i * NY, j * NU, NY, NU) = CAB[i - j];
            }
        }
    }

    // ==========================================
    // 💡 核心优化点 3: 提取常量为局部静态变量，避免每帧都在堆上分配巨量内存
    // ==========================================
    static bool is_matrix_initialized = false;
    static Eigen::MatrixXd QB_local = Eigen::MatrixXd::Zero(NP * NY, NP * NY);
    static Eigen::MatrixXd RB_local = Eigen::MatrixXd::Zero(NC * NU, NC * NU);
    static Eigen::MatrixXd A_cons = Eigen::MatrixXd::Zero(2 * NC * NU, NC * NU);
    
    if (!is_matrix_initialized) {
        Eigen::MatrixXd Q_step = Eigen::MatrixXd::Zero(NY, NY);
        Q_step.diagonal().head(NX) << 50.0, 50.0, 20.0;     
        // 保持适度的控制量跟踪权重，既避免超速冲出，又保留足够转向修正能力
        Q_step.diagonal().tail(NU) << 50.0, 50.0, 50.0, 50.0, 10.0, 10.0, 10.0, 10.0;

        // 将 X 和 Y 的权重从 50 提高到 200 或 300，逼迫小车必须死死咬住参考线
        // Q_step.diagonal().head(NX) << 200.0, 200.0, 10.0;     
        // // 稍微降低对控制量偏离的惩罚，允许小车更自由地打方向盘来纠偏
        // Q_step.diagonal().tail(NU) << 10.0, 10.0, 10.0, 10.0, 5.0, 5.0, 5.0, 5.0;

        for(int i = 0; i < NP; i++) { QB_local.block(i * NY, i * NY, NY, NY) = Q_step; }

        Eigen::MatrixXd R_step = Eigen::MatrixXd::Identity(NU, NU);
        R_step.diagonal() << 1.0, 1.0, 1.0, 1.0, 5.0, 5.0, 5.0, 5.0;

        // 降低后 4 个值（对应四个轮子的转向角增量惩罚），例如从 5.0 降到 1.0 或 2.0
        // R_step.diagonal() << 1.0, 1.0, 1.0, 1.0, 2.0, 2.0, 2.0, 2.0;

        for(int i = 0; i < NC; i++) { RB_local.block(i * NU, i * NU, NU, NU) = R_step; }

        Eigen::MatrixXd A_e = Eigen::MatrixXd::Zero(NC * NU, NC * NU);            
        for (int i = 0; i < NC; ++i) {
            for (int j = 0; j <= i; ++j) {
                A_e.block(i * NU, j * NU, NU, NU) = Eigen::MatrixXd::Identity(NU, NU);
            }
        }
        A_cons.topRows(NC * NU) = Eigen::MatrixXd::Identity(NC * NU, NC * NU); 
        A_cons.bottomRows(NC * NU) = A_e;   

        is_matrix_initialized = true;
    }

    // 生成二次规划标准型 H 和 g
    Eigen::MatrixXd H = Z.transpose() * QB_local * Z + RB_local;
    Eigen::VectorXd g = kesi.transpose() * W_mat.transpose() * QB_local * Z;

    Eigen::VectorXd Umax_seq = Eigen::VectorXd::Ones(NC * NU);
    Eigen::VectorXd Umin_seq = Eigen::VectorXd::Ones(NC * NU);
    Eigen::VectorXd delta_Umax_seq = Eigen::VectorXd::Ones(NC * NU);
    Eigen::VectorXd delta_Umin_seq = Eigen::VectorXd::Ones(NC * NU);
    Eigen::VectorXd U_rep = Eigen::VectorXd::Zero(NC * NU);
    
    for (int i = 0; i < NC; ++i) {
        Umax_seq.segment(i * NU, NU) = u_max;
        Umin_seq.segment(i * NU, NU) = u_min;
        delta_Umax_seq.segment(i * NU, NU) = delta_umax;
        delta_Umin_seq.segment(i * NU, NU) = delta_umin;
        U_rep.segment(i * NU, NU) = U; 
    }

    Eigen::VectorXd lowerBound(2 * NC * NU);
    Eigen::VectorXd upperBound(2 * NC * NU);
    
    lowerBound.head(NC * NU) = delta_Umin_seq;
    lowerBound.tail(NC * NU) = Umin_seq - U_rep;
    
    upperBound.head(NC * NU) = delta_Umax_seq;
    upperBound.tail(NC * NU) = Umax_seq - U_rep;

    OsqpEigen::Solver solver;
    solver.settings()->setVerbosity(false);
    solver.settings()->setWarmStart(true);
    // OSQP 建议在此处关闭一些耗时的检查选项以提高速度
    // solver.settings()->setCheckTermination(10); // 每10次迭代检查一次终止条件
    
    solver.data()->setNumberOfVariables(H.rows());
    solver.data()->setNumberOfConstraints(A_cons.rows());

    Eigen::SparseMatrix<double> H_sparse = H.sparseView();
    Eigen::SparseMatrix<double> A_cons_sparse = A_cons.sparseView();

    solver.data()->setHessianMatrix(H_sparse);  
    solver.data()->setGradient(g);                   
    solver.data()->setLinearConstraintsMatrix(A_cons_sparse);  
    solver.data()->setLowerBound(lowerBound);                    
    solver.data()->setUpperBound(upperBound);                    

    if (!solver.initSolver()) { throw std::runtime_error("Solver initialization failed"); }

    solver.solveProblem();

    Eigen::VectorXd solution = solver.getSolution();
    Eigen::VectorXd delta_U = solution.head(NU); 
    
    U += delta_U; 
    return U;
}

void KinematicModel_MPC::updatestate(const Eigen::VectorXd& U_real){
    double v[4] = {U_real(0), U_real(1), U_real(2), U_real(3)};
    double d[4] = {U_real(4), U_real(5), U_real(6), U_real(7)};

    double xw[4] = { L, -L, -L,  L};
    double yw[4] = { W,  W, -W, -W};
    double denom = 4.0 * (L * L + W * W);

    double dx = 0.25 * (v[0] * cos(d[0] + yaw) + v[1] * cos(d[1] + yaw) + v[2] * cos(d[2] + yaw) + v[3] * cos(d[3] + yaw));
    double dy = 0.25 * (v[0] * sin(d[0] + yaw) + v[1] * sin(d[1] + yaw) + v[2] * sin(d[2] + yaw) + v[3] * sin(d[3] + yaw));
    
    double dyaw = 0;
    for (int i = 0; i < 4; ++i) {
        dyaw += v[i] * (-yw[i] * cos(d[i]) + xw[i] * sin(d[i])) / denom;
    }

    x += dx * dt;
    y += dy * dt;
    yaw += dyaw * dt;
}

std::tuple<double, double, double, double> KinematicModel_MPC::getstate(){
    return std::make_tuple(x, y, yaw, v);
}