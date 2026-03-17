#include "nmpc_controller/mpc_controller.h"

#include <OsqpEigen/OsqpEigen.h>

#include <Eigen/Sparse>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <vector>

namespace {
template <typename T>
inline T clampVal(const T& v, const T& lo, const T& hi) {
    return std::max(lo, std::min(v, hi));
}

inline double normalizeAngle(double a) {
    while (a > M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
}

int nearestPathIndex(const Eigen::Vector2d& pos, const std::vector<Eigen::Vector2d>& path) {
    int best_idx = 0;
    double best_dist = std::numeric_limits<double>::infinity();
    for (int i = 0; i < static_cast<int>(path.size()); ++i) {
        const double d = (path[i] - pos).squaredNorm();
        if (d < best_dist) {
            best_dist = d;
            best_idx = i;
        }
    }
    return best_idx;
}

Eigen::Vector2d clippedGoalCmd(const Eigen::Vector3d& pose,
                               const Eigen::Vector2d& goal,
                               double v_max,
                               double w_max) {
    const double dx = goal.x() - pose.x();
    const double dy = goal.y() - pose.y();
    const double yaw_des = std::atan2(dy, dx);
    const double yaw_err = normalizeAngle(yaw_des - pose.z());

    const double v = clampVal(0.8 * std::sqrt(dx * dx + dy * dy), 0.0, v_max);
    const double w = clampVal(1.2 * yaw_err, -w_max, w_max);
    return Eigen::Vector2d(v, w);
}

Eigen::SparseMatrix<double> denseToSparse(const Eigen::MatrixXd& M) {
    Eigen::SparseMatrix<double> S(M.rows(), M.cols());
    std::vector<Eigen::Triplet<double>> triplets;
    triplets.reserve(M.rows() * M.cols() / 3);
    for (int i = 0; i < M.rows(); ++i) {
        for (int j = 0; j < M.cols(); ++j) {
            const double v = M(i, j);
            if (std::abs(v) > 1e-12) triplets.emplace_back(i, j, v);
        }
    }
    S.setFromTriplets(triplets.begin(), triplets.end());
    return S;
}
} // namespace

NmpcController::NmpcController() : cfg_(Config{}) {}
NmpcController::NmpcController(const Config& cfg) : cfg_(cfg) {}

geometry_msgs::Twist NmpcController::compute(const Eigen::Vector3d& robot_pose,
                                             const std::vector<Eigen::Vector2d>& path,
                                             bool& goal_reached) const {
    geometry_msgs::Twist cmd;
    goal_reached = false;
    if (path.size() < 2) return cmd;

    const Eigen::Vector2d goal = path.back();
    const Eigen::Vector2d pos(robot_pose.x(), robot_pose.y());
    if ((goal - pos).norm() < cfg_.goal_tolerance) {
        goal_reached = true;
        return cmd;
    }

    const int N = std::max(2, cfg_.horizon);
    const int nx = 3;
    const int nu = 2;

    std::vector<Eigen::Vector3d> x_ref(N + 1, Eigen::Vector3d::Zero());
    std::vector<Eigen::Vector2d> u_ref(N, Eigen::Vector2d::Zero());

    int idx = nearestPathIndex(pos, path);
    for (int k = 0; k <= N; ++k) {
        const int i0 = std::min<int>(path.size() - 2, idx + k);
        const int i1 = std::min<int>(path.size() - 1, i0 + 1);
        const Eigen::Vector2d p0 = path[i0];
        const Eigen::Vector2d p1 = path[i1];
        const double yaw = std::atan2(p1.y() - p0.y(), p1.x() - p0.x());

        x_ref[k] << p0.x(), p0.y(), yaw;

        if (k < N) {
            const double ds = (p1 - p0).norm();
            double v = ds / std::max(1e-3, cfg_.dt);
            if (cfg_.allow_reverse) {
                v = clampVal(v, -cfg_.max_linear_vel, cfg_.max_linear_vel);
            } else {
                v = clampVal(v, 0.0, cfg_.max_linear_vel);
            }
            const int i2 = std::min<int>(path.size() - 1, i1 + 1);
            const double yaw_next = std::atan2(path[i2].y() - p1.y(), path[i2].x() - p1.x());
            const double w = clampVal(normalizeAngle(yaw_next - yaw) / std::max(1e-3, cfg_.dt),
                                        -cfg_.max_angular_vel,
                                        cfg_.max_angular_vel);
            u_ref[k] << v, w;
        }
    }

    Eigen::Matrix<double, 3, 1> e0;
    e0 << robot_pose.x() - x_ref[0].x(),
          robot_pose.y() - x_ref[0].y(),
          normalizeAngle(robot_pose.z() - x_ref[0].z());

    std::vector<Eigen::Matrix3d> A_seq(N, Eigen::Matrix3d::Identity());
    std::vector<Eigen::Matrix<double, 3, 2>> B_seq(N, Eigen::Matrix<double, 3, 2>::Zero());
    for (int k = 0; k < N; ++k) {
        const double yaw = x_ref[k].z();
        const double v = u_ref[k].x();

        Eigen::Matrix3d A = Eigen::Matrix3d::Identity();
        A(0, 2) = -cfg_.dt * v * std::sin(yaw);
        A(1, 2) = cfg_.dt * v * std::cos(yaw);

        Eigen::Matrix<double, 3, 2> B = Eigen::Matrix<double, 3, 2>::Zero();
        B(0, 0) = cfg_.dt * std::cos(yaw);
        B(1, 0) = cfg_.dt * std::sin(yaw);
        B(2, 1) = cfg_.dt;

        A_seq[k] = A;
        B_seq[k] = B;
    }

    Eigen::MatrixXd Sx = Eigen::MatrixXd::Zero(nx * N, nx);
    Eigen::MatrixXd Su = Eigen::MatrixXd::Zero(nx * N, nu * N);
    for (int i = 0; i < N; ++i) {
        Eigen::Matrix3d Phi = Eigen::Matrix3d::Identity();
        for (int t = 0; t <= i; ++t) Phi = A_seq[t] * Phi;
        Sx.block(i * nx, 0, nx, nx) = Phi;

        for (int j = 0; j <= i; ++j) {
            Eigen::Matrix3d P = Eigen::Matrix3d::Identity();
            for (int t = j + 1; t <= i; ++t) P = A_seq[t] * P;
            Su.block(i * nx, j * nu, nx, nu) = P * B_seq[j];
        }
    }

    Eigen::MatrixXd Qbar = Eigen::MatrixXd::Zero(nx * N, nx * N);
    for (int i = 0; i < N; ++i) {
        Qbar(i * nx + 0, i * nx + 0) = cfg_.w_x;
        Qbar(i * nx + 1, i * nx + 1) = cfg_.w_y;
        Qbar(i * nx + 2, i * nx + 2) = cfg_.w_yaw;
    }

    Eigen::MatrixXd Rbar = Eigen::MatrixXd::Zero(nu * N, nu * N);
    for (int i = 0; i < N; ++i) {
        Rbar(i * nu + 0, i * nu + 0) = cfg_.w_v;
        Rbar(i * nu + 1, i * nu + 1) = cfg_.w_w;
    }

    Eigen::MatrixXd D = Eigen::MatrixXd::Zero(nu * (N - 1), nu * N);
    for (int i = 0; i < N - 1; ++i) {
        D.block(i * nu, i * nu, nu, nu) = -Eigen::Matrix2d::Identity();
        D.block(i * nu, (i + 1) * nu, nu, nu) = Eigen::Matrix2d::Identity();
    }

    const Eigen::MatrixXd H_dense = 2.0 * (Su.transpose() * Qbar * Su + Rbar + cfg_.w_du * D.transpose() * D);
    const Eigen::VectorXd f = 2.0 * Su.transpose() * Qbar * Sx * e0;

    const double v_min = cfg_.allow_reverse ? -cfg_.max_linear_vel : 0.0;
    Eigen::VectorXd lb = Eigen::VectorXd::Constant(nu * N, 0.0);
    Eigen::VectorXd ub = Eigen::VectorXd::Constant(nu * N, 0.0);
    for (int i = 0; i < N; ++i) {
        lb(i * nu + 0) = v_min - u_ref[i].x();
        ub(i * nu + 0) = cfg_.max_linear_vel - u_ref[i].x();
        lb(i * nu + 1) = -cfg_.max_angular_vel - u_ref[i].y();
        ub(i * nu + 1) = cfg_.max_angular_vel - u_ref[i].y();
    }

    OsqpEigen::Solver solver;
    solver.settings()->setWarmStart(true);
    solver.settings()->setVerbosity(false);

    const Eigen::SparseMatrix<double> H = denseToSparse(H_dense);
    Eigen::SparseMatrix<double> A(nu * N, nu * N);
    A.setIdentity();

    solver.data()->setNumberOfVariables(nu * N);
    solver.data()->setNumberOfConstraints(nu * N);
    if (!solver.data()->setHessianMatrix(H) ||
        !solver.data()->setGradient(f) ||
        !solver.data()->setLinearConstraintsMatrix(A) ||
        !solver.data()->setLowerBound(lb) ||
        !solver.data()->setUpperBound(ub) ||
        !solver.initSolver()) {
        const Eigen::Vector2d fb = clippedGoalCmd(robot_pose, goal, cfg_.max_linear_vel, cfg_.max_angular_vel);
        cmd.linear.x = fb.x();
        cmd.angular.z = fb.y();
        std::cerr << "[NmpcController] OSQP init failed, use fallback cmd." << std::endl;
        return cmd;
    }

    if (solver.solveProblem() != OsqpEigen::ErrorExitFlag::NoError) {
        const Eigen::Vector2d fb = clippedGoalCmd(robot_pose, goal, cfg_.max_linear_vel, cfg_.max_angular_vel);
        cmd.linear.x = fb.x();
        cmd.angular.z = fb.y();
        std::cerr << "[NmpcController] OSQP solve failed, use fallback cmd." << std::endl;
        return cmd;
    }

    const Eigen::VectorXd du = solver.getSolution();
    const Eigen::Vector2d u0 = u_ref[0] + du.segment<2>(0);
    cmd.linear.x = clampVal(u0.x(), v_min, cfg_.max_linear_vel);
    cmd.angular.z = clampVal(u0.y(), -cfg_.max_angular_vel, cfg_.max_angular_vel);
    return cmd;
}
