#include "nmpc_controller/mpc_controller.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace {
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

Eigen::Vector2d getLookaheadTarget(const Eigen::Vector2d& pos,
                                   const std::vector<Eigen::Vector2d>& path,
                                   double lookahead_dist) {
    const int idx = nearestPathIndex(pos, path);
    double remain = lookahead_dist;
    Eigen::Vector2d cur = path[idx];

    for (int i = idx + 1; i < static_cast<int>(path.size()); ++i) {
        const Eigen::Vector2d seg = path[i] - cur;
        const double len = seg.norm();
        if (len < 1e-6) continue;
        if (len >= remain) return cur + seg * (remain / len);
        remain -= len;
        cur = path[i];
    }
    return path.back();
}
} // namespace

NmpcController::NmpcController() : cfg_(Config{}) {}
NmpcController::NmpcController(const Config& cfg) : cfg_(cfg) {}

geometry_msgs::Twist NmpcController::compute(
    const Eigen::Vector3d& robot_pose,
    const std::vector<Eigen::Vector2d>& path,
    bool& goal_reached) const {
    geometry_msgs::Twist best_cmd;
    goal_reached = false;
    if (path.empty()) return best_cmd;

    const Eigen::Vector2d pos(robot_pose.x(), robot_pose.y());
    const Eigen::Vector2d goal = path.back();
    if ((goal - pos).norm() < cfg_.goal_tolerance) {
        goal_reached = true;
        return best_cmd;
    }

    const int nv = std::max(2, cfg_.linear_samples);
    const int nw = std::max(3, cfg_.angular_samples);
    const double v_min = cfg_.allow_reverse ? -cfg_.max_linear_vel : 0.0;
    const double v_span = cfg_.max_linear_vel - v_min;

    double best_cost = std::numeric_limits<double>::infinity();

    for (int iv = 0; iv < nv; ++iv) {
        const double v = v_min + v_span * static_cast<double>(iv) / (nv - 1);
        for (int iw = 0; iw < nw; ++iw) {
            const double w = -cfg_.max_angular_vel +
                             2.0 * cfg_.max_angular_vel * static_cast<double>(iw) / (nw - 1);

            Eigen::Vector3d x = robot_pose;
            double cost = cfg_.w_control * (v * v + 0.5 * w * w);

            for (int k = 0; k < cfg_.horizon; ++k) {
                x.x() += cfg_.dt * v * std::cos(x.z());
                x.y() += cfg_.dt * v * std::sin(x.z());
                x.z() = normalizeAngle(x.z() + cfg_.dt * w);

                const Eigen::Vector2d xp(x.x(), x.y());
                const Eigen::Vector2d target = getLookaheadTarget(xp, path, cfg_.lookahead_dist);

                const double track_err = (target - xp).norm();
                const double heading_des = std::atan2(target.y() - xp.y(), target.x() - xp.x());
                const double heading_err = normalizeAngle(heading_des - x.z());

                cost += cfg_.w_track * track_err * track_err +
                        cfg_.w_heading * heading_err * heading_err;
            }

            const Eigen::Vector2d xT(x.x(), x.y());
            cost += cfg_.w_terminal * (goal - xT).squaredNorm();

            if (cost < best_cost) {
                best_cost = cost;
                best_cmd.linear.x = v;
                best_cmd.angular.z = w;
            }
        }
    }

    return best_cmd;
}
