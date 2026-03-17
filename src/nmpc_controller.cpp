#include "nmpc_controller/mpc_controller.h"

#include <algorithm>
#include <cmath>

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
    if ((goal - pos).norm() < 0.2) {
        goal_reached = true;
        return best_cmd;
    }

    const int nv = 7;
    const int nw = 9;
    double best_cost = 1e18;

    for (int iv = 0; iv < nv; ++iv) {
        const double v = cfg_.max_linear_vel * static_cast<double>(iv) / (nv - 1);
        for (int iw = 0; iw < nw; ++iw) {
            const double w = -cfg_.max_angular_vel + 2.0 * cfg_.max_angular_vel * static_cast<double>(iw) / (nw - 1);

            Eigen::Vector3d x = robot_pose;
            double cost = cfg_.w_control * (v * v + 0.5 * w * w);

            for (int k = 0; k < cfg_.horizon; ++k) {
                x.x() += cfg_.dt * v * std::cos(x.z());
                x.y() += cfg_.dt * v * std::sin(x.z());
                x.z() += cfg_.dt * w;

                Eigen::Vector2d xp(x.x(), x.y());
                Eigen::Vector2d target = goal;
                for (const auto& p : path) {
                    if ((p - xp).norm() >= cfg_.lookahead_dist) {
                        target = p;
                        break;
                    }
                }

                const double track_err = (target - xp).norm();
                const double heading_des = std::atan2(target.y() - xp.y(), target.x() - xp.x());
                double heading_err = heading_des - x.z();
                while (heading_err > M_PI) heading_err -= 2.0 * M_PI;
                while (heading_err < -M_PI) heading_err += 2.0 * M_PI;

                cost += cfg_.w_track * track_err * track_err + cfg_.w_heading * heading_err * heading_err;
            }

            if (cost < best_cost) {
                best_cost = cost;
                best_cmd.linear.x = v;
                best_cmd.angular.z = w;
            }
        }
    }

    return best_cmd;
}
