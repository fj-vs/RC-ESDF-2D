#ifndef NMPC_CONTROLLER_MPC_CONTROLLER_H
#define NMPC_CONTROLLER_MPC_CONTROLLER_H

#include <Eigen/Core>
#include <geometry_msgs/Twist.h>
#include <vector>

class NmpcController {
public:
    struct Config {
        double lookahead_dist{0.8};
        double goal_tolerance{0.2};
        bool allow_reverse{false};
        double max_linear_vel{0.8};
        double max_angular_vel{1.0};
        double dt{0.1};
        int horizon{12};
        int linear_samples{7};
        int angular_samples{9};
        double w_track{2.0};
        double w_heading{0.8};
        double w_control{0.1};
        double w_terminal{3.0};
    };

    NmpcController();
    explicit NmpcController(const Config& cfg);

    geometry_msgs::Twist compute(
        const Eigen::Vector3d& robot_pose,
        const std::vector<Eigen::Vector2d>& path,
        bool& goal_reached) const;

private:
    Config cfg_;
};

#endif
