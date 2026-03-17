#ifndef NMPC_CONTROLLER_MPC_CONTROLLER_H
#define NMPC_CONTROLLER_MPC_CONTROLLER_H

#include <Eigen/Core>
#include <geometry_msgs/Twist.h>
#include <vector>

class NmpcController {
public:
    struct Config {
        double goal_tolerance{0.2};
        bool allow_reverse{false};

        double max_linear_vel{0.8};
        double max_angular_vel{1.0};
        double dt{0.1};
        int horizon{12};

        double w_x{12.0};
        double w_y{12.0};
        double w_yaw{2.0};
        double w_v{0.2};
        double w_w{0.2};
        double w_du{0.05};
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
