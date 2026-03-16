#include "global_esdf_planner.h"

#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <tf2/LinearMath/Quaternion.h>

#include <algorithm>
#include <cmath>
#include <vector>

class EsdfPlannerNode {
public:
    EsdfPlannerNode() : nh_("~") {
        double map_width = 40.0;
        double map_height = 40.0;
        double resolution = 0.1;
        double origin_x = -20.0;
        double origin_y = -20.0;

        nh_.param("map_width", map_width, map_width);
        nh_.param("map_height", map_height, map_height);
        nh_.param("resolution", resolution, resolution);
        nh_.param("origin_x", origin_x, origin_x);
        nh_.param("origin_y", origin_y, origin_y);

        BsplineEsdfPlanner::Config cfg;
        nh_.param("safe_distance", cfg.safe_distance, cfg.safe_distance);
        nh_.param("max_curvature", cfg.max_curvature, cfg.max_curvature);
        nh_.param("num_control_points", cfg.num_control_points, cfg.num_control_points);
        nh_.param("sample_per_segment", cfg.sample_per_segment, cfg.sample_per_segment);
        nh_.param("max_iterations", cfg.max_iterations, cfg.max_iterations);
        nh_.param("step_size", cfg.step_size, cfg.step_size);
        nh_.param("w_smooth", cfg.w_smooth, cfg.w_smooth);
        nh_.param("w_obstacle", cfg.w_obstacle, cfg.w_obstacle);
        nh_.param("w_length", cfg.w_length, cfg.w_length);
        nh_.param("w_kinematic", cfg.w_kinematic, cfg.w_kinematic);

        planner_ = BsplineEsdfPlanner(cfg);
        global_map_.initialize(map_width, map_height, resolution, Eigen::Vector2d(origin_x, origin_y));

        pose_sub_ = nh_.subscribe("/robot_pose", 1, &EsdfPlannerNode::poseCallback, this);
        scan_sub_ = nh_.subscribe("/scan", 1, &EsdfPlannerNode::scanCallback, this);
        goal_sub_ = nh_.subscribe("/move_base_simple/goal", 1, &EsdfPlannerNode::goalCallback, this);
        path_pub_ = nh_.advertise<nav_msgs::Path>("/planned_path", 1, true);
    }

private:
    void poseCallback(const geometry_msgs::Pose2D::ConstPtr& msg) {
        robot_pose_ = *msg;
        has_pose_ = true;
    }

    void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        goal_ = Eigen::Vector2d(msg->pose.position.x, msg->pose.position.y);
        has_goal_ = true;
        tryPlanAndPublish();
    }

    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {
        if (!has_pose_) return;

        global_map_.clearObstacles();

        for (size_t i = 0; i < scan->ranges.size(); ++i) {
            const float r = scan->ranges[i];
            if (!std::isfinite(r) || r < scan->range_min || r > scan->range_max) continue;

            const double angle = scan->angle_min + static_cast<double>(i) * scan->angle_increment;
            const double lx = static_cast<double>(r) * std::cos(angle);
            const double ly = static_cast<double>(r) * std::sin(angle);

            const double c = std::cos(robot_pose_.theta);
            const double s = std::sin(robot_pose_.theta);
            const double wx = robot_pose_.x + c * lx - s * ly;
            const double wy = robot_pose_.y + s * lx + c * ly;

            global_map_.setObstacle(Eigen::Vector2d(wx, wy));
        }

        global_map_.buildEsdf();
        tryPlanAndPublish();
    }

    void tryPlanAndPublish() {
        if (!has_pose_ || !has_goal_) return;

        const Eigen::Vector2d start(robot_pose_.x, robot_pose_.y);
        std::vector<Eigen::Vector2d> sampled_path;
        planner_.plan(start, goal_, global_map_, &sampled_path);
        if (sampled_path.size() < 2) return;

        nav_msgs::Path path;
        path.header.stamp = ros::Time::now();
        path.header.frame_id = "map";

        for (size_t i = 0; i < sampled_path.size(); ++i) {
            geometry_msgs::PoseStamped p;
            p.header = path.header;
            p.pose.position.x = sampled_path[i].x();
            p.pose.position.y = sampled_path[i].y();
            p.pose.position.z = 0.0;

            const size_t j = std::min(i + 1, sampled_path.size() - 1);
            const double yaw = std::atan2(sampled_path[j].y() - sampled_path[i].y(), sampled_path[j].x() - sampled_path[i].x());
            tf2::Quaternion q;
            q.setRPY(0.0, 0.0, yaw);
            p.pose.orientation.x = q.x();
            p.pose.orientation.y = q.y();
            p.pose.orientation.z = q.z();
            p.pose.orientation.w = q.w();

            path.poses.push_back(p);
        }

        // 简单碰撞校验（发布前）
        double min_dist = 1e6;
        for (const auto& pt : sampled_path) {
            double d;
            Eigen::Vector2d g;
            if (global_map_.query(pt, d, g)) min_dist = std::min(min_dist, d);
        }

        if (min_dist < 0.0) {
            ROS_WARN_THROTTLE(1.0, "Planned path intersects occupied cells. min_dist=%.3f", min_dist);
        } else {
            path_pub_.publish(path);
        }
    }

    ros::NodeHandle nh_;
    ros::Subscriber pose_sub_;
    ros::Subscriber scan_sub_;
    ros::Subscriber goal_sub_;
    ros::Publisher path_pub_;

    GlobalEsdfMap global_map_;
    BsplineEsdfPlanner planner_;

    geometry_msgs::Pose2D robot_pose_;
    Eigen::Vector2d goal_{0.0, 0.0};

    bool has_pose_{false};
    bool has_goal_{false};
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "esdf_bspline_planner_node");
    EsdfPlannerNode node;
    ros::spin();
    return 0;
}
