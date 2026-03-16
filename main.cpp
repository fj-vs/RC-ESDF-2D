#include "global_esdf_planner.h"

#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
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

        nh_.param("frame_id", frame_id_, std::string("map"));
        nh_.param("robot_pose_topic", robot_pose_topic_, std::string("/robot_pose"));
        nh_.param("scan_topic", scan_topic_, std::string("/scan"));
        nh_.param("goal_topic", goal_topic_, std::string("/move_base_simple/goal"));
        nh_.param("esdf_map_topic", esdf_map_topic_, std::string("/esdf_map"));
        nh_.param("robot_pose_vis_topic", robot_pose_vis_topic_, std::string("/robot_pose_vis"));
        nh_.param("planned_path_raw_topic", planned_path_raw_topic_, std::string("/planned_path_raw"));
        nh_.param("planned_path_opt_topic", planned_path_opt_topic_, std::string("/planned_path_opt"));
        nh_.param("max_vis_dist", max_vis_dist_, 3.0);
        nh_.param("publish_opt_path_when_collision", publish_opt_path_when_collision_, true);

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

        pose_sub_ = nh_.subscribe(robot_pose_topic_, 1, &EsdfPlannerNode::poseCallback, this);
        scan_sub_ = nh_.subscribe(scan_topic_, 1, &EsdfPlannerNode::scanCallback, this);
        goal_sub_ = nh_.subscribe(goal_topic_, 1, &EsdfPlannerNode::goalCallback, this);

        path_raw_pub_ = nh_.advertise<nav_msgs::Path>(planned_path_raw_topic_, 1, true);
        path_opt_pub_ = nh_.advertise<nav_msgs::Path>(planned_path_opt_topic_, 1, true);
        esdf_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>(esdf_map_topic_, 1, true);
        pose_vis_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(robot_pose_vis_topic_, 1, true);
    }

private:
    static geometry_msgs::PoseStamped toPoseStamped(const Eigen::Vector2d& p, double yaw, const std::string& frame_id) {
        geometry_msgs::PoseStamped out;
        out.header.frame_id = frame_id;
        out.pose.position.x = p.x();
        out.pose.position.y = p.y();
        out.pose.position.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, yaw);
        out.pose.orientation.x = q.x();
        out.pose.orientation.y = q.y();
        out.pose.orientation.z = q.z();
        out.pose.orientation.w = q.w();
        return out;
    }

    void publishRobotPose() {
        if (!has_pose_) return;
        geometry_msgs::PoseStamped pose = toPoseStamped(Eigen::Vector2d(robot_pose_.x, robot_pose_.y), robot_pose_.theta, frame_id_);
        pose.header.stamp = ros::Time::now();
        pose_vis_pub_.publish(pose);
    }

    void publishEsdfMap() {
        nav_msgs::OccupancyGrid grid;
        grid.header.stamp = ros::Time::now();
        grid.header.frame_id = frame_id_;

        grid.info.resolution = static_cast<float>(global_map_.resolution());
        grid.info.width = static_cast<uint32_t>(global_map_.sizeX());
        grid.info.height = static_cast<uint32_t>(global_map_.sizeY());
        grid.info.origin.position.x = global_map_.origin().x();
        grid.info.origin.position.y = global_map_.origin().y();
        grid.info.origin.orientation.w = 1.0;

        const auto& esdf = global_map_.esdfData();
        grid.data.resize(esdf.size(), -1);

        for (size_t i = 0; i < esdf.size(); ++i) {
            const double d = static_cast<double>(esdf[i]);
            if (d > 1e5) {
                grid.data[i] = -1;
            } else if (d < 0.0) {
                grid.data[i] = 100;
            } else {
                const double v = std::min(d, max_vis_dist_) / std::max(max_vis_dist_, 1e-6);
                grid.data[i] = static_cast<int8_t>(std::round(v * 100.0));
            }
        }

        esdf_pub_.publish(grid);
    }

    nav_msgs::Path buildPathMsg(const std::vector<Eigen::Vector2d>& points, const ros::Time& stamp) const {
        nav_msgs::Path path;
        path.header.stamp = stamp;
        path.header.frame_id = frame_id_;

        if (points.empty()) return path;

        for (size_t i = 0; i < points.size(); ++i) {
            const size_t j = std::min(i + 1, points.size() - 1);
            const double yaw = std::atan2(points[j].y() - points[i].y(), points[j].x() - points[i].x());
            geometry_msgs::PoseStamped p = toPoseStamped(points[i], yaw, frame_id_);
            p.header.stamp = stamp;
            path.poses.push_back(p);
        }
        return path;
    }

    std::vector<Eigen::Vector2d> buildRawPath(const Eigen::Vector2d& start, const Eigen::Vector2d& goal, size_t n) const {
        const size_t count = std::max<size_t>(n, 2);
        std::vector<Eigen::Vector2d> path(count);
        for (size_t i = 0; i < count; ++i) {
            const double t = static_cast<double>(i) / static_cast<double>(count - 1);
            path[i] = (1.0 - t) * start + t * goal;
        }
        return path;
    }

    void poseCallback(const geometry_msgs::Pose2D::ConstPtr& msg) {
        robot_pose_ = *msg;
        has_pose_ = true;
        publishRobotPose();
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
        publishEsdfMap();
        tryPlanAndPublish();
    }

    void tryPlanAndPublish() {
        if (!has_pose_ || !has_goal_) return;

        const Eigen::Vector2d start(robot_pose_.x, robot_pose_.y);
        std::vector<Eigen::Vector2d> opt_path;
        planner_.plan(start, goal_, global_map_, &opt_path);
        if (opt_path.size() < 2) return;

        const std::vector<Eigen::Vector2d> raw_path = buildRawPath(start, goal_, opt_path.size());

        double min_dist = 1e6;
        for (const auto& pt : opt_path) {
            double d;
            Eigen::Vector2d g;
            if (global_map_.query(pt, d, g)) min_dist = std::min(min_dist, d);
        }

        const ros::Time stamp = ros::Time::now();
        const nav_msgs::Path raw_msg = buildPathMsg(raw_path, stamp);
        const nav_msgs::Path opt_msg = buildPathMsg(opt_path, stamp);

        path_raw_pub_.publish(raw_msg);
        if (min_dist < 0.0) {
            ROS_WARN_THROTTLE(1.0, "Optimized path intersects occupied cells. min_dist=%.3f", min_dist);
            if (!publish_opt_path_when_collision_) {
                publishRobotPose();
                return;
            }
        }
        path_opt_pub_.publish(opt_msg);
        publishRobotPose();
    }

    ros::NodeHandle nh_;
    ros::Subscriber pose_sub_;
    ros::Subscriber scan_sub_;
    ros::Subscriber goal_sub_;
    ros::Publisher path_raw_pub_;
    ros::Publisher path_opt_pub_;
    ros::Publisher esdf_pub_;
    ros::Publisher pose_vis_pub_;

    GlobalEsdfMap global_map_;
    BsplineEsdfPlanner planner_;

    geometry_msgs::Pose2D robot_pose_;
    Eigen::Vector2d goal_{0.0, 0.0};

    std::string frame_id_{"map"};
    std::string robot_pose_topic_{"/robot_pose"};
    std::string scan_topic_{"/scan"};
    std::string goal_topic_{"/move_base_simple/goal"};
    std::string esdf_map_topic_{"/esdf_map"};
    std::string robot_pose_vis_topic_{"/robot_pose_vis"};
    std::string planned_path_raw_topic_{"/planned_path_raw"};
    std::string planned_path_opt_topic_{"/planned_path_opt"};

    double max_vis_dist_{3.0};
    bool publish_opt_path_when_collision_{true};

    bool has_pose_{false};
    bool has_goal_{false};
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "esdf_bspline_planner_node");
    EsdfPlannerNode node;
    ros::spin();
    return 0;
}
