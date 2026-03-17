#include "global_esdf_planner.h"
#include "nmpc_controller/mpc_controller.h"

#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
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
        double map_width = 40.0, map_height = 40.0, resolution = 0.1, origin_x = -20.0, origin_y = -20.0;
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
        nh_.param("cmd_vel_topic", cmd_vel_topic_, std::string("/cmd_vel"));
        nh_.param("max_vis_dist", max_vis_dist_, 3.0);
        nh_.param("publish_opt_path_when_collision", publish_opt_path_when_collision_, true);

        DdrEsdfPipelinePlanner::Config cfg;
        nh_.param("search_safe_distance", cfg.jps_cfg.search_safe_distance, cfg.jps_cfg.search_safe_distance);
        nh_.param("allow_diagonal", cfg.jps_cfg.allow_diagonal, cfg.jps_cfg.allow_diagonal);
        nh_.param("safe_distance", cfg.ms_cfg.safe_distance, cfg.ms_cfg.safe_distance);
        nh_.param("max_lbfgs_iterations", cfg.ms_cfg.max_lbfgs_iterations, cfg.ms_cfg.max_lbfgs_iterations);
        nh_.param("w_obstacle", cfg.ms_cfg.w_obstacle, cfg.ms_cfg.w_obstacle);
        nh_.param("w_ref", cfg.ms_cfg.w_ref, cfg.ms_cfg.w_ref);
        nh_.param("ms_verbose", cfg.ms_cfg.verbose, cfg.ms_cfg.verbose);

        NmpcController::Config mpc_cfg;
        nh_.param("mpc_lookahead_dist", mpc_cfg.lookahead_dist, mpc_cfg.lookahead_dist);
        nh_.param("mpc_goal_tolerance", mpc_cfg.goal_tolerance, mpc_cfg.goal_tolerance);
        nh_.param("mpc_allow_reverse", mpc_cfg.allow_reverse, mpc_cfg.allow_reverse);
        nh_.param("mpc_max_linear_vel", mpc_cfg.max_linear_vel, mpc_cfg.max_linear_vel);
        nh_.param("mpc_max_angular_vel", mpc_cfg.max_angular_vel, mpc_cfg.max_angular_vel);
        nh_.param("mpc_dt", mpc_cfg.dt, mpc_cfg.dt);
        nh_.param("mpc_horizon", mpc_cfg.horizon, mpc_cfg.horizon);
        nh_.param("mpc_linear_samples", mpc_cfg.linear_samples, mpc_cfg.linear_samples);
        nh_.param("mpc_angular_samples", mpc_cfg.angular_samples, mpc_cfg.angular_samples);
        nh_.param("mpc_w_track", mpc_cfg.w_track, mpc_cfg.w_track);
        nh_.param("mpc_w_heading", mpc_cfg.w_heading, mpc_cfg.w_heading);
        nh_.param("mpc_w_control", mpc_cfg.w_control, mpc_cfg.w_control);
        nh_.param("mpc_w_terminal", mpc_cfg.w_terminal, mpc_cfg.w_terminal);

        planner_ = DdrEsdfPipelinePlanner(cfg);
        nmpc_controller_ = NmpcController(mpc_cfg);
        global_map_.initialize(map_width, map_height, resolution, Eigen::Vector2d(origin_x, origin_y));

        pose_sub_ = nh_.subscribe(robot_pose_topic_, 1, &EsdfPlannerNode::poseCallback, this);
        scan_sub_ = nh_.subscribe(scan_topic_, 1, &EsdfPlannerNode::scanCallback, this);
        goal_sub_ = nh_.subscribe(goal_topic_, 1, &EsdfPlannerNode::goalCallback, this);

        path_raw_pub_ = nh_.advertise<nav_msgs::Path>(planned_path_raw_topic_, 1, true);
        path_opt_pub_ = nh_.advertise<nav_msgs::Path>(planned_path_opt_topic_, 1, true);
        esdf_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>(esdf_map_topic_, 1, true);
        pose_vis_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(robot_pose_vis_topic_, 1, true);
        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>(cmd_vel_topic_, 1);

        nmpc_timer_ = nh_.createTimer(ros::Duration(0.05), &EsdfPlannerNode::nmpcCallback, this);
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

    void nmpcCallback(const ros::TimerEvent&) {
        if (!has_pose_ || latest_opt_path_.size() < 2) return;
        bool goal_reached = false;
        const Eigen::Vector3d pose(robot_pose_.x, robot_pose_.y, robot_pose_.theta);
        const geometry_msgs::Twist cmd = nmpc_controller_.compute(pose, latest_opt_path_, goal_reached);
        cmd_vel_pub_.publish(cmd);
    }

    void publishRobotPose() {
        if (!has_pose_) return;
        auto pose = toPoseStamped(Eigen::Vector2d(robot_pose_.x, robot_pose_.y), robot_pose_.theta, frame_id_);
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
            if (d > 1e5) grid.data[i] = -1;
            else if (d < 0.0) grid.data[i] = 100;
            else grid.data[i] = static_cast<int8_t>(std::round(std::min(d, max_vis_dist_) / std::max(max_vis_dist_, 1e-6) * 100.0));
        }
        esdf_pub_.publish(grid);
    }

    nav_msgs::Path buildPathMsg(const std::vector<Eigen::Vector2d>& points, const ros::Time& stamp) const {
        nav_msgs::Path path;
        path.header.stamp = stamp;
        path.header.frame_id = frame_id_;
        for (size_t i = 0; i < points.size(); ++i) {
            const size_t j = std::min(i + 1, points.size() - 1);
            const double yaw = std::atan2(points[j].y() - points[i].y(), points[j].x() - points[i].x());
            auto p = toPoseStamped(points[i], yaw, frame_id_);
            p.header.stamp = stamp;
            path.poses.push_back(p);
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
            const double c = std::cos(robot_pose_.theta), s = std::sin(robot_pose_.theta);
            global_map_.setObstacle(Eigen::Vector2d(robot_pose_.x + c * lx - s * ly, robot_pose_.y + s * lx + c * ly));
        }

        global_map_.buildEsdf();
        publishEsdfMap();
        tryPlanAndPublish();
    }

    void tryPlanAndPublish() {
        if (!has_pose_ || !has_goal_) return;
        const Eigen::Vector2d start(robot_pose_.x, robot_pose_.y);
        std::vector<Eigen::Vector2d> raw_path, opt_path;
        if (!planner_.plan(start, goal_, global_map_, raw_path, opt_path)) return;
        if (opt_path.size() < 2) return;
        latest_opt_path_ = opt_path;

        double min_dist = 1e6;
        for (const auto& pt : opt_path) {
            double d; Eigen::Vector2d g;
            if (global_map_.query(pt, d, g)) min_dist = std::min(min_dist, d);
        }

        const ros::Time stamp = ros::Time::now();
        path_raw_pub_.publish(buildPathMsg(raw_path, stamp));
        if (min_dist < 0.0 && !publish_opt_path_when_collision_) return;
        path_opt_pub_.publish(buildPathMsg(opt_path, stamp));
        publishRobotPose();
    }

    ros::NodeHandle nh_;
    ros::Subscriber pose_sub_, scan_sub_, goal_sub_;
    ros::Publisher path_raw_pub_, path_opt_pub_, esdf_pub_, pose_vis_pub_, cmd_vel_pub_;
    ros::Timer nmpc_timer_;

    GlobalEsdfMap global_map_;
    DdrEsdfPipelinePlanner planner_;
    NmpcController nmpc_controller_;
    std::vector<Eigen::Vector2d> latest_opt_path_;

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
    std::string cmd_vel_topic_{"/cmd_vel"};

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
