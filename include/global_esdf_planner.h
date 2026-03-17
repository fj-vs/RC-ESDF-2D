#ifndef GLOBAL_ESDF_PLANNER_H
#define GLOBAL_ESDF_PLANNER_H

#include <Eigen/Core>
#include <vector>

class GlobalEsdfMap {
public:
    void initialize(double width_m, double height_m, double resolution, const Eigen::Vector2d& origin);
    void setObstacle(const Eigen::Vector2d& pos_world);
    void clearObstacles();
    void buildEsdf();
    bool query(const Eigen::Vector2d& pos_world, double& dist, Eigen::Vector2d& grad) const;

    int sizeX() const { return size_x_; }
    int sizeY() const { return size_y_; }
    double resolution() const { return resolution_; }
    const Eigen::Vector2d& origin() const { return origin_; }
    const std::vector<float>& esdfData() const { return esdf_; }

private:
    inline int index(int x, int y) const { return y * size_x_ + x; }
    inline bool inRange(int x, int y) const { return x >= 0 && x < size_x_ && y >= 0 && y < size_y_; }
    inline float getRaw(int x, int y) const {
        if (!inRange(x, y)) return 0.0f;
        return esdf_[index(x, y)];
    }

    double resolution_{0.1};
    double width_m_{0.0};
    double height_m_{0.0};
    Eigen::Vector2d origin_{0.0, 0.0};
    int size_x_{0};
    int size_y_{0};

    std::vector<uint8_t> occupancy_;
    std::vector<float> esdf_;
    std::vector<Eigen::Vector2i> obstacle_cells_;
};

class JPSPlanner {
public:
    struct Config {
        double search_safe_distance{0.25};
        bool allow_diagonal{true};
    };

    JPSPlanner();
    explicit JPSPlanner(const Config& cfg);

    bool plan(
        const Eigen::Vector2d& start,
        const Eigen::Vector2d& goal,
        const GlobalEsdfMap& map,
        std::vector<Eigen::Vector2d>& path) const;

private:
    Config cfg_;
};

class MSPlanner {
public:
    struct Config {
        int num_control_points{10};
        int sample_per_segment{16};
        int max_lbfgs_iterations{80};
        double initial_step{0.1};
        double safe_distance{0.35};
        double max_curvature{0.5};
        double w_smooth{1.5};
        double w_obstacle{12.0};
        double w_length{0.12};
        double w_kinematic{2.5};
        double w_ref{0.6};
        bool verbose{false};
    };

    MSPlanner();
    explicit MSPlanner(const Config& cfg);

    bool plan(
        const std::vector<Eigen::Vector2d>& reference_path,
        const Eigen::Vector2d& start,
        const Eigen::Vector2d& goal,
        const GlobalEsdfMap& map,
        std::vector<Eigen::Vector2d>& optimized_path) const;

private:
    Config cfg_;
};

class DdrEsdfPipelinePlanner {
public:
    struct Config {
        JPSPlanner::Config jps_cfg{};
        MSPlanner::Config ms_cfg{};
    };

    DdrEsdfPipelinePlanner();
    explicit DdrEsdfPipelinePlanner(const Config& cfg);

    bool plan(
        const Eigen::Vector2d& start,
        const Eigen::Vector2d& goal,
        const GlobalEsdfMap& map,
        std::vector<Eigen::Vector2d>& front_end_path,
        std::vector<Eigen::Vector2d>& optimized_path) const;

private:
    JPSPlanner jps_;
    MSPlanner ms_;
};

#endif
