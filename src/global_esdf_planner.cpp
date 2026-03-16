#include "global_esdf_planner.h"

#include <algorithm>
#include <cmath>
#include <limits>

BsplineEsdfPlanner::BsplineEsdfPlanner() : cfg_(Config{}) {}

BsplineEsdfPlanner::BsplineEsdfPlanner(const Config& cfg) : cfg_(cfg) {}


void GlobalEsdfMap::initialize(double width_m, double height_m, double resolution, const Eigen::Vector2d& origin) {
    width_m_ = width_m;
    height_m_ = height_m;
    resolution_ = resolution;
    origin_ = origin;

    size_x_ = static_cast<int>(std::ceil(width_m_ / resolution_));
    size_y_ = static_cast<int>(std::ceil(height_m_ / resolution_));

    occupancy_.assign(size_x_ * size_y_, 0);
    esdf_.assign(size_x_ * size_y_, 1e6f);
    obstacle_cells_.clear();
}

void GlobalEsdfMap::setObstacle(const Eigen::Vector2d& pos_world) {
    double gx, gy;
    posToGrid(pos_world, gx, gy);
    const int ix = static_cast<int>(std::floor(gx));
    const int iy = static_cast<int>(std::floor(gy));

    if (!inRange(ix, iy)) return;
    if (occupancy_[index(ix, iy)] == 1) return;

    occupancy_[index(ix, iy)] = 1;
    obstacle_cells_.push_back(Eigen::Vector2i(ix, iy));
}

void GlobalEsdfMap::clearObstacles() {
    std::fill(occupancy_.begin(), occupancy_.end(), 0);
    std::fill(esdf_.begin(), esdf_.end(), 1e6f);
    obstacle_cells_.clear();
}

void GlobalEsdfMap::buildEsdf() {
    if (obstacle_cells_.empty()) {
        std::fill(esdf_.begin(), esdf_.end(), 1e6f);
        return;
    }

    for (int y = 0; y < size_y_; ++y) {
        for (int x = 0; x < size_x_; ++x) {
            const int idx = index(x, y);
            if (occupancy_[idx] == 1) {
                esdf_[idx] = -0.5f * static_cast<float>(resolution_);
                continue;
            }

            double min_dist_sq = std::numeric_limits<double>::infinity();
            for (const auto& obs : obstacle_cells_) {
                const double dx = static_cast<double>(x - obs.x());
                const double dy = static_cast<double>(y - obs.y());
                const double d2 = dx * dx + dy * dy;
                if (d2 < min_dist_sq) min_dist_sq = d2;
            }
            esdf_[idx] = static_cast<float>(std::sqrt(min_dist_sq) * resolution_);
        }
    }
}

bool GlobalEsdfMap::query(const Eigen::Vector2d& pos_world, double& dist, Eigen::Vector2d& grad) const {
    double gx, gy;
    posToGrid(pos_world, gx, gy);

    const double u = gx - 0.5;
    const double v = gy - 0.5;

    if (u < 0.0 || v < 0.0 || u >= size_x_ - 1 || v >= size_y_ - 1) {
        dist = 0.0;
        grad.setZero();
        return false;
    }

    const int x0 = static_cast<int>(std::floor(u));
    const int y0 = static_cast<int>(std::floor(v));
    const double a = u - x0;
    const double b = v - y0;

    const float v00 = getRaw(x0, y0);
    const float v10 = getRaw(x0 + 1, y0);
    const float v01 = getRaw(x0, y0 + 1);
    const float v11 = getRaw(x0 + 1, y0 + 1);

    dist = (1 - a) * (1 - b) * v00 + a * (1 - b) * v10 + (1 - a) * b * v01 + a * b * v11;

    const double d_a = (1 - b) * (v10 - v00) + b * (v11 - v01);
    const double d_b = (1 - a) * (v01 - v00) + a * (v11 - v10);
    grad = Eigen::Vector2d(d_a / resolution_, d_b / resolution_);

    return true;
}

BsplineEsdfPlanner::KnotSpan BsplineEsdfPlanner::makeClampedUniformKnots(int num_ctrl, int degree) const {
    const int m = num_ctrl + degree + 1;
    const int interior = m - 2 * (degree + 1);

    std::vector<double> knots(m, 0.0);
    for (int i = 0; i <= degree; ++i) knots[i] = 0.0;
    for (int i = 0; i <= degree; ++i) knots[m - 1 - i] = 1.0;

    for (int i = 1; i <= interior; ++i) {
        knots[degree + i] = static_cast<double>(i) / (interior + 1);
    }

    return {std::max(1, num_ctrl - degree), knots};
}

std::vector<double> BsplineEsdfPlanner::basisAt(double u, int num_ctrl, int degree, const std::vector<double>& knots) const {
    std::vector<double> n(num_ctrl, 0.0);

    for (int i = 0; i < num_ctrl; ++i) {
        if ((u >= knots[i] && u < knots[i + 1]) ||
            (u == 1.0 && i == num_ctrl - 1 && u >= knots[i] && u <= knots[i + 1])) {
            n[i] = 1.0;
        }
    }

    for (int k = 1; k <= degree; ++k) {
        std::vector<double> next(num_ctrl, 0.0);
        for (int i = 0; i < num_ctrl; ++i) {
            double left = 0.0;
            double right = 0.0;
            const double left_den = knots[i + k] - knots[i];
            const double right_den = knots[i + k + 1] - knots[i + 1];

            if (left_den > 1e-12) left = (u - knots[i]) / left_den * n[i];
            if (right_den > 1e-12 && i + 1 < num_ctrl) right = (knots[i + k + 1] - u) / right_den * n[i + 1];
            next[i] = left + right;
        }
        n.swap(next);
    }

    return n;
}

Eigen::Vector2d BsplineEsdfPlanner::evaluate(const std::vector<Eigen::Vector2d>& ctrl_pts, const std::vector<double>& basis) const {
    Eigen::Vector2d p(0.0, 0.0);
    for (size_t i = 0; i < ctrl_pts.size(); ++i) {
        p += basis[i] * ctrl_pts[i];
    }
    return p;
}

std::vector<Eigen::Vector2d> BsplineEsdfPlanner::plan(
    const Eigen::Vector2d& start,
    const Eigen::Vector2d& goal,
    const GlobalEsdfMap& map,
    std::vector<Eigen::Vector2d>* sampled_path) const {
    const int degree = 3;
    const int n = std::max(cfg_.num_control_points, degree + 2);

    std::vector<Eigen::Vector2d> ctrl_pts(n);
    for (int i = 0; i < n; ++i) {
        const double t = static_cast<double>(i) / (n - 1);
        ctrl_pts[i] = (1 - t) * start + t * goal;
    }

    const auto knot = makeClampedUniformKnots(n, degree);
    const int sample_count = std::max(16, knot.span * cfg_.sample_per_segment);

    std::vector<double> sample_u(sample_count, 0.0);
    std::vector<std::vector<double>> sample_basis(sample_count);
    for (int s = 0; s < sample_count; ++s) {
        sample_u[s] = static_cast<double>(s) / (sample_count - 1);
        sample_basis[s] = basisAt(sample_u[s], n, degree, knot.knots);
    }

    for (int iter = 0; iter < cfg_.max_iterations; ++iter) {
        std::vector<Eigen::Vector2d> grad_ctrl(n, Eigen::Vector2d::Zero());

        for (int i = 1; i + 1 < n; ++i) {
            const Eigen::Vector2d a = ctrl_pts[i - 1] - 2.0 * ctrl_pts[i] + ctrl_pts[i + 1];
            grad_ctrl[i - 1] += 2.0 * cfg_.w_smooth * a;
            grad_ctrl[i] += -4.0 * cfg_.w_smooth * a;
            grad_ctrl[i + 1] += 2.0 * cfg_.w_smooth * a;
        }

        for (int i = 0; i + 1 < n; ++i) {
            const Eigen::Vector2d d = ctrl_pts[i + 1] - ctrl_pts[i];
            grad_ctrl[i] += -2.0 * cfg_.w_length * d;
            grad_ctrl[i + 1] += 2.0 * cfg_.w_length * d;
        }

        std::vector<Eigen::Vector2d> samples(sample_count, Eigen::Vector2d::Zero());
        for (int s = 0; s < sample_count; ++s) samples[s] = evaluate(ctrl_pts, sample_basis[s]);

        for (int s = 0; s < sample_count; ++s) {
            double dist;
            Eigen::Vector2d grad_dist;
            if (!map.query(samples[s], dist, grad_dist)) continue;

            const double violation = cfg_.safe_distance - dist;
            if (violation <= 0.0) continue;

            const Eigen::Vector2d dcost_dp = -2.0 * cfg_.w_obstacle * violation * grad_dist;
            for (int i = 0; i < n; ++i) grad_ctrl[i] += sample_basis[s][i] * dcost_dp;
        }

        for (int s = 1; s + 1 < sample_count; ++s) {
            const Eigen::Vector2d d1 = samples[s] - samples[s - 1];
            const Eigen::Vector2d d2 = samples[s + 1] - samples[s];
            const double n1 = d1.norm();
            const double n2 = d2.norm();
            if (n1 < 1e-3 || n2 < 1e-3) continue;

            double cos_theta = d1.dot(d2) / (n1 * n2);
            cos_theta = std::max(-1.0, std::min(1.0, cos_theta));
            const double theta = std::acos(cos_theta);
            const double ds = 0.5 * (n1 + n2);
            const double kappa = theta / std::max(ds, 1e-3);
            const double v = kappa - cfg_.max_curvature;
            if (v <= 0.0) continue;

            const Eigen::Vector2d dcost_dp = 2.0 * cfg_.w_kinematic * v * (2.0 * samples[s] - samples[s - 1] - samples[s + 1]);
            for (int i = 0; i < n; ++i) grad_ctrl[i] += sample_basis[s][i] * dcost_dp;
        }

        for (int i = 1; i + 1 < n; ++i) ctrl_pts[i] -= cfg_.step_size * grad_ctrl[i];
        ctrl_pts.front() = start;
        ctrl_pts.back() = goal;
    }

    if (sampled_path) {
        sampled_path->clear();
        sampled_path->reserve(sample_count);
        for (int s = 0; s < sample_count; ++s) sampled_path->push_back(evaluate(ctrl_pts, sample_basis[s]));
    }

    return ctrl_pts;
}
