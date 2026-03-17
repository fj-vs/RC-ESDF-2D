#include "global_esdf_planner.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>

JPSPlanner::JPSPlanner() : cfg_(Config{}) {}
JPSPlanner::JPSPlanner(const Config& cfg) : cfg_(cfg) {}
MSPlanner::MSPlanner() : cfg_(Config{}) {}
MSPlanner::MSPlanner(const Config& cfg) : cfg_(cfg) {}
DdrEsdfPipelinePlanner::DdrEsdfPipelinePlanner() : jps_(JPSPlanner::Config{}), ms_(MSPlanner::Config{}) {}
DdrEsdfPipelinePlanner::DdrEsdfPipelinePlanner(const Config& cfg) : jps_(cfg.jps_cfg), ms_(cfg.ms_cfg) {}

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
    const int ix = static_cast<int>(std::floor((pos_world.x() - origin_.x()) / resolution_));
    const int iy = static_cast<int>(std::floor((pos_world.y() - origin_.y()) / resolution_));
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
                min_dist_sq = std::min(min_dist_sq, dx * dx + dy * dy);
            }
            esdf_[idx] = static_cast<float>(std::sqrt(min_dist_sq) * resolution_);
        }
    }
}

bool GlobalEsdfMap::query(const Eigen::Vector2d& pos_world, double& dist, Eigen::Vector2d& grad) const {
    const double gx = (pos_world.x() - origin_.x()) / resolution_;
    const double gy = (pos_world.y() - origin_.y()) / resolution_;

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

bool JPSPlanner::plan(
    const Eigen::Vector2d& start,
    const Eigen::Vector2d& goal,
    const GlobalEsdfMap& map,
    std::vector<Eigen::Vector2d>& path) const {
    const int sx = map.sizeX();
    const int sy = map.sizeY();
    if (sx <= 2 || sy <= 2) return false;

    auto toId = [sx](int x, int y) { return y * sx + x; };
    auto toWorld = [&map](int x, int y) {
        return Eigen::Vector2d(map.origin().x() + (x + 0.5) * map.resolution(), map.origin().y() + (y + 0.5) * map.resolution());
    };
    auto toGrid = [&map](const Eigen::Vector2d& p, int& x, int& y) {
        x = static_cast<int>(std::floor((p.x() - map.origin().x()) / map.resolution()));
        y = static_cast<int>(std::floor((p.y() - map.origin().y()) / map.resolution()));
    };

    int start_x, start_y, goal_x, goal_y;
    toGrid(start, start_x, start_y);
    toGrid(goal, goal_x, goal_y);
    if (start_x < 0 || start_x >= sx || start_y < 0 || start_y >= sy) return false;
    if (goal_x < 0 || goal_x >= sx || goal_y < 0 || goal_y >= sy) return false;

    const auto& esdf = map.esdfData();
    auto traversable = [&](int x, int y) {
        const int id = toId(x, y);
        return id >= 0 && id < static_cast<int>(esdf.size()) && esdf[id] > cfg_.search_safe_distance;
    };
    if (!traversable(start_x, start_y) || !traversable(goal_x, goal_y)) return false;

    struct Node { int id; double f; };
    struct Cmp { bool operator()(const Node& a, const Node& b) const { return a.f > b.f; } };

    std::priority_queue<Node, std::vector<Node>, Cmp> open;
    std::vector<double> g(sx * sy, std::numeric_limits<double>::infinity());
    std::vector<int> parent(sx * sy, -1);
    std::vector<uint8_t> closed(sx * sy, 0);

    const int start_id = toId(start_x, start_y);
    const int goal_id = toId(goal_x, goal_y);

    auto heuristic = [&](int x, int y) {
        const double dx = static_cast<double>(x - goal_x);
        const double dy = static_cast<double>(y - goal_y);
        return std::sqrt(dx * dx + dy * dy);
    };

    g[start_id] = 0.0;
    open.push({start_id, heuristic(start_x, start_y)});

    const std::vector<Eigen::Vector2i> dirs4 = {{1,0},{-1,0},{0,1},{0,-1}};
    const std::vector<Eigen::Vector2i> dirs8 = {{1,0},{-1,0},{0,1},{0,-1},{1,1},{1,-1},{-1,1},{-1,-1}};
    const auto& dirs = cfg_.allow_diagonal ? dirs8 : dirs4;

    while (!open.empty()) {
        const Node cur = open.top();
        open.pop();
        if (closed[cur.id]) continue;
        closed[cur.id] = 1;
        if (cur.id == goal_id) break;

        const int cx = cur.id % sx;
        const int cy = cur.id / sx;
        for (const auto& d : dirs) {
            const int nx = cx + d.x();
            const int ny = cy + d.y();
            if (nx < 0 || nx >= sx || ny < 0 || ny >= sy) continue;
            if (!traversable(nx, ny)) continue;
            const int nid = toId(nx, ny);
            if (closed[nid]) continue;

            const double step = (std::abs(d.x()) + std::abs(d.y()) == 2) ? std::sqrt(2.0) : 1.0;
            const double ng = g[cur.id] + step;
            if (ng < g[nid]) {
                g[nid] = ng;
                parent[nid] = cur.id;
                open.push({nid, ng + heuristic(nx, ny)});
            }
        }
    }

    if (parent[goal_id] == -1) return false;

    std::vector<Eigen::Vector2d> rev;
    for (int id = goal_id; id != -1; id = parent[id]) {
        const int x = id % sx;
        const int y = id / sx;
        rev.push_back(toWorld(x, y));
    }
    path.assign(rev.rbegin(), rev.rend());
    return path.size() >= 2;
}

MSPlanner::KnotSpan MSPlanner::makeClampedUniformKnots(int num_ctrl, int degree) const {
    const int m = num_ctrl + degree + 1;
    const int interior = m - 2 * (degree + 1);
    std::vector<double> knots(m, 0.0);
    for (int i = 0; i <= degree; ++i) knots[i] = 0.0;
    for (int i = 0; i <= degree; ++i) knots[m - 1 - i] = 1.0;
    for (int i = 1; i <= interior; ++i) knots[degree + i] = static_cast<double>(i) / (interior + 1);
    return {std::max(1, num_ctrl - degree), knots};
}

std::vector<double> MSPlanner::basisAt(double u, int num_ctrl, int degree, const std::vector<double>& knots) const {
    std::vector<double> n(num_ctrl, 0.0);
    for (int i = 0; i < num_ctrl; ++i) {
        if ((u >= knots[i] && u < knots[i + 1]) || (u == 1.0 && i == num_ctrl - 1 && u >= knots[i] && u <= knots[i + 1])) {
            n[i] = 1.0;
        }
    }
    for (int k = 1; k <= degree; ++k) {
        std::vector<double> next(num_ctrl, 0.0);
        for (int i = 0; i < num_ctrl; ++i) {
            const double left_den = knots[i + k] - knots[i];
            const double right_den = knots[i + k + 1] - knots[i + 1];
            double left = 0.0, right = 0.0;
            if (left_den > 1e-12) left = (u - knots[i]) / left_den * n[i];
            if (right_den > 1e-12 && i + 1 < num_ctrl) right = (knots[i + k + 1] - u) / right_den * n[i + 1];
            next[i] = left + right;
        }
        n.swap(next);
    }
    return n;
}

Eigen::Vector2d MSPlanner::evaluate(const std::vector<Eigen::Vector2d>& ctrl_pts, const std::vector<double>& basis) const {
    Eigen::Vector2d p(0.0, 0.0);
    for (size_t i = 0; i < ctrl_pts.size(); ++i) p += basis[i] * ctrl_pts[i];
    return p;
}

bool MSPlanner::plan(
    const std::vector<Eigen::Vector2d>& reference_path,
    const Eigen::Vector2d& start,
    const Eigen::Vector2d& goal,
    const GlobalEsdfMap& map,
    std::vector<Eigen::Vector2d>& optimized_path) const {
    const int degree = 3;
    const int n = std::max(cfg_.num_control_points, degree + 2);

    std::vector<Eigen::Vector2d> ctrl_pts(n, start);
    if (reference_path.size() >= 2) {
        for (int i = 0; i < n; ++i) {
            const size_t idx = std::min(reference_path.size() - 1, static_cast<size_t>(i * reference_path.size() / n));
            ctrl_pts[i] = reference_path[idx];
        }
    } else {
        for (int i = 0; i < n; ++i) {
            const double t = static_cast<double>(i) / (n - 1);
            ctrl_pts[i] = (1.0 - t) * start + t * goal;
        }
    }
    ctrl_pts.front() = start;
    ctrl_pts.back() = goal;

    const auto knot = makeClampedUniformKnots(n, degree);
    const int sample_count = std::max(16, knot.span * cfg_.sample_per_segment);
    std::vector<std::vector<double>> sample_basis(sample_count);
    for (int s = 0; s < sample_count; ++s) {
        const double u = static_cast<double>(s) / (sample_count - 1);
        sample_basis[s] = basisAt(u, n, degree, knot.knots);
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
            if (violation > 0.0) {
                const Eigen::Vector2d dcost_dp = -2.0 * cfg_.w_obstacle * violation * grad_dist;
                for (int i = 0; i < n; ++i) grad_ctrl[i] += sample_basis[s][i] * dcost_dp;
            }

            if (!reference_path.empty()) {
                const size_t ridx = std::min(reference_path.size() - 1, static_cast<size_t>(s * reference_path.size() / sample_count));
                const Eigen::Vector2d dref = samples[s] - reference_path[ridx];
                const Eigen::Vector2d dcost_dp_ref = 2.0 * cfg_.w_ref * dref;
                for (int i = 0; i < n; ++i) grad_ctrl[i] += sample_basis[s][i] * dcost_dp_ref;
            }
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

    optimized_path.clear();
    optimized_path.reserve(sample_count);
    for (int s = 0; s < sample_count; ++s) optimized_path.push_back(evaluate(ctrl_pts, sample_basis[s]));
    return optimized_path.size() >= 2;
}

bool DdrEsdfPipelinePlanner::plan(
    const Eigen::Vector2d& start,
    const Eigen::Vector2d& goal,
    const GlobalEsdfMap& map,
    std::vector<Eigen::Vector2d>& front_end_path,
    std::vector<Eigen::Vector2d>& optimized_path) const {
    if (!jps_.plan(start, goal, map, front_end_path)) return false;
    return ms_.plan(front_end_path, start, goal, map, optimized_path);
}
