#include "global_esdf_planner.h"

#include "gcopter/lbfgs.hpp"
#include "gcopter/minco.hpp"

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

bool JPSPlanner::plan(const Eigen::Vector2d& start, const Eigen::Vector2d& goal, const GlobalEsdfMap& map,
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

    int sx0, sy0, gx0, gy0;
    toGrid(start, sx0, sy0);
    toGrid(goal, gx0, gy0);
    if (sx0 < 0 || sx0 >= sx || sy0 < 0 || sy0 >= sy) return false;
    if (gx0 < 0 || gx0 >= sx || gy0 < 0 || gy0 >= sy) return false;

    const auto& esdf = map.esdfData();
    auto freeCell = [&](int x, int y) {
        const int id = toId(x, y);
        return id >= 0 && id < static_cast<int>(esdf.size()) && esdf[id] > cfg_.search_safe_distance;
    };
    if (!freeCell(sx0, sy0) || !freeCell(gx0, gy0)) return false;

    struct Node { int id; double f; };
    struct Cmp { bool operator()(const Node& a, const Node& b) const { return a.f > b.f; } };
    std::priority_queue<Node, std::vector<Node>, Cmp> open;
    std::vector<double> g(sx * sy, std::numeric_limits<double>::infinity());
    std::vector<int> parent(sx * sy, -1);
    std::vector<uint8_t> closed(sx * sy, 0);

    const int sid = toId(sx0, sy0);
    const int gid = toId(gx0, gy0);
    auto h = [&](int x, int y) {
        const double dx = static_cast<double>(x - gx0);
        const double dy = static_cast<double>(y - gy0);
        return std::sqrt(dx * dx + dy * dy);
    };

    g[sid] = 0.0;
    open.push({sid, h(sx0, sy0)});

    const std::vector<Eigen::Vector2i> dirs4 = {{1,0},{-1,0},{0,1},{0,-1}};
    const std::vector<Eigen::Vector2i> dirs8 = {{1,0},{-1,0},{0,1},{0,-1},{1,1},{1,-1},{-1,1},{-1,-1}};
    const auto& dirs = cfg_.allow_diagonal ? dirs8 : dirs4;

    while (!open.empty()) {
        Node cur = open.top();
        open.pop();
        if (closed[cur.id]) continue;
        closed[cur.id] = 1;
        if (cur.id == gid) break;

        const int cx = cur.id % sx;
        const int cy = cur.id / sx;
        for (const auto& d : dirs) {
            const int nx = cx + d.x();
            const int ny = cy + d.y();
            if (nx < 0 || nx >= sx || ny < 0 || ny >= sy) continue;
            if (!freeCell(nx, ny)) continue;
            const int nid = toId(nx, ny);
            if (closed[nid]) continue;
            const double step = (std::abs(d.x()) + std::abs(d.y()) == 2) ? std::sqrt(2.0) : 1.0;
            const double ng = g[cur.id] + step;
            if (ng < g[nid]) {
                g[nid] = ng;
                parent[nid] = cur.id;
                open.push({nid, ng + h(nx, ny)});
            }
        }
    }

    if (parent[gid] == -1) return false;
    std::vector<Eigen::Vector2d> rev;
    for (int id = gid; id != -1; id = parent[id]) {
        const int x = id % sx;
        const int y = id / sx;
        rev.push_back(toWorld(x, y));
    }
    path.assign(rev.rbegin(), rev.rend());
    return path.size() >= 2;
}

namespace {
struct MsLbfgsCtx {
    const GlobalEsdfMap* map{nullptr};
    const std::vector<Eigen::Vector2d>* ref{nullptr};
    Eigen::Vector2d start{0, 0};
    Eigen::Vector2d goal{0, 0};
    MSPlanner::Config cfg;
    int piece_num{0};
};

static double evalCost(void* ptr, const Eigen::VectorXd& x, Eigen::VectorXd& g) {
    MsLbfgsCtx* ctx = reinterpret_cast<MsLbfgsCtx*>(ptr);
    const int inner_num = ctx->piece_num - 1;

    auto unpack = [&](const Eigen::VectorXd& v, Eigen::MatrixXd& inPs, Eigen::VectorXd& ts) {
        inPs.resize(2, std::max(0, inner_num));
        ts.resize(ctx->piece_num);
        for (int i = 0; i < inner_num; ++i) {
            inPs(0, i) = v(2 * i);
            inPs(1, i) = v(2 * i + 1);
        }
        for (int i = 0; i < ctx->piece_num; ++i) {
            ts(i) = std::max(0.03, std::exp(v(2 * inner_num + i)));
        }
    };

    auto compute_only = [&](const Eigen::VectorXd& v) {
        Eigen::MatrixXd inPs;
        Eigen::VectorXd ts;
        unpack(v, inPs, ts);

        Eigen::Matrix<double, 2, 3> head, tail;
        head.setZero(); tail.setZero();
        head.col(0) = ctx->start;
        tail.col(0) = ctx->goal;

        minco::MINCO_S3NU minco;
        minco.setConditions(head, tail, ctx->piece_num, Eigen::Vector2d(1.0, 1.0));
        minco.setParameters(inPs, ts);

        double energy = 0.0;
        minco.getEnergy(energy);
        double cost = 0.5 * energy;

        Trajectory<5, 2> traj;
        minco.getTrajectory(traj);

        const int sample_each = std::max(8, ctx->cfg.sample_per_segment);
        int ref_id = 0;
        for (int k = 0; k < traj.getPieceNum(); ++k) {
            const auto& piece = traj[k];
            for (int i = 0; i <= sample_each; ++i) {
                const double t = piece.getDuration() * static_cast<double>(i) / sample_each;
                const Eigen::Vector2d p = piece.getPos(t);

                double d; Eigen::Vector2d gd;
                if (ctx->map->query(p, d, gd)) {
                    const double vios = ctx->cfg.safe_distance - d;
                    if (vios > 0.0) cost += ctx->cfg.w_obstacle * vios * vios;
                }

                if (!ctx->ref->empty()) {
                    ref_id = std::min<int>(ctx->ref->size() - 1, ref_id);
                    const Eigen::Vector2d dr = p - (*ctx->ref)[ref_id];
                    cost += ctx->cfg.w_ref * dr.squaredNorm();
                    if (ref_id + 1 < static_cast<int>(ctx->ref->size())) ++ref_id;
                }
            }
        }

        cost += 0.1 * ts.sum();
        return cost;
    };

    const double fx = compute_only(x);
    g.resize(x.size());
    const double eps = 1e-4;
    for (int i = 0; i < x.size(); ++i) {
        Eigen::VectorXd xp = x;
        Eigen::VectorXd xm = x;
        xp(i) += eps;
        xm(i) -= eps;
        g(i) = (compute_only(xp) - compute_only(xm)) / (2.0 * eps);
    }
    return fx;
}
} // namespace

bool MSPlanner::plan(const std::vector<Eigen::Vector2d>& reference_path,
                     const Eigen::Vector2d& start,
                     const Eigen::Vector2d& goal,
                     const GlobalEsdfMap& map,
                     std::vector<Eigen::Vector2d>& optimized_path) const {
    if (reference_path.size() < 2) return false;

    const int piece_num = std::max(2, static_cast<int>(reference_path.size()) - 1);
    const int inner_num = piece_num - 1;

    Eigen::VectorXd x0(2 * inner_num + piece_num);
    for (int i = 0; i < inner_num; ++i) {
        const size_t idx = std::min(reference_path.size() - 2, static_cast<size_t>(i + 1));
        x0(2 * i) = reference_path[idx].x();
        x0(2 * i + 1) = reference_path[idx].y();
    }
    for (int i = 0; i < piece_num; ++i) {
        const Eigen::Vector2d p0 = (i == 0) ? start : reference_path[std::min<int>(reference_path.size() - 1, i)];
        const Eigen::Vector2d p1 = (i == piece_num - 1) ? goal : reference_path[std::min<int>(reference_path.size() - 1, i + 1)];
        const double t = std::max(0.05, (p1 - p0).norm() / 0.7);
        x0(2 * inner_num + i) = std::log(t);
    }

    MsLbfgsCtx ctx;
    ctx.map = &map;
    ctx.ref = &reference_path;
    ctx.start = start;
    ctx.goal = goal;
    ctx.cfg = cfg_;
    ctx.piece_num = piece_num;

    lbfgs::lbfgs_parameter_t param;
    param.max_iterations = cfg_.max_lbfgs_iterations;

    double fx = 0.0;
    int ret = lbfgs::lbfgs_optimize(x0, fx, evalCost, nullptr, nullptr, &ctx, param);
    if (ret < 0) return false;

    Eigen::MatrixXd inPs(2, std::max(0, inner_num));
    Eigen::VectorXd ts(piece_num);
    for (int i = 0; i < inner_num; ++i) {
        inPs(0, i) = x0(2 * i);
        inPs(1, i) = x0(2 * i + 1);
    }
    for (int i = 0; i < piece_num; ++i) ts(i) = std::max(0.03, std::exp(x0(2 * inner_num + i)));

    Eigen::Matrix<double, 2, 3> head, tail;
    head.setZero(); tail.setZero();
    head.col(0) = start;
    tail.col(0) = goal;

    minco::MINCO_S3NU minco;
    minco.setConditions(head, tail, piece_num, Eigen::Vector2d(1.0, 1.0));
    minco.setParameters(inPs, ts);

    Trajectory<5, 2> traj;
    minco.getTrajectory(traj);

    optimized_path.clear();
    const int sample_each = std::max(8, cfg_.sample_per_segment);
    for (int k = 0; k < traj.getPieceNum(); ++k) {
        const auto& piece = traj[k];
        for (int i = (k == 0 ? 0 : 1); i <= sample_each; ++i) {
            const double t = piece.getDuration() * static_cast<double>(i) / sample_each;
            optimized_path.push_back(piece.getPos(t));
        }
    }
    return optimized_path.size() >= 2;
}

bool DdrEsdfPipelinePlanner::plan(const Eigen::Vector2d& start,
                                  const Eigen::Vector2d& goal,
                                  const GlobalEsdfMap& map,
                                  std::vector<Eigen::Vector2d>& front_end_path,
                                  std::vector<Eigen::Vector2d>& optimized_path) const {
    if (!jps_.plan(start, goal, map, front_end_path)) return false;
    return ms_.plan(front_end_path, start, goal, map, optimized_path);
}
