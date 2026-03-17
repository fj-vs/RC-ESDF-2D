# RC-ESDF-2D
🚀 **A high-performance C++ implementation of the Robo-centric ESDF algorithm for any-shape robotic planning.**

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![C++: 14/17](https://img.shields.io/badge/C++-14%2F17-blue.svg)](https://isocpp.org/)
[![Origin: FAST-Lab](https://img.shields.io/badge/Origin-FAST--Lab-red.svg)](https://github.com/ZJU-FAST-Lab)

## 📖 简介 (Introduction)

**RC-ESDF-2D** 是基于浙大高飞团队（FAST-Lab）研究成果的高效复现版本。该库实现在机器人中心坐标系（Body Frame）下构建欧几里得符号距离场（2D ESDF），专为复杂形状机器人的局部路径规划（如 TEB, MPC）提供核心支撑。

本项目复现自以下学术论文：
> **Robo-centric ESDF: A Fast and Accurate Whole-body Collision Evaluation Tool for Any-shape Robotic Planning**, *Weijia Zhou, Fei Gao, et al.*

---

## ✨ 核心特性 (Features)

*   **论文算法复现**: 复现了论文中提出的机器人中心 ESDF 构建逻辑，适用于任意形状的多边形足迹（Any-shape Footprint）。
*   **机器人中心坐标系 (Robo-Centric)**: 所有计算均在 Body Frame 下实时生成，无需全局地图，天然适配动态环境避障。
*   **高速 $O(1)$ 查询**: 基于双线性插值（Bilinear Interpolation），单次查询耗时仅约 **2.4 μs**（测试环境：普通移动端 CPU），满足极致的实时性需求。
*   **解析梯度 (Analytic Gradient)**: 提供连续、平滑的一阶解析梯度，确保基于梯度的优化器（如 g2o, Ceres, NLopt）能够快速且稳定地收敛。
*   **可视化辅助**: 内置基于 OpenCV 的诊断工具，可直观对比物理轮廓（Yellow Box）与离散场（SDF Field）的对齐准确度。
*   **轻量化设计**: 仅依赖 Eigen3 核心库，易于集成到现有的 ROS 或嵌入式导航系统中。

---

## 📊 可视化效果 (Visualization)

![ESDF Visualization](https://github.com/JackJu-HIT/RC-ESDF-2D/blob/master/files/RC-ESDF%20Normalized.png) 

通过内置的 `visualizeEsdf()` 函数，您可以清晰地观察：
*   🔴 **红色区域**: 机器人内部 ($dist < 0$)。
*   🟢 **绿色区域**: 机器人外部安全区 ($dist > 0$)。
*   🟨 **黄色轮廓**: 输入的原始多边形物理边界。
*   ⚪ **白色箭头**: 解析梯度向量 $\nabla D$（始终指向最短路径脱离碰撞的方向）。

---


## 🧭 新增：全局 ESDF + DDR-opt 风格前后端规划 + MPC 控制

本仓库已新增一个全局地图版本的 ESDF 与优化式路径规划模块：


- 新增 DDR-opt 风格前后端 + 控制链路：
  - 前端：`JPSPlanner`（当前实现为栅格最短路搜索接口）输出无碰撞离散路径（`planned_path_raw`）
  - 后端：`MSPlanner`（MINCO-S3 + L-BFGS）以参考路径内点与分段时间为优化变量，使用 L-BFGS 优化轨迹参数，再结合 ESDF 约束输出连续路径（`planned_path_opt`）
  - 控制：`NmpcController`（基于 MINCO 优化轨迹参考的线性化 MPC，使用 OSQP-Eigen 求解 QP）发布 `cmd_vel`（`geometry_msgs::Twist`）

- `GlobalEsdfMap`：在世界坐标系下维护占据栅格并构建 ESDF，可查询任意点的距离与梯度。
- `BsplineEsdfPlanner`：基于三次 B 样条控制点做梯度下降优化，代价包含：
  - 平滑项（二阶差分）
  - 路径长度项
  - ESDF 安全距离约束项（通过 `safe_distance - dist` 的惩罚，并用 ESDF 梯度反向传播到控制点）

你可以直接参考 `main.cpp` 中的示例：
1. 构建全局障碍地图；
2. 生成全局 ESDF；
3. 输入起终点进行 B 样条轨迹优化；
4. 检查优化后轨迹最小 ESDF 距离。


## 🤖 ROS 实时规划节点（新增）

`main.cpp` 已改为 ROS 节点 `esdf_bspline_planner_node`，工作流程如下：

- 订阅 `/robot_pose` (`geometry_msgs/Pose2D`) 作为机器人当前位姿。
- 订阅 `/scan` (`sensor_msgs/LaserScan`) 将激光点投影到世界坐标并更新全局占据栅格，再重建 ESDF。
- 订阅 RViz 2D Nav Goal 话题 `/move_base_simple/goal` (`geometry_msgs/PoseStamped`) 作为规划目标。
- 以机器人当前位置到目标点进行 B 样条优化，代价同时考虑：
  - 平滑项
  - 路径长度项
  - ESDF 安全距离项（利用 ESDF 梯度）
  - 差速车曲率约束项（`max_curvature`）
- 发布 `/planned_path` (`nav_msgs/Path`) 到 RViz 可视化。

默认参数可通过 ROS 参数服务器设置：`safe_distance`、`max_lbfgs_iterations`、`w_obstacle`、`w_ref` 等。

## 🚀 快速开始 (Quick Start)


### RViz 可视化与启动

新增 `launch/esdf_planner.launch`，可一键启动规划节点与 RViz 配置。

```bash
roslaunch rc_esdf_global_planner esdf_planner.launch
```

默认可视化话题（均可在 launch 中改）：
- `/esdf_map` (`nav_msgs/OccupancyGrid`)：ESDF 栅格可视化（障碍区域高亮）。
- `/robot_pose_vis` (`geometry_msgs/PoseStamped`)：机器人位姿。
- `/planned_path_raw` (`nav_msgs/Path`)：优化前路径（起终点直线初始化）。
- `/planned_path_opt` (`nav_msgs/Path`)：优化后路径。
- `/cmd_vel` (`geometry_msgs/Twist`)：MPC 控制器输出控制指令。

当前已把地图参数、规划参数、输入输出话题、`frame_id`、`max_vis_dist`、`publish_opt_path_when_collision` 全部提取到 `launch/esdf_planner.launch`。
新增可调参数：`search_safe_distance`、`allow_diagonal`（前端 JPS）、`max_lbfgs_iterations`、`w_obstacle`、`w_ref`（后端 MS L-BFGS + MINCO）以及 `mpc_goal_tolerance`、`mpc_allow_reverse`、`mpc_max_linear_vel`、`mpc_max_angular_vel`、`mpc_dt`、`mpc_horizon`、`mpc_w_x`、`mpc_w_y`、`mpc_w_yaw`、`mpc_w_v`、`mpc_w_w`、`mpc_w_du`（MPC QP 权重与约束）。

RViz 工具栏中的 `2D Nav Goal` 会向 `/move_base_simple/goal` 发布目标点（可通过 launch 参数 `goal_topic` 改）。

### 依赖 (Dependencies)
*   [Eigen3](http://eigen.tuxfamily.org/) (核心计算)
*   [OpenCV](https://opencv.org/) (可选，仅用于可视化调试)
*   [OsqpEigen](https://github.com/robotology/osqp-eigen) (MPC QP 求解)
*   CMake (>= 3.10)

### 编译与运行 (Build)
```bash
git clone https://github.com/JackJu-HIT/RC-ESDF-2D.git
cd RC-ESDF-2D
mkdir build && cd build
cmake ..
make
./test_rc_esdf
```

### 核心代码示例 (Basic Usage)
```cpp
#include "rc_esdf.h"

RcEsdfMap esdf;
// 初始化地图：10m x 10m, 分辨率 0.1m
esdf.initialize(10.0, 10.0, 0.1); 

// 定义机器人多边形顶点 (Body Frame)
std::vector<Eigen::Vector2d> footprint;
footprint.push_back({0.7, 0.3});
footprint.push_back({-0.7, 0.3});
footprint.push_back({-0.7, -0.3});
footprint.push_back({0.7, -0.3});

// 离线/启动时生成 SDF 场
esdf.generateFromPolygon(footprint);

// 在线查询：输入障碍物在 Body Frame 的坐标
double dist;
Eigen::Vector2d grad;
if (esdf.query(Eigen::Vector2d(0.4, 0.2), dist, grad)) {
    if (dist < 0) {
        // 发生碰撞！利用 -grad 方向将机器人推离障碍物
    }
}
```

---

## 🛠 应用场景 (Applications)
*   **TEB Local Planner**: 增强碰撞检测逻辑，为非圆形状机器人提供更精确的代价约束。
*   **轨迹优化 (Trajectory Optimization)**: 在 MPC 或 EGO-Planner 框架中作为硬约束或惩罚项。
*   **势场法导航**: 生成高质量、无震荡的斥力场。

---


## 📄 协议 (License)
本项目基于 [MIT License](LICENSE) 开源。

---

## 💡 技术解析文章推荐
为了帮助您更好地理解本项目，建议阅读以下专题文章：
*   [【除夕礼&机器人轨迹优化算法】2.4μs极致查询！支持任意轮廓的RC-ESDF算法C++工程实现【附Github仓库链接】](https://mp.weixin.qq.com/s/5BpdH-d5nquTWPLqN1UMtg)
---
