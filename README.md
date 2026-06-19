# Kinodynamic Lazy Theta* Planner

> **面向实时机器人导航的两阶段运动学路径规划算法**
>
> 技术栈：C++17 / ROS1 Noetic / RViz

---

## 项目简介

**Kinodynamic Lazy Theta\*** 是一种面向非完整约束移动机器人的全局路径规划算法，采用 **"先搜索、后优化"** 的两阶段架构：

1. **搜索阶段**：利用 Lazy Theta\* 在栅格地图上快速搜索一条几何近优的全局路径；
2. **优化阶段**：通过 **Dubins 曲线平滑** 与 **约束感知弹性优化（Constraint-Aware Elastic Optimization）** 对路径进行运动学可行性投影、曲率约束 enforcement 与平滑处理，最终输出一条 **无折线、满足起终点朝向约束、可直接被局部规划器跟踪** 的高质量路径。

### 核心特性

- **实时性高**：Lazy Theta\* 的延迟 LOS 检查 + 路径快捷化将后处理耗时控制在亚毫秒级（典型值 ~0.2 ms）
- **运动学可行**：显式考虑最小转弯半径、最大角速度、横向加速度等约束
- **起终点朝向感知**：通过 Smooth Transition 机制保证航向角与任务要求一致
- **无折线输出**：基于 Dubins 曲线段拼接与弹性优化，彻底消除传统网格路径的锯齿现象
- **Dubins 失败兜底**：自动回退到前进型三次 Bezier 曲线，保证路径连续性
- **多底盘支持**：差速驱动、阿克曼转向、全向移动三种典型机器人运动学模型

### 应用场景

| 场景 | 说明 |
|------|------|
| 室内仓储物流 AGV | 在狭窄通道中规划满足最小转弯半径的平滑路径 |
| 室外自动驾驶低速车辆 | 结合激光雷达动态障碍物更新，实时重规划可行路径 |
| 服务机器人导航 | 在人群密集环境中生成符合动力学约束的舒适轨迹骨架 |
| 机器人竞赛/科研 | 作为 baseline 与 Hybrid A\*、RRT\* 等算法进行消融对比实验 |

---

## 算法原理

### 整体架构

```
┌─────────────────────────────────────────────────────────────────┐
│                    Kinodynamic Lazy Theta*                      │
├──────────────────────┬──────────────────────────────────────────┤
│   Phase 1: Search    │           Phase 2: Optimization          │
│   (Lazy Theta*)      │   (Constraint-Aware Elastic Optimizer)   │
├──────────────────────┼──────────────────────────────────────────┤
│ 1. 栅格地图构建       │ 1. 路径重采样 (Resample)                  │
│ 2. 延迟 LOS 检查      │ 2. 路径快捷化 (Shortcut)                  │
│ 3. 优先队列扩展       │ 3. Dubins 曲线平滑 (Dubins Smoothing)     │
│ 4. 父节点回溯优化     │ 4. 约束感知弹性平滑 (Smooth)              │
│                      │ 5. 航向角计算与起终点约束 (Heading)        │
│                      │ 6. 曲率计算与曲率率限制 (Curvature)        │
│                      │ 7. 运动学约束统计与质量评估 (Metrics)      │
└──────────────────────┴──────────────────────────────────────────┘
```

### 第一阶段：Lazy Theta\* 路径搜索

Lazy Theta\* 是对 Theta\* 的改进，核心思想是 **将直线可视性（Line-of-Sight, LOS）检查推迟到节点从优先队列中弹出时再进行**，而非在设置邻居节点时立即执行。这一延迟策略大幅减少了搜索过程中的 LOS 检查次数，显著提升规划速度。

**关键机制**：

- **延迟父节点重置（Lazy Parent Reset）**：当节点 `s` 从 Open 集合中取出时，才检查其祖父节点 `parent(parent(s))` 是否与 `s` 直线可视。若可视，则将 `s` 的父节点直接设为 `parent(parent(s))`，实现"跳点"效果
- **八连通邻居扩展**：支持 4-连通或 8-连通扩展
- **代价函数融合**：综合考虑欧几里得距离代价与栅格通行代价

### 第二阶段：运动学与动力学约束优化

后处理阶段包含七个顺序执行的子模块：

1. **路径重采样（Resample）**：按固定弧长间隔均匀重采样，为后续平滑提供足够密度的控制点
2. **路径快捷化（Shortcut）**：贪心策略用长直线段替代短折线段，同时满足转弯半径约束
3. **Dubins 曲线平滑**：用 Dubins 曲线段替代折线段，生成满足最小转弯半径约束的连续曲线路径
4. **约束感知弹性平滑**：将路径视为弹性带，通过迭代梯度下降最小化能量函数
5. **航向角计算与起终点约束**：中心差分估计 + 环向加权平均 + Smoothstep 过渡
6. **曲率计算与曲率率限制**：三点法曲率估计 + 前向-后向扫描限制曲率变化率
7. **运动学约束统计与质量评估**：综合评估路径长度、平滑度、约束满足率等指标

---

## 算法公式

### 1. Lazy Theta\* 代价函数

#### 启发函数

$$h(s) = w_{heuristic} \cdot \sqrt{(x - x_g)^2 + (y - y_g)^2}$$

#### 边代价

$$c(s, s') = w_{euc} \cdot \|s - s'\|_2 + \text{TraversalCost}(s')$$

其中栅格通行代价：

$$\text{TraversalCost}(s') = w_{traversal} \cdot \frac{\text{cost}(s')^2}{C_{max}^2}$$

这里 $\text{cost}(s') = 26 + 0.9 \cdot \text{map\_cost}(s')$，$C_{max} = 252$。二次项设计使高代价区域的通行惩罚呈非线性增长。

#### 直线可视性检查（LOS Check）

$$\text{LOS}(s_0, s_1) = \bigwedge_{(x,y) \in \text{Bresenham}(s_0, s_1)} \text{isSafe}(x, y)$$

若 LOS 成立，则进行父节点重置：

$$\text{if } \text{LOS}(s, \text{parent}(\text{parent}(s))) \text{ then } \text{parent}(s) \leftarrow \text{parent}(\text{parent}(s))$$

### 2. Dubins 曲线

Dubins 曲线是平面上满足最小转弯半径约束的最短路径，由两段圆弧（C）和一段直线（S）组成，共 6 种类型：LSL、LSR、RSL、RSR、RLR、LRL。

路径长度：

$$L = R \cdot (|t| + |p| + |q|)$$

**Dubins 段评分函数**：

$$\text{score} = L + \text{penalty}_{\text{type}} + \text{penalty}_{\text{continuity}} - \text{bonus}_{\text{skip}}$$

| 惩罚项 | 值 |
|--------|-----|
| CCC 类型（RLR/LRL） | $0.18 \cdot d_{euc}$ |
| 混合转向（LSR/RSL） | $0.08 \cdot d_{euc}$ |
| 航向连续性惩罚 | $0.12 \cdot \Delta\theta \cdot R_{eff}$ |
| 跳点奖励 | $0.03 \cdot \text{skip} \cdot d_{euc}$ |

### 3. 前进型 Bezier 兜底

当 Dubins 曲线无法生成时，构造三次 Bezier 曲线：

$$B(t) = (1-t)^3 P_0 + 3(1-t)^2 t C_1 + 3(1-t)t^2 C_2 + t^3 P_1, \quad t \in [0, 1]$$

控制点：

$$C_1 = P_0 + d_{ctrl} \cdot (\cos\theta_0, \sin\theta_0), \quad C_2 = P_1 - d_{ctrl} \cdot (\cos\theta_1, \sin\theta_1)$$

### 4. 运动学约束模型

#### 有效最小转弯半径

$$R_{eff} = \max\left(R_{kinematic}, \frac{v_{ref}^2}{a_{lat,max}}, \frac{v_{ref}}{\omega_{max}}\right)$$

#### 曲率约束

$$|\kappa| \leq \kappa_{max} = \frac{1}{R_{eff}}$$

对于阿克曼转向模型，还需满足转向角约束：

$$|\delta| = \left|\arctan(L \cdot \kappa)\right| \leq \delta_{max}$$

#### 角加速度约束（曲率率限制）

$$\left|\frac{d\kappa}{ds}\right| \leq \frac{\alpha_{max}}{v_{ref}^2}$$

### 5. 路径平滑能量函数

$$E = w_{smooth} \cdot E_{smooth} + w_{ref} \cdot E_{ref} + w_{curv} \cdot E_{curv}$$

- **拉普拉斯平滑项**：$\Delta p_i = p_{i-1} + p_{i+1} - 2p_i$，惩罚路径局部弯曲
- **参考路径保持项**：$\Delta p_i^{ref} = p_i^{ref} - p_i$，防止过度偏离安全走廊
- **曲率连续性项（Jerk）**：$J_i = d^2_{i-1} - 2d^2_i + d^2_{i+1}$，保证 G² 连续性

综合更新规则：

$$p_i^{new} = p_i + \eta \cdot \left(w_{smooth} \Delta p_i + w_{ref} \Delta p_i^{ref} + w_{curv} J_i\right)$$

### 6. 路径质量评估

**平滑度评分**：

$$S = 100 \cdot \exp\left(-2 \cdot (0.6 \bar{\kappa}_{norm} + 0.4 \Delta\bar{\kappa}_{norm})\right)$$

**约束满足率**：

$$\rho = 1 - \frac{N_{violated}}{N_{checked}}$$

---

## 算法对比

| 特性 | A\* | Dijkstra | Hybrid A\* | Lazy Theta\* | **Kinodynamic Lazy Theta\*** |
|------|-----|----------|------------|--------------|------------------------------|
| 完备性 | 是 | 是 | 是 | 是 | 是 |
| 最优性 | 最优 | 最优 | 次优 | 次优 | 次优 |
| 路径形状 | 锯齿折线 | 锯齿折线 | 连续曲线 | 折线 | **平滑曲线** |
| 运动学约束 | 无 | 无 | 显式 | 无 | **显式** |
| 起终点朝向 | 无 | 无 | 支持 | 无 | **支持** |
| Dubins/Bezier 平滑 | 无 | 无 | 部分 | 无 | **完整** |
| 典型耗时 | ~50 ms | ~100 ms | ~200 ms | ~30 ms | **~30 ms + ~0.2 ms** |
| 实时重规划 | 一般 | 差 | 差 | 好 | **优秀** |

---

## 项目结构

```
Kinodynamic_Lazy_Theta/
├── CMakeLists.txt                  # CMake 构建配置
├── package.xml                     # ROS 功能包描述
├── config/
│   └── theta_star_params.yaml      # 算法参数配置文件
├── include/theta_star/
│   ├── grid_map.hpp                # 栅格地图封装类
│   ├── theta_star.hpp              # Lazy Theta* 规划器头文件
│   ├── kinodynamic_optimizer.hpp   # 运动学优化器头文件
│   └── dubins_curve.hpp            # Dubins 曲线求解器
├── launch/
│   ├── theta_star_gazebo.launch    # Gazebo 仿真启动文件
│   ├── theta_star_pioneer.launch   # Pioneer 机器人仿真启动文件
│   └── theta_star_visualization.launch  # RViz 可视化启动文件
├── map/
│   ├── PM.pgm                      # 测试地图（栅格图像）
│   └── PM.yaml                     # 地图元数据
├── rviz/
│   └── theta_star.rviz             # RViz 可视化配置
└── src/
    ├── theta_star.cpp              # Lazy Theta* 核心实现
    ├── kinodynamic_optimizer.cpp   # 运动学优化器实现
    ├── theta_star_nav_node.cpp     # ROS 导航节点（主入口）
    └── main.cpp                    # 独立测试入口
```

### 核心模块

| 模块 | 文件 | 功能 |
|------|------|------|
| GridMap | `grid_map.hpp` | 轻量级二维栅格地图封装，支持坐标转换、代价管理、动态扩容 |
| ThetaStar | `theta_star.hpp/cpp` | Lazy Theta\* 完整实现，延迟 LOS 检查、代价融合、安全检测 |
| DubinsCurve | `dubins_curve.hpp` | 完整 Dubins 曲线求解器，6 种路径类型、短距离特化、碰撞验证 |
| KinodynamicOptimizer | `kinodynamic_optimizer.hpp/cpp` | 核心创新模块，折线路径到运动学可行路径的完整转换 |
| ThetaStarNavNode | `theta_star_nav_node.cpp` | ROS 封装层，传感器接入、动态代价图更新、TF 变换、路径发布 |

---

## 运行步骤

### 环境要求

| 依赖项 | 版本要求 |
|--------|----------|
| Ubuntu | 20.04 LTS |
| ROS | Noetic Ninjemys |
| C++ 编译器 | GCC 9+ / Clang 10+（C++17） |
| CMake | 3.10+ |

### 安装与编译

**1. 安装 ROS1 Noetic**

```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt update
sudo apt install ros-noetic-desktop-full
```

**2. 创建 catkin 工作空间**

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace
```

**3. 克隆项目**

```bash
cd ~/catkin_ws/src
git clone -b Kinodynamic_Lazy_Theta https://github.com/Robot-Nav/Lazy_Thetastar_planner.git
```

**4. 安装依赖**

```bash
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src -r -y
```

**5. 编译**

```bash
cd ~/catkin_ws
catkin_make -j$(nproc)
source devel/setup.bash
```

### 启动与运行

**启动仿真与规划器**

```bash
roslaunch theta_star_planner theta_star_pioneer.launch
```

**设置目标点**

- 方式 1：在 RViz 中使用 **2D Pose Estimate** 设置初始位姿，**2D Nav Goal** 设置目标点
- 方式 2：通过 Service 调用触发规划

```bash
rosservice call /theta_star/plan "{}"
```

**查看输出**

```bash
# 原始路径
rostopic echo /theta_star/path

# 优化后的运动学路径
rostopic echo /theta_star/kinematic_path

# 实时代价图
rostopic echo /theta_star/costmap
```

### 典型运行日志

```
[INFO] Using current robot position: (-0.00, -0.00), heading: 0.00
[INFO] Lazy Theta* path found with 155 points, 36814 nodes explored
[INFO] Path optimization successful: pts=5, smoothness=95.54, len=23.579->23.444,
       opt_time=0.21 ms, dubins_time=0.15 ms, dubins_segs=3,
       feasible_v=0.60 m/s, min_R=0.900 m, constraints=100.00% (0/9 violated)
```

---

## 参数配置

核心参数位于 `config/theta_star_params.yaml`，主要分为以下几组：

### 规划器参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `planner_frequency` | 1.0 | 规划频率 (Hz) |
| `w_euc_cost` | 1.0 | 欧氏距离权重 |
| `w_traversal_cost` | 2.0 | 通行代价权重（越大越远离障碍物） |
| `how_many_corners` | 8 | 连通性（8-连通更短，4-连通更规则） |
| `allow_unknown` | true | 允许穿越未知区域 |

### 运动学约束参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `max_linear_velocity` | 1.0 | 最大线速度 (m/s) |
| `max_linear_acceleration` | 0.5 | 最大线加速度 (m/s²) |
| `max_lateral_acceleration` | 0.3 | 最大横向加速度 (m/s²) |
| `max_angular_velocity` | 1.0 | 最大角速度 (rad/s) |
| `max_angular_acceleration` | 0.5 | 最大角加速度 (rad/s²) |
| `min_turning_radius` | 0.5 | 最小转弯半径 (m) |
| `wheel_base` | 0.3 | 轴距 (m) |
| `robot_type` | 0 | 0=差速, 1=阿克曼, 2=全向 |

### 路径平滑参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `enable_smoothing` | true | 启用平滑 |
| `resample_interval` | 0.08 | 路径重采样间隔 (m) |
| `smoothing_iterations` | 90 | 平滑优化迭代次数 |
| `optimizer_step_size` | 0.18 | 优化步长 |
| `smoothness_weight` | 0.65 | 二阶平滑权重 |
| `reference_weight` | 0.35 | 参考路径保持权重 |
| `curvature_continuity_weight` | 0.15 | 曲率连续性权重 |
| `smoothing_tolerance` | 0.12 | 相对原始路径允许偏移 (m) |

### Dubins 曲线参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `enable_dubins_smoothing` | true | 启用 Dubins 曲线平滑 |
| `dubins_sample_step` | 0.02 | Dubins 曲线采样步长 (m) |
| `dubins_max_detour_ratio` | 1.3 | 最大绕行比 |
| `dubins_max_skip_points` | 2 | 最大跳点数 |

### Bezier 兜底参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `control_dist_factor` | 0.35 | 控制点距离比例因子 |
| `control_dist_min` | 0.06 | 控制点最小距离 (m) |
| `tangent_direction_threshold` | -0.15 | 切线方向余弦阈值 |
| `heading_blend_factor` | 0.55 | 航向角混合权重 |

---

## ROS 话题与服务

### 订阅话题

| 话题 | 类型 | 说明 |
|------|------|------|
| `/map` | `nav_msgs/OccupancyGrid` | 静态地图 |
| `/scan` | `sensor_msgs/LaserScan` | 激光雷达数据 |
| `/initialpose` | `geometry_msgs/PoseWithCovarianceStamped` | 初始位姿 |
| `/move_base_simple/goal` | `geometry_msgs/PoseStamped` | 目标点 |

### 发布话题

| 话题 | 类型 | 说明 |
|------|------|------|
| `/theta_star/path` | `nav_msgs/Path` | Lazy Theta\* 原始路径 |
| `/theta_star/kinematic_path` | `nav_msgs/Path` | 优化后运动学路径（含航向四元数） |
| `/theta_star/costmap` | `nav_msgs/OccupancyGrid` | 实时代价图 |

### 服务

| 服务 | 类型 | 说明 |
|------|------|------|
| `/theta_star/plan` | `std_srvs/Empty` | 触发规划 |
| `/theta_star/reset` | `std_srvs/Empty` | 重置状态 |

---

## License

MIT License
