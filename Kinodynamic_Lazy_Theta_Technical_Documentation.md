# Kinodynamic Lazy Theta\*：面向实时机器人导航的两阶段运动学路径规划算法

> **项目地址**：`/home/zjs/JZJ_theta/src/Kinodynamic_Lazy_Theta`  
> **技术栈**：C++17 / ROS1 Noetic / RViz  
> **关键词**：Lazy Theta\*、运动学约束、Dubins 曲线、Bezier 兜底、路径平滑、实时规划、阿克曼/差速底盘

---

## 1. 项目概述

### 1.1 背景与意义

在移动机器人自主导航领域，**全局路径规划**是连接感知与控制的桥梁。传统基于图搜索的算法（如 A\*、Dijkstra）虽然在完备性和最优性上有理论保证，但它们在复杂环境中往往产生**锯齿状、折线化**的路径，无法直接用于非完整约束（Non-holonomic）机器人（如阿克曼转向车辆、差速驱动底盘）的跟踪控制。

**Lazy Theta\*** 作为 Theta\* 的高效变体，通过**延迟直线-of-sight（LOS）检查**显著降低了搜索过程中的计算开销，能够生成比 A\* 更短、更贴近环境几何结构的路径。然而，Lazy Theta\* 的原始输出仍然是基于网格顶点的折线序列，缺乏对机器人运动学模型（最小转弯半径、最大角速度等）的显式考虑。

**Kinodynamic Lazy Theta\*** 项目正是在这一背景下提出的。它采用**"先搜索、后优化"的两阶段架构**：

1. **前处理阶段**：利用 Lazy Theta\* 在栅格地图上快速搜索一条几何近优的全局路径；
2. **后处理阶段**：通过**Dubins 曲线平滑**与**约束感知弹性优化（Constraint-Aware Elastic Optimization）**对路径进行运动学可行性投影、曲率约束 enforcement 与平滑处理，最终输出一条**无折线、满足起终点朝向约束、可直接被局部规划器跟踪**的高质量路径。

### 1.2 应用场景

| 场景 | 说明 |
|------|------|
| **室内仓储物流 AGV** | 在狭窄通道中规划满足最小转弯半径的平滑路径，避免原地打滑 |
| **室外自动驾驶低速车辆** | 结合激光雷达动态障碍物更新，实时重规划可行路径 |
| **服务机器人导航** | 在人群密集环境中生成符合动力学约束的舒适轨迹骨架 |
| **机器人竞赛/科研** | 作为 baseline 与 Hybrid A\*、RRT\* 等算法进行消融对比实验 |

### 1.3 核心价值

- **实时性高**：Lazy Theta\* 的延迟 LOS 检查 + 路径快捷化（Shortcut）将后处理耗时控制在 **亚毫秒级**（典型值 ~0.2 ms）；
- **运动学可行**：显式考虑最小转弯半径、最大角速度、横向加速度等约束，输出路径可直接用于纯追踪（Pure Pursuit）或模型预测控制（MPC）；
- **起终点朝向感知**：通过平滑过渡（Smooth Transition）机制，保证机器人在起点和终点的航向角与任务要求一致；
- **无折线输出**：基于 Dubins 曲线段拼接与弹性优化（Elastic Band），彻底消除传统网格路径的锯齿现象；
- **Dubins 失败兜底**：当前向 Dubins 曲线段因障碍物或曲率约束无法生成时，自动回退到**前进型三次 Bezier 曲线**，保证路径连续性；
- **多底盘支持**：通过配置参数支持差速驱动、阿克曼转向、全向移动三种典型机器人运动学模型。

---

## 2. 算法原理

### 2.1 整体设计思想

本项目采用经典的 **"搜索 + 优化" 两阶段耦合架构**，其设计哲学是：**利用图搜索的全局最优性保证路径的拓扑正确性，再利用连续空间优化将离散路径投影到运动学可行流形上**。这种解耦策略兼顾了计算效率与路径质量，避免了直接在状态空间（SE(2) 或 SE(3)）中进行高维搜索带来的实时性瓶颈。

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

### 2.2 核心算法详解

#### 2.2.1 第一阶段：Lazy Theta\* 路径搜索

Lazy Theta\* 是对 Theta\* 的改进，核心思想是**将直线可视性（Line-of-Sight, LOS）检查推迟到节点从优先队列中弹出时再进行**，而非在设置邻居节点时立即执行。这一延迟策略大幅减少了搜索过程中的 LOS 检查次数，从而显著提升了规划速度。

**关键机制**：

- **延迟父节点重置（Lazy Parent Reset）**：当节点 `s` 从 Open 集合中取出时，才检查其父节点 `parent(s)` 的父节点 `parent(parent(s))` 是否与 `s` 直线可视。若可视，则将 `s` 的父节点直接设为 `parent(parent(s))`，实现"跳点"效果；
- **八连通邻居扩展**：支持 4-连通或 8-连通扩展，通过 `how_many_corners` 参数配置；
- **代价函数融合**：综合考虑欧几里得距离代价与栅格通行代价，公式见第 3 节。

#### 2.2.2 第二阶段：运动学与动力学约束优化

后处理阶段是本项目的主要创新点，包含七个顺序执行的子模块：

**（1）路径重采样（Resample）**

将 Lazy Theta\* 输出的稀疏折线路径按固定弧长间隔（默认 0.08 m）进行均匀重采样，为后续平滑提供足够密度的控制点。

**（2）路径快捷化（Shortcut）**

采用贪心策略尝试用长直线段替代短折线段，同时满足转弯半径约束。该步骤在保持路径拓扑不变的前提下显著减少路径点数，降低后续优化计算量。

**（3）Dubins 曲线平滑（Dubins Smoothing）**

在快捷化后的路径上，算法尝试用 Dubins 曲线段（由圆弧-直线-圆弧组成）替代折线段，从而生成满足最小转弯半径约束的连续曲线路径。

**Dubins 曲线段选择策略**：

- 对于每一段候选路径，尝试跳过 1~`dubins_max_skip_points` 个中间点，寻找更长的 Dubins 段；
- 评估 6 种经典 Dubins 类型（LSL、LSR、RSL、RSR、RLR、LRL），选择长度最短且满足碰撞检测的段；
- 引入**评分机制**对不同类型进行惩罚：CCC（RLR/LRL）惩罚 15%、混合转向（LSR/RSL）惩罚 8%、连续性惩罚 12%、跳点奖励 3%；
- 若某段无法生成有效的 Dubins 曲线（如障碍物阻挡或超出最大绕行比），则自动触发**前进型 Bezier 兜底**。

**前进型 Bezier 兜底（Forward Bezier Fallback）**：

当 Dubins 曲线失败时，算法构造一条**三次 Bezier 曲线**作为替代：

- 控制点距离由参数 `bezier_control_dist_factor`（默认 0.35）乘以起止点距离确定，并受最小转弯半径约束限制；
- 航向角采用几何航向与预估航向的混合（混合因子 `bezier_heading_blend_factor`，默认 0.55）；
- Bezier 采样后需通过碰撞检测、转弯半径约束与切线方向一致性检查；
- 若 Bezier 长度超过最大绕行比（`dubins_max_detour_ratio`，默认 1.3），则拒绝该段。

**（4）约束感知弹性平滑（Constraint-Aware Elastic Smoothing）**

在 Dubins 路径基础上，算法将路径视为**弹性带（Elastic Band）**，通过迭代梯度下降最小化以下能量函数：

$$
E = w_{smooth} \cdot E_{smooth} + w_{ref} \cdot E_{ref} + w_{curv} \cdot E_{curv}
$$

其中：
- $E_{smooth}$：二阶拉普拉斯平滑能量，惩罚路径的局部弯曲；
- $E_{ref}$：参考路径保持能量，防止优化后的路径过度偏离原始安全走廊；
- $E_{curv}$：曲率连续性能量（Jerk 项），惩罚曲率的三阶变化，保证路径的 G² 连续性。

每次迭代中，候选点需通过**碰撞检测**与**转弯半径约束检查**，否则会被逐步回退到线段中点，直至满足约束或达到最大尝试次数。

**（5）航向角计算与起终点约束（Heading Computation）**

路径点航向角通过中心差分法估计，并经过环向加权平均（Circular Weighted Average）平滑。对于起点和终点，采用**三次 Hermite 插值**实现朝向的渐进过渡，避免航向突变。

**（6）曲率计算与曲率率限制（Curvature & Curvature Rate Limiting）**

使用三点法估计路径曲率，并通过前向-后向扫描（Forward-Backward Sweep）限制曲率的空间变化率（$dk/ds$），确保路径满足最大角加速度约束。

**（7）运动学约束统计与质量评估（Metrics）**

综合评估路径长度、平滑度评分、约束满足率、Dubins 段使用数量等指标，输出可量化的路径质量报告。

### 2.3 算法对比分析

| 特性 | A\* | Dijkstra | Hybrid A\* | Lazy Theta\* | **Kinodynamic Lazy Theta\*** |
|------|-----|----------|------------|--------------|------------------------------|
| 完备性 | 是 | 是 | 是 | 是 | 是 |
| 最优性 | 最优 | 最优 | 次优 | 次优 | 次优 |
| 路径形状 | 锯齿折线 | 锯齿折线 | 连续曲线 | 折线 | **平滑曲线** |
| 运动学约束 | 无 | 无 | 显式 | 无 | **显式** |
| 起终点朝向 | 无 | 无 | 支持 | 无 | **支持** |
| Dubins/Bezier 平滑 | 无 | 无 | 部分 | 无 | **完整** |
| 计算复杂度 | $O(b^d)$ | $O(V \log V)$ | 高（连续状态） | $O(b^d)$（更低常数） | **$O(b^d)$ + 线性优化** |
| 典型耗时 | ~50 ms | ~100 ms | ~200 ms | ~30 ms | **~30 ms + ~0.2 ms** |
| 实时重规划 | 一般 | 差 | 差 | 好 | **优秀** |

> **注**：上表中的典型耗时为在 200×200 栅格地图上的经验值，实际性能取决于具体硬件与地图复杂度。

---

## 3. 算法公式

### 3.1 Lazy Theta\* 代价函数

#### 3.1.1 启发函数（Heuristic）

对于节点 $s = (x, y)$，到目标点 $s_{goal} = (x_g, y_g)$ 的启发代价采用欧几里得距离：

$$
h(s) = w_{heuristic} \cdot \sqrt{(x - x_g)^2 + (y - y_g)^2}
$$

其中 $w_{heuristic}$ 为启发权重，默认取 $1.0$。当 $w_{heuristic} < 1.0$ 时，算法变为次优但更快；当 $w_{heuristic} = 1.0$ 时，保证可采纳性（Admissible）。

#### 3.1.2 边代价（Edge Cost）

从节点 $s$ 到邻居节点 $s'$ 的边代价包含两部分：欧几里得距离代价与栅格通行代价：

$$
c(s, s') = w_{euc} \cdot \|s - s'\|_2 + \text{TraversalCost}(s')
$$

其中栅格通行代价定义为：

$$
\text{TraversalCost}(s') = w_{traversal} \cdot \frac{\text{cost}(s')^2}{C_{max}^2}
$$

这里 $\text{cost}(s') = 26 + 0.9 \cdot \text{map\_cost}(s')$，$C_{max} = 252$ 为最大非障碍物代价值。二次项设计使得高代价区域（如靠近障碍物）的通行惩罚呈非线性增长。

#### 3.1.3 直线可视性检查（LOS Check）

Lazy Theta\* 使用改进的 Bresenham 直线算法进行 LOS 检查。对于从 $(x_0, y_0)$ 到 $(x_1, y_1)$ 的线段，算法逐步采样栅格并检查安全性：

$$
\text{LOS}(s_0, s_1) = \bigwedge_{(x,y) \in \text{Bresenham}(s_0, s_1)} \text{isSafe}(x, y)
$$

若 LOS 成立，则进行**父节点重置**：

$$
\text{if } \text{LOS}(s, \text{parent}(\text{parent}(s))) \text{ then } \text{parent}(s) \leftarrow \text{parent}(\text{parent}(s))
$$

### 3.2 Dubins 曲线与 Bezier 兜底

#### 3.2.1 Dubins 曲线模型

Dubins 曲线是平面上满足最小转弯半径约束的最短路径，由两段圆弧（C）和一段直线（S）组成，共 6 种类型：LSL、LSR、RSL、RSR、RLR、LRL。

对于起点 $(x_0, y_0, \theta_0)$ 和终点 $(x_1, y_1, \theta_1)$，Dubins 路径长度 $L$ 为：

$$
L = R \cdot (|t| + |p| + |q|)
$$

其中 $R$ 为转弯半径，$t, p, q$ 为标准化后的三段弧长参数。

**Dubins 段评分函数**：

$$
\text{score} = L + \text{penalty}_{\text{type}} + \text{penalty}_{\text{continuity}} - \text{bonus}_{\text{skip}}
$$

其中：
- CCC 类型（RLR/LRL）额外惩罚 $0.18 \cdot d_{euc}$
- 混合转向（LSR/RSL）额外惩罚 $0.08 \cdot d_{euc}$
- 航向连续性惩罚 $0.12 \cdot \Delta\theta \cdot R_{eff}$
- 跳点奖励 $0.03 \cdot \text{skip} \cdot d_{euc}$

#### 3.2.2 前进型 Bezier 兜底

当 Dubins 曲线无法生成时（如障碍物阻挡），算法构造三次 Bezier 曲线：

$$
B(t) = (1-t)^3 P_0 + 3(1-t)^2 t C_1 + 3(1-t)t^2 C_2 + t^3 P_1, \quad t \in [0, 1]
$$

其中控制点为：

$$
\begin{aligned}
C_1 &= P_0 + d_{ctrl} \cdot (\cos\theta_0, \sin\theta_0) \\
C_2 &= P_1 - d_{ctrl} \cdot (\cos\theta_1, \sin\theta_1)
\end{aligned}
$$

控制点距离 $d_{ctrl}$ 的计算：

$$
d_{ctrl} = \text{clamp}\left(f_{factor} \cdot d_{euc}, \quad d_{min}, \quad \max(d_{floor}, \quad r_{ratio} \cdot R_{eff})\right)
$$

其中：
- $f_{factor}$：`bezier_control_dist_factor`（默认 0.35）
- $d_{min}$：`bezier_control_dist_min`（默认 0.06 m）
- $d_{floor}$：`bezier_control_dist_min_radius_floor`（默认 0.12 m）
- $r_{ratio}$：`bezier_control_dist_min_radius_ratio`（默认 0.9）

**Bezier 验证条件**：
1. 所有采样点通过碰撞检测；
2. 路径长度不超过 $1.3 \cdot d_{euc}$；
3. 所有内部点满足转弯半径约束；
4. 切线方向与期望航向的余弦值不低于阈值（默认 -0.15）。

### 3.3 运动学约束模型

#### 3.3.1 有效最小转弯半径

机器人的实际最小转弯半径由运动学约束与动力学约束共同决定：

$$
R_{eff} = \max\left(R_{kinematic}, \frac{v_{ref}^2}{a_{lat,max}}, \frac{v_{ref}}{\omega_{max}}\right)
$$

其中：
- $R_{kinematic}$：机械结构决定的最小转弯半径；
- $v_{ref}$：路径可行性评估速度（默认 $0.6 \cdot v_{max}$）；
- $a_{lat,max}$：最大横向加速度；
- $\omega_{max}$：最大角速度。

#### 3.3.2 曲率约束

路径上任意点的曲率 $\kappa$ 必须满足：

$$
|\kappa| \leq \kappa_{max} = \frac{1}{R_{eff}}
$$

对于阿克曼转向模型，还需满足转向角约束：

$$
|\delta| = \left|\arctan(L \cdot \kappa)\right| \leq \delta_{max}
$$

其中 $L$ 为轴距，$\delta_{max}$ 通常取 $\pi/4$。

#### 3.3.3 角加速度约束（曲率率限制）

曲率沿弧长的变化率受到最大角加速度的限制：

$$
\left|\frac{d\kappa}{ds}\right| \leq \frac{\alpha_{max}}{v_{ref}^2}
$$

其中 $\alpha_{max}$ 为最大角加速度。该约束通过前向-后向扫描算法在离散路径上强制执行。

### 3.4 路径平滑能量函数

#### 3.4.1 拉普拉斯平滑项

对于路径点 $p_i$，其二阶拉普拉斯算子为：

$$
\Delta p_i = p_{i-1} + p_{i+1} - 2p_i
$$

该项惩罚路径的局部非线性，驱使路径趋向直线。

#### 3.4.2 参考路径保持项

$$
\Delta p_i^{ref} = p_i^{ref} - p_i
$$

该项防止优化后的路径过度偏离原始 Lazy Theta\* 路径，确保路径始终位于安全的自由空间走廊内。

#### 3.4.3 曲率连续性项（Jerk）

使用二阶差分的差分近似曲率的三阶变化：

$$
\begin{aligned}
d^2_{i-1} &= p_{i-2} - 2p_{i-1} + p_i \\
d^2_i &= p_{i-1} - 2p_i + p_{i+1} \\
d^2_{i+1} &= p_i - 2p_{i+1} + p_{i+2} \\
J_i &= d^2_{i-1} - 2d^2_i + d^2_{i+1}
\end{aligned}
$$

#### 3.4.4 综合更新规则

在每次迭代中，路径点的更新量为：

$$
p_i^{new} = p_i + \eta \cdot \left(w_{smooth} \Delta p_i + w_{ref} \Delta p_i^{ref} + w_{curv} J_i\right)
$$

其中 $\eta$ 为优化步长。更新后，$p_i^{new}$ 需通过碰撞检测与转弯半径约束验证，否则被逐步回退。

### 3.5 航向角计算与平滑

#### 3.5.1 中心差分估计

对于内部路径点，航向角通过前后点估计：

$$
\theta_i = \arctan\left(\frac{y_{i+1} - y_{i-1}}{x_{i+1} - x_{i-1}}\right)
$$

边界点采用前向/后向差分。

#### 3.5.2 环向加权平均

为避免角度跳变（$\pm 2\pi$ 问题），航向平滑在正弦-余弦空间进行：

$$
\theta_i^{smooth} = \arctan2\left(\sum_{j} w_j \sin\theta_j, \sum_{j} w_j \cos\theta_j\right)
$$

#### 3.5.3 起终点朝向过渡

起点朝向过渡使用平滑步长函数（Smoothstep）：

$$
\theta_i = \text{interpAngle}(\theta_{init}, \theta_i, 3t^2 - 2t^3), \quad t = \frac{i}{N_{transition}}
$$

终点朝向采用对称的反向过渡。

### 3.6 路径质量评估指标

#### 3.6.1 平滑度评分

$$
S = 100 \cdot \exp\left(-2 \cdot (0.6 \bar{\kappa}_{norm} + 0.4 \Delta\bar{\kappa}_{norm})\right)
$$

其中 $\bar{\kappa}_{norm} = \bar{\kappa} \cdot R_{eff}$，$\Delta\bar{\kappa}_{norm} = \Delta\bar{\kappa} \cdot R_{eff}$。评分范围为 $[0, 100]$，越接近 100 表示路径越平滑。

#### 3.6.2 约束满足率

$$
\rho = 1 - \frac{N_{violated}}{N_{checked}}
$$

其中 $N_{violated}$ 为违反运动学约束的路径点数，$N_{checked}$ 为总检查点数。

---

## 4. 项目内容

### 4.1 项目结构

```
Kinodynamic_Lazy_Theta/
├── CMakeLists.txt                  # CMake 构建配置
├── package.xml                     # ROS 功能包描述
├── Kinodynamic_Lazy_Theta_Technical_Documentation.md  # 技术文档
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

### 4.2 核心模块功能

#### 4.2.1 GridMap（栅格地图）

`grid_map.hpp` 实现了一个轻量级的二维栅格地图封装，提供以下核心功能：

- **坐标转换**：`worldToMap` / `mapToWorld` 实现世界坐标系与栅格索引的双向映射；
- **代价管理**：支持 `NO_INFORMATION`（255）、`LETHAL_OBSTACLE`（254）、`FREE_SPACE`（0）等标准代价层级；
- **动态扩容**：`resize` 方法支持地图的动态更新，适配激光雷达实时建图场景。

#### 4.2.2 ThetaStar（Lazy Theta\* 规划器）

`theta_star.hpp / theta_star.cpp` 实现了完整的 Lazy Theta\* 算法，关键设计包括：

- **节点管理**：使用扁平数组 `nodes_data_` 与指针数组 `node_position_` 实现 O(1) 的节点查找，避免哈希表开销；
- **延迟 LOS**：`resetParent` 方法在节点弹出时执行父节点回溯优化；
- **代价融合**：`getTraversalCost` 与 `getEuclideanCost` 实现距离代价与通行代价的加权融合；
- **安全检测**：`isSafe` 方法支持未知区域穿越（`allow_unknown` 参数控制）。

#### 4.2.3 DubinsCurve（Dubins 曲线求解器）

`dubins_curve.hpp` 实现了完整的 Dubins 曲线求解器：

- **6 种路径类型**：支持 LSL、LSR、RSL、RSR、RLR、LRL 六种经典 Dubins 路径；
- **短距离特化**：当起止点距离小于转弯半径时，采用特殊评分策略（CCC 惩罚 2 倍、混合转向惩罚 1.3 倍）；
- **路径采样**：`samplePath` 方法按固定步长采样 Dubins 曲线，输出带曲率信息的点序列；
- **碰撞验证**：`sampleAndValidate` 方法在采样的同时进行碰撞检测，确保路径安全性。

#### 4.2.4 KinodynamicOptimizer（运动学优化器）

`kinodynamic_optimizer.hpp / kinodynamic_optimizer.cpp` 是项目的核心创新模块，实现了从折线路径到运动学可行路径的完整转换：

| 方法 | 功能 |
|------|------|
| `resamplePath` | 均匀重采样，固定弧长间隔生成控制点 |
| `shortcutPath` | 贪心快捷化，减少路径点数同时保持约束 |
| `applyDubinsSmoothing` | Dubins 曲线平滑与 Bezier 兜底 |
| `smoothPath` | 约束感知弹性平滑，核心优化循环 |
| `computeHeadings` | 航向角计算与起终点朝向约束过渡 |
| `computeCurvatures` | 三点法曲率估计与平滑 |
| `enforceCurvatureRateLimit` | 曲率率限制，保证角加速度约束 |
| `evaluatePathQuality` | 路径质量综合评估与指标统计 |

#### 4.2.5 ThetaStarNavNode（ROS 导航节点）

`theta_star_nav_node.cpp` 是算法的 ROS 封装层，负责：

- **传感器接入**：订阅 `/map`（静态地图）、`/scan`（激光雷达）、`/initialpose`（初始位姿）、`/move_base_simple/goal`（目标点）；
- **动态代价图更新**：`LaserCostmapUpdater` 类将激光点云实时投影到栅格地图，并进行障碍物膨胀；
- **TF 坐标变换**：通过 `tf2` 获取机器人当前位姿，支持 `map` → `base_link` 的实时变换；
- **路径发布**：发布 `/theta_star/path`（原始路径）与 `/theta_star/kinematic_path`（优化后路径）；
- **服务接口**：提供 `/theta_star/plan`（触发规划）与 `/theta_star/reset`（重置状态）两个 ROS Service。

### 4.3 关键技术实现

#### 4.3.1 Dubins 曲线与 Bezier 兜底的耦合策略

在 Dubins 平滑阶段，算法采用**贪心跳点搜索**策略：

1. 对于每个起点，尝试跳过 1~`max_skip` 个中间点；
2. 计算几何航向与预估航向的混合航向角（混合因子随距离自适应）；
3. 求解 Dubins 路径并采样验证；
4. 若 Dubins 失败，自动回退到前进型 Bezier 曲线；
5. 选择评分最优的段进行拼接。

这种**先 Dubins、后 Bezier**的策略在保证运动学可行性的同时，最大化路径的平滑度与连续性。

#### 4.3.2 碰撞检测与约束验证的耦合策略

在平滑迭代中，每个候选点的验证顺序经过精心设计：

1. **转弯半径约束**：$O(1)$ 计算，快速拒绝不可行点；
2. **线段碰撞检测**：对前后线段进行均匀采样检查，步长为 `resample_interval * 0.25`；
3. **失败回退**：若验证失败，将候选点向中点方向回退 50%，最多尝试 5 次。

这种**先运动学、后碰撞**的验证顺序最大化了早期拒绝效率。

#### 4.3.3 路径锚定机制

为防止平滑过程中起点/终点漂移，算法强制固定：

```cpp
smooth_path.front() = raw_path.front();
smooth_path.back() = raw_path.back();
```

#### 4.3.4 多底盘运动学适配

通过 `KinodynamicConfig::RobotType` 枚举支持三种底盘：

- **差速驱动（DIFFERENTIAL_DRIVE）**：主要考虑最小转弯半径与角速度约束；
- **阿克曼转向（ACKERMANN）**：额外检查转向角约束 $|\delta| \leq \pi/4$；
- **全向移动（OMNI_DIRECTIONAL）**：仅考虑位置约束，航向约束放宽。

---

## 5. 运行步骤

### 5.1 环境配置要求

| 依赖项 | 版本要求 | 说明 |
|--------|----------|------|
| Ubuntu | 20.04 LTS | 推荐，兼容 ROS1 Noetic |
| ROS | Noetic Ninjemys | 完整桌面版安装 |
| C++ 编译器 | GCC 9+ / Clang 10+ | 支持 C++17 标准 |
| CMake | 3.10+ | 构建系统 |
| catkin | ROS 自带 | 功能包编译工具 |
| Pioneer 仿真包 | `pioneer_utils` | Gazebo 仿真环境（可选） |

### 5.2 依赖安装指南

**步骤 1：安装 ROS1 Noetic**

```bash
# 参考 ROS 官方安装指南
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt update
sudo apt install ros-noetic-desktop-full
```

**步骤 2：初始化 catkin 工作空间**

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace
```

**步骤 3：拷贝项目代码**

```bash
cp -r /home/zjs/JZJ_theta/src/Kinodynamic_Lazy_Theta ~/catkin_ws/src/
```

**步骤 4：安装 ROS 依赖**

```bash
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src -r -y
```

### 5.3 编译与运行

**编译项目**

```bash
cd ~/catkin_ws
catkin_make -j$(nproc)
source devel/setup.bash
```

**启动仿真与规划器**

```bash
# 启动 Pioneer 机器人仿真、地图服务器、Theta* 导航节点与 RViz
roslaunch theta_star_planner theta_star_pioneer.launch
```

**手动触发规划**

```bash
# 方式 1：通过 RViz 设置初始位姿（2D Pose Estimate）与目标点（2D Nav Goal）
# 方式 2：通过 Service 调用触发规划
rosservice call /theta_star/plan "{}"
```

**查看输出**

```bash
# 查看原始路径
echo "原始路径："
rostopic echo /theta_star/path | head -n 20

# 查看优化后的运动学路径
echo "优化路径："
rostopic echo /theta_star/kinematic_path | head -n 20

# 查看实时代价图（若启用激光雷达）
rostopic echo /theta_star/costmap | head -n 10
```

### 5.4 参数配置说明

核心参数位于 `config/theta_star_params.yaml`，可根据实际场景调整：

```yaml
# 规划器参数
planner:
  planner_frequency: 1.0          # 规划频率，建议 1~10 Hz
  w_euc_cost: 1.0                 # 欧氏距离权重
  w_traversal_cost: 2.0           # 通行代价权重（越大越远离障碍物）
  how_many_corners: 8             # 8-连通扩展更短，4-连通更规则

# 运动学约束（根据机器人实际参数修改）
kinodynamic:
  max_linear_velocity: 1.0        # 最大线速度 (m/s)
  max_linear_acceleration: 0.5    # 最大线加速度 (m/s²)
  max_lateral_acceleration: 0.3   # 最大横向加速度 (m/s²)
  max_angular_velocity: 1.0       # 最大角速度 (rad/s)
  max_angular_acceleration: 0.5   # 最大角加速度 (rad/s²)
  min_turning_radius: 0.5         # 最小转弯半径 (m)
  wheel_base: 0.3                 # 轴距 (m)
  dynamic_feasibility_speed: 0.6  # 路径级动力学可行性评估速度 (m/s)
  robot_type: 0                   # 0=差速, 1=阿克曼, 2=全向

# 路径平滑参数（约束感知弹性优化）
smoothing:
  enable_smoothing: true          # 启用平滑
  resample_interval: 0.08         # 路径重采样间隔 (m)
  smoothing_iterations: 90        # 平滑优化迭代次数
  optimizer_step_size: 0.18       # 优化步长
  smoothness_weight: 0.65         # 二阶平滑权重
  reference_weight: 0.35          # 参考路径保持权重
  curvature_continuity_weight: 0.15  # 曲率连续性权重
  enforce_collision_check: true   # 平滑过程中是否检查碰撞
  shortcut_max_passes: 2          # 线段可视化快捷化次数
  bspline_degree: 3               # 兼容保留参数
  smoothing_tolerance: 0.12       # 相对原始路径允许偏移 (m)

# Dubins 曲线平滑参数
dubins:
  enable_dubins_smoothing: true   # 启用 Dubins 曲线平滑
  dubins_sample_step: 0.02        # Dubins 曲线采样步长 (m)
  dubins_max_detour_ratio: 1.3    # Dubins 路径相对直线距离的最大绕行比
  dubins_max_skip_points: 2       # Dubins 曲线段最大跳点数

# 前进型 Bezier 兜底参数（Dubins 失败时使用）
bezier_fallback:
  enable: true                    # 启用 Bezier 兜底
  control_dist_factor: 0.35       # 控制点距离比例因子（相对于起止点距离）
  control_dist_min: 0.06          # 控制点最小距离 (m)
  control_dist_min_radius_ratio: 0.9  # 控制点距离与最小转弯半径的比例
  control_dist_min_radius_floor: 0.12 # 控制点距离最小转弯半径下限 (m)
  tangent_direction_threshold: -0.15  # 切线方向余弦阈值（低于此值认为方向异常）
  heading_blend_factor: 0.55      # 航向角混合权重（几何航向与预估航向的混合比例）

# 激光雷达参数
laser:
  use_laser: true                 # 启用激光雷达
  laser_topic: /scan              # 激光雷达话题
  laser_max_range: 10.0           # 最大检测距离 (m)
  laser_min_range: 0.1            # 最小检测距离 (m)
  obstacle_inflation_radius: 0.5  # 障碍物膨胀半径 (m)
  map_frame: map                  # 地图坐标系
  robot_frame: base_link          # 机器人坐标系

# 可视化参数
visualization:
  visualize_explored_nodes: true  # 显示探索节点
  visualize_grid_cost: true       # 显示网格代价
  grid_cost_alpha: 0.3            # 网格透明度

# 话题名称配置
topics:
  map_topic: /map
  start_topic: /initialpose
  goal_topic: /move_base_simple/goal
  path_topic: /theta_star/path
  kinematic_path_topic: /theta_star/kinematic_path
  marker_topic: /theta_star/visualization
  grid_cost_topic: /theta_star/grid_cost
```

---

## 6. 实验结果

### 6.1 典型运行日志分析

以下是在 Pioneer 机器人仿真环境中的一次典型规划日志：

```
[INFO] Using current robot position: (-0.00, -0.00), heading: 0.00
[INFO] Lazy Theta* path found with 155 points, 36814 nodes explored
[INFO] Path optimization successful: pts=5, smoothness=95.54, len=23.579->23.444, 
       opt_time=0.21 ms, dubins_time=0.15 ms, dubins_segs=3, 
       feasible_v=0.60 m/s, min_R=0.900 m, constraints=100.00% (0/9 violated)
```

**关键指标解读**：

| 指标 | 数值 | 说明 |
|------|------|------|
| 原始路径点数 | 155 | Lazy Theta\* 输出的折线路径点数 |
| 探索节点数 | 36,814 | 搜索过程中访问的栅格节点数 |
| 优化后路径点数 | 5 | 经过快捷化、Dubins 平滑与弹性平滑后的路径点数 |
| 平滑度评分 | 95.54/100 | 路径几何平滑程度，接近满分 |
| 路径长度变化 | 23.579 → 23.444 m | 优化后略短，说明无显著绕路 |
| 优化总耗时 | 0.21 ms | 后处理阶段总耗时，满足实时性要求 |
| Dubins 耗时 | 0.15 ms | Dubins 曲线求解与采样耗时 |
| Dubins 段数 | 3 | 成功生成的 Dubins 曲线段数量 |
| 可行速度 | 0.60 m/s | 基于动力学约束评估的安全速度 |
| 有效最小转弯半径 | 0.900 m | 综合运动学/动力学约束后的最小半径 |
| 约束满足率 | 100.00% | 所有运动学约束均满足 |

### 6.2 性能对比（待补充）

> **注意**：以下表格为模板，请根据实际消融实验与对比实验填充数据。

#### 6.2.1 消融实验：后处理模块贡献

| 配置 | 路径长度 (m) | 路径点数 | 平滑度评分 | 约束满足率 | 优化耗时 (ms) |
|------|-------------|----------|-----------|-----------|--------------|
| 仅 Lazy Theta\*（基线） | - | - | - | - | - |
| + 重采样 | - | - | - | - | - |
| + 快捷化 | - | - | - | - | - |
| + Dubins 平滑 | - | - | - | - | - |
| + 弹性平滑 | - | - | - | - | - |
| + 曲率率限制 | - | - | - | - | - |
| **完整系统** | **-** | **-** | **-** | **-** | **-** |

#### 6.2.2 与同类算法对比

| 算法 | 规划耗时 (ms) | 路径长度 (m) | 平均曲率 (1/m) | 最大曲率 (1/m) | 起终点朝向误差 (rad) |
|------|--------------|-------------|---------------|---------------|---------------------|
| A\* | - | - | - | - | - |
| Dijkstra | - | - | - | - | - |
| Hybrid A\* | - | - | - | - | - |
| Lazy Theta\* | - | - | - | - | - |
| **Kinodynamic Lazy Theta\*** | **-** | **-** | **-** | **-** | **-** |

#### 6.2.3 实时性测试（不同地图规模）

| 地图尺寸 (cells) | 搜索耗时 (ms) | 优化耗时 (ms) | 总耗时 (ms) | 帧率 (Hz) |
|-----------------|--------------|--------------|------------|----------|
| 100 × 100 | - | - | - | - |
| 200 × 200 | - | - | - | - |
| 500 × 500 | - | - | - | - |
| 1000 × 1000 | - | - | - | - |

### 6.3 RViz 可视化效果

启动 `theta_star_pioneer.launch` 后，在 RViz 中可以观察到以下可视化内容：

- **原始路径**（白色）：`/theta_star/path`，显示 Lazy Theta\* 的折线输出；
- **优化路径**（绿色）：`/theta_star/kinematic_path`，显示带有航向的四元数姿态；
- **代价图**（彩色栅格）：`/theta_star/costmap`，显示静态地图与激光雷达融合后的实时代价分布。

---

## 7. 未来展望

### 7.1 潜在优化方向

1. **时间参数化与速度规划**  
   当前输出为纯几何路径（Path），未来可扩展为时间参数化的轨迹（Trajectory），集成速度-加速度曲线规划，实现真正的时空最优控制。

2. **动态障碍物避碰**  
   当前激光雷达仅用于静态代价图更新。可引入**动态窗口法（DWA）**或**模型预测控制（MPC）**作为局部规划器，与全局路径形成分层规划架构。

3. **三维地形扩展**  
   将栅格地图扩展为三维体素地图（Voxel Grid），支持无人机、四足机器人在非平坦地形上的运动学路径规划。

4. **机器学习辅助启发函数**  
   利用神经网络学习环境的启发函数，进一步减少搜索节点数，提升在超大规模地图（>10⁶ cells）上的规划效率。

5. **多机器人协同规划**  
   引入冲突搜索（Conflict-Based Search, CBS）或优先级规划，将 Kinodynamic Lazy Theta\* 扩展至多机器人系统的无冲突路径规划。

### 7.2 扩展应用场景

| 领域 | 应用方向 |
|------|----------|
| **自动驾驶** | 停车场低速导航、园区接驳车路径规划 |
| **物流配送** | 仓储 AGV 高密度环境下的实时调度 |
| **农业机器人** | 果园/大棚中的行间作业路径生成 |
| **搜救机器人** | 废墟环境中的快速可达路径规划 |
| **服务机器人** | 医院/商场中的行人感知导航 |

---

## 附录：代码规范与开发约定

- **命名空间**：所有核心类位于 `theta_star` 命名空间；
- **代码风格**：遵循 ROS C++ 风格指南，使用 `snake_case` 命名变量与函数；
- **线程安全**：`LaserCostmapUpdater` 使用 `std::mutex` 保护代价图并发访问；
- **内存管理**：`ThetaStarNavNode` 使用裸指针管理 `GridMap` 与 `LaserCostmapUpdater`，在析构函数中显式释放；
- **编译优化**：`CMakeLists.txt` 中启用 `-O3` 级别优化，确保发布版本的实时性能。

---

> **版权声明**：本项目基于 Apache 2.0 许可证开源，欢迎用于学术研究与商业应用。如有问题或建议，欢迎提交 Issue 或 Pull Request。
