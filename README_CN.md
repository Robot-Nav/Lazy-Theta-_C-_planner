# ThetaStar_CPP

**Lazy Theta* 路径规划算法的纯 C++ 实现，完全脱离 ROS/ROS2 依赖。**

[![License](https://img.shields.io/badge/License-Apache--2.0-blue.svg)](./LICENSE)

---

## 目录

- [项目概述](#项目概述)
- [算法原理](#算法原理)
- [算法公式](#算法公式)
- [项目结构](#项目结构)
- [编译构建](#编译构建)
- [快速上手](#快速上手)
- [API 参考](#api-参考)
- [与其他规划器对比](#与其他规划器对比)
- [性能指标](#性能指标)
- [许可证](#许可证)

---

## 项目概述

**ThetaStar_CPP** 是一个独立、精简的 C++17 库，实现了 **Lazy Theta***（懒Theta*）任意角度路径规划算法。它是对 ROS2 Navigation2 框架中 [nav2_theta_star_planner](https://github.com/SteveMacenski/nav2_theta_star_planner) 插件的完全解耦重构，移除了所有 ROS/ROS2 依赖。

核心算法搜索逻辑完全保留原实现，可作为任何需要高效任意角度路径规划的机器人、游戏开发或仿真项目的直接替代方案，无需引入完整的 ROS 系统开销。

### 核心特性

- **任意角度路径** — 路径不受网格方向约束，可产生平滑的任意角度路径
- **纯 C++17** — 仅依赖标准库和 pthreads，无外部依赖
- **高度可配置** — 支持 4/8 连通性、成本权重、未知区域穿越等参数调节
- **跨平台** — 支持 Linux（GCC/Clang）、Windows（MSVC/MinGW）、macOS
- **算法与接口分离** — 清晰的头文件与实现分离设计

---

## 算法原理

### 什么是 Theta*？

Theta*（读作 "theta star"）是一种任意角度（any-angle）路径规划算法，它在 A* 搜索框架的基础上引入了**视线检查（Line-of-Sight, LOS）**机制。与传统 A* 只能沿 4 或 8 个离散方向移动不同，Theta* 可以产生任意角度的路径——路径更短、更自然。

其核心思想非常简洁：每当一个节点被扩展时，Theta* 检查通过该节点的**祖父节点**（父节点的父节点）是否存在一条捷径。如果当前节点与其祖父节点之间视线畅通，则将祖父节点设为当前节点的父节点，从而"拉直"路径。

### Lazy Theta* 变体

本实现采用 **Lazy Theta*** 变体。与标准 Theta* 在节点插入时立即进行视线检查不同，Lazy Theta* 延迟到节点位于优先队列顶端（即将被扩展）时才进行视线检查。这种懒评估策略显著减少了视线检查的次数，同时不损失路径质量，实际运行更快。

### 算法流程

1. **初始化**：将起始节点以 `g=0`、`h=heuristic(start, goal)`、`f=g+h` 放入开放列表优先队列。

2. **主循环**：重复从开放列表中弹出 `f` 值最低的节点。

3. **视线检查（懒父节点重置）**：在扩展节点之前，检查通过其祖父节点直接连接是否能使 `g` 值更低。如果是，更新父节点和 `g` 值。这是 Theta* 的核心步骤。

4. **邻居扩展**：对每个有效邻居（4 或 8 连通）：
   - 计算经由当前节点的暂定 `g` 成本
   - 如果优于邻居现有的 `g`，则更新并加入开放列表

5. **终止**：当目标节点从开放列表弹出时，通过父指针回溯重建路径。

### 视线检查 — Bresenham 算法

视线检查使用改进的 **Bresenham 直线算法** 判断两个网格单元格之间的直线是否无障碍。它同时累加沿线的通行成本，实现更精确的代价感知规划。

---

## 算法公式

### 代价函数（g）

从一个节点移动到邻居节点的总代价：

```
g(neighbor) = g(current)
            + w_euc_cost × 欧几里得距离(current, neighbor)
            + w_traversal_cost × (costmap_cost(current, neighbor) / LETHAL_COST)²
```

### 启发函数（h）

到目标点的可容许启发值（欧几里得距离）：

```
h(neighbor) = w_heuristic_cost × 欧几里得距离(neighbor, goal)
```

其中 `w_heuristic_cost = min(w_euc_cost, 1.0)`，以保证可容许性和一致性。

### 总分（f）

```
f(node) = g(node) + h(node)
```

### 代价缩放

原始代价地图的值（0~252）被重新映射到平滑区间：

```
scaled_cost = 26 + 0.9 × original_cost
```

这避免了零代价单元格，并创建更平缓的势场。

---

## 项目结构

```
ThetaStar_CPP/
├── CMakeLists.txt                    # CMake 构建配置
├── include/theta_star/
│   ├── grid_map.hpp                  # 替代 nav2_costmap_2d::Costmap2D
│   └── theta_star.hpp                 # 算法接口与声明
└── src/
    ├── theta_star.cpp                # 算法完整实现
    └── main.cpp                      # Demo 演示程序
```

### 文件说明

| 文件 | 用途 |
|------|------|
| `grid_map.hpp` | 精简自包含的 `GridMap` 类，复刻 `nav2_costmap_2d::Costmap2D` 接口。提供代价读写、世界/网格坐标转换。 |
| `theta_star.hpp` | Theta* 规划器的公开 API：`ThetaStar` 类、`PlannerConfig` 结构体、坐标类型和常量定义。 |
| `theta_star.cpp` | 完整算法实现：`generatePath()`、`resetParent()`、`setNeighbors()`、`losCheck()`、`backtrace()`。 |
| `main.cpp` | 可运行的演示程序，创建含障碍物的测试地图，运行规划器并以 ASCII 艺术可视化结果。 |

---

## 编译构建

### 环境要求

- C++17 兼容编译器（GCC 9+、Clang 10+、MSVC 2019+）
- CMake 3.10+
- POSIX 线程（Linux/macOS 自动链接；Windows 请使用 MinGW 或 MSVC）

### 构建步骤

```bash
cd ThetaStar_CPP
mkdir build && cd build
cmake ..
cmake --build . --config Release
```

Windows (PowerShell)：

```powershell
cmake .. -G "MinGW Makefiles" -DCMAKE_BUILD_TYPE=Release
mingw32-make
```

### 构建产物

- `libtheta_star_lib.a`（静态库）
- `theta_star_demo`（可执行文件）

---

## 快速上手

### 代码示例

```cpp
#include "theta_star/theta_star.hpp"
#include <vector>
#include <iostream>

int main() {
    // 1. 创建网格地图 (宽=50, 高=50, 分辨率=1.0, 原点=(0,0))
    theta_star::GridMap map(50, 50, 1.0, 0.0, 0.0);

    // 2. 设置障碍物 (代价 >= 254 表示致死障碍)
    for (int i = 10; i <= 15; ++i) {
        for (int j = 10; j <= 30; ++j) {
            map.setCost(i, j, 254);
        }
    }

    // 3. 创建并配置规划器
    theta_star::ThetaStar planner;
    planner.setCostmap(&map);

    theta_star::PlannerConfig config;
    config.w_euc_cost = 1.0;         // 路径长度权重
    config.w_traversal_cost = 2.0;   // 高代价区域惩罚力度
    config.how_many_corners = 8;      // 8 = 8连通搜索, 4 = 4连通搜索
    config.allow_unknown = true;      // 允许穿越未知区域
    planner.setConfig(config);

    // 4. 设置起点和终点 (网格坐标)
    planner.setStartAndGoal(3, 3, 47, 47);
    // 或使用世界坐标:
    // planner.setStartAndGoalWorld(3.0, 3.0, 47.0, 47.0);

    // 5. 执行规划
    std::vector<theta_star::coordsW> path;
    if (planner.generatePath(path)) {
        for (const auto& pt : path) {
            std::cout << "(" << pt.x << ", " << pt.y << ")\n";
        }
    }

    return 0;
}
```

### 运行 Demo

```bash
./theta_star_demo
```

---

## API 参考

### GridMap 类

```cpp
namespace theta_star {

class GridMap {
public:
  GridMap();
  GridMap(unsigned int size_x, unsigned int size_y, double resolution,
          double origin_x = 0.0, double origin_y = 0.0,
          uint8_t default_value = FREE_SPACE);

  uint8_t getCost(unsigned int mx, unsigned int my) const;
  void setCost(unsigned int mx, unsigned int my, uint8_t cost);

  bool worldToMap(double wx, double wy,
                  unsigned int& mx, unsigned int& my) const;
  void mapToWorld(unsigned int mx, unsigned int my,
                  double& wx, double& wy) const;

  unsigned int getSizeInCellsX() const;
  unsigned int getSizeInCellsY() const;
  double getResolution() const;
  double getOriginX() const;
  double getOriginY() const;
};
}
```

### ThetaStar 类

```cpp
namespace theta_star {

struct PlannerConfig {
  double w_traversal_cost = 2.0;       // 通行代价权重
  double w_euc_cost = 1.0;            // 欧几里得距离权重
  int how_many_corners = 8;           // 4 或 8 连通性
  bool allow_unknown = true;          // 允许穿越未知单元格
  int terminal_checking_interval = 5000; // 每 N 个节点检查一次取消
};

struct coordsM { int x, y; };         // 网格坐标
struct coordsW { double x, y; };      // 世界坐标

class ThetaStar {
public:
  ThetaStar();
  ~ThetaStar();

  void setCostmap(GridMap* costmap);

  void setStartAndGoal(int start_x, int start_y,
                       int goal_x, int goal_y);
  void setStartAndGoalWorld(double start_wx, double start_wy,
                            double goal_wx, double goal_wy);

  void setConfig(const PlannerConfig& config);

  bool generatePath(std::vector<coordsW>& raw_path,
                    std::function<bool()> cancel_checker = nullptr);

  bool isUnsafeToPlan() const;
  void clearStart();

  int nodes_opened = 0;  // 扩展的节点数（用于诊断）
};
}
```

### 代价常量

| 常量 | 值 | 含义 |
|------|-------|------|
| `FREE_SPACE` | 0 | 无障碍 |
| `INSCRIBED_INFLATED_OBSTACLE` | 253 | 离障碍物最近的安全距离 |
| `LETHAL_OBSTACLE` | 254 | 单元格为障碍物 |
| `NO_INFORMATION` | 255 | 未知区域 |
| `MAX_NON_OBSTACLE_COST` | 252 | 可通行的最大代价 |

---

## 与其他规划器对比

| 特性 | A* | 跳点搜索 (JPS) | Theta*（本实现） |
|------|-----|-----------------|-------------------|
| **路径质量** | 次优（网格对齐） | 次优 | 近似最优（任意角度） |
| **速度** | 良好 | 在均匀网格上极快 | 良好 |
| **视线检查** | 无 | 无 | 有 |
| **8方向支持** | 有 | 有 | 有 |
| **ROS依赖** | 取决于实现 | 取决于实现 | **无（本实现）** |
| **网格分辨率敏感度** | 高 | 中 | 较低 |

### 为什么要用 Theta* 而不是 A*？

传统 A* 在网格上只能沿 4 或 8 个离散方向移动，在绕障时会产生锯齿状路径。Theta* 通过检查是否存在经过非相邻节点的"捷径"来消除这一问题，无需后处理平滑即可产生平滑的、近似最优的任意角度路径。

### 为什么要用纯 C++？

原始 [nav2_theta_star_planner](https://github.com/SteveMacenski/nav2_theta_star_planner) 是 ROS2 插件，依赖：
- `rclcpp` / `rclcpp_lifecycle`
- `nav2_costmap_2d`
- `nav2_core`
- `pluginlib`
- `tf2_ros`

对于非 ROS 项目（游戏引擎、嵌入式系统、自定义仿真器），这些依赖不切实际。ThetaStar_CPP 仅需：
- 标准 C++ 库（`<vector>`、`<queue>`、`<cmath>` 等）
- POSIX 线程（`<threads>` / `-lpthread`）

---

## 性能指标

算法保持了与原始 ROS2 实现相同的性能特征：

- **规划时间**：在 100×100 网格上规划 87.5m 路径约需 46ms（原始 nav2 基准测试数据）
- **内存占用**：与扩展节点数成正比；有界于网格大小
- **可扩展性**：性能随地图增大而平滑下降；`terminal_checking_interval` 参数允许长时搜索的周期性取消

### 影响性能的因素

1. **地图分辨率** — 越精细的网格扩展节点越多，但路径越平滑
2. **w_traversal_cost** — 值越高对高代价区域惩罚越重，可能扩展更多节点
3. **障碍物数量** — 复杂的障碍物布局会增加视线检查失败次数
4. **连通性（4 vs 8）** — 8 连通搜索略慢但路径质量更好

---

## 许可证

本项目采用 **Apache 许可证 2.0**。详见 [LICENSE](./LICENSE)。

原始 `nav2_theta_star_planner` 由 Steve Macenski 和 Anshumaan Singh 开发，同样采用 Apache 2.0 许可证。
