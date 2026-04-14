# ThetaStar_CPP

**A pure C++ implementation of the Lazy Theta* path planning algorithm, completely free of ROS/ROS2 dependencies.**

[![License](https://img.shields.io/badge/License-Apache--2.0-blue.svg)](./LICENSE)

---

## Table of Contents

- [Overview](#overview)
- [Algorithm Principle](#algorithm-principle)
- [Algorithm Formulas](#algorithm-formulas)
- [Project Structure](#project-structure)
- [Building](#building)
- [Usage](#usage)
- [API Reference](#api-reference)
- [Comparison with Other Planners](#comparison-with-other-planners)
- [Performance](#performance)
- [License](#license)

---

## Overview

**ThetaStar_CPP** is a standalone, header-style C++ library implementing the **Lazy Theta\*** variant of the Theta\* any-angle path planning algorithm. It is a direct refactor of the [nav2_theta_star_planner](https://github.com/SteveMacenski/nav2_theta_star_planner) plugin from the ROS2 Navigation2 framework, with all ROS/ROS2 dependencies removed.

The core algorithm search logic is preserved verbatim, making it a drop-in replacement for any robotics, game development, or simulation project that needs fast any-angle path planning without the overhead of a full ROS installation.

### Key Features

- **Any-Angle Path Planning** — Paths are not constrained to grid-aligned directions
- **Pure C++17** — Zero external dependencies beyond the standard library and pthreads
- **Configurable** — Tunable corner connectivity, cost weights, and unknown-space traversal
- **Portable** — Verified compilation on Linux (GCC/Clang), Windows (MSVC/MinGW), and macOS
- **Header-free** — Clean separation between interface and implementation

---

## Algorithm Principle

### What is Theta\*?

Theta\* (pronounced "theta star") is an any-angle path planning algorithm that combines the A\* search framework with line-of-sight checks. Unlike traditional A\*, which only moves in discrete cardinal or diagonal directions, Theta\* can produce paths that cut through grid cells at arbitrary angles — resulting in significantly shorter and more natural-looking paths.

The core insight is simple: after a node is expanded, Theta\* checks whether a shortcut exists through the node's **grandparent** (the parent of its parent). If the line of sight between the current node and the grandparent is unobstructed, the grandparent becomes the parent instead of the intermediate node. This "propagates the wire" and straightens the path.

### Lazy Theta\* Variant

This implementation uses the **Lazy Theta\*** variant, which defers the line-of-sight (LOS) check until a node is at the top of the priority queue (about to be expanded), rather than checking immediately upon insertion. This lazy evaluation reduces the number of LOS checks without sacrificing path quality, making it faster in practice.

### Algorithm Walkthrough

1. **Initialization**: The start node is inserted into the open priority queue with `g = 0`, `h = heuristic(start, goal)`, and `f = g + h`.

2. **Main Loop**: Repeatedly pop the node with the lowest `f` score from the open list.

3. **Line-of-Sight Check (Lazy Parent Reset)**: Before expanding the node, check if connecting it directly to its grandparent yields a lower `g` cost. If so, update the parent and `g` value. This is the defining step of Theta\*.

4. **Neighbor Expansion**: For each valid neighbor (4 or 8 connectivity):
   - Calculate tentative `g` cost through the current node
   - If this is better than the neighbor's existing `g`, update it and push onto the open list

5. **Termination**: When the goal node is popped from the open list, backtrace through parent pointers to reconstruct the path.

### Line-of-Sight Check — Bresenham's Algorithm

The LOS check uses a modified **Bresenham's line algorithm** to determine whether a straight line between two grid cells is obstacle-free. It simultaneously accumulates the traversal cost along the line, enabling more accurate cost-aware planning.

---

## Algorithm Formulas

### Cost Function

The total cost of traversing from the current node to a neighbor is:

```
g(neighbor) = g(current)
            + w_euc_cost × EuclideanDistance(current, neighbor)
            + w_traversal_cost × (costmap_cost(current, neighbor) / LETHAL_COST)²
```

### Heuristic Function

The admissible heuristic is the Euclidean distance to the goal:

```
h(neighbor) = w_heuristic_cost × EuclideanDistance(neighbor, goal)
```

where `w_heuristic_cost = min(w_euc_cost, 1.0)` to ensure admissibility.

### Total Score

```
f(node) = g(node) + h(node)
```

### Cost Scaling

The original costmap cost (0–252) is remapped to a smoothed range:

```
scaled_cost = 26 + 0.9 × original_cost
```

This prevents zero-cost cells and creates a gentler potential field.

---

## Project Structure

```
ThetaStar_CPP/
├── CMakeLists.txt                    # Build configuration
├── include/theta_star/
│   ├── grid_map.hpp                  # Pure C++ costmap replacement
│   └── theta_star.hpp                # Algorithm interface & declarations
└── src/
    ├── theta_star.cpp                # Algorithm implementation
    └── main.cpp                      # Demo application
```

### File Descriptions

| File | Purpose |
|------|---------|
| `grid_map.hpp` | A minimal, self-contained `GridMap` class that replicates the interface of `nav2_costmap_2d::Costmap2D`. Provides cost reading/writing and world/map coordinate conversion. |
| `theta_star.hpp` | The public API of the Theta\* planner: `ThetaStar` class, `PlannerConfig` struct, coordinate types, and constants. |
| `theta_star.cpp` | The full algorithm implementation: `generatePath()`, `resetParent()`, `setNeighbors()`, `losCheck()`, `backtrace()`. |
| `main.cpp` | A runnable demo that creates a test map with obstacles, runs the planner, and visualizes the result in ASCII art. |

---

## Building

### Prerequisites

- A C++17-compatible compiler (GCC 9+, Clang 10+, MSVC 2019+)
- CMake 3.10+
- POSIX threads (automatically linked on Linux/macOS; use MinGW or MSVC on Windows)

### Build Steps

```bash
cd ThetaStar_CPP
mkdir build && cd build
cmake ..
cmake --build . --config Release
```

On Windows (PowerShell):

```powershell
cmake .. -G "MinGW Makefiles" -DCMAKE_BUILD_TYPE=Release
mingw32-make
```

### Build Output

After a successful build, you will have:

- `libtheta_star_lib.a` (static library)
- `theta_star_demo` (executable)

---

## Usage

### Quick Start

```cpp
#include "theta_star/theta_star.hpp"
#include <vector>

int main() {
    // 1. Create a grid map (width=50, height=50, resolution=1.0, origin=(0,0))
    theta_star::GridMap map(50, 50, 1.0, 0.0, 0.0);

    // 2. Set some obstacles (cost >= 254 means lethal obstacle)
    for (int i = 10; i <= 15; ++i) {
        for (int j = 10; j <= 30; ++j) {
            map.setCost(i, j, 254);
        }
    }

    // 3. Create and configure the planner
    theta_star::ThetaStar planner;
    planner.setCostmap(&map);

    theta_star::PlannerConfig config;
    config.w_euc_cost = 1.0;
    config.w_traversal_cost = 2.0;
    config.how_many_corners = 8;   // 8 = 8-connected, 4 = 4-connected
    config.allow_unknown = true;
    planner.setConfig(config);

    // 4. Set start and goal (grid coordinates or world coordinates)
    planner.setStartAndGoal(3, 3, 47, 47);
    // Or in world coordinates:
    // planner.setStartAndGoalWorld(3.0, 3.0, 47.0, 47.0);

    // 5. Plan
    std::vector<coordsW> path;
    if (planner.generatePath(path)) {
        // path[i].x and path[i].y are world coordinates
        for (const auto& pt : path) {
            printf("(%.2f, %.2f)\n", pt.x, pt.y);
        }
    }

    return 0;
}
```

### Running the Demo

```bash
./theta_star_demo
```

Expected output (ASCII map with obstacles `#`, path `*`):

```
============================================
  Lazy Theta* Path Planner - Pure C++ Demo
============================================

  Map (size: 30x30):
  '#' = obstacle, '+' = cost, '.' = free, '*' = path

  ...

  Path waypoints (N points):
  [0] (3.50, 3.50)  dist_from_prev: 0
  [1] (5.50, 5.50)  dist_from_prev: 2.83
  ...
  Total path length: 65.23
```

---

## API Reference

### GridMap Class

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

### ThetaStar Class

```cpp
namespace theta_star {

struct PlannerConfig {
  double w_traversal_cost = 2.0;       // Weight on traversal cost
  double w_euc_cost = 1.0;             // Weight on Euclidean distance
  int how_many_corners = 8;            // 4 or 8 connectivity
  bool allow_unknown = true;           // Allow planning through unknown cells
  int terminal_checking_interval = 5000; // Check cancel every N nodes
};

struct coordsM { int x, y; };          // Map coordinates
struct coordsW { double x, y; };       // World coordinates

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

  int nodes_opened = 0;  // Number of nodes expanded (for diagnostics)
};
}
```

### Cost Constants

| Constant | Value | Meaning |
|----------|-------|---------|
| `FREE_SPACE` | 0 | No obstacle |
| `INSCRIBED_INFLATED_OBSTACLE` | 253 | Closest safe distance to obstacle |
| `LETHAL_OBSTACLE` | 254 | Cell is an obstacle |
| `NO_INFORMATION` | 255 | Unknown cell |
| `MAX_NON_OBSTACLE_COST` | 252 | Maximum cost considered traversable |

---

## Comparison with Other Planners

| Feature | A\* | Jump Point Search | Theta\* (this) |
|---------|-----|-------------------|-----------------|
| **Path Quality** | Suboptimal (grid-aligned) | Suboptimal | Near-optimal (any-angle) |
| **Speed** | Good | Excellent on uniform grids | Good |
| **Line-of-Sight** | No | No | Yes |
| **8-Direction Support** | Yes | Yes | Yes |
| **ROS Dependency** | Depends on impl | Depends on impl | **None (this impl)** |
| **Grid Resolution Sensitivity** | High | Medium | Lower |

### Why Theta\* over A\*?

Traditional A\* on a grid can only move in 4 or 8 discrete directions, resulting in paths that锯齿状 (zig-zag) when navigating around obstacles. Theta\* eliminates this by checking if a "shortcut" exists through non-adjacent cells, producing smooth, near-optimal any-angle paths without post-processing smoothing.

### Why Pure C++?

The original [nav2_theta_star_planner](https://github.com/SteveMacenski/nav2_theta_star_planner) is a ROS2 plugin that depends on:
- `rclcpp` / `rclcpp_lifecycle`
- `nav2_costmap_2d`
- `nav2_core`
- `pluginlib`
- `tf2_ros`

For non-ROS projects (game engines, embedded systems, custom simulators), these dependencies are impractical. ThetaStar_CPP provides the same algorithm with only:
- Standard C++ library (`<vector>`, `<queue>`, `<cmath>`, etc.)
- POSIX threads (`<threads>` / `-lpthread`)

---

## Performance

The algorithm maintains the same performance characteristics as the original ROS2 implementation:

- **Planning Time**: ~46ms for an 87.5m path on a 100×100 grid (as reported in original nav2 benchmarks)
- **Memory**: Proportional to the number of expanded nodes; bounded by grid size
- **Scalability**: Performance degrades gracefully with map size; `terminal_checking_interval` parameter allows periodic cancellation for long-running searches

### Factors Affecting Performance

1. **Map Resolution** — Finer grids expand more nodes but produce smoother paths
2. **w_traversal_cost** — Higher values penalize high-cost regions more, potentially expanding more nodes
3. **Number of Obstacles** — Complex obstacle layouts increase LOS check failures
4. **Connectivity (4 vs 8)** — 8-connected search is slightly slower but produces better paths

---

## License

This project is licensed under the **Apache License 2.0**. See [LICENSE](./LICENSE) for details.

The original `nav2_theta_star_planner` is also Apache 2.0 licensed by Steve Macenski and Anshumaan Singh.
