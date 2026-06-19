#include <iostream>
#include <iomanip>
#include <chrono>
#include <vector>
#include "theta_star/theta_star.hpp"

void printMap(const theta_star::GridMap & map, const std::vector<coordsW> & path)
{
  int size_x = static_cast<int>(map.getSizeInCellsX());
  int size_y = static_cast<int>(map.getSizeInCellsY());

  std::vector<std::vector<char>> display(size_y, std::vector<char>(size_x, '.'));

  for (int y = 0; y < size_y; y++) {
    for (int x = 0; x < size_x; x++) {
      uint8_t cost = map.getCost(x, y);
      if (cost >= 254) {
        display[y][x] = '#';
      } else if (cost > 0) {
        display[y][x] = '+';
      }
    }
  }

  for (const auto & pt : path) {
    int mx = static_cast<int>((pt.x - map.getOriginX()) / map.getResolution());
    int my = static_cast<int>((pt.y - map.getOriginY()) / map.getResolution());
    if (mx >= 0 && mx < size_x && my >= 0 && my < size_y) {
      display[my][mx] = '*';
    }
  }

  std::cout << "\n  Map (size: " << size_x << "x" << size_y << "):\n";
  std::cout << "  '#' = obstacle, '+' = cost, '.' = free, '*' = path\n\n";

  for (int y = size_y - 1; y >= 0; y--) {
    std::cout << "  ";
    for (int x = 0; x < size_x; x++) {
      std::cout << display[y][x] << ' ';
    }
    std::cout << "\n";
  }
  std::cout << "\n";
}

void printPath(const std::vector<coordsW> & path)
{
  std::cout << "  Path waypoints (" << path.size() << " points):\n";
  std::cout << "  " << std::fixed << std::setprecision(3);
  for (size_t i = 0; i < path.size(); i++) {
    std::cout << "  [" << i << "] (" << path[i].x << ", " << path[i].y << ")";
    if (i > 0) {
      double dist = std::hypot(path[i].x - path[i - 1].x, path[i].y - path[i - 1].y);
      std::cout << "  dist_from_prev: " << dist;
    }
    std::cout << "\n";
  }

  double total_dist = 0.0;
  for (size_t i = 1; i < path.size(); i++) {
    total_dist += std::hypot(path[i].x - path[i - 1].x, path[i].y - path[i - 1].y);
  }
  std::cout << "  Total path length: " << total_dist << "\n\n";
}

int main()
{
  std::cout << "============================================\n";
  std::cout << "  Lazy Theta* Path Planner - Pure C++ Demo\n";
  std::cout << "============================================\n\n";

  const unsigned int MAP_SIZE_X = 30;
  const unsigned int MAP_SIZE_Y = 30;
  const double RESOLUTION = 1.0;

  theta_star::GridMap map(MAP_SIZE_X, MAP_SIZE_Y, RESOLUTION, 0.0, 0.0);

  for (unsigned int i = 8; i <= 14; i++) {
    for (unsigned int j = 5; j <= 22; j++) {
      map.setCost(i, j, 254);
    }
  }

  for (unsigned int i = 20; i <= 26; i++) {
    for (unsigned int j = 8; j <= 25; j++) {
      map.setCost(i, j, 254);
    }
  }

  for (unsigned int i = 12; i <= 20; i++) {
    for (unsigned int j = 18; j <= 22; j++) {
      map.setCost(i, j, 254);
    }
  }

  theta_star::ThetaStar planner;
  planner.setCostmap(&map);

  theta_star::PlannerConfig config;
  config.w_euc_cost = 1.0;
  config.w_traversal_cost = 2.0;
  config.how_many_corners = 8;
  config.allow_unknown = true;
  config.terminal_checking_interval = 5000;
  planner.setConfig(config);

  int start_x = 3, start_y = 3;
  int goal_x = 27, goal_y = 27;

  planner.setStartAndGoal(start_x, start_y, goal_x, goal_y);

  std::cout << "  Start: (" << start_x << ", " << start_y << ")\n";
  std::cout << "  Goal:  (" << goal_x << ", " << goal_y << ")\n";
  std::cout << "  Config: w_euc=" << config.w_euc_cost
            << ", w_traversal=" << config.w_traversal_cost
            << ", corners=" << config.how_many_corners << "\n\n";

  if (planner.isUnsafeToPlan()) {
    std::cerr << "  ERROR: Start or goal is on an obstacle!\n";
    return 1;
  }

  std::vector<coordsW> path;

  auto start_time = std::chrono::steady_clock::now();
  bool found = planner.generatePath(path);
  auto end_time = std::chrono::steady_clock::now();

  auto duration_us = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);

  if (found) {
    std::cout << "  Path found!\n";
    std::cout << "  Planning time: " << duration_us.count() << " us ("
              << duration_us.count() / 1000.0 << " ms)\n";
    std::cout << "  Nodes opened: " << planner.nodes_opened << "\n\n";
    printPath(path);
    printMap(map, path);
  } else {
    std::cout << "  No path found!\n";
    std::cout << "  Nodes opened: " << planner.nodes_opened << "\n";
    printMap(map, {});
  }

  std::cout << "============================================\n";
  std::cout << "  Demo 2: World coordinate input\n";
  std::cout << "============================================\n\n";

  theta_star::GridMap map2(50, 50, 0.5, -5.0, -5.0);

  for (unsigned int i = 15; i <= 30; i++) {
    for (unsigned int j = 10; j <= 40; j++) {
      map2.setCost(i, j, 254);
    }
  }

  theta_star::ThetaStar planner2;
  planner2.setCostmap(&map2);
  planner2.setConfig(config);

  double start_wx = -3.0, start_wy = -3.0;
  double goal_wx = 15.0, goal_wy = 15.0;

  planner2.setStartAndGoalWorld(start_wx, start_wy, goal_wx, goal_wy);

  std::cout << "  Start (world): (" << start_wx << ", " << start_wy << ")\n";
  std::cout << "  Goal  (world): (" << goal_wx << ", " << goal_wy << ")\n";
  std::cout << "  Start (map):   (" << planner2.src_.x << ", " << planner2.src_.y << ")\n";
  std::cout << "  Goal  (map):   (" << planner2.dst_.x << ", " << planner2.dst_.y << ")\n\n";

  std::vector<coordsW> path2;
  start_time = std::chrono::steady_clock::now();
  found = planner2.generatePath(path2);
  end_time = std::chrono::steady_clock::now();
  duration_us = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);

  if (found) {
    std::cout << "  Path found!\n";
    std::cout << "  Planning time: " << duration_us.count() << " us ("
              << duration_us.count() / 1000.0 << " ms)\n";
    std::cout << "  Nodes opened: " << planner2.nodes_opened << "\n\n";
    printPath(path2);
  } else {
    std::cout << "  No path found!\n";
    std::cout << "  Nodes opened: " << planner2.nodes_opened << "\n";
  }

  return 0;
}
