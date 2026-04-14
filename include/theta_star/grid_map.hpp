#ifndef THETA_STAR_GRID_MAP_HPP_
#define THETA_STAR_GRID_MAP_HPP_

#include <vector>
#include <cmath>
#include <cstdint>
#include <stdexcept>

namespace theta_star
{

static constexpr uint8_t NO_INFORMATION = 255;
static constexpr uint8_t LETHAL_OBSTACLE = 254;
static constexpr uint8_t INSCRIBED_INFLATED_OBSTACLE = 253;
static constexpr uint8_t MAX_NON_OBSTACLE_COST = 252;
static constexpr uint8_t FREE_SPACE = 0;

class GridMap
{
public:
  GridMap()
  : size_x_(0), size_y_(0), resolution_(0.0), origin_x_(0.0), origin_y_(0.0)
  {
  }

  GridMap(unsigned int size_x, unsigned int size_y, double resolution,
          double origin_x = 0.0, double origin_y = 0.0, uint8_t default_value = FREE_SPACE)
  : size_x_(size_x), size_y_(size_y), resolution_(resolution),
    origin_x_(origin_x), origin_y_(origin_y),
    data_(size_x * size_y, default_value)
  {
  }

  uint8_t getCost(unsigned int mx, unsigned int my) const
  {
    return data_[my * size_x_ + mx];
  }

  uint8_t getCost(int mx, int my) const
  {
    return data_[static_cast<size_t>(my) * size_x_ + static_cast<size_t>(mx)];
  }

  void setCost(unsigned int mx, unsigned int my, uint8_t cost)
  {
    data_[my * size_x_ + mx] = cost;
  }

  void setCost(int mx, int my, uint8_t cost)
  {
    data_[static_cast<size_t>(my) * size_x_ + static_cast<size_t>(mx)] = cost;
  }

  bool worldToMap(double wx, double wy, unsigned int & mx, unsigned int & my) const
  {
    if (wx < origin_x_ || wy < origin_y_) {
      return false;
    }
    mx = static_cast<unsigned int>((wx - origin_x_) / resolution_);
    my = static_cast<unsigned int>((wy - origin_y_) / resolution_);
    if (mx >= size_x_ || my >= size_y_) {
      return false;
    }
    return true;
  }

  void mapToWorld(unsigned int mx, unsigned int my, double & wx, double & wy) const
  {
    wx = origin_x_ + (mx + 0.5) * resolution_;
    wy = origin_y_ + (my + 0.5) * resolution_;
  }

  void mapToWorld(int mx, int my, double & wx, double & wy) const
  {
    wx = origin_x_ + (static_cast<double>(mx) + 0.5) * resolution_;
    wy = origin_y_ + (static_cast<double>(my) + 0.5) * resolution_;
  }

  unsigned int getSizeInCellsX() const { return size_x_; }
  unsigned int getSizeInCellsY() const { return size_y_; }
  double getResolution() const { return resolution_; }
  double getOriginX() const { return origin_x_; }
  double getOriginY() const { return origin_y_; }

  void resize(unsigned int size_x, unsigned int size_y, double resolution,
              double origin_x = 0.0, double origin_y = 0.0, uint8_t default_value = FREE_SPACE)
  {
    size_x_ = size_x;
    size_y_ = size_y;
    resolution_ = resolution;
    origin_x_ = origin_x;
    origin_y_ = origin_y;
    data_.assign(size_x * size_y, default_value);
  }

  uint8_t * getData() { return data_.data(); }
  const uint8_t * getData() const { return data_.data(); }

private:
  unsigned int size_x_;
  unsigned int size_y_;
  double resolution_;
  double origin_x_;
  double origin_y_;
  std::vector<uint8_t> data_;
};

}  // namespace theta_star

#endif  // THETA_STAR_GRID_MAP_HPP_
