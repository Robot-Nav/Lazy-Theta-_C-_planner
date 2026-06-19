#ifndef THETA_STAR_KINODYNAMIC_OPTIMIZER_HPP_
#define THETA_STAR_KINODYNAMIC_OPTIMIZER_HPP_

#include <vector>
#include <cmath>
#include <algorithm>
#include <functional>

#include "theta_star/theta_star.hpp"
#include "theta_star/dubins_curve.hpp"

namespace theta_star
{

struct KinodynamicConfig
{
  double max_linear_velocity = 1.0;
  double max_linear_acceleration = 0.5;
  double max_lateral_acceleration = 0.4;
  double max_angular_velocity = 1.0;
  double max_angular_acceleration = 0.5;
  double min_turning_radius = 0.3;
  double wheel_base = 0.3;
  double dynamic_feasibility_speed = 0.6;

  bool enable_smoothing = true;
  double resample_interval = 0.08;
  int smoothing_iterations = 90;
  double optimizer_step_size = 0.18;
  double smoothness_weight = 0.55;
  double reference_weight = 0.35;
  double curvature_continuity_weight = 0.15;
  bool enforce_collision_check = true;
  int shortcut_max_passes = 2;
  int bspline_degree = 3;
  double smoothing_tolerance = 0.12;

  bool enable_dubins_smoothing = true;
  double dubins_sample_step = 0.05;
  double dubins_max_detour_ratio = 1.3;
  int dubins_max_skip_points = 2;

  double bezier_control_dist_factor = 0.35;
  double bezier_control_dist_min = 0.06;
  double bezier_control_dist_min_radius_ratio = 0.9;
  double bezier_control_dist_min_radius_floor = 0.12;
  double bezier_tangent_direction_threshold = -0.15;
  double bezier_heading_blend_factor = 0.55;

  enum RobotType {
    DIFFERENTIAL_DRIVE,
    ACKERMANN,
    OMNI_DIRECTIONAL
  };
  RobotType robot_type = DIFFERENTIAL_DRIVE;
};

struct OptimizedPathPoint
{
  double x, y;
  double heading;
  double curvature;
};

struct PathQualityMetrics
{
  double optimization_time_ms = 0.0;
  double raw_path_length = 0.0;
  double optimized_path_length = 0.0;
  double smoothness_score = 0.0;

  double max_curvature = 0.0;
  double max_curvature_rate = 0.0;
  double effective_min_turning_radius = 0.0;
  double path_feasibility_speed = 0.0;

  double constraint_satisfaction_ratio = 0.0;
  int total_constraints_checked = 0;
  int violated_constraints = 0;

  int dubins_segments_used = 0;
  double dubins_time_ms = 0.0;
};

class KinodynamicOptimizer
{
public:
  KinodynamicOptimizer();

  void setConfig(const KinodynamicConfig& config);

  bool optimizePath(
    const std::vector<coordsW>& raw_path,
    std::vector<OptimizedPathPoint>& optimized_path,
    double initial_heading = 0.0,
    double goal_heading = 0.0,
    const std::function<bool(double, double)>& state_validator = nullptr);

  const PathQualityMetrics& getLastMetrics() const
  {
    return last_metrics_;
  }

private:
  KinodynamicConfig config_;
  PathQualityMetrics last_metrics_;
  DubinsCurve dubins_solver_;

  void resetMetrics();

  void resamplePath(
    const std::vector<coordsW>& path,
    std::vector<coordsW>& resampled_path) const;

  void shortcutPath(
    const std::vector<coordsW>& path,
    std::vector<coordsW>& shortcut_path,
    const std::function<bool(double, double)>& state_validator) const;

  void applyDubinsSmoothing(
    const std::vector<coordsW>& shortcut_path,
    const std::vector<double>& headings,
    std::vector<coordsW>& dubins_path,
    const std::function<bool(double, double)>& state_validator);

  void smoothPath(
    const std::vector<coordsW>& reference_path,
    std::vector<coordsW>& smooth_path,
    const std::function<bool(double, double)>& state_validator);

  double computeCurvatureFromThreePoints(
    const coordsW& p0,
    const coordsW& p1,
    const coordsW& p2) const;

  bool isSegmentValid(
    const coordsW& p0,
    const coordsW& p1,
    const std::function<bool(double, double)>& state_validator) const;

  bool satisfyTurningRadiusConstraint(
    const coordsW& p_prev,
    const coordsW& p_curr,
    const coordsW& p_next) const;

  double interpolateAngle(double from, double to, double ratio) const;

  void computeHeadings(
    const std::vector<coordsW>& smooth_path,
    std::vector<double>& headings,
    double initial_heading = 0.0,
    double goal_heading = 0.0);

  void computeCurvatures(
    const std::vector<coordsW>& smooth_path,
    const std::vector<double>& headings,
    std::vector<double>& curvatures);

  void enforceCurvatureRateLimit(
    const std::vector<coordsW>& smooth_path,
    std::vector<double>& curvatures) const;

  bool checkKinematicConstraints(
    const OptimizedPathPoint& point) const;

  void generateOptimizedPath(
    const std::vector<coordsW>& smooth_path,
    const std::vector<double>& headings,
    const std::vector<double>& curvatures,
    std::vector<OptimizedPathPoint>& optimized_path);

  void evaluatePathQuality(
    const std::vector<coordsW>& raw_path,
    const std::vector<OptimizedPathPoint>& optimized_path,
    double optimization_time_ms,
    PathQualityMetrics& metrics) const;

  void accumulateConstraintStatistics(
    const std::vector<OptimizedPathPoint>& optimized_path,
    PathQualityMetrics& metrics) const;

  double computePathLength(const std::vector<coordsW>& path) const;

  double computePathLength(const std::vector<OptimizedPathPoint>& path) const;

  double getPathFeasibilitySpeed() const;

  double getEffectiveMinTurningRadius() const;

  inline double computeDistance(const coordsW& p1, const coordsW& p2) const
  {
    return std::hypot(p2.x - p1.x, p2.y - p1.y);
  }

  inline double normalizeAngle(double angle) const
  {
    while (angle > M_PI) angle -= 2 * M_PI;
    while (angle < -M_PI) angle += 2 * M_PI;
    return angle;
  }

  inline double computeSteeringAngle(double curvature) const
  {
    return std::atan(config_.wheel_base * curvature);
  }

  bool checkTurningRadius(double curvature) const
  {
    if (std::abs(curvature) < 1e-10) return true;
    const double turning_radius = 1.0 / std::abs(curvature);
    return turning_radius >= getEffectiveMinTurningRadius();
  }

  double limitCurvature(double curvature) const
  {
    const double max_curvature = 1.0 / getEffectiveMinTurningRadius();
    return std::clamp(curvature, -max_curvature, max_curvature);
  }
};

}  // namespace theta_star

#endif  // THETA_STAR_KINODYNAMIC_OPTIMIZER_HPP_
