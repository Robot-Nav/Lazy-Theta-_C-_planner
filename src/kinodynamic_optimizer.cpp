#include "theta_star/kinodynamic_optimizer.hpp"

#include <algorithm>
#include <chrono>
#include <limits>

namespace theta_star
{

KinodynamicOptimizer::KinodynamicOptimizer()
{
}

void KinodynamicOptimizer::setConfig(const KinodynamicConfig& config)
{
  config_ = config;
  dubins_solver_.setTurningRadius(getEffectiveMinTurningRadius());
}

void KinodynamicOptimizer::resetMetrics()
{
  last_metrics_ = PathQualityMetrics();
}

double KinodynamicOptimizer::getPathFeasibilitySpeed() const
{
  const double vmax = std::max(config_.max_linear_velocity, 0.05);
  const double speed = (config_.dynamic_feasibility_speed > 1e-6) ?
    config_.dynamic_feasibility_speed : 0.6 * vmax;
  return std::clamp(speed, 0.05, vmax);
}

double KinodynamicOptimizer::getEffectiveMinTurningRadius() const
{
  const double kinematic_radius = std::max(config_.min_turning_radius, 1e-3);

  const double v_ref = getPathFeasibilitySpeed();
  double dynamic_radius = 0.0;

  if (config_.max_lateral_acceleration > 1e-6) {
    dynamic_radius = std::max(dynamic_radius,
      (v_ref * v_ref) / config_.max_lateral_acceleration);
  }

  if (config_.max_angular_velocity > 1e-6) {
    dynamic_radius = std::max(dynamic_radius, v_ref / config_.max_angular_velocity);
  }

  return std::max(kinematic_radius, dynamic_radius);
}

double KinodynamicOptimizer::computePathLength(const std::vector<coordsW>& path) const
{
  if (path.size() < 2) {
    return 0.0;
  }

  double length = 0.0;
  for (size_t i = 1; i < path.size(); ++i) {
    length += computeDistance(path[i - 1], path[i]);
  }
  return length;
}

double KinodynamicOptimizer::computePathLength(const std::vector<OptimizedPathPoint>& path) const
{
  if (path.size() < 2) {
    return 0.0;
  }

  double length = 0.0;
  for (size_t i = 1; i < path.size(); ++i) {
    const double dx = path[i].x - path[i - 1].x;
    const double dy = path[i].y - path[i - 1].y;
    length += std::hypot(dx, dy);
  }
  return length;
}

void KinodynamicOptimizer::resamplePath(
  const std::vector<coordsW>& path,
  std::vector<coordsW>& resampled_path) const
{
  resampled_path.clear();
  if (path.empty()) {
    return;
  }

  if (path.size() == 1) {
    resampled_path = path;
    return;
  }

  const double interval = std::max(config_.resample_interval, 1e-3);
  resampled_path.push_back(path.front());

  double carry = 0.0;
  for (size_t i = 1; i < path.size(); ++i) {
    coordsW seg_start = path[i - 1];
    const coordsW seg_end = path[i];

    double seg_len = computeDistance(seg_start, seg_end);
    if (seg_len < 1e-9) {
      continue;
    }

    const double ux = (seg_end.x - seg_start.x) / seg_len;
    const double uy = (seg_end.y - seg_start.y) / seg_len;
    double distance_on_segment = 0.0;

    while (carry + (seg_len - distance_on_segment) >= interval) {
      const double step = interval - carry;
      distance_on_segment += step;

      coordsW p;
      p.x = seg_start.x + ux * distance_on_segment;
      p.y = seg_start.y + uy * distance_on_segment;

      if (computeDistance(resampled_path.back(), p) > 1e-6) {
        resampled_path.push_back(p);
      }

      carry = 0.0;
    }

    carry += (seg_len - distance_on_segment);
  }

  if (computeDistance(resampled_path.back(), path.back()) > 1e-6) {
    resampled_path.push_back(path.back());
  }

  if (resampled_path.size() < 2) {
    resampled_path = path;
  }
}

void KinodynamicOptimizer::shortcutPath(
  const std::vector<coordsW>& path,
  std::vector<coordsW>& shortcut_path,
  const std::function<bool(double, double)>& state_validator) const
{
  if (path.size() < 3) {
    shortcut_path = path;
    return;
  }

  std::vector<coordsW> current = path;
  const int passes = std::max(0, config_.shortcut_max_passes);

  for (int pass = 0; pass < passes; ++pass) {
    std::vector<coordsW> simplified;
    simplified.reserve(current.size());
    simplified.push_back(current.front());

    size_t i = 0;
    while (i + 1 < current.size()) {
      size_t best_next = i + 1;

      for (size_t j = current.size() - 1; j > i + 1; --j) {
        if (!isSegmentValid(current[i], current[j], state_validator)) {
          continue;
        }

        if (simplified.size() >= 2) {
          const coordsW& p_prev = simplified[simplified.size() - 2];
          if (!satisfyTurningRadiusConstraint(p_prev, current[i], current[j])) {
            continue;
          }
        }

        if (j + 1 < current.size() &&
          !satisfyTurningRadiusConstraint(current[i], current[j], current[j + 1]))
        {
          continue;
        }

        best_next = j;
        break;
      }

      simplified.push_back(current[best_next]);
      i = best_next;
    }

    if (simplified.size() >= current.size()) {
      break;
    }

    current.swap(simplified);
  }

  shortcut_path = current;
}

void KinodynamicOptimizer::applyDubinsSmoothing(
  const std::vector<coordsW>& shortcut_path,
  const std::vector<double>& headings,
  std::vector<coordsW>& dubins_path,
  const std::function<bool(double, double)>& state_validator)
{
  dubins_path.clear();
  if (shortcut_path.size() < 2 || headings.size() != shortcut_path.size()) {
    dubins_path = shortcut_path;
    last_metrics_.dubins_segments_used = 0;
    return;
  }

  dubins_solver_.setTurningRadius(getEffectiveMinTurningRadius());
  const double step = std::clamp(config_.dubins_sample_step, 0.01, 0.1);
  const double max_detour = std::max(config_.dubins_max_detour_ratio, 1.0);
  const int max_skip = std::clamp(config_.dubins_max_skip_points, 1, 4);
  const double max_heading_gap = M_PI / 2.0;
  const double ccc_penalty = 0.18;
  const double mixed_turn_penalty = 0.08;
  const double continuity_penalty = 0.12;
  const double skip_bonus = 0.03;

  int dubins_segments = 0;
  double prev_end_heading = headings[0];

  auto appendPoints = [this](
                        const std::vector<coordsW>& in_points,
                        std::vector<coordsW>& out_points) {
    if (in_points.empty()) {
      return;
    }

    size_t start_idx = 0;
    if (!out_points.empty() &&
        computeDistance(out_points.back(), in_points.front()) <= 1e-6)
    {
      start_idx = 1;
    }

    for (size_t k = start_idx; k < in_points.size(); ++k) {
      out_points.push_back(in_points[k]);
    }
  };

  auto sampleDubinsSegment = [&](const DubinsPoint& start,
                                 const DubinsPath& path,
                                 std::vector<coordsW>& out_points) {
    out_points.clear();

    std::vector<DubinsSamplePoint> samples;
    bool ok = false;
    if (state_validator) {
      ok = dubins_solver_.sampleAndValidate(start, path, step, state_validator, samples);
    } else {
      samples = dubins_solver_.samplePath(start, path, step);
      ok = !samples.empty();
    }

    if (!ok || samples.empty()) {
      return false;
    }

    out_points.reserve(samples.size());
    for (const auto& sp : samples) {
      out_points.push_back({sp.x, sp.y});
    }

    for (size_t k = 1; k < out_points.size(); ++k) {
      if (!isSegmentValid(out_points[k - 1], out_points[k], state_validator)) {
        return false;
      }
    }

    for (size_t k = 1; k + 1 < out_points.size(); ++k) {
      if (!satisfyTurningRadiusConstraint(out_points[k - 1], out_points[k], out_points[k + 1])) {
        return false;
      }
    }

    return true;
  };

  auto tryForwardBezierFallback = [&](const coordsW& p0,
                                      const coordsW& p1,
                                      double heading0,
                                      double heading1,
                                      std::vector<coordsW>& out_points) {
    out_points.clear();

    const double dist = computeDistance(p0, p1);
    if (dist < 1e-6) {
      return false;
    }

    const double min_radius = getEffectiveMinTurningRadius();
    const double control_dist = std::clamp(config_.bezier_control_dist_factor * dist,
      config_.bezier_control_dist_min,
      std::max(config_.bezier_control_dist_min_radius_floor,
               config_.bezier_control_dist_min_radius_ratio * min_radius));

    const coordsW c1{
      p0.x + control_dist * std::cos(heading0),
      p0.y + control_dist * std::sin(heading0)};
    const coordsW c2{
      p1.x - control_dist * std::cos(heading1),
      p1.y - control_dist * std::sin(heading1)};

    const int sample_count = std::max(8, static_cast<int>(std::ceil(dist / step)) + 2);
    out_points.reserve(sample_count);

    for (int s = 0; s < sample_count; ++s) {
      const double t = (sample_count <= 1) ? 1.0 : static_cast<double>(s) / (sample_count - 1);
      const double one_minus_t = 1.0 - t;
      const double b0 = one_minus_t * one_minus_t * one_minus_t;
      const double b1 = 3.0 * one_minus_t * one_minus_t * t;
      const double b2 = 3.0 * one_minus_t * t * t;
      const double b3 = t * t * t;

      coordsW point;
      point.x = b0 * p0.x + b1 * c1.x + b2 * c2.x + b3 * p1.x;
      point.y = b0 * p0.y + b1 * c1.y + b2 * c2.y + b3 * p1.y;
      out_points.push_back(point);
    }

    double bezier_length = 0.0;
    for (size_t k = 1; k < out_points.size(); ++k) {
      if (!isSegmentValid(out_points[k - 1], out_points[k], state_validator)) {
        return false;
      }

      bezier_length += computeDistance(out_points[k - 1], out_points[k]);
    }

    if (bezier_length > max_detour * dist) {
      return false;
    }

    for (size_t k = 1; k + 1 < out_points.size(); ++k) {
      if (!satisfyTurningRadiusConstraint(out_points[k - 1], out_points[k], out_points[k + 1])) {
        return false;
      }

      const coordsW& prev = out_points[k - 1];
      const coordsW& next = out_points[k + 1];
      const double tangent = std::atan2(next.y - prev.y, next.x - prev.x);
      const double ratio = static_cast<double>(k) / (out_points.size() - 1);
      const double expected = interpolateAngle(heading0, heading1, ratio);
      if (std::cos(normalizeAngle(tangent - expected)) < config_.bezier_tangent_direction_threshold) {
        return false;
      }
    }

    return true;
  };

  size_t i = 0;
  while (i < shortcut_path.size() - 1) {
    const coordsW& p_start = shortcut_path[i];

    size_t best_j = i + 1;
    double best_score = std::numeric_limits<double>::infinity();
    std::vector<coordsW> best_samples;
    bool use_dubins = false;

    for (int skip = 1; skip <= max_skip; ++skip) {
      const size_t j = i + static_cast<size_t>(skip);
      if (j >= shortcut_path.size()) {
        break;
      }

      const coordsW& p_end = shortcut_path[j];
      const double euc_dist = computeDistance(p_start, p_end);
      if (euc_dist < 1e-6) {
        continue;
      }

      const double geom_heading = std::atan2(p_end.y - p_start.y, p_end.x - p_start.x);
      const double blend = std::clamp(
        euc_dist / std::max(2.0 * getEffectiveMinTurningRadius(), 1e-3), 0.0, 1.0);

      const double start_heading = interpolateAngle(geom_heading, headings[i], blend);
      const double end_heading = interpolateAngle(geom_heading, headings[j], blend);

      if (!dubins_path.empty()) {
        const double heading_gap = std::abs(normalizeAngle(start_heading - prev_end_heading));
        if (heading_gap > max_heading_gap) {
          continue;
        }
      }

      DubinsPoint dp_start{p_start.x, p_start.y, start_heading};
      DubinsPoint dp_end{p_end.x, p_end.y, end_heading};
      DubinsPath dpath = dubins_solver_.shortestPath(dp_start, dp_end);
      if (!dpath.valid || dpath.total_length > max_detour * euc_dist) {
        continue;
      }

      std::vector<coordsW> samples;
      if (!sampleDubinsSegment(dp_start, dpath, samples)) {
        continue;
      }

      double score = dpath.total_length;
      if (dpath.type == DubinsPath::RLR || dpath.type == DubinsPath::LRL) {
        score += ccc_penalty * euc_dist;
      } else if (dpath.type == DubinsPath::LSR || dpath.type == DubinsPath::RSL) {
        score += mixed_turn_penalty * euc_dist;
      }

      const double heading_gap = std::abs(normalizeAngle(end_heading - headings[j]));
      score += continuity_penalty * heading_gap * getEffectiveMinTurningRadius();
      score -= skip_bonus * static_cast<double>(skip) * euc_dist;

      if (score < best_score) {
        best_score = score;
        best_j = j;
        best_samples.swap(samples);
        use_dubins = true;
      }
    }

    if (!use_dubins) {
      std::vector<coordsW> bezier_samples;
      const size_t j = i + 1;
      const coordsW& p_end = shortcut_path[j];
      const double geom_heading = std::atan2(p_end.y - p_start.y, p_end.x - p_start.x);
      const double blend = config_.bezier_heading_blend_factor;
      const double start_heading = interpolateAngle(geom_heading, headings[i], blend);
      const double end_heading = interpolateAngle(geom_heading, headings[j], blend);

      if (tryForwardBezierFallback(p_start, p_end, start_heading, end_heading, bezier_samples)) {
        best_samples.swap(bezier_samples);
        best_j = j;
      }
    } else {
      ++dubins_segments;
    }

    if (!best_samples.empty()) {
      appendPoints(best_samples, dubins_path);
      if (best_samples.size() >= 2) {
        const coordsW& last_prev = best_samples[best_samples.size() - 2];
        const coordsW& last = best_samples.back();
        prev_end_heading = std::atan2(last.y - last_prev.y, last.x - last_prev.x);
      } else {
        prev_end_heading = headings[best_j];
      }
      i = best_j;
      continue;
    }

    if (dubins_path.empty() ||
        computeDistance(dubins_path.back(), shortcut_path[i]) > 1e-6)
    {
      dubins_path.push_back(shortcut_path[i]);
    }
    prev_end_heading = headings[i];
    ++i;
  }

  if (dubins_path.empty() ||
      computeDistance(dubins_path.back(), shortcut_path.back()) > 1e-6) {
    dubins_path.push_back(shortcut_path.back());
  }

  last_metrics_.dubins_segments_used = dubins_segments;
}

void KinodynamicOptimizer::smoothPath(
  const std::vector<coordsW>& reference_path,
  std::vector<coordsW>& smooth_path,
  const std::function<bool(double, double)>& state_validator)
{
  if (reference_path.size() < 3) {
    smooth_path = reference_path;
    return;
  }

  smooth_path = reference_path;

  const int n = static_cast<int>(smooth_path.size());
  const int iterations = std::max(1, config_.smoothing_iterations);
  const double step_size = std::clamp(config_.optimizer_step_size, 0.01, 0.5);
  const double max_deviation = std::max(config_.smoothing_tolerance, 0.03);

  const double w_smooth = std::max(0.0, config_.smoothness_weight);
  const double w_ref = std::max(0.0, config_.reference_weight);
  const double w_curv = std::max(0.0, config_.curvature_continuity_weight);

  const double convergence_thresh = std::max(config_.resample_interval * 0.02, 1e-4);

  for (int iter = 0; iter < iterations; ++iter) {
    double max_update = 0.0;
    for (int i = 1; i < n - 1; ++i) {
      const coordsW& p_prev = smooth_path[i - 1];
      const coordsW& p_curr = smooth_path[i];
      const coordsW& p_next = smooth_path[i + 1];
      const coordsW& p_ref = reference_path[i];

      const coordsW laplacian{
        p_prev.x + p_next.x - 2.0 * p_curr.x,
        p_prev.y + p_next.y - 2.0 * p_curr.y};

      const coordsW reference_pull{
        p_ref.x - p_curr.x,
        p_ref.y - p_curr.y};

      coordsW jerk_term{0.0, 0.0};
      if (i >= 2 && i + 2 < n) {
        const coordsW& p_prev2 = smooth_path[i - 2];
        const coordsW& p_next2 = smooth_path[i + 2];

        const coordsW d2_prev{
          p_prev2.x - 2.0 * p_prev.x + p_curr.x,
          p_prev2.y - 2.0 * p_prev.y + p_curr.y};

        const coordsW d2_curr{
          p_prev.x - 2.0 * p_curr.x + p_next.x,
          p_prev.y - 2.0 * p_curr.y + p_next.y};

        const coordsW d2_next{
          p_curr.x - 2.0 * p_next.x + p_next2.x,
          p_curr.y - 2.0 * p_next.y + p_next2.y};

        jerk_term.x = d2_prev.x - 2.0 * d2_curr.x + d2_next.x;
        jerk_term.y = d2_prev.y - 2.0 * d2_curr.y + d2_next.y;
      }

      coordsW candidate{
        p_curr.x + step_size * (w_smooth * laplacian.x + w_ref * reference_pull.x + w_curv * jerk_term.x),
        p_curr.y + step_size * (w_smooth * laplacian.y + w_ref * reference_pull.y + w_curv * jerk_term.y)};

      double dev_x = candidate.x - p_ref.x;
      double dev_y = candidate.y - p_ref.y;
      double dev_norm = std::hypot(dev_x, dev_y);
      if (dev_norm > max_deviation && dev_norm > 1e-9) {
        const double scale = max_deviation / dev_norm;
        candidate.x = p_ref.x + dev_x * scale;
        candidate.y = p_ref.y + dev_y * scale;
      }

      bool accepted = false;
      for (int attempt = 0; attempt < 5; ++attempt) {
        const bool turn_ok = satisfyTurningRadiusConstraint(p_prev, candidate, p_next);
        const bool seg_prev_ok = isSegmentValid(p_prev, candidate, state_validator);
        const bool seg_next_ok = isSegmentValid(candidate, p_next, state_validator);

        if (turn_ok && seg_prev_ok && seg_next_ok) {
          accepted = true;
          break;
        }

        const coordsW midpoint{
          0.5 * (p_prev.x + p_next.x),
          0.5 * (p_prev.y + p_next.y)};
        candidate.x = 0.5 * (candidate.x + midpoint.x);
        candidate.y = 0.5 * (candidate.y + midpoint.y);

        dev_x = candidate.x - p_ref.x;
        dev_y = candidate.y - p_ref.y;
        dev_norm = std::hypot(dev_x, dev_y);
        if (dev_norm > max_deviation && dev_norm > 1e-9) {
          const double scale = max_deviation / dev_norm;
          candidate.x = p_ref.x + dev_x * scale;
          candidate.y = p_ref.y + dev_y * scale;
        }
      }

      if (accepted) {
        max_update = std::max(max_update, computeDistance(p_curr, candidate));
        smooth_path[i] = candidate;
      }
    }

    if (max_update < convergence_thresh) {
      break;
    }
  }

  for (int pass = 0; pass < 3; ++pass) {
    for (int i = 1; i < n - 1; ++i) {
      if (!satisfyTurningRadiusConstraint(smooth_path[i - 1], smooth_path[i], smooth_path[i + 1])) {
        const coordsW midpoint{
          0.5 * (smooth_path[i - 1].x + smooth_path[i + 1].x),
          0.5 * (smooth_path[i - 1].y + smooth_path[i + 1].y)};

        if (isSegmentValid(smooth_path[i - 1], midpoint, state_validator) &&
            isSegmentValid(midpoint, smooth_path[i + 1], state_validator) &&
            satisfyTurningRadiusConstraint(smooth_path[i - 1], midpoint, smooth_path[i + 1]))
        {
          smooth_path[i] = midpoint;
          continue;
        }

        const coordsW& ref = reference_path[i];
        if (isSegmentValid(smooth_path[i - 1], ref, state_validator) &&
            isSegmentValid(ref, smooth_path[i + 1], state_validator) &&
            satisfyTurningRadiusConstraint(smooth_path[i - 1], ref, smooth_path[i + 1]))
        {
          smooth_path[i] = ref;
        }
      }
    }
  }

  for (int i = 1; i < n - 1; ++i) {
    if (!isSegmentValid(smooth_path[i - 1], smooth_path[i], state_validator) ||
        !isSegmentValid(smooth_path[i], smooth_path[i + 1], state_validator))
    {
      const coordsW& ref = reference_path[i];
      if (isSegmentValid(smooth_path[i - 1], ref, state_validator) &&
          isSegmentValid(ref, smooth_path[i + 1], state_validator))
      {
        smooth_path[i] = ref;
      }
    }
  }
}

bool KinodynamicOptimizer::optimizePath(
  const std::vector<coordsW>& raw_path,
  std::vector<OptimizedPathPoint>& optimized_path,
  double initial_heading,
  double goal_heading,
  const std::function<bool(double, double)>& state_validator)
{
  resetMetrics();
  if (raw_path.size() < 2) {
    optimized_path.clear();
    return false;
  }

  const auto start_time = std::chrono::steady_clock::now();

  std::vector<coordsW> reference_path;
  resamplePath(raw_path, reference_path);

  if (reference_path.size() < 2) {
    optimized_path.clear();
    return false;
  }

  std::vector<coordsW> shortcut_path;
  shortcutPath(reference_path, shortcut_path, state_validator);

  std::vector<double> pre_headings;
  computeHeadings(shortcut_path, pre_headings, initial_heading, goal_heading);

  std::vector<coordsW> dubins_path;
  if (config_.enable_dubins_smoothing && shortcut_path.size() >= 2) {
    const auto dubins_start = std::chrono::steady_clock::now();
    applyDubinsSmoothing(shortcut_path, pre_headings, dubins_path, state_validator);
    const auto dubins_end = std::chrono::steady_clock::now();
    last_metrics_.dubins_time_ms =
      std::chrono::duration_cast<std::chrono::microseconds>(dubins_end - dubins_start).count() / 1000.0;
  } else {
    dubins_path = shortcut_path;
    last_metrics_.dubins_segments_used = 0;
    last_metrics_.dubins_time_ms = 0.0;
  }

  std::vector<coordsW> smooth_path;
  if (config_.enable_smoothing && dubins_path.size() >= 3) {
    smoothPath(dubins_path, smooth_path, state_validator);
  } else {
    smooth_path = dubins_path;
  }

  if (smooth_path.size() < 2) {
    optimized_path.clear();
    return false;
  }

  smooth_path.front() = raw_path.front();
  smooth_path.back() = raw_path.back();

  if (config_.enforce_collision_check && state_validator) {
    bool valid_after_smooth = true;
    for (size_t i = 1; i < smooth_path.size(); ++i) {
      if (!isSegmentValid(smooth_path[i - 1], smooth_path[i], state_validator)) {
        valid_after_smooth = false;
        break;
      }
    }
    if (!valid_after_smooth) {
      smooth_path = dubins_path;
      smooth_path.front() = raw_path.front();
      smooth_path.back() = raw_path.back();
    }
  }

  std::vector<double> headings;
  computeHeadings(smooth_path, headings, initial_heading, goal_heading);

  std::vector<double> curvatures;
  computeCurvatures(smooth_path, headings, curvatures);
  enforceCurvatureRateLimit(smooth_path, curvatures);

  generateOptimizedPath(smooth_path, headings, curvatures, optimized_path);

  const auto end_time = std::chrono::steady_clock::now();
  const double optimization_time_ms =
    std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count() / 1000.0;

  evaluatePathQuality(raw_path, optimized_path, optimization_time_ms, last_metrics_);

  return !optimized_path.empty();
}

double KinodynamicOptimizer::computeCurvatureFromThreePoints(
  const coordsW& p0,
  const coordsW& p1,
  const coordsW& p2) const
{
  const double a = computeDistance(p0, p1);
  const double b = computeDistance(p1, p2);
  const double c = computeDistance(p0, p2);

  if (a < 1e-6 || b < 1e-6 || c < 1e-6) {
    return 0.0;
  }

  const double cross =
    (p1.x - p0.x) * (p2.y - p0.y) -
    (p1.y - p0.y) * (p2.x - p0.x);

  return (2.0 * cross) / (a * b * c);
}

bool KinodynamicOptimizer::isSegmentValid(
  const coordsW& p0,
  const coordsW& p1,
  const std::function<bool(double, double)>& state_validator) const
{
  if (!config_.enforce_collision_check || !state_validator) {
    return true;
  }

  const double distance = computeDistance(p0, p1);
  const double sample_step = std::clamp(config_.resample_interval * 0.25, 0.01, 0.05);
  const int steps = std::max(2, static_cast<int>(std::ceil(distance / sample_step)));

  for (int i = 0; i <= steps; ++i) {
    const double t = static_cast<double>(i) / steps;
    const double x = p0.x + t * (p1.x - p0.x);
    const double y = p0.y + t * (p1.y - p0.y);
    if (!state_validator(x, y)) {
      return false;
    }
  }

  return true;
}

bool KinodynamicOptimizer::satisfyTurningRadiusConstraint(
  const coordsW& p_prev,
  const coordsW& p_curr,
  const coordsW& p_next) const
{
  const double min_radius = getEffectiveMinTurningRadius();
  if (min_radius <= 1e-6) {
    return true;
  }

  const double abs_k = std::abs(computeCurvatureFromThreePoints(p_prev, p_curr, p_next));
  const double max_k = 1.0 / min_radius;
  if (abs_k > max_k + 1e-4) {
    return false;
  }

  const double ds_prev = computeDistance(p_prev, p_curr);
  const double ds_next = computeDistance(p_curr, p_next);
  const double ds = 0.5 * (ds_prev + ds_next);
  if (ds < 1e-6) {
    return true;
  }

  const double yaw_prev = std::atan2(p_curr.y - p_prev.y, p_curr.x - p_prev.x);
  const double yaw_next = std::atan2(p_next.y - p_curr.y, p_next.x - p_curr.x);
  const double dyaw = std::abs(normalizeAngle(yaw_next - yaw_prev));

  return dyaw <= (ds / min_radius + 1e-3);
}

double KinodynamicOptimizer::interpolateAngle(double from, double to, double ratio) const
{
  const double t = std::clamp(ratio, 0.0, 1.0);
  const double delta = normalizeAngle(to - from);
  return normalizeAngle(from + t * delta);
}

void KinodynamicOptimizer::computeHeadings(
  const std::vector<coordsW>& smooth_path,
  std::vector<double>& headings,
  double initial_heading,
  double goal_heading)
{
  headings.clear();
  headings.resize(smooth_path.size());

  if (smooth_path.size() < 2) return;

  headings[0] = std::atan2(
    smooth_path[1].y - smooth_path[0].y,
    smooth_path[1].x - smooth_path[0].x);

  for (size_t i = 1; i + 1 < smooth_path.size(); ++i) {
    const double dx = smooth_path[i + 1].x - smooth_path[i - 1].x;
    const double dy = smooth_path[i + 1].y - smooth_path[i - 1].y;
    headings[i] = std::atan2(dy, dx);
  }

  headings.back() = std::atan2(
    smooth_path.back().y - smooth_path[smooth_path.size() - 2].y,
    smooth_path.back().x - smooth_path[smooth_path.size() - 2].x);

  const int window_size = 5;
  std::vector<double> smoothed_headings = headings;

  for (int pass = 0; pass < 3; ++pass) {
    for (size_t i = 0; i < headings.size(); ++i) {
      double sum_sin = 0.0;
      double sum_cos = 0.0;
      double sum_weight = 0.0;

      for (int j = -window_size; j <= window_size; ++j) {
        const int idx = static_cast<int>(i) + j;
        if (idx < 0 || idx >= static_cast<int>(headings.size())) continue;

        const double weight = static_cast<double>(window_size + 1 - std::abs(j));
        sum_sin += std::sin(smoothed_headings[idx]) * weight;
        sum_cos += std::cos(smoothed_headings[idx]) * weight;
        sum_weight += weight;
      }

      if (sum_weight > 1e-6) {
        headings[i] = std::atan2(sum_sin, sum_cos);
      }
    }

    if (pass < 2) {
      smoothed_headings = headings;
    }
  }

  const double max_heading_change_per_step = M_PI / 4.0;
  for (size_t i = 1; i < headings.size(); ++i) {
    double delta = normalizeAngle(headings[i] - headings[i - 1]);
    if (std::abs(delta) > max_heading_change_per_step) {
      delta = std::copysign(max_heading_change_per_step, delta);
      headings[i] = normalizeAngle(headings[i - 1] + delta);
    }
  }

  if (!headings.empty()) {
    const int n = static_cast<int>(headings.size());
    const int start_transition_len = std::max(5, static_cast<int>(n * 0.2));
    for (int i = 0; i < start_transition_len && i < n; ++i) {
      const double t =
        start_transition_len <= 1 ? 1.0 : static_cast<double>(i) / (start_transition_len - 1);
      const double blend = t * t * (3.0 - 2.0 * t);
      headings[i] = interpolateAngle(initial_heading, headings[i], blend);
    }
    headings.front() = initial_heading;

    const int end_transition_len = std::max(5, static_cast<int>(n * 0.2));
    const int start_idx = std::max(0, n - end_transition_len);
    for (int i = start_idx; i < n; ++i) {
      const double t =
        end_transition_len <= 1 ? 1.0 : static_cast<double>(i - start_idx) / (end_transition_len - 1);
      const double blend = t * t * (3.0 - 2.0 * t);
      headings[i] = interpolateAngle(headings[i], goal_heading, blend);
    }
    headings.back() = goal_heading;
  }
}

void KinodynamicOptimizer::computeCurvatures(
  const std::vector<coordsW>& smooth_path,
  const std::vector<double>& headings,
  std::vector<double>& curvatures)
{
  (void)headings;
  curvatures.clear();
  curvatures.resize(smooth_path.size(), 0.0);

  if (smooth_path.size() < 3) return;

  for (size_t i = 1; i < smooth_path.size() - 1; ++i) {
    curvatures[i] = limitCurvature(computeCurvatureFromThreePoints(
      smooth_path[i - 1], smooth_path[i], smooth_path[i + 1]));
  }

  if (!curvatures.empty() && curvatures.size() >= 2) {
    curvatures.front() = curvatures[1];
    curvatures.back() = curvatures[curvatures.size() - 2];
  }

  for (int pass = 0; pass < 3; ++pass) {
    std::vector<double> tmp = curvatures;
    for (size_t i = 1; i + 1 < curvatures.size(); ++i) {
      tmp[i] = 0.25 * curvatures[i - 1] + 0.5 * curvatures[i] + 0.25 * curvatures[i + 1];
    }
    curvatures = tmp;

    for (auto& k : curvatures) {
      k = limitCurvature(k);
    }
  }
}

void KinodynamicOptimizer::enforceCurvatureRateLimit(
  const std::vector<coordsW>& smooth_path,
  std::vector<double>& curvatures) const
{
  if (smooth_path.size() < 2 || curvatures.size() != smooth_path.size()) {
    return;
  }

  const double v_ref = getPathFeasibilitySpeed();
  const double v2 = std::max(v_ref * v_ref, 1e-4);
  const double max_dk_ds =
    (config_.max_angular_acceleration > 1e-6) ?
    config_.max_angular_acceleration / v2 : std::numeric_limits<double>::infinity();

  if (!std::isfinite(max_dk_ds)) {
    return;
  }

  for (int pass = 0; pass < 4; ++pass) {
    for (size_t i = 1; i < curvatures.size(); ++i) {
      const double ds = std::max(computeDistance(smooth_path[i - 1], smooth_path[i]), 1e-4);
      const double max_delta = max_dk_ds * ds;
      const double delta = curvatures[i] - curvatures[i - 1];

      if (std::abs(delta) > max_delta) {
        curvatures[i] = curvatures[i - 1] + std::copysign(max_delta, delta);
      }
    }

    for (int i = static_cast<int>(curvatures.size()) - 2; i >= 0; --i) {
      const double ds = std::max(computeDistance(smooth_path[i], smooth_path[i + 1]), 1e-4);
      const double max_delta = max_dk_ds * ds;
      const double delta = curvatures[i] - curvatures[i + 1];

      if (std::abs(delta) > max_delta) {
        curvatures[i] = curvatures[i + 1] + std::copysign(max_delta, delta);
      }
    }

    for (double& k : curvatures) {
      k = limitCurvature(k);
    }
  }
}

void KinodynamicOptimizer::generateOptimizedPath(
  const std::vector<coordsW>& smooth_path,
  const std::vector<double>& headings,
  const std::vector<double>& curvatures,
  std::vector<OptimizedPathPoint>& optimized_path)
{
  optimized_path.clear();

  if (smooth_path.empty()) {
    return;
  }

  const size_t n = smooth_path.size();
  optimized_path.resize(n);

  for (size_t i = 0; i < n; ++i) {
    optimized_path[i].x = smooth_path[i].x;
    optimized_path[i].y = smooth_path[i].y;
    optimized_path[i].heading = headings[i];
    optimized_path[i].curvature = curvatures[i];
  }
}

bool KinodynamicOptimizer::checkKinematicConstraints(
  const OptimizedPathPoint& point) const
{
  const double abs_k = std::abs(point.curvature);
  if (!checkTurningRadius(point.curvature)) {
    return false;
  }

  const double v_ref = getPathFeasibilitySpeed();
  if (abs_k * v_ref > config_.max_angular_velocity + 1e-6) {
    return false;
  }

  const double lateral_accel = v_ref * v_ref * abs_k;
  if (lateral_accel > config_.max_lateral_acceleration + 1e-6) {
    return false;
  }

  if (config_.robot_type == KinodynamicConfig::ACKERMANN) {
    const double steering_angle = std::abs(computeSteeringAngle(point.curvature));
    if (steering_angle > (M_PI / 4.0 + 1e-6)) {
      return false;
    }
  }

  return true;
}

void KinodynamicOptimizer::accumulateConstraintStatistics(
  const std::vector<OptimizedPathPoint>& optimized_path,
  PathQualityMetrics& metrics) const
{
  metrics.total_constraints_checked = 0;
  metrics.violated_constraints = 0;

  const double v_ref = getPathFeasibilitySpeed();
  const double v2 = std::max(v_ref * v_ref, 1e-4);
  const double max_dk_ds =
    (config_.max_angular_acceleration > 1e-6) ?
    config_.max_angular_acceleration / v2 : std::numeric_limits<double>::infinity();

  for (size_t i = 0; i < optimized_path.size(); ++i) {
    const OptimizedPathPoint& pt = optimized_path[i];
    metrics.max_curvature = std::max(metrics.max_curvature, std::abs(pt.curvature));

    ++metrics.total_constraints_checked;
    if (!checkKinematicConstraints(pt)) {
      ++metrics.violated_constraints;
    }

    if (i == 0) {
      continue;
    }

    const double ds = std::max(computeDistance(
      {optimized_path[i - 1].x, optimized_path[i - 1].y},
      {optimized_path[i].x, optimized_path[i].y}), 1e-4);
    const double dk_ds = std::abs(pt.curvature - optimized_path[i - 1].curvature) / ds;
    metrics.max_curvature_rate = std::max(metrics.max_curvature_rate, dk_ds);

    ++metrics.total_constraints_checked;
    if (std::isfinite(max_dk_ds) && dk_ds > max_dk_ds + 1e-6) {
      ++metrics.violated_constraints;
    }
  }

  if (metrics.total_constraints_checked > 0) {
    metrics.constraint_satisfaction_ratio =
      1.0 - static_cast<double>(metrics.violated_constraints) / metrics.total_constraints_checked;
  } else {
    metrics.constraint_satisfaction_ratio = 1.0;
  }
}

void KinodynamicOptimizer::evaluatePathQuality(
  const std::vector<coordsW>& raw_path,
  const std::vector<OptimizedPathPoint>& optimized_path,
  double optimization_time_ms,
  PathQualityMetrics& metrics) const
{
  metrics.optimization_time_ms = optimization_time_ms;
  metrics.raw_path_length = computePathLength(raw_path);
  metrics.optimized_path_length = computePathLength(optimized_path);
  metrics.path_feasibility_speed = getPathFeasibilitySpeed();
  metrics.effective_min_turning_radius = getEffectiveMinTurningRadius();

  if (optimized_path.size() >= 2) {
    double mean_abs_curvature = 0.0;
    double mean_delta_curvature = 0.0;

    for (size_t i = 0; i < optimized_path.size(); ++i) {
      mean_abs_curvature += std::abs(optimized_path[i].curvature);
      if (i > 0) {
        mean_delta_curvature += std::abs(optimized_path[i].curvature - optimized_path[i - 1].curvature);
      }
    }

    mean_abs_curvature /= optimized_path.size();
    mean_delta_curvature /= (optimized_path.size() - 1);

    const double radius_scale = std::max(metrics.effective_min_turning_radius, 1e-3);
    const double norm_curv = mean_abs_curvature * radius_scale;
    const double norm_dcurv = mean_delta_curvature * radius_scale;
    const double penalty = 0.6 * norm_curv + 0.4 * norm_dcurv;
    metrics.smoothness_score = std::clamp(100.0 * std::exp(-2.0 * penalty), 0.0, 100.0);
  } else {
    metrics.smoothness_score = 0.0;
  }

  accumulateConstraintStatistics(optimized_path, metrics);
}

}  // namespace theta_star
