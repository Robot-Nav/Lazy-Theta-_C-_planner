#ifndef THETA_STAR_DUBINS_CURVE_HPP_
#define THETA_STAR_DUBINS_CURVE_HPP_

#include <vector>
#include <cmath>
#include <algorithm>
#include <functional>
#include <limits>

namespace theta_star
{

struct DubinsPoint
{
  double x, y, theta;
};

struct DubinsPath
{
  enum Type
  {
    LSL = 0,
    LSR = 1,
    RSL = 2,
    RSR = 3,
    RLR = 4,
    LRL = 5
  };

  Type type;
  double param[3];
  double total_length;
  double cost;
  bool valid;
};

struct DubinsSamplePoint
{
  double x, y, theta, curvature;
  double arc_length;
};

class DubinsCurve
{
public:
  DubinsCurve(double turning_radius = 0.5)
    : turning_radius_(std::max(turning_radius, 1e-3))
  {
  }

  void setTurningRadius(double r)
  {
    turning_radius_ = std::max(r, 1e-3);
  }

  double getTurningRadius() const { return turning_radius_; }

  DubinsPath shortestPath(
    const DubinsPoint& start,
    const DubinsPoint& goal) const
  {
    double alpha, beta, d;
    toStandardForm(start, goal, alpha, beta, d);

    if (d < 1e-6) {
      DubinsPath result;
      result.valid = false;
      return result;
    }

    if (d < 1.0) {
      return shortestPathShortDistance(alpha, beta, d, start, goal);
    }

    DubinsPath candidates[6];
    computeLSL(alpha, beta, d, candidates[0]);
    computeLSR(alpha, beta, d, candidates[1]);
    computeRSL(alpha, beta, d, candidates[2]);
    computeRSR(alpha, beta, d, candidates[3]);
    computeRLR(alpha, beta, d, candidates[4]);
    computeLRL(alpha, beta, d, candidates[5]);

    DubinsPath best;
    best.valid = false;
    best.cost = std::numeric_limits<double>::infinity();
    best.total_length = std::numeric_limits<double>::infinity();

    for (int i = 0; i < 6; ++i) {
      if (!candidates[i].valid) continue;

      double candidate_cost = candidates[i].total_length;
      if (candidates[i].type == DubinsPath::RLR ||
          candidates[i].type == DubinsPath::LRL) {
        candidate_cost *= 1.15;
      }

      if (candidate_cost < best.cost) {
        best = candidates[i];
        best.cost = candidate_cost;
      }
    }

    return best;
  }

  std::vector<DubinsSamplePoint> samplePath(
    const DubinsPoint& start,
    const DubinsPath& path,
    double step_size = 0.05) const
  {
    std::vector<DubinsSamplePoint> result;
    if (!path.valid) return result;

    const double d = path.total_length;
    if (d < 1e-9) return result;

    const int num_samples = std::max(2, static_cast<int>(std::ceil(d / step_size)) + 1);

    for (int i = 0; i < num_samples; ++i) {
      const double t = (i == num_samples - 1) ? d : (static_cast<double>(i) / (num_samples - 1)) * d;
      DubinsSamplePoint sp;
      sp.arc_length = t;
      computePointOnPath(start, path, t, sp.x, sp.y, sp.theta, sp.curvature);
      result.push_back(sp);
    }

    return result;
  }

  bool sampleAndValidate(
    const DubinsPoint& start,
    const DubinsPath& path,
    double step_size,
    const std::function<bool(double, double)>& state_validator,
    std::vector<DubinsSamplePoint>& out_samples) const
  {
    out_samples = samplePath(start, path, step_size);
    if (out_samples.empty()) return false;

    for (const auto& sp : out_samples) {
      if (!state_validator(sp.x, sp.y)) {
        return false;
      }
    }
    return true;
  }

  void computePointOnPath(
    const DubinsPoint& start,
    const DubinsPath& path,
    double arc_length,
    double& x, double& y, double& theta, double& curvature) const
  {
    const double R = turning_radius_;
    const double seg_lengths[3] = {path.param[0] * R, path.param[1] * R, path.param[2] * R};

    double cx = start.x;
    double cy = start.y;
    double ctheta = start.theta;
    curvature = 0.0;

    double remaining = arc_length;

    int dir[3] = {0, 0, 0};
    switch (path.type) {
      case DubinsPath::LSL: dir[0]=1;  dir[1]=0;  dir[2]=1;  break;
      case DubinsPath::LSR: dir[0]=1;  dir[1]=0;  dir[2]=-1; break;
      case DubinsPath::RSL: dir[0]=-1; dir[1]=0;  dir[2]=1;  break;
      case DubinsPath::RSR: dir[0]=-1; dir[1]=0;  dir[2]=-1; break;
      case DubinsPath::RLR: dir[0]=-1; dir[1]=1;  dir[2]=-1; break;
      case DubinsPath::LRL: dir[0]=1;  dir[1]=-1; dir[2]=1;  break;
    }

    for (int i = 0; i < 3; ++i) {
      if (remaining <= 1e-12) break;

      const double seg_len = seg_lengths[i];
      if (seg_len < 1e-12) continue;

      const double consumed = std::min(remaining, seg_len);

      if (dir[i] == 0) {
        cx += std::cos(ctheta) * consumed;
        cy += std::sin(ctheta) * consumed;
        curvature = 0.0;
      } else {
        const double d = static_cast<double>(dir[i]);
        const double delta_theta = d * consumed / R;
        const double new_theta = ctheta + delta_theta;

        const double circle_cx = cx - d * R * std::sin(ctheta);
        const double circle_cy = cy + d * R * std::cos(ctheta);

        cx = circle_cx + d * R * std::sin(new_theta);
        cy = circle_cy - d * R * std::cos(new_theta);
        ctheta = new_theta;
        curvature = d / R;
      }

      remaining -= consumed;
    }

    x = cx;
    y = cy;
    theta = normalizeAngle(ctheta);
  }

private:
  double turning_radius_;

  void toStandardForm(
    const DubinsPoint& start,
    const DubinsPoint& goal,
    double& alpha, double& beta, double& d) const
  {
    const double dx = goal.x - start.x;
    const double dy = goal.y - start.y;
    const double D = std::hypot(dx, dy);
    d = D / turning_radius_;

    const double theta = std::atan2(dy, dx);
    alpha = mod2pi(start.theta - theta);
    beta = mod2pi(goal.theta - theta);
  }

  inline double mod2pi(double angle) const
  {
    const double two_pi = 2.0 * M_PI;
    angle = std::fmod(angle, two_pi);
    if (angle < 0.0) angle += two_pi;
    return angle;
  }

  inline double normalizeAngle(double angle) const
  {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
  }

  DubinsPath shortestPathShortDistance(
    double alpha, double beta, double d,
    const DubinsPoint& start, const DubinsPoint& goal) const
  {
    DubinsPath candidates[6];
    computeLSL(alpha, beta, d, candidates[0]);
    computeLSR(alpha, beta, d, candidates[1]);
    computeRSL(alpha, beta, d, candidates[2]);
    computeRSR(alpha, beta, d, candidates[3]);
    computeRLR(alpha, beta, d, candidates[4]);
    computeLRL(alpha, beta, d, candidates[5]);

    DubinsPath best;
    best.valid = false;
    best.cost = std::numeric_limits<double>::infinity();
    best.total_length = std::numeric_limits<double>::infinity();

    for (int i = 0; i < 6; ++i) {
      if (!candidates[i].valid) continue;

      double cost = candidates[i].total_length;

      if (candidates[i].type == DubinsPath::RLR ||
          candidates[i].type == DubinsPath::LRL) {
        cost *= 2.0;
      }

      if (candidates[i].type == DubinsPath::LSR ||
          candidates[i].type == DubinsPath::RSL) {
        cost *= 1.3;
      }

      if (cost < best.cost) {
        best = candidates[i];
        best.cost = cost;
      }
    }

    if (!best.valid) {
      return best;
    }

    const double euc_dist = d * turning_radius_;
    if (best.total_length > euc_dist * 3.0) {
      DubinsPath straight_path;
      straight_path.type = DubinsPath::LSL;
      straight_path.param[0] = 0.0;
      straight_path.param[1] = d;
      straight_path.param[2] = 0.0;
      straight_path.total_length = euc_dist;
      straight_path.cost = euc_dist;
      straight_path.valid = true;
      return straight_path;
    }

    return best;
  }

  void computeLSL(double alpha, double beta, double d, DubinsPath& result) const
  {
    const double sa = std::sin(alpha);
    const double ca = std::cos(alpha);
    const double sb = std::sin(beta);
    const double cb = std::cos(beta);

    const double p_sq = 2.0 + d * d - 2.0 * ca * cb + 2.0 * d * (sa - sb);
    if (p_sq < 0) {
      result.valid = false;
      return;
    }

    const double tmp = std::atan2(cb - ca, d + sa - sb);
    const double t = mod2pi(-alpha + tmp);
    const double p = std::sqrt(std::max(0.0, p_sq));
    const double q = mod2pi(beta - tmp);

    if (t < -1e-10 || p < -1e-10 || q < -1e-10) {
      result.valid = false;
      return;
    }

    result.type = DubinsPath::LSL;
    result.param[0] = std::max(0.0, t);
    result.param[1] = std::max(0.0, p);
    result.param[2] = std::max(0.0, q);
    result.total_length = (result.param[0] + result.param[1] + result.param[2]) * turning_radius_;
    result.cost = result.total_length;
    result.valid = true;
  }

  void computeLSR(double alpha, double beta, double d, DubinsPath& result) const
  {
    const double sa = std::sin(alpha);
    const double ca = std::cos(alpha);
    const double sb = std::sin(beta);
    const double cb = std::cos(beta);

    const double p_sq = -2.0 + d * d + 2.0 * ca * cb + 2.0 * d * (sa + sb);
    if (p_sq < 0) {
      result.valid = false;
      return;
    }

    const double p = std::sqrt(std::max(0.0, p_sq));
    const double tmp = std::atan2(-ca - cb, d + sa + sb) -
      std::atan2(-2.0, p);
    const double t = mod2pi(-alpha + tmp);
    const double q = mod2pi(-beta + tmp);

    if (t < -1e-10 || p < -1e-10 || q < -1e-10) {
      result.valid = false;
      return;
    }

    result.type = DubinsPath::LSR;
    result.param[0] = std::max(0.0, t);
    result.param[1] = std::max(0.0, p);
    result.param[2] = std::max(0.0, q);
    result.total_length = (result.param[0] + result.param[1] + result.param[2]) * turning_radius_;
    result.cost = result.total_length;
    result.valid = true;
  }

  void computeRSL(double alpha, double beta, double d, DubinsPath& result) const
  {
    const double sa = std::sin(alpha);
    const double ca = std::cos(alpha);
    const double sb = std::sin(beta);
    const double cb = std::cos(beta);

    const double p_sq = -2.0 + d * d + 2.0 * ca * cb - 2.0 * d * (sa + sb);
    if (p_sq < 0) {
      result.valid = false;
      return;
    }

    const double p = std::sqrt(std::max(0.0, p_sq));
    const double tmp = std::atan2(ca + cb, d - sa - sb) -
      std::atan2(2.0, p);
    const double t = mod2pi(alpha - tmp);
    const double q = mod2pi(beta - tmp);

    if (t < -1e-10 || p < -1e-10 || q < -1e-10) {
      result.valid = false;
      return;
    }

    result.type = DubinsPath::RSL;
    result.param[0] = std::max(0.0, t);
    result.param[1] = std::max(0.0, p);
    result.param[2] = std::max(0.0, q);
    result.total_length = (result.param[0] + result.param[1] + result.param[2]) * turning_radius_;
    result.cost = result.total_length;
    result.valid = true;
  }

  void computeRSR(double alpha, double beta, double d, DubinsPath& result) const
  {
    const double sa = std::sin(alpha);
    const double ca = std::cos(alpha);
    const double sb = std::sin(beta);
    const double cb = std::cos(beta);

    const double p_sq = 2.0 + d * d - 2.0 * ca * cb - 2.0 * d * (sa - sb);
    if (p_sq < 0) {
      result.valid = false;
      return;
    }

    const double tmp = std::atan2(ca - cb, d - sa + sb);
    const double t = mod2pi(alpha - tmp);
    const double p = std::sqrt(std::max(0.0, p_sq));
    const double q = mod2pi(-beta + tmp);

    if (t < -1e-10 || p < -1e-10 || q < -1e-10) {
      result.valid = false;
      return;
    }

    result.type = DubinsPath::RSR;
    result.param[0] = std::max(0.0, t);
    result.param[1] = std::max(0.0, p);
    result.param[2] = std::max(0.0, q);
    result.total_length = (result.param[0] + result.param[1] + result.param[2]) * turning_radius_;
    result.cost = result.total_length;
    result.valid = true;
  }

  void computeRLR(double alpha, double beta, double d, DubinsPath& result) const
  {
    const double sa = std::sin(alpha);
    const double ca = std::cos(alpha);
    const double sb = std::sin(beta);
    const double cb = std::cos(beta);

    const double tmp = (6.0 - d * d + 2.0 * ca * cb + 2.0 * d * (sa - sb)) / 8.0;
    if (std::abs(tmp) > 1.0) {
      result.valid = false;
      return;
    }

    const double p = mod2pi(2.0 * M_PI - std::acos(std::clamp(tmp, -1.0, 1.0)));
    const double t = mod2pi(alpha - std::atan2(ca - cb, d - sa + sb) + p / 2.0);
    const double q = mod2pi(alpha - beta - t + p);

    if (t < -1e-10 || p < -1e-10 || q < -1e-10) {
      result.valid = false;
      return;
    }

    result.type = DubinsPath::RLR;
    result.param[0] = std::max(0.0, t);
    result.param[1] = std::max(0.0, p);
    result.param[2] = std::max(0.0, q);
    result.total_length = (result.param[0] + result.param[1] + result.param[2]) * turning_radius_;
    result.cost = result.total_length;
    result.valid = true;
  }

  void computeLRL(double alpha, double beta, double d, DubinsPath& result) const
  {
    const double sa = std::sin(alpha);
    const double ca = std::cos(alpha);
    const double sb = std::sin(beta);
    const double cb = std::cos(beta);

    const double tmp = (6.0 - d * d + 2.0 * ca * cb - 2.0 * d * (sa - sb)) / 8.0;
    if (std::abs(tmp) > 1.0) {
      result.valid = false;
      return;
    }

    const double p = mod2pi(2.0 * M_PI - std::acos(std::clamp(tmp, -1.0, 1.0)));
    const double t = mod2pi(-alpha - std::atan2(ca - cb, d + sa - sb) + p / 2.0);
    const double q = mod2pi(beta - alpha - t + p);

    if (t < -1e-10 || p < -1e-10 || q < -1e-10) {
      result.valid = false;
      return;
    }

    result.type = DubinsPath::LRL;
    result.param[0] = std::max(0.0, t);
    result.param[1] = std::max(0.0, p);
    result.param[2] = std::max(0.0, q);
    result.total_length = (result.param[0] + result.param[1] + result.param[2]) * turning_radius_;
    result.cost = result.total_length;
    result.valid = true;
  }
};

}  // namespace theta_star

#endif  // THETA_STAR_DUBINS_CURVE_HPP_
