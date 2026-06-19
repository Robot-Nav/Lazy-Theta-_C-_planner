#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>
#include <std_msgs/ColorRGBA.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_srvs/Empty.h>

#include <vector>
#include <string>
#include <memory>
#include <mutex>
#include <cmath>
#include <algorithm>

#include "theta_star/theta_star.hpp"
#include "theta_star/kinodynamic_optimizer.hpp"

namespace
{

inline unsigned char occupancyToCostmapCost(int8_t occupancy)
{
  if (occupancy < 0) {
    return theta_star::NO_INFORMATION;
  }

  const int occ = std::max(0, std::min(100, static_cast<int>(occupancy)));
  if (occ >= 100) {
    return theta_star::LETHAL_OBSTACLE;
  }

  return static_cast<unsigned char>(
    (occ * static_cast<int>(theta_star::MAX_NON_OBSTACLE_COST)) / 100);
}

inline int8_t costmapToOccupancy(unsigned char cost)
{
  if (cost == theta_star::NO_INFORMATION) {
    return -1;
  }

  if (cost >= theta_star::INSCRIBED_INFLATED_OBSTACLE) {
    return 100;
  }

  return static_cast<int8_t>((static_cast<int>(cost) * 100) /
         static_cast<int>(theta_star::MAX_NON_OBSTACLE_COST));
}

}  // namespace

class LaserCostmapUpdater
{
public:
  LaserCostmapUpdater(ros::NodeHandle& nh, ros::NodeHandle& global_nh, theta_star::GridMap* costmap)
    : nh_(nh), global_nh_(global_nh), costmap_(costmap), tf_buffer_(ros::Duration(30.0)), tf_listener_(tf_buffer_)
  {
    global_nh_.param("laser/laser_max_range", laser_max_range_, 10.0);
    global_nh_.param("laser/laser_min_range", laser_min_range_, 0.1);
    global_nh_.param("laser/obstacle_inflation_radius", inflation_radius_, 0.3);
    global_nh_.param("laser/laser_topic", laser_topic_, std::string("/scan"));
    global_nh_.param("laser/map_frame", map_frame_, std::string("map"));
    global_nh_.param("laser/robot_frame", robot_frame_, std::string("base_link"));

    laser_sub_ = nh_.subscribe(laser_topic_, 1, &LaserCostmapUpdater::laserCallback, this);
    ROS_INFO("Laser costmap updater initialized, listening to %s", laser_topic_.c_str());
  }

  void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
  {
    if (!costmap_) return;

    geometry_msgs::TransformStamped transform;
    try {
      transform = tf_buffer_.lookupTransform(map_frame_, msg->header.frame_id, msg->header.stamp, ros::Duration(0.1));
    } catch (tf2::TransformException& ex) {
      ROS_WARN_THROTTLE(5.0, "Transform error: %s", ex.what());
      return;
    }

    std::lock_guard<std::mutex> lock(costmap_mutex_);

    for (size_t i = 0; i < msg->ranges.size(); ++i) {
      float range = msg->ranges[i];

      if (range < msg->range_min || range > msg->range_max ||
          range < laser_min_range_ || range > laser_max_range_ ||
          std::isnan(range) || std::isinf(range)) {
        continue;
      }

      float angle = msg->angle_min + i * msg->angle_increment;
      float x_laser = range * cos(angle);
      float y_laser = range * sin(angle);

      geometry_msgs::PointStamped point_laser, point_map;
      point_laser.header = msg->header;
      point_laser.point.x = x_laser;
      point_laser.point.y = y_laser;
      point_laser.point.z = 0.0;

      tf2::doTransform(point_laser, point_map, transform);
      markObstacle(point_map.point.x, point_map.point.y);
    }
  }

  void markObstacle(double wx, double wy)
  {
    unsigned int mx, my;
    if (!costmap_->worldToMap(wx, wy, mx, my)) {
      return;
    }

    costmap_->setCost(mx, my, 254);

    int inflation_cells = static_cast<int>(inflation_radius_ / costmap_->getResolution());
    for (int dy = -inflation_cells; dy <= inflation_cells; ++dy) {
      for (int dx = -inflation_cells; dx <= inflation_cells; ++dx) {
        int nx = static_cast<int>(mx) + dx;
        int ny = static_cast<int>(my) + dy;

        if (nx < 0 || nx >= static_cast<int>(costmap_->getSizeInCellsX()) ||
            ny < 0 || ny >= static_cast<int>(costmap_->getSizeInCellsY())) {
          continue;
        }

        double dist = std::hypot(dx, dy) * costmap_->getResolution();
        if (dist > inflation_radius_) continue;

        unsigned char cost = static_cast<unsigned char>(253 * (1.0 - dist / inflation_radius_));
        unsigned char current_cost = costmap_->getCost(nx, ny);
        if (cost > current_cost && current_cost < 254) {
          costmap_->setCost(nx, ny, cost);
        }
      }
    }
  }

  std::mutex& getMutex() { return costmap_mutex_; }

  void setCostmap(theta_star::GridMap * costmap)
  {
    std::lock_guard<std::mutex> lock(costmap_mutex_);
    costmap_ = costmap;
  }

private:
  ros::NodeHandle nh_;
  ros::NodeHandle global_nh_;
  theta_star::GridMap* costmap_;
  ros::Subscriber laser_sub_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  double laser_max_range_;
  double laser_min_range_;
  double inflation_radius_;
  std::string laser_topic_;
  std::string map_frame_;
  std::string robot_frame_;

  std::mutex costmap_mutex_;
};

class ThetaStarNavNode
{
public:
  ThetaStarNavNode()
    : nh_("~"), global_nh_(), tf_buffer_(ros::Duration(30.0)), tf_listener_(tf_buffer_)
  {
    initParams();
    initSubscribers();
    initPublishers();
    initServices();

    map_received_ = false;
    start_received_ = false;
    goal_received_ = false;

    costmap_adapter_ = nullptr;
    laser_updater_ = nullptr;

    ROS_INFO("Kinodynamic Lazy Theta* Navigation Node initialized");
  }

  ~ThetaStarNavNode()
  {
    delete laser_updater_;
    delete costmap_adapter_;
  }

  void initParams()
  {
    // 规划器参数
    global_nh_.param("planner/planner_frequency", planner_frequency_, 1.0);
    global_nh_.param("planner/w_euc_cost", w_euc_cost_, 1.0);
    global_nh_.param("planner/w_traversal_cost", w_traversal_cost_, 2.0);
    global_nh_.param("planner/how_many_corners", how_many_corners_, 8);
    global_nh_.param("planner/allow_unknown", allow_unknown_, true);
    global_nh_.param("planner/terminal_checking_interval", terminal_checking_interval_, 5000);

    // 激光雷达参数
    global_nh_.param("laser/use_laser", use_laser_, true);
    global_nh_.param("laser/map_frame", map_frame_, std::string("map"));

    global_nh_.param<std::string>("topics/map_topic", map_topic_, "/map");
    global_nh_.param<std::string>("topics/start_topic", start_topic_, "/initialpose");
    global_nh_.param<std::string>("topics/goal_topic", goal_topic_, "/move_base_simple/goal");
    global_nh_.param<std::string>("topics/path_topic", path_topic_, "/theta_star/path");
    global_nh_.param<std::string>("topics/kinematic_path_topic", kinematic_path_topic_, "/theta_star/kinematic_path");

    // 诊断：从参数服务器读取原始值
    bool ps_enable_opt = true;
    bool ps_enable_smooth = true;
    bool ps_enable_dubins = true;
    global_nh_.getParam("kinodynamic/enable_kinodynamic_optimization", ps_enable_opt);
    global_nh_.getParam("smoothing/enable_smoothing", ps_enable_smooth);
    global_nh_.getParam("dubins/enable_dubins_smoothing", ps_enable_dubins);
    ROS_INFO(
      "Param server values: enable_opt=%d, enable_smooth=%d, enable_dubins=%d",
      ps_enable_opt, ps_enable_smooth, ps_enable_dubins);

    // 运动学约束参数
    global_nh_.param("kinodynamic/enable_kinodynamic_optimization", enable_kinodynamic_opt_, true);
    global_nh_.param("kinodynamic/max_linear_velocity", max_linear_vel_, 1.0);
    global_nh_.param("kinodynamic/max_linear_acceleration", max_linear_accel_, 0.5);
    global_nh_.param("kinodynamic/max_lateral_acceleration", max_lateral_accel_, 0.4);
    global_nh_.param("kinodynamic/max_angular_velocity", max_angular_vel_, 1.0);
    global_nh_.param("kinodynamic/max_angular_acceleration", max_angular_accel_, 0.5);
    global_nh_.param("kinodynamic/min_turning_radius", min_turning_radius_, 0.3);
    global_nh_.param("kinodynamic/wheel_base", wheel_base_, 0.3);
    global_nh_.param("kinodynamic/dynamic_feasibility_speed", dynamic_feasibility_speed_, 0.6);

    // 路径平滑参数
    global_nh_.param("smoothing/enable_smoothing", enable_smoothing_, true);
    global_nh_.param("smoothing/resample_interval", resample_interval_, 0.08);
    global_nh_.param("smoothing/smoothing_iterations", smoothing_iterations_, 90);
    global_nh_.param("smoothing/optimizer_step_size", optimizer_step_size_, 0.18);
    global_nh_.param("smoothing/smoothness_weight", smoothness_weight_, 0.55);
    global_nh_.param("smoothing/reference_weight", reference_weight_, 0.35);
    global_nh_.param("smoothing/curvature_continuity_weight", curvature_continuity_weight_, 0.15);
    global_nh_.param("smoothing/enforce_collision_check", enforce_collision_check_, true);
    global_nh_.param("smoothing/shortcut_max_passes", shortcut_max_passes_, 2);
    global_nh_.param("smoothing/bspline_degree", bspline_degree_, 3);
    global_nh_.param("smoothing/smoothing_tolerance", smoothing_tolerance_, 0.12);

    global_nh_.param("dubins/enable_dubins_smoothing", enable_dubins_smoothing_, true);
    global_nh_.param("dubins/dubins_sample_step", dubins_sample_step_, 0.05);
    global_nh_.param("dubins/dubins_max_detour_ratio", dubins_max_detour_ratio_, 1.3);
    global_nh_.param("dubins/dubins_max_skip_points", dubins_max_skip_points_, 2);

    global_nh_.param("bezier_fallback/enable", enable_bezier_fallback_, true);
    global_nh_.param("bezier_fallback/control_dist_factor", bezier_control_dist_factor_, 0.35);
    global_nh_.param("bezier_fallback/control_dist_min", bezier_control_dist_min_, 0.06);
    global_nh_.param("bezier_fallback/control_dist_min_radius_ratio", bezier_control_dist_min_radius_ratio_, 0.9);
    global_nh_.param("bezier_fallback/control_dist_min_radius_floor", bezier_control_dist_min_radius_floor_, 0.12);
    global_nh_.param("bezier_fallback/tangent_direction_threshold", bezier_tangent_direction_threshold_, -0.15);
    global_nh_.param("bezier_fallback/heading_blend_factor", bezier_heading_blend_factor_, 0.55);

    int robot_type_int = 0;
    global_nh_.param("kinodynamic/robot_type", robot_type_int, 0);
    robot_type_ = static_cast<theta_star::KinodynamicConfig::RobotType>(robot_type_int);

    // 配置运动学优化器
    kinodynamic_config_.max_linear_velocity = max_linear_vel_;
    kinodynamic_config_.max_linear_acceleration = max_linear_accel_;
    kinodynamic_config_.max_lateral_acceleration = max_lateral_accel_;
    kinodynamic_config_.max_angular_velocity = max_angular_vel_;
    kinodynamic_config_.max_angular_acceleration = max_angular_accel_;
    kinodynamic_config_.min_turning_radius = min_turning_radius_;
    kinodynamic_config_.wheel_base = wheel_base_;
    kinodynamic_config_.dynamic_feasibility_speed = dynamic_feasibility_speed_;
    kinodynamic_config_.enable_smoothing = enable_smoothing_;
    kinodynamic_config_.resample_interval = resample_interval_;
    kinodynamic_config_.smoothing_iterations = smoothing_iterations_;
    kinodynamic_config_.optimizer_step_size = optimizer_step_size_;
    kinodynamic_config_.smoothness_weight = smoothness_weight_;
    kinodynamic_config_.reference_weight = reference_weight_;
    kinodynamic_config_.curvature_continuity_weight = curvature_continuity_weight_;
    kinodynamic_config_.enforce_collision_check = enforce_collision_check_;
    kinodynamic_config_.shortcut_max_passes = shortcut_max_passes_;
    kinodynamic_config_.bspline_degree = bspline_degree_;
    kinodynamic_config_.smoothing_tolerance = smoothing_tolerance_;
    kinodynamic_config_.enable_dubins_smoothing = enable_dubins_smoothing_;
    kinodynamic_config_.dubins_sample_step = dubins_sample_step_;
    kinodynamic_config_.dubins_max_detour_ratio = dubins_max_detour_ratio_;
    kinodynamic_config_.dubins_max_skip_points = dubins_max_skip_points_;
    kinodynamic_config_.bezier_control_dist_factor = bezier_control_dist_factor_;
    kinodynamic_config_.bezier_control_dist_min = bezier_control_dist_min_;
    kinodynamic_config_.bezier_control_dist_min_radius_ratio = bezier_control_dist_min_radius_ratio_;
    kinodynamic_config_.bezier_control_dist_min_radius_floor = bezier_control_dist_min_radius_floor_;
    kinodynamic_config_.bezier_tangent_direction_threshold = bezier_tangent_direction_threshold_;
    kinodynamic_config_.bezier_heading_blend_factor = bezier_heading_blend_factor_;
    kinodynamic_config_.robot_type = robot_type_;

    kinodynamic_optimizer_.setConfig(kinodynamic_config_);

    ROS_INFO(
      "Kinodynamic config loaded: enable_opt=%d, enable_smooth=%d, enable_dubins=%d, "
      "enable_bezier=%d, min_R=%.3f, smooth_iter=%d, dubins_skip=%d, "
      "bezier_control=%.2f, bezier_blend=%.2f",
      enable_kinodynamic_opt_,
      enable_smoothing_,
      enable_dubins_smoothing_,
      enable_bezier_fallback_,
      min_turning_radius_,
      smoothing_iterations_,
      dubins_max_skip_points_,
      bezier_control_dist_factor_,
      bezier_heading_blend_factor_);
  }

  void initSubscribers()
  {
    map_sub_ = nh_.subscribe(map_topic_, 1, &ThetaStarNavNode::mapCallback, this);
    start_sub_ = nh_.subscribe(start_topic_, 1, &ThetaStarNavNode::startCallback, this);
    goal_sub_ = nh_.subscribe(goal_topic_, 1, &ThetaStarNavNode::goalCallback, this);
  }

  void initPublishers()
  {
    path_pub_ = nh_.advertise<nav_msgs::Path>(path_topic_, 1);
    kinematic_path_pub_ = nh_.advertise<nav_msgs::Path>(kinematic_path_topic_, 1);
    costmap_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("/theta_star/costmap", 1);
    optimized_marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/theta_star/optimized_path", 1);
  }

  void initServices()
  {
    plan_srv_ = nh_.advertiseService("/theta_star/plan", &ThetaStarNavNode::planService, this);
    reset_srv_ = nh_.advertiseService("/theta_star/reset", &ThetaStarNavNode::resetService, this);
  }

  void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
  {
    ROS_INFO_ONCE("Received static map");

    if (msg->info.width == 0 || msg->info.height == 0) {
      ROS_WARN("Received empty map");
      return;
    }

    std::lock_guard<std::mutex> lock(costmap_mutex_);
    std::unique_lock<std::mutex> laser_lock;
    if (laser_updater_) {
      laser_lock = std::unique_lock<std::mutex>(laser_updater_->getMutex());
    }

    if (!costmap_adapter_) {
      costmap_adapter_ = new theta_star::GridMap(
        msg->info.width, msg->info.height, msg->info.resolution,
        msg->info.origin.position.x, msg->info.origin.position.y, 0);
    } else {
      costmap_adapter_->resize(
        msg->info.width, msg->info.height, msg->info.resolution,
        msg->info.origin.position.x, msg->info.origin.position.y, 0);
    }

    for (unsigned int y = 0; y < msg->info.height; ++y) {
      for (unsigned int x = 0; x < msg->info.width; ++x) {
        costmap_adapter_->setCost(x, y, occupancyToCostmapCost(msg->data[y * msg->info.width + x]));
      }
    }

    if (use_laser_ && !laser_updater_) {
      laser_updater_ = new LaserCostmapUpdater(nh_, global_nh_, costmap_adapter_);
    } else if (laser_updater_) {
      laser_updater_->setCostmap(costmap_adapter_);
    }

    theta_star_planner_.setCostmap(costmap_adapter_);

    theta_star::PlannerConfig config;
    config.w_euc_cost = w_euc_cost_;
    config.w_traversal_cost = w_traversal_cost_;
    config.how_many_corners = how_many_corners_;
    config.allow_unknown = allow_unknown_;
    config.terminal_checking_interval = terminal_checking_interval_;
    theta_star_planner_.setConfig(config);

    map_received_ = true;
    ROS_INFO("Map loaded: %ux%u, resolution: %.3f",
             msg->info.width, msg->info.height, msg->info.resolution);
  }

  void startCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
  {
    ROS_INFO("Received start: (%.2f, %.2f), heading: %.2f",
             msg->pose.pose.position.x, msg->pose.pose.position.y,
             tf2::getYaw(msg->pose.pose.orientation));
    start_received_ = true;
    start_pose_.header = msg->header;
    start_pose_.pose = msg->pose.pose;
  }

  void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
  {
    ROS_INFO("Received goal: (%.2f, %.2f)",
             msg->pose.position.x, msg->pose.position.y);
    goal_received_ = true;
    goal_pose_ = *msg;
  }

  bool planService(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
  {
    return runPlanning();
  }

  bool resetService(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
  {
    start_received_ = false;
    goal_received_ = false;
    last_path_.poses.clear();
    last_kinematic_path_.poses.clear();
    ROS_INFO("Kinodynamic Theta* reset successful");
    return true;
  }

  bool runPlanning()
  {
    if (!map_received_ || !costmap_adapter_) {
      ROS_WARN("No map received yet");
      return false;
    }

    if (!goal_received_) {
      ROS_WARN("Goal not set yet");
      return false;
    }

    // 获取机器人当前位置（通过TF）
    geometry_msgs::PoseStamped current_pose;
    try {
      geometry_msgs::TransformStamped transform = tf_buffer_.lookupTransform(
        map_frame_, "base_link", ros::Time(0), ros::Duration(1.0));
      current_pose.header.frame_id = map_frame_;
      current_pose.pose.position.x = transform.transform.translation.x;
      current_pose.pose.position.y = transform.transform.translation.y;
      current_pose.pose.orientation = transform.transform.rotation;
    } catch (tf2::TransformException& ex) {
      ROS_WARN_THROTTLE(5.0, "TF lookup failed, using last start pose: %s", ex.what());
      if (!start_received_) {
        ROS_WARN("No start pose available");
        return false;
      }
      current_pose = start_pose_;
    }

    double current_heading = tf2::getYaw(current_pose.pose.orientation);
    ROS_INFO("Using current robot position: (%.2f, %.2f), heading: %.2f",
             current_pose.pose.position.x, current_pose.pose.position.y, current_heading);

    std::lock_guard<std::mutex> lock(costmap_mutex_);
    std::unique_lock<std::mutex> laser_lock;
    if (laser_updater_) {
      laser_lock = std::unique_lock<std::mutex>(laser_updater_->getMutex());
    }

    if (!theta_star_planner_.setStartAndGoalWorld(
        current_pose.pose.position.x, current_pose.pose.position.y,
        goal_pose_.pose.position.x, goal_pose_.pose.position.y))
    {
      ROS_WARN("Start or goal is outside map bounds");
      return false;
    }

    if (theta_star_planner_.isUnsafeToPlan()) {
      ROS_WARN("Start or goal is in an obstacle!");
      return false;
    }

    // Step 1: Lazy Theta* 全局路径规划
    std::vector<coordsW> path;
    bool found = theta_star_planner_.generatePath(path);

    if (!found) {
      ROS_WARN("No path found by Lazy Theta*!");
      return false;
    }

    // 发布原始路径
    publishPath(path, path_pub_);
    ROS_INFO("Lazy Theta* path found with %zu points, %d nodes explored",
             path.size(), theta_star_planner_.nodes_opened);

    // Step 2: 运动学与动力学约束优化
    if (enable_kinodynamic_opt_ && path.size() >= 2) {
      std::vector<theta_star::OptimizedPathPoint> optimized_path;
      double initial_heading = current_heading;
      double goal_heading = tf2::getYaw(goal_pose_.pose.orientation);

      auto state_validator = [this](double wx, double wy) -> bool {
        if (!costmap_adapter_) {
          return false;
        }

        unsigned int mx = 0;
        unsigned int my = 0;
        if (!costmap_adapter_->worldToMap(wx, wy, mx, my)) {
          return false;
        }

        const unsigned char cost = costmap_adapter_->getCost(mx, my);
        if (cost == theta_star::NO_INFORMATION) {
          return allow_unknown_;
        }

        return cost <= theta_star::MAX_NON_OBSTACLE_COST;
      };

      bool opt_success = kinodynamic_optimizer_.optimizePath(
        path, optimized_path, initial_heading, goal_heading, state_validator);

      if (opt_success) {
        // 发布优化后的路径
        publishKinematicPath(optimized_path);

        const auto& metrics = kinodynamic_optimizer_.getLastMetrics();

        ROS_INFO(
          "Path optimization successful: pts=%zu, smoothness=%.2f, len=%.3f->%.3f, "
          "opt_time=%.2f ms, dubins_time=%.2f ms, dubins_segs=%d, "
          "feasible_v=%.2f m/s, min_R=%.3f m, constraints=%.2f%% (%d/%d violated)",
          optimized_path.size(),
          metrics.smoothness_score,
          metrics.raw_path_length,
          metrics.optimized_path_length,
          metrics.optimization_time_ms,
          metrics.dubins_time_ms,
          metrics.dubins_segments_used,
          metrics.path_feasibility_speed,
          metrics.effective_min_turning_radius,
          metrics.constraint_satisfaction_ratio * 100.0,
          metrics.violated_constraints,
          metrics.total_constraints_checked);

        if (metrics.constraint_satisfaction_ratio < 0.999) {
          ROS_WARN(
            "Constraint violations detected: max|k|=%.3f, max|dk/ds|=%.3f",
            metrics.max_curvature,
            metrics.max_curvature_rate);
        }
      } else {
        ROS_WARN("Kinodynamic optimization failed, no kinematic path published");
      }
    }

    return true;
  }

  void publishPath(const std::vector<coordsW>& path, ros::Publisher& publisher)
  {
    nav_msgs::Path path_msg;
    path_msg.header.frame_id = map_frame_;
    path_msg.header.stamp = ros::Time::now();

    for (const auto& pt : path) {
      geometry_msgs::PoseStamped pose;
      pose.header.frame_id = map_frame_;
      pose.header.stamp = ros::Time::now();
      pose.pose.position.x = pt.x;
      pose.pose.position.y = pt.y;
      pose.pose.position.z = 0.0;
      pose.pose.orientation.w = 1.0;
      path_msg.poses.push_back(pose);
    }

    publisher.publish(path_msg);
    if (&publisher == &path_pub_) {
      last_path_ = path_msg;
    }
  }

  void publishKinematicPath(const std::vector<theta_star::OptimizedPathPoint>& optimized_path)
  {
    nav_msgs::Path path_msg;
    path_msg.header.frame_id = map_frame_;
    path_msg.header.stamp = ros::Time::now();

    for (const auto& pt : optimized_path) {
      geometry_msgs::PoseStamped pose;
      pose.header.frame_id = map_frame_;
      pose.header.stamp = ros::Time::now();
      pose.pose.position.x = pt.x;
      pose.pose.position.y = pt.y;
      pose.pose.position.z = 0.0;

      // 设置航向角到四元数
      tf2::Quaternion q;
      q.setRPY(0, 0, pt.heading);
      pose.pose.orientation = tf2::toMsg(q);

      path_msg.poses.push_back(pose);
    }

    kinematic_path_pub_.publish(path_msg);
    last_kinematic_path_ = path_msg;
  }

  void publishCostmap()
  {
    if (!costmap_adapter_) return;

    nav_msgs::OccupancyGrid costmap_msg;
    costmap_msg.header.frame_id = map_frame_;
    costmap_msg.header.stamp = ros::Time::now();
    costmap_msg.info.width = costmap_adapter_->getSizeInCellsX();
    costmap_msg.info.height = costmap_adapter_->getSizeInCellsY();
    costmap_msg.info.resolution = costmap_adapter_->getResolution();
    costmap_msg.info.origin.position.x = costmap_adapter_->getOriginX();
    costmap_msg.info.origin.position.y = costmap_adapter_->getOriginY();
    costmap_msg.info.origin.orientation.w = 1.0;

    costmap_msg.data.resize(costmap_msg.info.width * costmap_msg.info.height);
    for (unsigned int y = 0; y < costmap_msg.info.height; ++y) {
      for (unsigned int x = 0; x < costmap_msg.info.width; ++x) {
        costmap_msg.data[y * costmap_msg.info.width + x] = costmapToOccupancy(costmap_adapter_->getCost(x, y));
      }
    }

    costmap_pub_.publish(costmap_msg);
  }

  void run()
  {
    ros::Rate rate(planner_frequency_);
    while (ros::ok()) {
      ros::spinOnce();

      if (goal_received_) {
        runPlanning();
      }

      if (use_laser_ && costmap_adapter_) {
        publishCostmap();
      }

      rate.sleep();
    }
  }

private:
  ros::NodeHandle nh_;
  ros::NodeHandle global_nh_;

  ros::Subscriber map_sub_;
  ros::Subscriber start_sub_;
  ros::Subscriber goal_sub_;

  ros::Publisher path_pub_;
  ros::Publisher kinematic_path_pub_;
  ros::Publisher costmap_pub_;
  ros::Publisher optimized_marker_pub_;

  ros::ServiceServer plan_srv_;
  ros::ServiceServer reset_srv_;

  theta_star::GridMap* costmap_adapter_;
  theta_star::ThetaStar theta_star_planner_;
  LaserCostmapUpdater* laser_updater_;
  theta_star::KinodynamicOptimizer kinodynamic_optimizer_;
  theta_star::KinodynamicConfig kinodynamic_config_;

  bool map_received_;
  bool start_received_;
  bool goal_received_;

  geometry_msgs::PoseStamped start_pose_;
  geometry_msgs::PoseStamped goal_pose_;
  nav_msgs::Path last_path_;
  nav_msgs::Path last_kinematic_path_;

  std::mutex costmap_mutex_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  double planner_frequency_;
  double w_euc_cost_;
  double w_traversal_cost_;
  int how_many_corners_;
  bool allow_unknown_;
  int terminal_checking_interval_;
  bool use_laser_;
  std::string map_frame_;

  // 运动学约束参数
  bool enable_kinodynamic_opt_;
  double max_linear_vel_;
  double max_linear_accel_;
  double max_lateral_accel_;
  double max_angular_vel_;
  double max_angular_accel_;
  double min_turning_radius_;
  double wheel_base_;
  double dynamic_feasibility_speed_;
  bool enable_smoothing_;
  double resample_interval_;
  int smoothing_iterations_;
  double optimizer_step_size_;
  double smoothness_weight_;
  double reference_weight_;
  double curvature_continuity_weight_;
  bool enforce_collision_check_;
  int shortcut_max_passes_;
  int bspline_degree_;
  double smoothing_tolerance_;
  bool enable_dubins_smoothing_;
  double dubins_sample_step_;
  double dubins_max_detour_ratio_;
  int dubins_max_skip_points_;
  bool enable_bezier_fallback_;
  double bezier_control_dist_factor_;
  double bezier_control_dist_min_;
  double bezier_control_dist_min_radius_ratio_;
  double bezier_control_dist_min_radius_floor_;
  double bezier_tangent_direction_threshold_;
  double bezier_heading_blend_factor_;
  theta_star::KinodynamicConfig::RobotType robot_type_;

  std::string map_topic_;
  std::string start_topic_;
  std::string goal_topic_;
  std::string path_topic_;
  std::string kinematic_path_topic_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "kinodynamic_theta_star_nav_node");

  ThetaStarNavNode planner;
  planner.run();

  return 0;
}
