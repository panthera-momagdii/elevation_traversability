// ============================================================================
// elevation_traversability_node.cpp
//
// Real-time elevation-based traversability analysis. ROS 2 Jazzy port.
// SLAM-agnostic: the only contract with the outside world is a configurable
// PointCloud2 topic plus a configurable map_frame -> robot_frame TF.
//
// Design for real-time:
//   1. Cloud callback is FAST: ~10ms — uses PointCloud2Iterator (no PCL copy),
//      precomputed rotation matrix for TF, min/max-z per cell.
//   2. Terrain analysis runs LOCK-FREE on a snapshot of elevation layers,
//      so the cloud callback is never blocked by heavy computation.
//   3. Incremental covariance for surface normals — no per-cell allocations.
//   4. Precomputed circular kernel offsets.
//   5. Map only moves when robot shifts > threshold (avoids thrashing).
//   6. Decoupled publish rates for grid_map / local occupancy / global.
//
// Outputs:
//   /elevation_grid_map              (grid_map_msgs/GridMap with all layers)
//   /occupancy_map_local             (nav_msgs/OccupancyGrid)
//   /occupancy_map_local_height      (OccupancyElevation — with elevation + cost)
//   /occupancy_map_global            (persistent accumulated OccupancyGrid, latched)
//   /elevation_pointcloud            (global elevation visualization)
//   /traversability_pointcloud       (global traversability visualization)
//   /pointcloud_2_laserscan          (for 2D nav stacks)
// ============================================================================

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/exceptions.h>
#include <tf2/time.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <Eigen/Core>
#include <Eigen/Eigenvalues>

#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>

#include <thread>
#include <mutex>
#include <atomic>
#include <chrono>
#include <climits>
#include <cmath>
#include <cfloat>
#include <fstream>
#include <algorithm>
#include <vector>
#include <memory>

#include "elevation_traversability/msg/occupancy_elevation.hpp"
#include "elevation_traversability/srv/save_map.hpp"

using PointType = pcl::PointXYZI;

// ============================================================================
//  Global-map sparse cube structure (persistent occupancy accumulation)
// ============================================================================
static constexpr float kGlobalRes    = 0.15f;
static constexpr float kCubeLength   = 1.5f;
static constexpr int   kCellsPerCube = static_cast<int>(kCubeLength / kGlobalRes);
static constexpr int   kMaxCubes     = 1500;
static constexpr int   kCubeOrigin   = kMaxCubes / 2;

struct GlobalCell {
  float elevation     = -FLT_MAX;
  float log_odds      = 0.0f;
  float traversability = -1.0f;
  int8_t occupancy    = -1;
};

struct GlobalCube {
  int indX, indY;
  float originX, originY;
  GlobalCell cells[kCellsPerCube][kCellsPerCube];

  GlobalCube(int ix, int iy) : indX(ix), indY(iy) {
    originX = (ix - kCubeOrigin) * kCubeLength - kCubeLength / 2.0f;
    originY = (iy - kCubeOrigin) * kCubeLength - kCubeLength / 2.0f;
  }
};

// ============================================================================
//  Precomputed circular kernel  (neighbor offsets within radius)
// ============================================================================
struct KernelOffset { int dr; int dc; };

static std::vector<KernelOffset> buildKernel(float radius_m, float resolution) {
  int r_cells = static_cast<int>(std::ceil(radius_m / resolution));
  float r_sq = (radius_m / resolution) * (radius_m / resolution);
  std::vector<KernelOffset> kernel;
  kernel.reserve(static_cast<size_t>((2 * r_cells + 1) * (2 * r_cells + 1)));
  for (int dr = -r_cells; dr <= r_cells; ++dr) {
    for (int dc = -r_cells; dc <= r_cells; ++dc) {
      if (dr * dr + dc * dc <= r_sq) kernel.push_back({dr, dc});
    }
  }
  return kernel;
}

// ============================================================================
//  Main node
// ============================================================================
class ElevationTraversability : public rclcpp::Node {
public:
  ElevationTraversability()
      : rclcpp::Node("elevation_traversability"),
        tf_buffer_(this->get_clock()),
        tf_listener_(tf_buffer_),
        running_(true) {
    declareParams();
    initGridMap();
    initLaserScan();
    initGlobalMap();
    buildKernels();
    initIO();

    analysis_thread_ = std::thread(
        &ElevationTraversability::analysisThread, this);

    RCLCPP_INFO(this->get_logger(), "\033[1;32m---->\033[0m elevation_traversability ready.");
    RCLCPP_INFO(this->get_logger(),
                "  raw_topic=%s  height_band=[%.1f, %.1f]  obstacle_diff=%.2f",
                raw_topic_.c_str(), min_height_rel_, max_height_rel_,
                obstacle_height_diff_);
    RCLCPP_INFO(this->get_logger(),
                "  map: %.1fm x %.1fm @ %.2fm  range=%.1fm  analysis=%.1fHz",
                map_length_, map_length_, map_resolution_,
                sensor_range_limit_, analysis_rate_);
  }

  ~ElevationTraversability() {
    running_ = false;
    if (analysis_thread_.joinable()) analysis_thread_.join();
    if (global_index_) {
      for (int i = 0; i < kMaxCubes; ++i) delete[] global_index_[i];
      delete[] global_index_;
    }
  }

private:
  // ================================================================
  //  Parameter declaration
  // ================================================================
  void declareParams() {
    map_frame_   = this->declare_parameter<std::string>("map_frame",   "map");
    robot_frame_ = this->declare_parameter<std::string>("robot_frame", "base_link");
    raw_topic_   = this->declare_parameter<std::string>("raw_topic",   "/cloud_registered");

    min_height_rel_       = static_cast<float>(this->declare_parameter<double>("min_height_rel",       -1.5));
    max_height_rel_       = static_cast<float>(this->declare_parameter<double>("max_height_rel",        2.0));
    obstacle_height_diff_ = static_cast<float>(this->declare_parameter<double>("obstacle_height_diff",  0.30));

    map_length_         = static_cast<float>(this->declare_parameter<double>("map_length",         20.0));
    map_resolution_     = static_cast<float>(this->declare_parameter<double>("map_resolution",     0.15));
    sensor_range_limit_ = static_cast<float>(this->declare_parameter<double>("sensor_range_limit", 12.0));
    move_threshold_     = static_cast<float>(this->declare_parameter<double>("move_threshold",     0.30));

    robot_height_ = static_cast<float>(this->declare_parameter<double>("robot_height", 1.6));
    robot_radius_ = static_cast<float>(this->declare_parameter<double>("robot_radius", 0.40));

    normal_radius_    = static_cast<float>(this->declare_parameter<double>("normal_estimation_radius", 0.45));
    smoothing_radius_ = static_cast<float>(this->declare_parameter<double>("smoothing_radius",         0.45));

    slope_threshold_deg_ = static_cast<float>(this->declare_parameter<double>("slope_threshold",     35.0));
    step_threshold_      = static_cast<float>(this->declare_parameter<double>("step_threshold",      0.25));
    roughness_threshold_ = static_cast<float>(this->declare_parameter<double>("roughness_threshold", 0.20));

    slope_weight_       = static_cast<float>(this->declare_parameter<double>("slope_weight",       0.40));
    roughness_weight_   = static_cast<float>(this->declare_parameter<double>("roughness_weight",   0.30));
    step_weight_        = static_cast<float>(this->declare_parameter<double>("step_weight",        0.30));
    slope_critical_     = static_cast<float>(this->declare_parameter<double>("slope_critical",     0.60));
    roughness_critical_ = static_cast<float>(this->declare_parameter<double>("roughness_critical", 0.35));
    step_critical_      = static_cast<float>(this->declare_parameter<double>("step_critical",      0.30));

    trav_threshold_   = static_cast<float>(this->declare_parameter<double>("traversability_threshold", 0.25));
    min_observations_ = static_cast<int>(this->declare_parameter<int>("min_observations", 3));

    min_obstacle_hits_   = static_cast<int>(this->declare_parameter<int>("min_obstacle_hits", 3));
    obstacle_decay_rate_ = static_cast<float>(this->declare_parameter<double>("obstacle_decay_rate", 0.5));
    ground_clear_rate_   = static_cast<float>(this->declare_parameter<double>("ground_clear_rate",   2.0));

    p_occ_obs_  = static_cast<float>(this->declare_parameter<double>("p_occupied_when_obstacle", 0.90));
    p_occ_free_ = static_cast<float>(this->declare_parameter<double>("p_occupied_when_free",     0.30));
    large_lo_   = static_cast<float>(this->declare_parameter<double>("large_log_odds",           50.0));
    max_lo_     = static_cast<float>(this->declare_parameter<double>("max_log_odds_for_belief",   8.0));

    analysis_rate_         = static_cast<float>(this->declare_parameter<double>("analysis_rate",       4.0));
    publish_rate_          = static_cast<float>(this->declare_parameter<double>("publish_rate",        4.0));
    publish_global_        = this->declare_parameter<bool>("publish_global", true);
    global_publish_rate_   = this->declare_parameter<double>("global_publish_rate", 0.5);
    global_update_every_n_ = static_cast<int>(this->declare_parameter<int>("global_update_every_n", 2));

    vis_radius_       = static_cast<float>(this->declare_parameter<double>("visualization_radius", 60.0));
    publish_grid_map_ = this->declare_parameter<bool>("publish_grid_map", true);

    slope_threshold_rad_ = slope_threshold_deg_ * M_PI / 180.0f;
  }

  // ================================================================
  //  Grid-map initialisation
  // ================================================================
  void initGridMap() {
    map_.setFrameId(map_frame_);
    map_.setGeometry(grid_map::Length(map_length_, map_length_),
                     map_resolution_,
                     grid_map::Position(0.0, 0.0));
    map_.add("elevation",        NAN);
    map_.add("max_elevation",    NAN);
    map_.add("observations",     NAN);
    map_.add("obstacle_count",   NAN);
    map_.add("ground_count",     NAN);
    map_.add("elevation_smooth", NAN);
    map_.add("normal_x",         NAN);
    map_.add("normal_y",         NAN);
    map_.add("normal_z",         NAN);
    map_.add("slope",            NAN);
    map_.add("roughness",        NAN);
    map_.add("step",             NAN);
    map_.add("traversability",   NAN);
    map_.add("occupancy",        NAN);
    RCLCPP_INFO(this->get_logger(),
                "  grid_map: %d x %d cells, %zu layers",
                map_.getSize()(0), map_.getSize()(1), map_.getLayers().size());
  }

  void initLaserScan() {
    laser_scan_.header.frame_id = robot_frame_;
    laser_scan_.angle_min       = -M_PI;
    laser_scan_.angle_max       =  M_PI;
    laser_scan_.angle_increment =  1.0f / 180.0f * M_PI;
    laser_scan_.time_increment  =  0;
    laser_scan_.scan_time       =  0.1f;
    laser_scan_.range_min       =  0.3f;
    laser_scan_.range_max       =  100.0f;
    int n = std::ceil((laser_scan_.angle_max - laser_scan_.angle_min)
                       / laser_scan_.angle_increment);
    laser_scan_.ranges.assign(n, laser_scan_.range_max + 1.0f);
  }

  void initGlobalMap() {
    global_index_ = new int*[kMaxCubes];
    for (int i = 0; i < kMaxCubes; ++i) {
      global_index_[i] = new int[kMaxCubes];
      for (int j = 0; j < kMaxCubes; ++j) global_index_[i][j] = -1;
    }
  }

  void buildKernels() {
    normal_kernel_   = buildKernel(normal_radius_,    map_resolution_);
    smooth_kernel_   = buildKernel(smoothing_radius_, map_resolution_);
    step_kernel_     = buildKernel(robot_radius_,     map_resolution_);
    RCLCPP_INFO(this->get_logger(),
                "  kernels: normal=%zu smooth=%zu step=%zu",
                normal_kernel_.size(), smooth_kernel_.size(), step_kernel_.size());
  }

  // ================================================================
  //  Publishers / subscriber / service / timers
  // ================================================================
  void initIO() {
    // Live cloud sub — matches ROS 1 queue=2 + tcpNoDelay intent.
    auto cloud_qos = rclcpp::SensorDataQoS().keep_last(2);
    sub_cloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        raw_topic_, cloud_qos,
        std::bind(&ElevationTraversability::cloudCallback, this, std::placeholders::_1));

    auto live_qos = rclcpp::QoS(1);
    pub_grid_map_ = this->create_publisher<grid_map_msgs::msg::GridMap>(
        "/elevation_grid_map", live_qos);
    pub_local_occ_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
        "/occupancy_map_local", live_qos);
    pub_local_occ_height_ = this->create_publisher<elevation_traversability::msg::OccupancyElevation>(
        "/occupancy_map_local_height", live_qos);
    pub_elevation_cloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/elevation_pointcloud", live_qos);
    pub_traversability_cloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/traversability_pointcloud", live_qos);
    pub_laser_scan_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
        "/pointcloud_2_laserscan", live_qos);

    // Latched in ROS 1 -> transient_local + reliable.
    auto latched_qos = rclcpp::QoS(1).transient_local().reliable();
    pub_global_occ_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
        "/occupancy_map_global", latched_qos);

    save_srv_ = this->create_service<elevation_traversability::srv::SaveMap>(
        "~/save_map",
        std::bind(&ElevationTraversability::onSaveMap, this,
                  std::placeholders::_1, std::placeholders::_2));

    publish_timer_ = this->create_wall_timer(
        std::chrono::duration<double>(1.0 / publish_rate_),
        std::bind(&ElevationTraversability::publishTimerCb, this));

    if (publish_global_) {
      global_timer_ = this->create_wall_timer(
          std::chrono::duration<double>(1.0 / std::max(0.05, global_publish_rate_)),
          std::bind(&ElevationTraversability::globalTimerCb, this));
    }
  }

  // ================================================================
  //  Fast robot pose lookup
  // ================================================================
  bool lookupRobotPose() {
    geometry_msgs::msg::TransformStamped t;
    try {
      t = tf_buffer_.lookupTransform(map_frame_, robot_frame_, tf2::TimePointZero);
    } catch (const tf2::TransformException& ex) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "[elev_trav] TF: %s", ex.what());
      return false;
    }
    robot_x_ = t.transform.translation.x;
    robot_y_ = t.transform.translation.y;
    robot_z_ = t.transform.translation.z;
    return true;
  }

  // ================================================================
  //  Raw cloud callback — FAST (~10ms)
  //  No PCL copy: direct PointCloud2Iterator + precomputed rotation.
  // ================================================================
  void cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
    if (!lookupRobotPose()) return;

    // --- lookup sensor→map transform (using precomputed rotation matrix) ---
    geometry_msgs::msg::TransformStamped sensor_to_map;
    try {
      sensor_to_map = tf_buffer_.lookupTransform(
          map_frame_, msg->header.frame_id, tf2::TimePointZero);
    } catch (const tf2::TransformException& ex) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "[elev_trav] cloud TF: %s", ex.what());
      return;
    }

    // Build 3x3 rotation from quaternion (faster than per-point ops).
    const double qw = sensor_to_map.transform.rotation.w;
    const double qx = sensor_to_map.transform.rotation.x;
    const double qy = sensor_to_map.transform.rotation.y;
    const double qz = sensor_to_map.transform.rotation.z;
    const double r11 = 1 - 2*(qy*qy + qz*qz);
    const double r12 =     2*(qx*qy - qz*qw);
    const double r13 =     2*(qx*qz + qy*qw);
    const double r21 =     2*(qx*qy + qz*qw);
    const double r22 = 1 - 2*(qx*qx + qz*qz);
    const double r23 =     2*(qy*qz - qx*qw);
    const double r31 =     2*(qx*qz - qy*qw);
    const double r32 =     2*(qy*qz + qx*qw);
    const double r33 = 1 - 2*(qx*qx + qy*qy);
    const double tx  = sensor_to_map.transform.translation.x;
    const double ty  = sensor_to_map.transform.translation.y;
    const double tz  = sensor_to_map.transform.translation.z;

    const float range_sq  = sensor_range_limit_ * sensor_range_limit_;
    const float robot_x = robot_x_, robot_y = robot_y_, robot_z = robot_z_;

    std::lock_guard<std::mutex> lock(mtx_);

    // --- move map only if robot shifted significantly ---
    grid_map::Position cur_pos = map_.getPosition();
    float shift_sq = (robot_x - cur_pos.x()) * (robot_x - cur_pos.x())
                   + (robot_y - cur_pos.y()) * (robot_y - cur_pos.y());
    if (shift_sq > move_threshold_ * move_threshold_) {
      map_.move(grid_map::Position(robot_x, robot_y));
    }

    auto& elev_layer    = map_["elevation"];
    auto& maxelev_layer = map_["max_elevation"];
    auto& obs_layer     = map_["observations"];
    auto& obst_cnt      = map_["obstacle_count"];
    auto& gnd_cnt       = map_["ground_count"];

    // --- iterate cloud directly (no pcl::fromROSMsg copy) ---
    sensor_msgs::PointCloud2ConstIterator<float> it_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> it_y(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> it_z(*msg, "z");

    const bool same_frame = (msg->header.frame_id == map_frame_);

    for (; it_x != it_x.end(); ++it_x, ++it_y, ++it_z) {
      const float sx = *it_x, sy = *it_y, sz = *it_z;
      if (!std::isfinite(sx) || !std::isfinite(sy) || !std::isfinite(sz))
        continue;

      float px, py, pz;
      if (same_frame) {
        px = sx; py = sy; pz = sz;
      } else {
        px = static_cast<float>(r11*sx + r12*sy + r13*sz + tx);
        py = static_cast<float>(r21*sx + r22*sy + r23*sz + ty);
        pz = static_cast<float>(r31*sx + r32*sy + r33*sz + tz);
      }

      // range filter (sq-dist avoids sqrt)
      float dx = px - robot_x, dy = py - robot_y;
      if (dx * dx + dy * dy > range_sq) continue;

      // height band relative to robot z
      float rel_z = pz - robot_z;
      if (rel_z < min_height_rel_ || rel_z > max_height_rel_) continue;

      grid_map::Index idx;
      if (!map_.getIndex(grid_map::Position(px, py), idx)) continue;
      const int r = idx(0), c = idx(1);

      float& elev    = elev_layer(r, c);
      float& maxelev = maxelev_layer(r, c);

      if (!std::isfinite(elev) || pz < elev) elev = pz;
      if (!std::isfinite(maxelev) || pz > maxelev) maxelev = pz;

      float& obs = obs_layer(r, c);
      obs = std::isfinite(obs) ? obs + 1.0f : 1.0f;

      // Obstacle / ground classification of THIS POINT (not cell history).
      //
      // Previously we used `diff = maxelev - elev`, a cell-wide property.
      // That lets a single noisy flyer point lock maxelev high: every
      // subsequent ground-level point is then mis-classified as obstacle
      // (diff stays large), obst_cnt accumulates past min_obstacle_hits,
      // and the cell becomes a permanent false obstacle. Same mechanism
      // causes moving-person trails: as the body points push maxelev up,
      // ground points arriving after they leave still read as obstacles
      // until maxelev decays enough.
      //
      // Classifying the incoming point by its own height above the
      // cell's current min kills both pathologies: a lone noise point
      // contributes at most +1 to obst_cnt, and subsequent ground points
      // correctly decrement it via ground_clear_rate.
      float diff = pz - elev;
      if (diff > obstacle_height_diff_) {
        // Real obstacle evidence: increment obst_cnt only. Do NOT reset
        // gnd_cnt — the ratio check `obst > gnd * 0.5` below handles
        // transient-vs-static distinction.
        float& oc = obst_cnt(r, c);
        oc = std::isfinite(oc) ? oc + 1.0f : 1.0f;
      } else {
        // Ground hit: increment gnd_cnt only. Do NOT subtract from
        // obst_cnt. Previously we did `obst_cnt -= ground_clear_rate`
        // (2.0/hit) which outpaced the `+1` per obstacle hit for any
        // cell with less than ~70% obstacle hit ratio — real walls and
        // tree trunks with mixed ground/obstacle returns would never
        // accumulate past min_obstacle_hits. Clearing of transient
        // obstacles now relies on gnd_cnt outgrowing obst_cnt in the
        // `has_obstacle` ratio test, which is the correct mechanism.
        float& gc = gnd_cnt(r, c);
        gc = std::isfinite(gc) ? gc + 1.0f : 1.0f;
        // Slowly pull max_elevation down toward elevation for viz.
        maxelev = elev + (maxelev - elev) * 0.9f;
      }
    }
  }

  // ================================================================
  //  Analysis thread — lock-free heavy compute
  //
  //  1. Snapshot required layers under brief lock (~5ms).
  //  2. Record map geometry (position + start index) at snapshot time.
  //  3. Release lock, compute features on local copies (no lock held).
  //  4. Reacquire lock, verify geometry unchanged, write results.
  //  5. If geometry changed during compute, skip this cycle's write
  //     (next cycle will catch up).
  // ================================================================
  void analysisThread() {
    rclcpp::Rate rate(analysis_rate_);

    // Pre-allocate snapshot + output buffers (reused every cycle)
    Eigen::MatrixXf elev_s, maxelev_s, obs_s, obst_s, gnd_s;
    Eigen::MatrixXf out_smooth, out_nx, out_ny, out_nz;
    Eigen::MatrixXf out_slope, out_rough, out_step, out_trav, out_occ;

    int global_tick = 0;

    while (rclcpp::ok() && running_) {
      int rows = 0, cols = 0;
      grid_map::Position snap_pos;
      grid_map::Size snap_size;
      Eigen::Array2i snap_start;
      float snap_res = 0.0f;

      // --- SNAPSHOT (brief lock) ---
      {
        std::lock_guard<std::mutex> lock(mtx_);
        elev_s    = map_["elevation"];
        maxelev_s = map_["max_elevation"];
        obs_s     = map_["observations"];
        obst_s    = map_["obstacle_count"];
        gnd_s     = map_["ground_count"];
        snap_pos   = map_.getPosition();
        snap_size  = map_.getSize();
        snap_start = map_.getStartIndex();
        snap_res   = map_.getResolution();
        rows = snap_size(0);
        cols = snap_size(1);
      }

      if (rows == 0 || cols == 0) { rate.sleep(); continue; }

      // --- Resize output buffers if needed ---
      if (out_smooth.rows() != rows || out_smooth.cols() != cols) {
        out_smooth.resize(rows, cols);
        out_nx.resize(rows, cols);
        out_ny.resize(rows, cols);
        out_nz.resize(rows, cols);
        out_slope.resize(rows, cols);
        out_rough.resize(rows, cols);
        out_step.resize(rows, cols);
        out_trav.resize(rows, cols);
        out_occ.resize(rows, cols);
      }

      // --- HEAVY COMPUTE (no lock held) ---
      computeTerrainFeatures(
          elev_s, maxelev_s, obs_s, obst_s, gnd_s,
          rows, cols, snap_res,
          out_smooth, out_nx, out_ny, out_nz,
          out_slope, out_rough, out_step, out_trav, out_occ);

      // --- WRITE BACK (brief lock) ---
      {
        std::lock_guard<std::mutex> lock(mtx_);

        // Geometry unchanged? Only write if yes.
        if (map_.getSize()(0) == rows && map_.getSize()(1) == cols &&
            (map_.getStartIndex() == snap_start).all() &&
            map_.getPosition().isApprox(snap_pos, 1e-6)) {
          map_["elevation_smooth"] = out_smooth;
          map_["normal_x"]         = out_nx;
          map_["normal_y"]         = out_ny;
          map_["normal_z"]         = out_nz;
          map_["slope"]            = out_slope;
          map_["roughness"]        = out_rough;
          map_["step"]             = out_step;
          map_["traversability"]   = out_trav;
          map_["occupancy"]        = out_occ;

          // Decay obstacle_count in place (NaN cells remain NaN)
          auto& oc = map_["obstacle_count"];
          for (int r = 0; r < rows; ++r) {
            for (int c = 0; c < cols; ++c) {
              float& v = oc(r, c);
              if (std::isfinite(v) && v > 0.0f)
                v = std::max(0.0f, v - obstacle_decay_rate_);
            }
          }
        }

        if ((global_tick++ % std::max(1, global_update_every_n_)) == 0)
          updateGlobalFromLocal();
      }

      rate.sleep();
    }
  }

  // ================================================================
  //  Terrain feature computation — pure compute, no locks
  //  Uses incremental covariance (no per-cell std::vector allocation).
  // ================================================================
  void computeTerrainFeatures(
      const Eigen::MatrixXf& elev,
      const Eigen::MatrixXf& maxelev,
      const Eigen::MatrixXf& obs,
      const Eigen::MatrixXf& obst_cnt,
      const Eigen::MatrixXf& gnd_cnt,
      int rows, int cols, float res,
      Eigen::MatrixXf& out_smooth,
      Eigen::MatrixXf& out_nx, Eigen::MatrixXf& out_ny, Eigen::MatrixXf& out_nz,
      Eigen::MatrixXf& out_slope, Eigen::MatrixXf& out_rough,
      Eigen::MatrixXf& out_step, Eigen::MatrixXf& out_trav,
      Eigen::MatrixXf& out_occ)
  {
    (void)maxelev;
    for (int r = 0; r < rows; ++r) {
      for (int c = 0; c < cols; ++c) {
        const float e = elev(r, c);

        const float obst_count = std::isfinite(obst_cnt(r, c))
                                     ? obst_cnt(r, c) : 0.0f;
        const float gnd_count  = std::isfinite(gnd_cnt(r, c))
                                     ? gnd_cnt(r, c) : 0.0f;
        const bool has_obstacle = (obst_count >= min_obstacle_hits_) &&
                                  (obst_count > gnd_count * 0.5f);

        // No elevation AND no obstacle → skip entirely
        if (!std::isfinite(e) && !has_obstacle) {
          out_smooth(r, c) = NAN;
          out_nx(r, c)     = NAN;
          out_ny(r, c)     = NAN;
          out_nz(r, c)     = NAN;
          out_slope(r, c)  = NAN;
          out_rough(r, c)  = NAN;
          out_step(r, c)   = NAN;
          out_trav(r, c)   = NAN;
          out_occ(r, c)    = NAN;
          continue;
        }

        // Obstacle only, no elevation
        if (!std::isfinite(e) && has_obstacle) {
          out_smooth(r, c) = NAN;
          out_nx(r, c)     = NAN;
          out_ny(r, c)     = NAN;
          out_nz(r, c)     = NAN;
          out_slope(r, c)  = NAN;
          out_rough(r, c)  = NAN;
          out_step(r, c)   = NAN;
          out_trav(r, c)   = 0.0f;
          out_occ(r, c)    = 1.0f;
          continue;
        }

        // Enough ground observations?
        const int obs_count = std::isfinite(obs(r, c))
                                  ? static_cast<int>(obs(r, c)) : 0;
        if (obs_count < min_observations_) {
          out_smooth(r, c) = NAN;
          out_nx(r, c)     = NAN;
          out_ny(r, c)     = NAN;
          out_nz(r, c)     = NAN;
          out_slope(r, c)  = NAN;
          out_rough(r, c)  = NAN;
          out_step(r, c)   = NAN;
          out_trav(r, c)   = NAN;
          out_occ(r, c)    = has_obstacle ? 1.0f : NAN;
          continue;
        }

        // --- Pass 1: smooth + step (shares neighbour iteration) ---
        float sum_e = 0.0f;
        int   n_e   = 0;
        float emin  =  FLT_MAX, emax = -FLT_MAX;

        for (const auto& k : smooth_kernel_) {
          int nr = r + k.dr, nc = c + k.dc;
          if (nr < 0 || nr >= rows || nc < 0 || nc >= cols) continue;
          float ne = elev(nr, nc);
          if (!std::isfinite(ne)) continue;
          sum_e += ne;
          ++n_e;
        }
        for (const auto& k : step_kernel_) {
          int nr = r + k.dr, nc = c + k.dc;
          if (nr < 0 || nr >= rows || nc < 0 || nc >= cols) continue;
          float ne = elev(nr, nc);
          if (!std::isfinite(ne)) continue;
          emin = std::min(emin, ne);
          emax = std::max(emax, ne);
        }

        float elev_sm = (n_e > 0) ? (sum_e / n_e) : e;
        float rough  = std::fabs(e - elev_sm);
        float step   = (emin < FLT_MAX) ? (emax - emin) : 0.0f;

        out_smooth(r, c) = elev_sm;
        out_rough(r, c)  = rough;
        out_step(r, c)   = step;

        // --- Pass 2: surface normal via INCREMENTAL covariance ---
        // cov = E[xx^T] - μμ^T → just accumulate sum_x and sum_xx^T
        Eigen::Vector3f sum_x(0.0f, 0.0f, 0.0f);
        float sxx = 0, sxy = 0, sxz = 0;
        float       syy = 0, syz = 0;
        float              szz = 0;
        int n = 0;

        for (const auto& k : normal_kernel_) {
          int nr = r + k.dr, nc = c + k.dc;
          if (nr < 0 || nr >= rows || nc < 0 || nc >= cols) continue;
          float ne = elev(nr, nc);
          if (!std::isfinite(ne)) continue;

          // Use RELATIVE coords (dx, dy, dz). Subtracting a constant from
          // all points doesn't change the covariance, so this is identical
          // to using absolute world coordinates.
          float dx = k.dr * res;
          float dy = k.dc * res;
          float dz = ne;

          sum_x[0] += dx;
          sum_x[1] += dy;
          sum_x[2] += dz;
          sxx += dx * dx;  sxy += dx * dy;  sxz += dx * dz;
          syy += dy * dy;  syz += dy * dz;
          szz += dz * dz;
          ++n;
        }

        if (n < 3) {
          out_nx(r, c) = NAN;
          out_ny(r, c) = NAN;
          out_nz(r, c) = NAN;
          out_slope(r, c) = NAN;
          out_trav(r, c) = NAN;
          out_occ(r, c)  = has_obstacle ? 1.0f : NAN;
          continue;
        }

        float inv_n = 1.0f / static_cast<float>(n);
        float mx = sum_x[0] * inv_n;
        float my = sum_x[1] * inv_n;
        float mz = sum_x[2] * inv_n;

        Eigen::Matrix3f cov;
        cov(0,0) = sxx * inv_n - mx * mx;
        cov(1,1) = syy * inv_n - my * my;
        cov(2,2) = szz * inv_n - mz * mz;
        cov(0,1) = cov(1,0) = sxy * inv_n - mx * my;
        cov(0,2) = cov(2,0) = sxz * inv_n - mx * mz;
        cov(1,2) = cov(2,1) = syz * inv_n - my * mz;

        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eig(cov);
        Eigen::Vector3f normal = eig.eigenvectors().col(0);
        if (normal.z() < 0) normal = -normal;

        out_nx(r, c) = normal.x();
        out_ny(r, c) = normal.y();
        out_nz(r, c) = normal.z();

        float slope_v = std::acos(std::min(std::fabs(normal.z()), 1.0f));
        out_slope(r, c) = slope_v;

        // --- Traversability + occupancy ---
        if (has_obstacle ||
            slope_v > slope_threshold_rad_ ||
            step    > step_threshold_ ||
            rough   > roughness_threshold_) {
          out_trav(r, c) = 0.0f;
          out_occ(r, c)  = 1.0f;
        } else {
          float t_slope = std::max(0.0f, 1.0f - slope_v / slope_critical_);
          float t_rough = std::max(0.0f, 1.0f - rough   / roughness_critical_);
          float t_step  = std::max(0.0f, 1.0f - step    / step_critical_);
          float trav = slope_weight_     * t_slope
                     + roughness_weight_ * t_rough
                     + step_weight_      * t_step;
          trav = std::max(0.0f, std::min(1.0f, trav));
          out_trav(r, c) = trav;
          out_occ(r, c)  = (trav < trav_threshold_) ? 1.0f : 0.0f;
        }
      }
    }
  }

  // ================================================================
  //  Update global cubes from current local map (called under lock)
  // ================================================================
  void updateGlobalFromLocal() {
    const auto& elev  = map_["elevation"];
    const auto& trav  = map_["traversability"];
    const auto& occ   = map_["occupancy"];
    const int rows = map_.getSize()(0);
    const int cols = map_.getSize()(1);

    for (int r = 0; r < rows; ++r) {
      for (int c = 0; c < cols; ++c) {
        float e  = elev(r, c);
        float t  = trav(r, c);
        float o  = occ(r, c);
        if (!std::isfinite(e) && !std::isfinite(o)) continue;
        bool is_occupied = std::isfinite(o) && o > 0.5f;

        grid_map::Position pos;
        map_.getPosition(grid_map::Index(r, c), pos);

        int cx = static_cast<int>((pos.x() + kCubeLength / 2.0f) / kCubeLength)
                 + kCubeOrigin;
        int cy = static_cast<int>((pos.y() + kCubeLength / 2.0f) / kCubeLength)
                 + kCubeOrigin;
        if (pos.x() + kCubeLength / 2.0f < 0) --cx;
        if (pos.y() + kCubeLength / 2.0f < 0) --cy;
        if (cx < 0 || cx >= kMaxCubes || cy < 0 || cy >= kMaxCubes) continue;

        if (global_index_[cx][cy] == -1) {
          global_cubes_.emplace_back(new GlobalCube(cx, cy));
          global_index_[cx][cy] = static_cast<int>(global_cubes_.size()) - 1;
        }

        GlobalCube* cube = global_cubes_[global_index_[cx][cy]].get();
        int gx = static_cast<int>((pos.x() - cube->originX) / kGlobalRes);
        int gy = static_cast<int>((pos.y() - cube->originY) / kGlobalRes);
        if (gx < 0 || gx >= kCellsPerCube || gy < 0 || gy >= kCellsPerCube)
          continue;

        GlobalCell& gc = cube->cells[gx][gy];
        if (std::isfinite(e)) gc.elevation = e;
        if (std::isfinite(t)) gc.traversability = t;

        float p = is_occupied ? p_occ_obs_ : p_occ_free_;
        gc.log_odds += std::log(p / (1.0f - p));
        gc.log_odds = std::max(-large_lo_, std::min(large_lo_, gc.log_odds));

        if (gc.log_odds < -max_lo_)      gc.occupancy = 0;
        else if (gc.log_odds > max_lo_)  gc.occupancy = 100;
        else
          gc.occupancy = static_cast<int8_t>(
              std::lround((1.0f - 1.0f / (1.0f + std::exp(gc.log_odds))) * 100.0f));
      }
    }
  }

  // ================================================================
  //  Publish timer
  // ================================================================
  void publishTimerCb() {
    std::lock_guard<std::mutex> lock(mtx_);
    if (publish_grid_map_ && pub_grid_map_->get_subscription_count() > 0) {
      map_.setTimestamp(this->now().nanoseconds());
      auto msg = grid_map::GridMapRosConverter::toMessage(map_);
      pub_grid_map_->publish(std::move(msg));
    }
    publishLocalOccupancy();
    publishLaserScan();
    publishVisualization();
  }

  void publishLocalOccupancy() {
    if (pub_local_occ_->get_subscription_count() == 0 &&
        pub_local_occ_height_->get_subscription_count() == 0) return;

    nav_msgs::msg::OccupancyGrid occ_msg;
    grid_map::GridMapRosConverter::toOccupancyGrid(
        map_, "occupancy", 0.0f, 1.0f, occ_msg);
    pub_local_occ_->publish(occ_msg);

    if (pub_local_occ_height_->get_subscription_count() > 0) {
      const auto& elev_layer = map_["elevation"];
      const auto& trav_layer = map_["traversability"];
      const int n = static_cast<int>(occ_msg.data.size());
      const int rows = map_.getSize()(0);
      const int cols = map_.getSize()(1);
      std::vector<float> heights(n, -FLT_MAX);
      std::vector<float> costs(n, 0.0f);
      for (int r = 0; r < rows; ++r) {
        for (int c = 0; c < cols; ++c) {
          int idx = (rows - 1 - r) * cols + c;
          if (idx < 0 || idx >= n) continue;
          float e = elev_layer(r, c);
          float t = trav_layer(r, c);
          if (std::isfinite(e)) heights[idx] = e;
          if (std::isfinite(t)) costs[idx]   = 1.0f - t;
        }
      }
      elevation_traversability::msg::OccupancyElevation elev_msg;
      elev_msg.header    = occ_msg.header;
      elev_msg.occupancy = occ_msg;
      elev_msg.height    = heights;
      elev_msg.cost_map  = costs;
      pub_local_occ_height_->publish(elev_msg);
    }
  }

  void publishLaserScan() {
    if (pub_laser_scan_->get_subscription_count() == 0) return;

    std::fill(laser_scan_.ranges.begin(), laser_scan_.ranges.end(),
              laser_scan_.range_max + 1.0f);

    geometry_msgs::msg::TransformStamped map_to_robot;
    try {
      map_to_robot = tf_buffer_.lookupTransform(
          robot_frame_, map_frame_, tf2::TimePointZero);
    } catch (const tf2::TransformException&) { return; }

    // Pre-compute the 3x3 rotation from quaternion so per-cell math is cheap.
    const double qw = map_to_robot.transform.rotation.w;
    const double qx = map_to_robot.transform.rotation.x;
    const double qy = map_to_robot.transform.rotation.y;
    const double qz = map_to_robot.transform.rotation.z;
    const double r11 = 1 - 2*(qy*qy + qz*qz);
    const double r12 =     2*(qx*qy - qz*qw);
    const double r13 =     2*(qx*qz + qy*qw);
    const double r21 =     2*(qx*qy + qz*qw);
    const double r22 = 1 - 2*(qx*qx + qz*qz);
    const double r23 =     2*(qy*qz - qx*qw);
    const double tx  = map_to_robot.transform.translation.x;
    const double ty  = map_to_robot.transform.translation.y;

    const auto& occ = map_["occupancy"];
    const int rows = map_.getSize()(0);
    const int cols = map_.getSize()(1);
    for (int r = 0; r < rows; ++r) {
      for (int c = 0; c < cols; ++c) {
        if (!std::isfinite(occ(r, c)) || occ(r, c) < 0.5f) continue;
        grid_map::Position pos;
        map_.getPosition(grid_map::Index(r, c), pos);
        const double px = pos.x(), py = pos.y(), pzv = robot_z_;
        const double prx = r11*px + r12*py + r13*pzv + tx;
        const double pry = r21*px + r22*py + r23*pzv + ty;
        float range = static_cast<float>(std::sqrt(prx * prx + pry * pry));
        float angle = static_cast<float>(std::atan2(pry, prx));
        int idx = static_cast<int>(
            (angle - laser_scan_.angle_min) / laser_scan_.angle_increment);
        if (idx >= 0 && idx < static_cast<int>(laser_scan_.ranges.size()))
          laser_scan_.ranges[idx] = std::min(laser_scan_.ranges[idx], range);
      }
    }
    laser_scan_.header.stamp = this->now();
    pub_laser_scan_->publish(laser_scan_);
  }

  // ================================================================
  //  Visualization — global persistent clouds from cubes
  // ================================================================
  void publishVisualization() {
    bool pub_elev = pub_elevation_cloud_->get_subscription_count() > 0;
    bool pub_trav = pub_traversability_cloud_->get_subscription_count() > 0;
    if ((!pub_elev && !pub_trav) || global_cubes_.empty()) return;

    int robot_cx = static_cast<int>((robot_x_ + kCubeLength / 2.0f) / kCubeLength)
                   + kCubeOrigin;
    int robot_cy = static_cast<int>((robot_y_ + kCubeLength / 2.0f) / kCubeLength)
                   + kCubeOrigin;
    if (robot_x_ + kCubeLength / 2.0f < 0) --robot_cx;
    if (robot_y_ + kCubeLength / 2.0f < 0) --robot_cy;
    int vis_cubes = static_cast<int>(vis_radius_ / kCubeLength);

    pcl::PointCloud<PointType> elev_c, trav_c;
    for (const auto& cube : global_cubes_) {
      int dx_c = cube->indX - robot_cx;
      int dy_c = cube->indY - robot_cy;
      if (dx_c * dx_c + dy_c * dy_c > vis_cubes * vis_cubes) continue;
      for (int i = 0; i < kCellsPerCube; ++i) {
        for (int j = 0; j < kCellsPerCube; ++j) {
          const GlobalCell& gc = cube->cells[i][j];
          if (gc.elevation <= -FLT_MAX + 1.0f) continue;
          float px = cube->originX + i * kGlobalRes;
          float py = cube->originY + j * kGlobalRes;
          if (pub_elev) {
            PointType p;
            p.x = px; p.y = py; p.z = gc.elevation;
            p.intensity = (gc.occupancy > 80) ? 100.0f :
                          (gc.occupancy >= 0  ? 0.0f : 50.0f);
            elev_c.push_back(p);
          }
          if (pub_trav) {
            PointType p;
            p.x = px; p.y = py; p.z = gc.elevation;
            p.intensity = (gc.traversability >= 0.0f)
                ? gc.traversability * 100.0f : 50.0f;
            trav_c.push_back(p);
          }
        }
      }
    }
    if (pub_elev && !elev_c.empty()) {
      sensor_msgs::msg::PointCloud2 msg;
      pcl::toROSMsg(elev_c, msg);
      msg.header.stamp = this->now();
      msg.header.frame_id = map_frame_;
      pub_elevation_cloud_->publish(msg);
    }
    if (pub_trav && !trav_c.empty()) {
      sensor_msgs::msg::PointCloud2 msg;
      pcl::toROSMsg(trav_c, msg);
      msg.header.stamp = this->now();
      msg.header.frame_id = map_frame_;
      pub_traversability_cloud_->publish(msg);
    }
  }

  // ================================================================
  //  Global occupancy grid publish
  // ================================================================
  void globalTimerCb() {
    std::lock_guard<std::mutex> lock(mtx_);
    if (global_cubes_.empty()) return;

    int min_cx = INT_MAX, min_cy = INT_MAX, max_cx = INT_MIN, max_cy = INT_MIN;
    for (const auto& cube : global_cubes_) {
      min_cx = std::min(min_cx, cube->indX);
      min_cy = std::min(min_cy, cube->indY);
      max_cx = std::max(max_cx, cube->indX);
      max_cy = std::max(max_cy, cube->indY);
    }
    int cubes_x = max_cx - min_cx + 1, cubes_y = max_cy - min_cy + 1;
    int width   = cubes_x * kCellsPerCube, height = cubes_y * kCellsPerCube;
    float origin_x = (min_cx - kCubeOrigin) * kCubeLength - kCubeLength / 2.0f;
    float origin_y = (min_cy - kCubeOrigin) * kCubeLength - kCubeLength / 2.0f;

    std::vector<int8_t> data(static_cast<size_t>(width) * height, -1);
    for (const auto& cube : global_cubes_) {
      int cx_off = cube->indX - min_cx, cy_off = cube->indY - min_cy;
      for (int i = 0; i < kCellsPerCube; ++i) {
        for (int j = 0; j < kCellsPerCube; ++j) {
          const GlobalCell& gc = cube->cells[i][j];
          if (gc.occupancy < 0) continue;
          int gx = cx_off * kCellsPerCube + i;
          int gy = cy_off * kCellsPerCube + j;
          data[static_cast<size_t>(gy) * width + gx] =
              gc.occupancy > 80 ? 100 : 0;
        }
      }
    }

    nav_msgs::msg::OccupancyGrid msg;
    msg.header.stamp    = this->now();
    msg.header.frame_id = map_frame_;
    msg.info.resolution = kGlobalRes;
    msg.info.width      = width;
    msg.info.height     = height;
    msg.info.origin.position.x = origin_x;
    msg.info.origin.position.y = origin_y;
    msg.info.origin.orientation.w = 1.0;
    msg.data = data;
    pub_global_occ_->publish(msg);
  }

  // ================================================================
  //  Save map service
  // ================================================================
  void onSaveMap(
      const std::shared_ptr<elevation_traversability::srv::SaveMap::Request> req,
      std::shared_ptr<elevation_traversability::srv::SaveMap::Response> res) {
    std::lock_guard<std::mutex> lock(mtx_);
    const std::string dir  = req->directory.empty() ? "." : req->directory;
    const std::string stem = req->name.empty() ? "elevation_trav_map" : req->name;
    if (global_cubes_.empty()) {
      res->success = false;
      res->message = "global map is empty — nothing to save";
      return;
    }
    int min_cx = INT_MAX, min_cy = INT_MAX, max_cx = INT_MIN, max_cy = INT_MIN;
    for (const auto& cube : global_cubes_) {
      min_cx = std::min(min_cx, cube->indX);
      min_cy = std::min(min_cy, cube->indY);
      max_cx = std::max(max_cx, cube->indX);
      max_cy = std::max(max_cy, cube->indY);
    }
    int cubes_x = max_cx - min_cx + 1, cubes_y = max_cy - min_cy + 1;
    int width   = cubes_x * kCellsPerCube, height = cubes_y * kCellsPerCube;
    float origin_x = (min_cx - kCubeOrigin) * kCubeLength - kCubeLength / 2.0f;
    float origin_y = (min_cy - kCubeOrigin) * kCubeLength - kCubeLength / 2.0f;
    std::vector<int8_t> data(static_cast<size_t>(width) * height, -1);
    for (const auto& cube : global_cubes_) {
      int cx_off = cube->indX - min_cx, cy_off = cube->indY - min_cy;
      for (int i = 0; i < kCellsPerCube; ++i) {
        for (int j = 0; j < kCellsPerCube; ++j) {
          const GlobalCell& gc = cube->cells[i][j];
          if (gc.occupancy < 0) continue;
          int gx = cx_off * kCellsPerCube + i;
          int gy = cy_off * kCellsPerCube + j;
          data[static_cast<size_t>(gy) * width + gx] =
              gc.occupancy > 80 ? 100 : 0;
        }
      }
    }
    const std::string pgm_path  = dir + "/" + stem + ".pgm";
    const std::string yaml_path = dir + "/" + stem + ".yaml";
    std::ofstream pgm(pgm_path, std::ios::binary);
    if (!pgm) { res->success = false; res->message = "cannot open " + pgm_path; return; }
    pgm << "P5\n" << width << " " << height << "\n255\n";
    std::vector<unsigned char> row(width);
    for (int j = height - 1; j >= 0; --j) {
      for (int i = 0; i < width; ++i) {
        int8_t v = data[static_cast<size_t>(j) * width + i];
        row[i] = (v < 0) ? 205 : (v >= 100 ? 0 : 254);
      }
      pgm.write(reinterpret_cast<const char*>(row.data()), row.size());
    }
    pgm.close();
    std::ofstream yaml(yaml_path);
    if (!yaml) { res->success = false; res->message = "cannot open " + yaml_path; return; }
    yaml << "image: " << stem << ".pgm\n"
         << "resolution: " << kGlobalRes << "\n"
         << "origin: [" << origin_x << ", " << origin_y << ", 0.0]\n"
         << "negate: 0\n"
         << "occupied_thresh: 0.65\n"
         << "free_thresh: 0.196\n"
         << "frame_id: " << map_frame_ << "\n";
    yaml.close();
    res->success = true;
    res->message = "saved " + dir + "/" + stem + ".{pgm,yaml}";
    RCLCPP_INFO_STREAM(this->get_logger(), "[elev_trav] " << res->message);
  }

  // ================================================================
  //  Members
  // ================================================================
  std::mutex mtx_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud_;

  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr pub_grid_map_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_local_occ_;
  rclcpp::Publisher<elevation_traversability::msg::OccupancyElevation>::SharedPtr
      pub_local_occ_height_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_global_occ_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_elevation_cloud_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_traversability_cloud_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_laser_scan_;

  rclcpp::Service<elevation_traversability::srv::SaveMap>::SharedPtr save_srv_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  rclcpp::TimerBase::SharedPtr global_timer_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  grid_map::GridMap map_;
  float robot_x_ = 0, robot_y_ = 0, robot_z_ = 0;
  sensor_msgs::msg::LaserScan laser_scan_;

  std::thread analysis_thread_;
  std::atomic<bool> running_;

  int** global_index_ = nullptr;
  std::vector<std::unique_ptr<GlobalCube>> global_cubes_;

  // Precomputed kernels
  std::vector<KernelOffset> normal_kernel_;
  std::vector<KernelOffset> smooth_kernel_;
  std::vector<KernelOffset> step_kernel_;

  // Parameters
  std::string map_frame_, robot_frame_, raw_topic_;
  float map_length_, map_resolution_, sensor_range_limit_;
  float move_threshold_;
  float robot_height_, robot_radius_;
  float min_height_rel_, max_height_rel_, obstacle_height_diff_;
  float normal_radius_, smoothing_radius_;
  float slope_threshold_deg_, slope_threshold_rad_;
  float step_threshold_, roughness_threshold_;
  float slope_weight_, roughness_weight_, step_weight_;
  float slope_critical_, roughness_critical_, step_critical_;
  float trav_threshold_;
  int   min_observations_;
  int   min_obstacle_hits_;
  float obstacle_decay_rate_, ground_clear_rate_;
  float p_occ_obs_, p_occ_free_;
  float large_lo_, max_lo_;
  float analysis_rate_, publish_rate_;
  bool  publish_global_, publish_grid_map_;
  double global_publish_rate_;
  int   global_update_every_n_;
  float vis_radius_;
};

// ============================================================================
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ElevationTraversability>();
  rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 2);
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
