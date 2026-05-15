// ============================================================================
// offline_pcd_traversability.cpp — ROS 2 Jazzy port.
//
// Offline utility: load a single (pre-registered, typically SLAM-output) PCD
// file, run the SAME elevation → terrain-feature → occupancy pipeline as the
// online node, publish the results on LATCHED topics (TRANSIENT_LOCAL +
// RELIABLE) so RViz can inspect the final map, and (optionally) save the
// occupancy grid as a map_server-compatible PGM + YAML pair.
//
// No robot pose, no TF, no rolling window, no analysis thread. Single pass.
//
// Usage:
//   ros2 run elevation_traversability pcd_traversability_offline_node
//     --ros-args -p pcd_file:=/path/to/cloud.pcd
//                -p output_pgm:=/tmp/site_map
//                -p map_resolution:=0.15
//                -p frame_id:=map
//
// Or use the launch file:
//   ros2 launch elevation_traversability offline_pcd.launch.py
//     pcd_file:=/path/to/cloud.pcd output_pgm:=/tmp/site_map
// ============================================================================

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

#include <Eigen/Core>
#include <Eigen/Eigenvalues>

#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>

#include <algorithm>
#include <cfloat>
#include <cmath>
#include <fstream>
#include <string>
#include <vector>
#include <memory>

using Pt = pcl::PointXYZ;

// Circular kernel offsets in (row, col) cell-index space.
struct KOff { int dr; int dc; };
static std::vector<KOff> buildKernel(float radius_m, float resolution) {
  const int r_cells = static_cast<int>(std::ceil(radius_m / resolution));
  const float r_sq_cells = (radius_m / resolution) * (radius_m / resolution);
  std::vector<KOff> k;
  k.reserve(static_cast<size_t>((2 * r_cells + 1) * (2 * r_cells + 1)));
  for (int dr = -r_cells; dr <= r_cells; ++dr) {
    for (int dc = -r_cells; dc <= r_cells; ++dc) {
      if (dr * dr + dc * dc <= r_sq_cells) k.push_back({dr, dc});
    }
  }
  return k;
}

class OfflinePcdTraversability : public rclcpp::Node {
public:
  OfflinePcdTraversability()
      : rclcpp::Node("pcd_traversability_offline") {
    declareParams();

    auto latched_qos = rclcpp::QoS(1).transient_local().reliable();
    pub_grid_map_ = this->create_publisher<grid_map_msgs::msg::GridMap>(
        "/elevation_grid_map_offline", latched_qos);
    pub_occ_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
        "/occupancy_map_offline", latched_qos);
    pub_cloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/cloud_offline", latched_qos);

    if (!loadAndProcess()) {
      RCLCPP_ERROR(this->get_logger(),
                   "[offline_pcd] pipeline failed, shutting down.");
      rclcpp::shutdown();
      return;
    }
    RCLCPP_INFO(this->get_logger(),
                "[offline_pcd] done. Latched outputs published on:");
    RCLCPP_INFO(this->get_logger(),
                "  /elevation_grid_map_offline  (grid_map_msgs/GridMap)");
    RCLCPP_INFO(this->get_logger(),
                "  /occupancy_map_offline       (nav_msgs/OccupancyGrid)");
    RCLCPP_INFO(this->get_logger(),
                "  /cloud_offline               (sensor_msgs/PointCloud2)");
    RCLCPP_INFO(this->get_logger(), "Ctrl-C to exit.");
  }

private:
  // ==========================================================================
  //  Parameters
  // ==========================================================================
  void declareParams() {
    pcd_file_   = this->declare_parameter<std::string>("pcd_file",   "");
    output_pgm_ = this->declare_parameter<std::string>("output_pgm", "");
    frame_id_   = this->declare_parameter<std::string>("frame_id",   "map");

    map_resolution_ = static_cast<float>(this->declare_parameter<double>("map_resolution", 0.15));
    padding_        = static_cast<float>(this->declare_parameter<double>("padding",        2.0));
    min_z_          = static_cast<float>(this->declare_parameter<double>("min_z",          -1.0e6));
    max_z_          = static_cast<float>(this->declare_parameter<double>("max_z",           1.0e6));

    obstacle_height_diff_ = static_cast<float>(this->declare_parameter<double>("obstacle_height_diff", 0.20));
    min_obstacle_hits_    = static_cast<int>(this->declare_parameter<int>("min_obstacle_hits", 2));

    // Per-cell overhang filter: reject points more than this above the
    // cell's own min-z (ground). A canopy / sunshade / bridge / ceiling
    // lives above this band and won't block the cell.
    // Default 1e6 = disabled. Typical value: 2.0 m.
    max_height_above_ground_ = static_cast<float>(this->declare_parameter<double>("max_height_above_ground", 1.0e6));

    // Morphological opening on the occupancy layer: an occupied cell with
    // fewer than this many occupied 3x3 neighbours is demoted to free,
    // killing isolated noise dots. 0 = disabled, 2 = mild, 3 = aggressive.
    noise_removal_neighbors_ = static_cast<int>(this->declare_parameter<int>("noise_removal_neighbors", 0));

    normal_radius_    = static_cast<float>(this->declare_parameter<double>("normal_estimation_radius", 0.45));
    smoothing_radius_ = static_cast<float>(this->declare_parameter<double>("smoothing_radius",         0.45));
    robot_radius_     = static_cast<float>(this->declare_parameter<double>("robot_radius",             0.40));

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

    slope_threshold_rad_ = slope_threshold_deg_ * M_PI / 180.0f;

    RCLCPP_INFO(this->get_logger(), "[offline_pcd] params:");
    RCLCPP_INFO(this->get_logger(), "  pcd_file     = %s", pcd_file_.c_str());
    if (!output_pgm_.empty())
      RCLCPP_INFO(this->get_logger(), "  output_pgm   = %s(.pgm,.yaml)", output_pgm_.c_str());
    RCLCPP_INFO(this->get_logger(), "  resolution   = %.3f m", map_resolution_);
    RCLCPP_INFO(this->get_logger(), "  padding      = %.2f m", padding_);
    RCLCPP_INFO(this->get_logger(), "  obst_diff    = %.2f m  min_hits = %d",
                obstacle_height_diff_, min_obstacle_hits_);
    if (max_height_above_ground_ < 1.0e5f)
      RCLCPP_INFO(this->get_logger(), "  overhang cut = %.2f m above cell ground",
                  max_height_above_ground_);
    if (noise_removal_neighbors_ > 0)
      RCLCPP_INFO(this->get_logger(), "  morph open   = require >= %d occupied 3x3 neighbours",
                  noise_removal_neighbors_);
  }

  // ==========================================================================
  //  Top-level flow
  // ==========================================================================
  bool loadAndProcess() {
    if (pcd_file_.empty()) {
      RCLCPP_ERROR(this->get_logger(),
                   "[offline_pcd] pcd_file parameter is required");
      return false;
    }
    pcl::PointCloud<Pt>::Ptr cloud(new pcl::PointCloud<Pt>);
    if (pcl::io::loadPCDFile<Pt>(pcd_file_, *cloud) < 0) {
      RCLCPP_ERROR(this->get_logger(),
                   "[offline_pcd] failed to load PCD: %s", pcd_file_.c_str());
      return false;
    }
    RCLCPP_INFO(this->get_logger(),
                "[offline_pcd] loaded %zu points", cloud->size());

    // Compute xy bounds (respecting min_z/max_z z-band).
    float minx =  FLT_MAX, miny =  FLT_MAX;
    float maxx = -FLT_MAX, maxy = -FLT_MAX;
    size_t n_valid = 0;
    for (const auto& p : cloud->points) {
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z))
        continue;
      if (p.z < min_z_ || p.z > max_z_) continue;
      minx = std::min(minx, p.x);
      miny = std::min(miny, p.y);
      maxx = std::max(maxx, p.x);
      maxy = std::max(maxy, p.y);
      ++n_valid;
    }
    if (n_valid == 0) {
      RCLCPP_ERROR(this->get_logger(),
                   "[offline_pcd] no valid points after z/NaN filter");
      return false;
    }
    RCLCPP_INFO(this->get_logger(),
                "[offline_pcd] valid %zu pts, xy=[%.2f,%.2f]x[%.2f,%.2f]",
                n_valid, minx, maxx, miny, maxy);

    const float lx = (maxx - minx) + 2.0f * padding_;
    const float ly = (maxy - miny) + 2.0f * padding_;
    const float cx = 0.5f * (minx + maxx);
    const float cy = 0.5f * (miny + maxy);

    setupGridMap(lx, ly, cx, cy);
    accumulate(*cloud);
    computeFeatures();
    removeIsolatedOccupied();
    publishAll(*cloud);
    if (!output_pgm_.empty()) saveOccupancyPgm();
    return true;
  }

  // ==========================================================================
  //  Morphological opening on the occupancy layer.
  //  An occupied cell with fewer than `noise_removal_neighbors_` occupied
  //  neighbours in its 3x3 window is demoted to free. Kills isolated
  //  single-cell noise dots without touching well-supported obstacles.
  //  No-op when noise_removal_neighbors_ == 0.
  // ==========================================================================
  void removeIsolatedOccupied() {
    if (noise_removal_neighbors_ <= 0) return;
    auto& occ = map_["occupancy"];
    auto& trav = map_["traversability"];
    const int rows = map_.getSize()(0);
    const int cols = map_.getSize()(1);
    Eigen::MatrixXf occ_in = occ;
    size_t cleaned = 0;

    for (int r = 0; r < rows; ++r) {
      for (int c = 0; c < cols; ++c) {
        if (!std::isfinite(occ_in(r, c)) || occ_in(r, c) < 0.5f) continue;
        int n_occ = 0;
        for (int dr = -1; dr <= 1; ++dr) {
          for (int dc = -1; dc <= 1; ++dc) {
            if (dr == 0 && dc == 0) continue;
            int nr = r + dr, nc = c + dc;
            if (nr < 0 || nr >= rows || nc < 0 || nc >= cols) continue;
            if (std::isfinite(occ_in(nr, nc)) && occ_in(nr, nc) >= 0.5f)
              ++n_occ;
          }
        }
        if (n_occ < noise_removal_neighbors_) {
          occ(r, c) = 0.0f;
          if (std::isfinite(trav(r, c)) && trav(r, c) < 0.5f)
            trav(r, c) = 0.6f;
          ++cleaned;
        }
      }
    }
    RCLCPP_INFO(this->get_logger(),
                "[offline_pcd] morph open removed %zu isolated occupied cells",
                cleaned);
  }

  // ==========================================================================
  //  Grid map setup (sized to cloud extent + padding)
  // ==========================================================================
  void setupGridMap(float lx, float ly, float cx, float cy) {
    map_.setFrameId(frame_id_);
    map_.setGeometry(grid_map::Length(lx, ly),
                     map_resolution_,
                     grid_map::Position(cx, cy));
    map_.add("elevation",        NAN);
    map_.add("max_elevation",    NAN);
    map_.add("observations",     0.0f);
    map_.add("obstacle_count",   0.0f);
    map_.add("ground_count",     0.0f);
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
                "[offline_pcd] grid: %d x %d cells (%.1f x %.1f m @ %.3fm)",
                map_.getSize()(0), map_.getSize()(1),
                lx, ly, map_resolution_);
  }

  // ==========================================================================
  //  Two-pass accumulation.
  //
  //  Pass 1: find each cell's min-z (ground). We need this before we can
  //          correctly classify points relative to ground height — needed
  //          for the overhang filter in Pass 2.
  //  Pass 2: full accumulation (max-z, obs, obstacle/ground counters),
  //          rejecting points that sit more than max_height_above_ground
  //          above the cell's ground. That cuts canopies / sunshades /
  //          bridges / overhangs so they don't block the space under them.
  // ==========================================================================
  void accumulate(const pcl::PointCloud<Pt>& cloud) {
    auto& elev    = map_["elevation"];
    auto& maxelev = map_["max_elevation"];
    auto& obs     = map_["observations"];
    auto& obst    = map_["obstacle_count"];
    auto& gnd     = map_["ground_count"];

    // --- Pass 1: per-cell min-z ---
    for (const auto& p : cloud.points) {
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z))
        continue;
      if (p.z < min_z_ || p.z > max_z_) continue;
      grid_map::Index idx;
      if (!map_.getIndex(grid_map::Position(p.x, p.y), idx)) continue;
      float& e = elev(idx(0), idx(1));
      if (!std::isfinite(e) || p.z < e) e = p.z;
    }

    // --- Pass 2: counters + overhang filter ---
    size_t used = 0, overhang_dropped = 0;
    for (const auto& p : cloud.points) {
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z))
        continue;
      if (p.z < min_z_ || p.z > max_z_) continue;

      grid_map::Index idx;
      if (!map_.getIndex(grid_map::Position(p.x, p.y), idx)) continue;
      const int r = idx(0), c = idx(1);

      const float e = elev(r, c);
      if (std::isfinite(e) && (p.z - e) > max_height_above_ground_) {
        ++overhang_dropped;
        continue;
      }

      float& me = maxelev(r, c);
      if (!std::isfinite(me) || p.z > me) me = p.z;
      obs(r, c) += 1.0f;

      const float diff = p.z - e;
      if (diff > obstacle_height_diff_) {
        obst(r, c) += 1.0f;
      } else {
        gnd(r, c) += 1.0f;
        me = e + (me - e) * 0.9f;
      }
      ++used;
    }
    RCLCPP_INFO(this->get_logger(),
                "[offline_pcd] accumulated %zu pts (%zu dropped as overhang)",
                used, overhang_dropped);
  }

  // ==========================================================================
  //  Terrain features: smoothed elevation, normals (PCA), slope, roughness,
  //  step, traversability score, binary occupancy. Same logic as the online
  //  computeTerrainFeatures, parallelised with OpenMP.
  // ==========================================================================
  void computeFeatures() {
    const auto smooth_k = buildKernel(smoothing_radius_, map_resolution_);
    const auto normal_k = buildKernel(normal_radius_,    map_resolution_);
    const auto step_k   = buildKernel(robot_radius_,     map_resolution_);

    const auto& elev     = map_["elevation"];
    const auto& obs      = map_["observations"];
    const auto& obst_cnt = map_["obstacle_count"];
    const auto& gnd_cnt  = map_["ground_count"];
    auto& smooth = map_["elevation_smooth"];
    auto& nx     = map_["normal_x"];
    auto& ny     = map_["normal_y"];
    auto& nz     = map_["normal_z"];
    auto& slope_l = map_["slope"];
    auto& rough_l = map_["roughness"];
    auto& step_l  = map_["step"];
    auto& trav_l  = map_["traversability"];
    auto& occ_l   = map_["occupancy"];

    const int rows = map_.getSize()(0);
    const int cols = map_.getSize()(1);
    const float res = map_resolution_;

    RCLCPP_INFO(this->get_logger(),
                "[offline_pcd] kernels: smooth=%zu normal=%zu step=%zu",
                smooth_k.size(), normal_k.size(), step_k.size());

    #pragma omp parallel for schedule(static)
    for (int r = 0; r < rows; ++r) {
      for (int c = 0; c < cols; ++c) {
        const float e = elev(r, c);
        const float obst_count = obst_cnt(r, c);
        const float gnd_count  = gnd_cnt(r, c);
        const bool has_obstacle = (obst_count >= min_obstacle_hits_) &&
                                  (obst_count > gnd_count * 0.5f);

        if (!std::isfinite(e) && !has_obstacle) {
          smooth(r, c) = NAN; nx(r, c) = NAN; ny(r, c) = NAN; nz(r, c) = NAN;
          slope_l(r, c) = NAN; rough_l(r, c) = NAN; step_l(r, c) = NAN;
          trav_l(r, c) = NAN; occ_l(r, c) = NAN;
          continue;
        }
        if (!std::isfinite(e) && has_obstacle) {
          smooth(r, c) = NAN; nx(r, c) = NAN; ny(r, c) = NAN; nz(r, c) = NAN;
          slope_l(r, c) = NAN; rough_l(r, c) = NAN; step_l(r, c) = NAN;
          trav_l(r, c) = 0.0f; occ_l(r, c) = 1.0f;
          continue;
        }
        const int obs_c = static_cast<int>(obs(r, c));
        if (obs_c < min_observations_) {
          smooth(r, c) = NAN; nx(r, c) = NAN; ny(r, c) = NAN; nz(r, c) = NAN;
          slope_l(r, c) = NAN; rough_l(r, c) = NAN; step_l(r, c) = NAN;
          trav_l(r, c) = NAN;
          occ_l(r, c) = has_obstacle ? 1.0f : NAN;
          continue;
        }

        // --- smoothing + step (shared neighbour iteration) ---
        float sum_e = 0.0f;
        int n_e = 0;
        float emin = FLT_MAX, emax = -FLT_MAX;
        for (const auto& k : smooth_k) {
          int nr = r + k.dr, nc = c + k.dc;
          if (nr < 0 || nr >= rows || nc < 0 || nc >= cols) continue;
          float ne = elev(nr, nc);
          if (!std::isfinite(ne)) continue;
          sum_e += ne; ++n_e;
        }
        for (const auto& k : step_k) {
          int nr = r + k.dr, nc = c + k.dc;
          if (nr < 0 || nr >= rows || nc < 0 || nc >= cols) continue;
          float ne = elev(nr, nc);
          if (!std::isfinite(ne)) continue;
          emin = std::min(emin, ne);
          emax = std::max(emax, ne);
        }
        const float elev_sm = (n_e > 0) ? (sum_e / n_e) : e;
        const float rough   = std::fabs(e - elev_sm);
        const float step    = (emin < FLT_MAX) ? (emax - emin) : 0.0f;
        smooth(r, c)  = elev_sm;
        rough_l(r, c) = rough;
        step_l(r, c)  = step;

        // --- surface normal via incremental PCA ---
        Eigen::Vector3f sum_x(0.0f, 0.0f, 0.0f);
        float sxx = 0, sxy = 0, sxz = 0, syy = 0, syz = 0, szz = 0;
        int n = 0;
        for (const auto& k : normal_k) {
          int nr = r + k.dr, nc = c + k.dc;
          if (nr < 0 || nr >= rows || nc < 0 || nc >= cols) continue;
          float ne = elev(nr, nc);
          if (!std::isfinite(ne)) continue;
          const float dx = k.dr * res;
          const float dy = k.dc * res;
          const float dz = ne;
          sum_x[0] += dx; sum_x[1] += dy; sum_x[2] += dz;
          sxx += dx*dx; sxy += dx*dy; sxz += dx*dz;
          syy += dy*dy; syz += dy*dz;
          szz += dz*dz;
          ++n;
        }
        if (n < 3) {
          nx(r, c) = NAN; ny(r, c) = NAN; nz(r, c) = NAN;
          slope_l(r, c) = NAN; trav_l(r, c) = NAN;
          occ_l(r, c) = has_obstacle ? 1.0f : NAN;
          continue;
        }
        const float inv_n = 1.0f / static_cast<float>(n);
        const float mx = sum_x[0] * inv_n;
        const float my = sum_x[1] * inv_n;
        const float mz = sum_x[2] * inv_n;
        Eigen::Matrix3f cov;
        cov(0, 0) = sxx * inv_n - mx * mx;
        cov(1, 1) = syy * inv_n - my * my;
        cov(2, 2) = szz * inv_n - mz * mz;
        cov(0, 1) = cov(1, 0) = sxy * inv_n - mx * my;
        cov(0, 2) = cov(2, 0) = sxz * inv_n - mx * mz;
        cov(1, 2) = cov(2, 1) = syz * inv_n - my * mz;
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eig(cov);
        Eigen::Vector3f normal = eig.eigenvectors().col(0);
        if (normal.z() < 0) normal = -normal;
        nx(r, c) = normal.x();
        ny(r, c) = normal.y();
        nz(r, c) = normal.z();
        const float slope_v = std::acos(std::min(std::fabs(normal.z()), 1.0f));
        slope_l(r, c) = slope_v;

        if (has_obstacle ||
            slope_v > slope_threshold_rad_ ||
            step    > step_threshold_ ||
            rough   > roughness_threshold_) {
          trav_l(r, c) = 0.0f;
          occ_l(r, c)  = 1.0f;
        } else {
          const float t_slope = std::max(0.0f, 1.0f - slope_v / slope_critical_);
          const float t_rough = std::max(0.0f, 1.0f - rough   / roughness_critical_);
          const float t_step  = std::max(0.0f, 1.0f - step    / step_critical_);
          float t = slope_weight_     * t_slope
                  + roughness_weight_ * t_rough
                  + step_weight_      * t_step;
          t = std::max(0.0f, std::min(1.0f, t));
          trav_l(r, c) = t;
          occ_l(r, c)  = (t < trav_threshold_) ? 1.0f : 0.0f;
        }
      }
    }
    RCLCPP_INFO(this->get_logger(), "[offline_pcd] terrain features computed.");
  }

  // ==========================================================================
  //  Publish (latched via transient_local QoS)
  // ==========================================================================
  void publishAll(const pcl::PointCloud<Pt>& cloud) {
    map_.setTimestamp(this->now().nanoseconds());
    auto gm_msg = grid_map::GridMapRosConverter::toMessage(map_);
    pub_grid_map_->publish(std::move(gm_msg));

    nav_msgs::msg::OccupancyGrid occ_msg;
    grid_map::GridMapRosConverter::toOccupancyGrid(
        map_, "occupancy", 0.0f, 1.0f, occ_msg);
    pub_occ_->publish(occ_msg);

    sensor_msgs::msg::PointCloud2 pc_msg;
    pcl::toROSMsg(cloud, pc_msg);
    pc_msg.header.frame_id = frame_id_;
    pc_msg.header.stamp = this->now();
    pub_cloud_->publish(pc_msg);

    RCLCPP_INFO(this->get_logger(), "[offline_pcd] publishing complete.");
  }

  // ==========================================================================
  //  map_server compatible save (PGM + YAML)
  // ==========================================================================
  void saveOccupancyPgm() {
    nav_msgs::msg::OccupancyGrid occ;
    grid_map::GridMapRosConverter::toOccupancyGrid(
        map_, "occupancy", 0.0f, 1.0f, occ);

    std::string stem = output_pgm_;
    if (stem.size() >= 4 &&
        stem.substr(stem.size() - 4) == ".pgm") {
      stem = stem.substr(0, stem.size() - 4);
    }
    const std::string pgm_path  = stem + ".pgm";
    const std::string yaml_path = stem + ".yaml";

    std::ofstream pgm(pgm_path, std::ios::binary);
    if (!pgm) {
      RCLCPP_ERROR(this->get_logger(),
                   "[offline_pcd] cannot open %s", pgm_path.c_str());
      return;
    }
    const int w = static_cast<int>(occ.info.width);
    const int h = static_cast<int>(occ.info.height);
    pgm << "P5\n" << w << " " << h << "\n255\n";
    std::vector<uint8_t> row(w);
    // OccupancyGrid is row-major, bottom-left origin. PGM P5 is row-major,
    // top-left origin. Flip rows.
    for (int j = h - 1; j >= 0; --j) {
      for (int i = 0; i < w; ++i) {
        const int8_t v = occ.data[static_cast<size_t>(j) * w + i];
        uint8_t px;
        if (v < 0)          px = 205;
        else if (v >= 100)  px = 0;
        else                px = 254;
        row[i] = px;
      }
      pgm.write(reinterpret_cast<const char*>(row.data()),
                static_cast<std::streamsize>(row.size()));
    }
    pgm.close();

    std::ofstream yaml(yaml_path);
    if (!yaml) {
      RCLCPP_ERROR(this->get_logger(),
                   "[offline_pcd] cannot open %s", yaml_path.c_str());
      return;
    }
    const size_t slash = stem.find_last_of('/');
    const std::string img_name =
        ((slash == std::string::npos) ? stem : stem.substr(slash + 1)) + ".pgm";
    yaml << "image: " << img_name << "\n"
         << "resolution: " << occ.info.resolution << "\n"
         << "origin: [" << occ.info.origin.position.x << ", "
                        << occ.info.origin.position.y << ", 0.0]\n"
         << "negate: 0\n"
         << "occupied_thresh: 0.65\n"
         << "free_thresh: 0.196\n"
         << "frame_id: " << frame_id_ << "\n";
    yaml.close();
    RCLCPP_INFO(this->get_logger(),
                "[offline_pcd] wrote %s + %s", pgm_path.c_str(), yaml_path.c_str());
  }

  // ==========================================================================
  //  Members
  // ==========================================================================
  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr  pub_grid_map_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_occ_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_;
  grid_map::GridMap map_;

  std::string pcd_file_, output_pgm_, frame_id_;
  float map_resolution_, padding_;
  float min_z_, max_z_;
  float obstacle_height_diff_;
  int   min_obstacle_hits_;
  float max_height_above_ground_;
  int   noise_removal_neighbors_;
  float normal_radius_, smoothing_radius_;
  float robot_radius_;
  float slope_threshold_deg_, slope_threshold_rad_;
  float step_threshold_, roughness_threshold_;
  float slope_weight_, roughness_weight_, step_weight_;
  float slope_critical_, roughness_critical_, step_critical_;
  float trav_threshold_;
  int   min_observations_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<OfflinePcdTraversability>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
