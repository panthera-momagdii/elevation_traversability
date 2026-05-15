# elevation_traversability (ROS 2 Jazzy)

Real-time elevation-based terrain analysis for ground robots. Subscribes to a
registered point cloud, builds a rolling 2D elevation map, computes
slope/roughness/step/normals, and publishes traversability + occupancy outputs
(local rolling + global accumulated).

## SLAM contract

The package is SLAM-agnostic. The only external contract is:

1. A `sensor_msgs/PointCloud2` on a configurable topic (`raw_topic`, default
   `/cloud_registered`).
2. A TF chain such that `map_frame -> robot_frame` is resolvable at any time.

Both frame names and the cloud topic are parameters. Set them to whatever your
SLAM/odometry source publishes.

## Build

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select elevation_traversability \
             --symlink-install --parallel-workers 3 \
             --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

## Launch examples

The launch args override the YAML — no config edits needed to swap SLAM.

```bash
# FAST-LIO / FAST-LIVO2
ros2 launch elevation_traversability elevation_traversability.launch.py \
  map_frame:=camera_init robot_frame:=body raw_topic:=/cloud_registered

# GLIM
ros2 launch elevation_traversability elevation_traversability.launch.py \
  map_frame:=map robot_frame:=base_link raw_topic:=/glim_ros/points

# LIO-SAM
ros2 launch elevation_traversability elevation_traversability.launch.py \
  map_frame:=map robot_frame:=base_link \
  raw_topic:=/lio_sam/deskew/cloud_deskewed
```

Offline rosbag replay:

```bash
ros2 launch elevation_traversability offline.launch.py \
  bag:=/path/to/bag rate:=1.0 map_frame:=map robot_frame:=base_link
```

Offline single PCD → latched grid + optional PGM:

```bash
ros2 launch elevation_traversability offline_pcd.launch.py \
  pcd_file:=/path/to/cloud.pcd output_pgm:=/tmp/site_map map_resolution:=0.15
```

## Published topics

| Topic                                   | Type                                | QoS          |
|-----------------------------------------|-------------------------------------|--------------|
| `/elevation_grid_map`                   | `grid_map_msgs/GridMap`             | reliable, depth 1 |
| `/occupancy_map_local`                  | `nav_msgs/OccupancyGrid`            | reliable, depth 1 |
| `/occupancy_map_local_height`           | `elevation_traversability/OccupancyElevation` | reliable, depth 1 |
| `/occupancy_map_global`                 | `nav_msgs/OccupancyGrid`            | transient_local (latched) |
| `/elevation_pointcloud`                 | `sensor_msgs/PointCloud2`           | reliable, depth 1 |
| `/traversability_pointcloud`            | `sensor_msgs/PointCloud2`           | reliable, depth 1 |
| `/pointcloud_2_laserscan`               | `sensor_msgs/LaserScan`             | reliable, depth 1 |
| `/elevation_grid_map_offline` (offline) | `grid_map_msgs/GridMap`             | transient_local |
| `/occupancy_map_offline` (offline)      | `nav_msgs/OccupancyGrid`            | transient_local |
| `/cloud_offline` (offline)              | `sensor_msgs/PointCloud2`           | transient_local |

## Save the global map

```bash
ros2 service call /elevation_traversability/save_map \
  elevation_traversability/srv/SaveMap \
  "{directory: '/tmp', name: 'elev_trav_map'}"
```

Produces a `map_server`-compatible `.pgm` + `.yaml` pair.

## Subscribed topic

| Topic                | Type                      | QoS                       |
|----------------------|---------------------------|---------------------------|
| `<raw_topic>`        | `sensor_msgs/PointCloud2` | SensorDataQoS, depth 2    |
