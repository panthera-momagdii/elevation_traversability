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

## Docker

All Docker assets live under [`docker/`](docker/): `Dockerfile`,
`docker-entrypoint.sh`, and a BuildKit-scoped `Dockerfile.dockerignore`.
The image is based on `ros:jazzy-ros-base`, builds the package via colcon,
and ships both `rmw_fastrtps_cpp` and `rmw_cyclonedds_cpp` so the RMW can be
swapped at runtime.

### Build

Run from the package root (the build context must include the source):

```bash
docker build -f docker/Dockerfile -t elevation_traversability:jazzy .
```

### Run — share host DDS so it sees existing ROS 2 Jazzy topics

```bash
docker run --rm -it \
  --network host --ipc=host --pid=host \
  -e ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0} \
  -v $(pwd)/config:/config:ro \
  elevation_traversability:jazzy
```

- `--network host --ipc=host` lets the container discover and subscribe to
  topics on the host (multicast + shared memory).
- `ROS_DOMAIN_ID` must match what your other Jazzy nodes use.
- Edit [`config/params.yaml`](config/params.yaml) on the host → rerun →
  the new values are picked up. The entrypoint defaults to
  `PARAMS_FILE=/config/params.yaml` and forwards it to the launch as
  `params_file:=…`.

### Selecting the RMW at runtime

The image installs both implementations. Pick one per `docker run`:

```bash
# Fast DDS (default)
docker run --rm -it --network host --ipc=host \
  -v $(pwd)/config:/config:ro elevation_traversability:jazzy

# CycloneDDS, no config file
docker run --rm -it --network host --ipc=host \
  -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  -v $(pwd)/config:/config:ro elevation_traversability:jazzy

# CycloneDDS with a config XML: drop config/cyclonedds.xml next to
# params.yaml and the entrypoint auto-sets CYCLONEDDS_URI + RMW.
docker run --rm -it --network host --ipc=host \
  -v $(pwd)/config:/config:ro elevation_traversability:jazzy
```

Override the auto-detected path explicitly:

```bash
-e CYCLONEDDS_URI=file:///config/my_cyclone.xml
```

Example minimal `config/cyclonedds.xml`:

```xml
<?xml version="1.0" encoding="UTF-8"?>
<CycloneDDS xmlns="https://cdds.io/config">
  <Domain id="any">
    <General>
      <Interfaces><NetworkInterface name="lo"/></Interfaces>
      <AllowMulticast>true</AllowMulticast>
    </General>
  </Domain>
</CycloneDDS>
```

### Override launch args / pass through commands

Anything after the image name replaces the default launch:

```bash
docker run --rm -it --network host --ipc=host elevation_traversability:jazzy \
  ros2 launch elevation_traversability elevation_traversability.launch.py \
    map_frame:=map robot_frame:=base_link raw_topic:=/glim_ros/points rviz:=false
```

Drop into a shell:

```bash
docker run --rm -it --network host --ipc=host elevation_traversability:jazzy bash
```

### RViz inside the container

```bash
xhost +local:root  # on the host
docker run --rm -it --network host --ipc=host \
  -e DISPLAY=$DISPLAY -e RVIZ=true \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v $(pwd)/config:/config:ro \
  elevation_traversability:jazzy
```

### Environment variables (entrypoint-aware)

| Variable             | Default                 | Effect                                                                 |
|----------------------|-------------------------|------------------------------------------------------------------------|
| `PARAMS_FILE`        | `/config/params.yaml`   | Forwarded to the launch as `params_file:=…` when the file exists.      |
| `RVIZ`               | `false`                 | Forwarded as `rviz:=…`.                                                |
| `RMW_IMPLEMENTATION` | unset → Fast DDS        | `rmw_fastrtps_cpp` or `rmw_cyclonedds_cpp`.                            |
| `CYCLONEDDS_URI`     | auto if `/config/cyclonedds.xml` exists | Cyclone config URI; also auto-flips RMW to Cyclone when set. |
| `ROS_DOMAIN_ID`      | unset → 0               | Must match the rest of your Jazzy network.                             |
