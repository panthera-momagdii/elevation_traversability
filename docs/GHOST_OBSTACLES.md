# Ghost Ring Obstacles — Root Cause & Fix

## Symptom

The local occupancy map (`/occupancy_map_local`) and the global occupancy map
(`/occupancy_map_global`) show **concentric ring / arc obstacles** extending
radially from the robot's traversed path — even when no real obstacles exist
there. The pattern:

- Looks like LiDAR scan rings projected onto the floor at multiple ranges.
- Persists when the robot is stationary.
- Forms parallel 1-cell-thick lines spreading outward from the robot.
- Is unchanged when the input topic is switched from
  `/glim_ros/aligned_points` to raw `/panther/ouster/points` — confirming the
  source is downstream of GLIM, in `elevation_traversability` itself.

## Two distinct bugs, both fixed

The symptom is produced by **two independent algorithmic bugs** in
`computeTerrainFeatures()`. They reinforce each other: in the raw output some
ring cells have `elevation = NaN` (bug 1), and others have finite elevation but
get classified as 90° vertical walls (bug 2). Together they produce the full
ring pattern observed in RViz.

---

### Bug 1 — "Obstacle only, no elevation" branch lets `obstacle_count` stand alone

Original code in `computeTerrainFeatures()`:

```cpp
// (a) No elevation AND no obstacle → unknown   (correct)
if (!std::isfinite(e) && !has_obstacle) {
  ...
  out_occ(r, c) = NAN;
  continue;
}

// (b) Obstacle only, no elevation → OBSTACLE   (BUG)
if (!std::isfinite(e) && has_obstacle) {
  ...
  out_occ(r, c) = 1.0f;       // marks occupancy = 1 with no current elevation
  continue;
}
```

Branch **(b)** trusts `obstacle_count` alone to mark a cell as obstacle, even
when the cell has **no current elevation evidence**. Empirically cells reach
the state `elevation = NaN, obstacle_count > 0` (likely via interaction of
`grid_map::move()` circular-buffer reuse with the per-layer reset semantics).
With branch (b), that stale `obstacle_count` permanently locks the cell at
`occupancy = 1`.

#### Fix

Collapse the two NaN-elevation branches into a single rule:

> A cell with no current elevation evidence is **UNKNOWN**, never obstacle.

```cpp
if (!std::isfinite(e)) {
  out_smooth(r, c) = NAN;
  out_nx(r, c)     = NAN;
  out_ny(r, c)     = NAN;
  out_nz(r, c)     = NAN;
  out_slope(r, c)  = NAN;
  out_rough(r, c)  = NAN;
  out_step(r, c)   = NAN;
  out_trav(r, c)   = NAN;
  out_occ(r, c)    = NAN;   // was 1.0f when has_obstacle — now NaN
  continue;
}
```

#### Behavioral diff

| Cell state | Before | After |
|------------|--------|-------|
| Finite elev + `has_obstacle` | obstacle (1) | obstacle (1) — unchanged |
| Finite elev + slope/step/rough over threshold | obstacle (1) | obstacle (1) — unchanged |
| Finite elev + traversable | free (0) | free (0) — unchanged |
| **NaN elev + `has_obstacle`** (stale `obst_cnt`) | **obstacle (1)** | **unknown (-1)** ← fixed |
| NaN elev + no obstacle | unknown (-1) | unknown (-1) — unchanged |

---

### Bug 2 — Surface normal from rank-deficient covariance produces phantom 90° slopes

The slope path computes a per-cell surface normal via PCA on the elevation
field in a small neighborhood:

```cpp
Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eig(cov);
Eigen::Vector3f normal = eig.eigenvectors().col(0);   // smallest eigenvalue
if (normal.z() < 0) normal = -normal;
float slope_v = std::acos(std::min(std::fabs(normal.z()), 1.0f));
```

This works when the local neighborhood is genuinely planar. **It breaks for
colinear point sets.** For a ground cell on a single LiDAR scan ring, the only
finite-elevation neighbors are other cells **on the same ring** (the cells
between rings are NaN because that LiDAR beam never hit there). The
neighborhood is effectively a 1D line of points in 3D, not a 2D patch:

- λ2 (largest eigenvalue): variance along the ring direction — large.
- **λ0 ≈ λ1 ≈ 0** — the two perpendicular directions have near-zero variance.
- The smallest eigenvector is therefore **numerically ambiguous**: it can
  point anywhere in the 2D null-space orthogonal to the ring.

When the eigenvector lands horizontal (or near-horizontal):
`normal.z ≈ 0` → `slope_v = acos(0) ≈ π/2 = 90°` → exceeds
`slope_threshold = 35°` → cell marked obstacle.

This is **geometrically wrong**: a flat floor cell on a LiDAR ring is not a
90° vertical wall. It's the algorithm failing to recognize that it has too
little data to fit a plane.

#### Diagnostic that proved it

After applying Bug 1's fix, ghost rings still appeared. Setting **every**
hard threshold to a value that should never fire still left ~560 obstacle
cells in `/occupancy_map_local`. Setting `slope_threshold: 89` did not help.
Setting `slope_threshold: 100` (above the physical maximum of 90°) **did**
clear them — proving that slope was the path firing, and that the slope
value was landing very close to 90°. That's the signature of a degenerate
plane fit.

#### Fix

Detect rank-deficient covariance via the **planarity ratio**
`(λ1 − λ0) / λ2`. When this ratio is below a small threshold, the neighborhood
is not planar enough to support a reliable normal — treat the cell as flat
for the slope path. `step` and `roughness` still apply and will catch any
real obstacle.

```cpp
Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eig(cov);
const Eigen::Vector3f& eigvals = eig.eigenvalues();

const float lam0 = eigvals(0);
const float lam1 = eigvals(1);
const float lam2 = eigvals(2);
const float kPlanarityMin = 0.05f;
const bool planar = (lam2 > 1e-9f) &&
                    ((lam1 - lam0) / lam2 >= kPlanarityMin);

Eigen::Vector3f normal;
float slope_v;
if (planar) {
  normal = eig.eigenvectors().col(0);
  if (normal.z() < 0) normal = -normal;
  slope_v = std::acos(std::min(std::fabs(normal.z()), 1.0f));
} else {
  // Non-planar local neighborhood (line / cluster). Surface normal is
  // ill-defined; treat the cell as locally flat for the slope path.
  // step and roughness still apply and will catch real obstacles.
  normal = Eigen::Vector3f::UnitZ();
  slope_v = 0.0f;
}
```

#### Why the threshold value is 0.05

It's a heuristic. Empirically:

- A genuine plane (3+ points distributed in 2D): `(λ1 − λ0) / λ2` ≈ 0.3–1.0.
- A degenerate line (colinear ring cells): `(λ1 − λ0) / λ2` ≈ 0.0–0.01.
- Threshold of 0.05 cleanly separates the two without rejecting real planar
  cells.

If real obstacles are being missed in your environment, lowering this to 0.02
or 0.01 may help; the trade-off is more false 90° slopes from colinear data.

---

## Why earlier hypotheses didn't explain it

| Hypothesis | Verdict |
|------------|---------|
| Wrong `T_lidar_imu` extrinsic | Verified correct: GLIM `[0.006,-0.01,-0.273,id]` matches URDF TF `panther/imu_link → panther/os_lidar` |
| Bad LiDAR deskew (no per-point times) | Setting `global_shutter_lidar: true` made it WORSE — not the dominant cause |
| Source cloud has bad points | Subscribing to raw `/panther/ouster/points` produced identical ghosts |
| Local elevation min-z-ever ratchet | Real issue at small scale (1-cell halos around real walls), but not the primary mechanism |
| Global accumulator log-odds asymmetry | Real issue for `/occupancy_map_global` persistence, but local map shows the same ghosts independently |
| Per-scan rotation noise from GLIM | At sensor_range_limit ≤ 15 m, 0.4° rotation noise = 10 cm z-spread — below the 0.20 m threshold; raising `obstacle_height_diff` to 0.30 m didn't help, ruling this out as the primary mechanism |

The two real bugs above are independent of pose noise, deskewing, or
extrinsic accuracy — they're algorithm-internal issues in
`computeTerrainFeatures()` that produce ghost obstacles even with perfect
LiDAR data and perfect pose.

---

## Diagnostic path (replication recipe)

1. **Visualize each grid_map layer in RViz** (GridMap display, change Color
   Layer):
   - `elevation`, `slope`, `roughness`, `step`: NaN at the ghost cells.
   - `obstacle_count`: ghost cells have **finite, non-zero** values.
   - `ground_count`: ghost cells have **NaN or very low** values.
2. **Compare with `/occupancy_map_local`**: ghost cells are `occupancy = 100`
   while `/elevation_grid_map`'s `elevation` layer is `NaN`. ⇒ Bug 1.
3. **After Bug 1 fix, ghosts still appear with finite elevation**.
4. **Disable all hard thresholds** in params (`slope_threshold: 89`,
   `step_threshold: 100`, `roughness_threshold: 100`, `min_obstacle_hits:
   100000`, `traversability_threshold: 0.0`) — ghosts persist.
5. **Set `slope_threshold: 100` (above 90°)** — ghosts disappear. ⇒ Bug 2:
   slope path is producing exactly 90° from rank-deficient covariance.

---

## Files changed

- `src/elevation_traversability_node.cpp`:
  - `computeTerrainFeatures()`: collapsed two NaN-elevation branches into a
    single `if (!std::isfinite(e))` → unknown (Bug 1 fix, ~10 lines).
  - `computeTerrainFeatures()`: added planarity gate before slope from
    eigenvectors, falls back to `slope_v = 0` for non-planar neighborhoods
    (Bug 2 fix, ~15 lines).
- `config/params.yaml`: reverted to upstream defaults after diagnostic
  exploration. Bug fixes are in code, not config.
- `docs/GHOST_OBSTACLES.md`: this file.

## Rebuild & deploy

Only the `elevation_traversability` Docker image needs rebuilding:

```bash
cd husarion_ugv_autonomy_ros
docker compose -f compose.simulation.glim.yaml \
               -f compose.simulation.glim.elevation.yaml \
               build elevation_traversability

docker compose -f compose.simulation.glim.yaml \
               -f compose.simulation.glim.elevation.yaml \
               up -d elevation_traversability
```

Gazebo, GLIM, Nav2, docking all keep running.

## Verification after deploy

1. Drive the robot around the simulated arena.
2. Stop. Inspect `/occupancy_map_local` and `/occupancy_map_global` in RViz
   (RAW topics, not Nav2's inflated costmap).
3. **Pass criteria**:
   - No concentric ring / arc obstacles along the robot's prior path.
   - Real walls and pillars are marked as obstacles (lethal).
   - GridMap `obstacle_count` Color Layer shows non-zero values only on real
     walls/pillars; floor stays near zero.
4. Check raw cell counts to confirm:
   ```bash
   docker exec navigation bash -lc \
     "source /opt/ros/jazzy/setup.bash; \
      timeout 3 ros2 topic echo --once /occupancy_map_local --field data \
        2>&1 | tr ',' '\n' | grep -oE '\-?[0-9]+' | sort | uniq -c"
   ```
   Expect: a few hundred `100` cells (the actual obstacles), no ghost ring
   contribution.
