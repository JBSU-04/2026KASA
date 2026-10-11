# K-City ground perception pipeline

## Common engine

Every LiDAR mission uses the official `dcmlr/groundgrid` terrain estimator at
revision `7d6a5d7dff12b048424014a2377aa1b6ed203b52`. Its origin and local patch
boundary are recorded in `groundgrid/UPSTREAM.md`.

```text
/velodyne_points (x,y,z,intensity,ring,time)   /imu
                    \                           /
                     groundgrid_gravity_align
           point-time roll/pitch compensation + rotational deskew
                                |
              /perception/points_gravity [velodyne_ground]
                                |
                official GroundGrid terrain map + classifier
                 /              |                  \
 /groundgrid/ground   /groundgrid/segmented_sensor   /nonground
       ground only        all points + labels         obstacles only
                                |
                    /groundgrid/segmented_map
                         map-frame diagnostic
```

GroundGrid classifies a point with its rolling 120 m × 120 m elevation grid.
The project adapter returns the result to the gravity-aligned sensor frame and
adds `ground_relative`, the point height above the estimated terrain cell.
`intensity=49` denotes ground and `intensity=99` denotes non-ground, following
the upstream classifier's label convention.

The IMU adapter interpolates attitude at `cloud.header.stamp + point.time`.
This matters during hard acceleration and braking: one attitude for the whole
VLP-16 revolution leaves a scan-shaped road wedge, while per-point correction
keeps the road level throughout the revolution. A cloud is dropped if more
than 5% of its points cannot be matched to IMU data. GroundGrid waits up to
50 ms for the matching odometry transform, covering the measured asynchronous
arrival without substituting a pose from the wrong time.

Runtime entry point:

```bash
roslaunch total_bringup competition.launch
```

Ground-only validation and recording:

```bash
roslaunch total_bringup tuning_perception.launch record_heavy:=true
```

The GroundGrid parameters are in `groundgrid/config/kcity_vlp16.yaml`. Mission
clustering parameters remain in their mission package configs.

## Mission split

| Mission | GroundGrid input used by mission | Mission-specific fitting and result |
|---|---|---|
| M1, M4 traffic lights | none | camera detector/localizer produces the stop constraint |
| M2 static obstacle | `/nonground` with `ground_relative` | `obstacle_perception_node` uses the path corridor, DBSCAN, shape filters, and tracking; `obstacle_avoidance_node` fits the avoidance path |
| M3 dynamic obstacle | `/nonground` with `ground_relative` | the same detector keeps motion history; `obstacle_stop_node` applies the dynamic-zone stop logic |
| M5 roundabout | `/nonground` | `roundabout_lidar_tracker` uses its wide-range voxel, vehicle-shape clustering, and motion confirmation; `roundabout_mission` computes the merge gap |
| M6/M7 highway | `/groundgrid/segmented_sensor` | `lidar_ttc_tracker` uses GroundGrid labels and terrain height while retaining all rings for sparse-vehicle recovery; SMPC handles ACC/AEB and lane change |
| M8 GPS shadow | `/nonground` through `lidar_perception_pkg` when enabled | the legacy shadow-zone cluster/tracker remains available; its duplicate ground filter was removed |

This split gives all LiDAR missions one terrain model while preserving the
different object assumptions: small low obstacles in M2/M3, distant moving
vehicles in M5, and sparse ring continuity in M6/M7.

## Topics and ownership

| Topic | Publisher | Consumers |
|---|---|---|
| `/perception/points_gravity` | `groundgrid_gravity_align` | GroundGrid only |
| `/groundgrid/grid_map` | GroundGrid | terrain diagnostics |
| `/groundgrid/ground` | GroundGrid | RViz and bag analysis |
| `/groundgrid/segmented_sensor` | GroundGrid | highway/TTC tracker |
| `/groundgrid/segmented_map` | GroundGrid | map-frame diagnostics |
| `/nonground` | GroundGrid | static/dynamic, roundabout, and optional shadow-zone clustering |

Do not launch another ground-removal publisher on `/nonground`. The common
engine depends on `/imu`, `/odom`, the `map -> base_link` transform, and the
static `base_link -> velodyne_ground` transform supplied by its launch file.

## Removed from the active workspace

The replaced `linefit_ground_segmentation`, `patchworkpp`, and unused
`adaptive_clustering` repositories are stored under
`unused_packages/ground_segmentation_replaced_20260910/`. They no longer take
part in catkin discovery, builds, or launch files. The local
`lidar_perception_pkg/ground_grid_node` implementation was deleted; the
package remains because its clustering and tracking nodes support M8.

## Validation gates

Before changing GroundGrid thresholds, record raw cloud, IMU, odometry,
gravity-aligned cloud, full labels, ground, and non-ground together. Check:

1. `/nonground` remains continuous through hard braking and acceleration.
2. Road points stay in `/groundgrid/ground` on flat, banked, and ramp surfaces.
3. The lowest required obstacle remains in `/nonground` at the maximum mission
   detection range.
4. `/groundgrid/segmented_sensor` retains `ring`, `time`, and
   `ground_relative` fields.
5. End-to-end cloud output rate remains close to the input LiDAR rate and TF
   extrapolation warnings do not repeat.
