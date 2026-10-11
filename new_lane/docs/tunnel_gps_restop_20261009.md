# 2026-10-09 M8 GPS map and stop recovery

Applies only to M8. MPC configs, other missions, reference admission corridor,
front/reverse speed limits and the 0.3m / 10deg tracking limits are unchanged.

## Applied behavior

- GPS NORMAL user-confirmed empty drives 19:05:14 and 19:06:02 supply the new
  `tunnel_static_gps_20261009.npz` and `tunnel_walls_gps_20261009.pcd`.
- Read sensor offset and measured road height from lidar_extrinsic.yaml. Interpolate
  scan-time odometry (including z); reject unsupported time gaps and non-normal GPS.
- Keep measured 5cm geometry on 20cm surface patches seen in >=3 distinct scans.
  This preserves sparse 16-channel wall returns without inventing/extruding surfaces.
  Independently store both drives. Exclude >12cm cross-drive disagreements only
  where the other drive actually observed free space by a measured ray. Unobserved
  regions are not filled. Online obstacle drives never update the static map.
- Background and foreground use 3D distance and independent floor support. Keep
  wall-adjacent low residuals; no blanket wall-distance strip removal. Batch local
  floor seed calculations (same robust plane/coverage limits), reuse map KD trees,
  avoid the duplicate floor lookup and restrict the filter's BLAS threads to one.
- Missing map classification cannot turn repeated raw/pending returns into a
  persistent confirmed static object. Immediate transient collision evidence stays.
- Accepted-path handoff guard uses actual future swept rectangles. A known-clear
  sweep releases that object's guard immediately; unknown/stale path does not.
- Project recovery progress onto segments. A tracking error first brakes; below
  0.15m/s clear the old latch and replan at the current pose. Final-owner rejection
  of the same plan id or newly invalid geometry also schedules this transition.
- Existing 20m preslow / 6m/s cap and .08m, 3 points, 2 distinct scan confirmation
  remain unchanged. Sensor-error stops and actual collision stops remain active.

## Checks and pending drive

Before the user's request to stop bag verification: Python 266 tests, C++ 66
checks and isolated path-owner integration 16 checks passed. An initial sparse
map offline evaluation still had occupancy failures; map support was subsequently
changed to repeated physical surface patches. That initial evaluation is not a
validation of the final map and does not establish drive success. No further bag
comparison/replay will be run in this task at the user's request.

Final build/config/hash checks only. Real closed-loop drive must verify empty
entrance, S-shaped avoidance, guard release, replan after tracking error, 20m
preslow for sufficiently sampled low objects, and forward/reverse handoff. The
80ms p95 and preslow latency targets are not certified on the final full launch.

## Rollback

Backups: `backups/tunnel_gps_restop_20261009/source_before.tar.gz`,
`core_before.cpp`, `mpc_hashes.json`. Historical map files remain untouched.

- Map-only rollback: change `tunnel_static_map` default in obstacle_avoidance.launch
  to the retained `tunnel_static_3d.npz`, and observer.launch map/metadata defaults
  to the retained `tunnel_walls_empty_20261009.pcd/.json`; restart launch.
- Guard-only rollback: set `obstacle_stop_node/tunnel_swept_guard_enabled: false`
  in tunnel_recovery.yaml and restart launch.
- Replan-only rollback: set
  `obstacle_avoidance_node/embedded_engine/tunnel_replan_enabled: false` and restart.
- Prior overall tunnel policy: `tunnel_recovery_enabled:=false`; source backup is
  available for exact pre-change restoration. No automatic running-node restart
  or simulator control command is issued by this change.

Diagnostics: map_state map_version/compute_ms; candidate_state tracking_cte_m,
tracking_heading_error_rad, replan_pending; dynamic_state handoff_guard_reason,
handoff_cleared_ids. Existing topics and MPC interfaces are retained.
