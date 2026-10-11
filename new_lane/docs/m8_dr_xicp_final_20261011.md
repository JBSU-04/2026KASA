# M8 DR + X-ICP and shared object policy

Applied only to M8. MPC tuning, speed settings, the 1.3 m trigger corridor,
the GPS 3D map and the physical wall profile are unchanged. `src_1009` and
`src_음영구간수정전` were compared as behavior references, not copied wholesale.

## Execution and inputs

`roslaunch total_bringup competition.launch` now selects `tunnel_heading_mode:=xicp`.
UFLD and straight-wall heading fallback are not enabled. Existing X-ICP quality,
300 ms result age and 0.2 degree observation correction limits remain in force.
M8 map lookup uses the corrected DR pose without another registration offset or
independent lateral wall alignment. Scan poses are interpolated, or extrapolated
for at most 80 ms using measured odometry twist, instead of relabeling old poses.

The simulator must actually send vehicle status to the participant computer's
UDP port **9099** (existing simulator Host Port 9088). `/Competition_topic` must
carry real velocity. Internet/network profiles and UDP ports were not changed.
No live launch/listener was present initially. A receive-only five-second check
on UDP 9099 received **zero packets**, so actual status reception is **not
verified**. The bridge parser tests passed. This does not prove a configuration
fault when the simulator is not transmitting.

`/localization/speed_valid` and `/localization/speed_state` separate valid measured
zero speed from missing speed. In M8, missing real vehicle speed does not create
a GPS-derived or zero-speed twist observation. The existing stop node emits
`speed_input_expired`; reverse control cannot override this stop. Restore actual
status reception before driving. X-ICP yaw cannot replace a missing speed input.

## Common evidence and object policy

Raw gravity and nonground use the same scan pose and exact background thresholds.
The known-floor distance is 0.055 m; general 3D background distance remains 0.12 m.
Exact XY floor lookups and background queries are reused within a scan.
Only matched points are removed. A long fitted cluster cannot delete unmatched
protrusions. Physical walls constrain the vehicle center by half-width 0.95 m
plus margin 0.1 m, with no extra wall inset or virtual entrance extension.

Known-ground raw residuals confirmed by 3 points/two distinct scans are attached
to the matching nonground scan and enter the same shared detector/tracker. Exact
duplicate returns are removed, but M8 detection input is not voxel-thinned.
Legacy far proposals are not fed back into the M8 merger: their raw-map evidence
already enters the common tracker. Parent retention, real-ray clearance and
native scene IDs remain authoritative.

The shared detector keeps one pending latest scan. Exact XY radius connectivity
and swept-body tests now use the C++ `m8_geometry` library; Python implementations
remain available. Thresholds and world-coordinate double precision are preserved.
The dense-cloud point-count cap does not reject M8 objects merely because input
thinning was removed. Other mission clustering remains unchanged.

New object metadata:

- `ever_dynamic`: tracked dynamic history, retained after handoff.
- `m8_stationary_seconds`: stationary proof from original observed scan times.
- `m8_restatic`: a previously dynamic, stationary blocking object is available
  to the existing static avoidance engine.
- `m8_handoff_reason`: `observed_stationary_blocker_5s` on that transition.

Handoff requires observed surface speed <=0.18 m/s, surface displacement <=0.3 m,
5 seconds of observations and intrusion into the reference corridor. Missing
observations over 350 ms or coordinate-version changes reset confirmation.
Box growth is not movement proof. Two independent moving observations revoke
the handoff. Actual body proximity remains a stop reason throughout.

Preslow remains 20 m / 6 m/s. A certified static object's own limit is released
only when a fresh approved path clears its owned cells. Other owners, crossing
risks, actual near-body danger and sensor faults are not released with it.

Reverse escape can start only after a fresh spatial/kinematic failure from the
forward owner. `stale_lidar`, expired inputs, dynamic waiting and search timeout
do not trigger reverse. The existing integrated forward/reverse planner must
validate forward passage before authorizing reverse; speed, gear, tracking and
wall safety limits are unchanged.

## Diagnostics and rollback

Added `/tunnel/detection_processing_ms` and `/tunnel/detection_queue_ms`; existing
map, merger, scene and plan-input metrics retain original scan timestamps.
Default RViz remains compact: walls, object state, paths, stop reason and heading.

Backup and test artifacts: `analysis/m8_dr_xicp_final_20261011/`.
MPC hashes and map-asset hashes are recorded there. No map was rebuilt.

- Heading rollback: `tunnel_heading_mode:=dr` on competition launch.
- Wall policy rollback: `tunnel_physical_boundary_policy:=false`.
- Object handoff rollback: `tunnel_recovery.stationary_handoff_enabled: false`
  in `tunnel_recovery.yaml`, then restart.
- Numeric fallback: `tunnel_recovery.native_geometry_enabled: false`.
- Callback processing fallback: `tunnel_recovery.detection_worker_enabled: false`.
- Full source restoration: restore the backed-up relative paths and rebuild.
  Keep speed reception checks; do not disable input validity to conceal loss.

## Validation and remaining acceptance

No bags were replayed. Python/unit, C++ map/pose, existing forward/escape and
isolated ROS scene checks were executed. Isolated scene tests inject valid
measured-speed health while supplying synthetic odometry; they do not validate
the simulator connection, actual X-ICP tracking or closed-loop passage.

Real-drive targets: map aggregate p95 <=60 ms, detector stage <=60 ms, merger
<=30 ms, scene <=10 ms, scan-to-plan <=250 ms, X-ICP <=50 ms. The 350 ms expiry
was not relaxed. Synthetic timing is not evidence these targets hold live.

Next: verify actual vehicle status, drive an empty tunnel with DR+X-ICP, then
test low static avoidance, crossing stops, five-second handoff and joint escape.
Only after delay/localization acceptance should two user-confirmed GPS-normal
empty runs be used to rebuild background and physical boundaries.
