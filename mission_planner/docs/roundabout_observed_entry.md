# Observed-gap departure admission (ROS1)

The competition profile enables `roundabout/gap_select/observed_entry_enabled`.
This is a ROS1 implementation in the existing mission node, not an installation
of Autoware or SUMO and not a claim to reproduce either project's entire planner.
The existing local ego-profile integration, circular tracking and ACC are reused.
No upstream source was copied verbatim in this change.

## Algorithm references / provenance

- Autoware `autoware_behavior_velocity_roundabout_module`, reference revision
  `c31096851b321828b0c2664765c36cb1998c0f62` (Apache-2.0).
  [Decision source](https://github.com/autowarefoundation/autoware_universe/blob/c31096851b321828b0c2664765c36cb1998c0f62/planning/behavior_velocity_planner/autoware_behavior_velocity_roundabout_module/src/scene_roundabout.cpp):
  continuous SAFE confirmation, geometry-based entry areas, and ego/object
  spatiotemporal occupancy. Unlike upstream permanent-GO handling, our user-selected
  policy latches at the final departure command and does not recheck rear collisions.
- Eclipse SUMO, `MSLink.cpp`, reference revision
  `53aa031d2592c432e140b24d82a3f37be3d439cd`.
  [Junction-model documentation](https://sumo.dlr.de/docs/Definition_of_Vehicles,_Vehicle_Types,_and_Routes.html#junction_model_parameters):
  `jmAdvance` times arrival behind a crossing/merging priority vehicle.
  Our existing `timedApproachSpeedCap` provides analogous arrival timing through
  speed constraints; SUMO simulation state, exact NPC states and TraCI are not used.

## Live inputs and behavior

- Complete unfiltered `/perception/points_gravity` scan (raw LiDAR with existing
  IMU gravity alignment), plus same-scan `/nonground`, both `velodyne_ground`.
- Scan-aligned odometry and the same sensor extrinsic as the roundabout tracker.
  Pitch must not be applied again. The global `/lidar_extrinsic/z_from_ground`
  overrides the site's fallback measured sensor height (1.765 m).
- A 0.5 m scan-local grid carves FREE only before actual returns at vehicle height.
  Non-ground endpoints override FREE. Occlusion, unobserved areas, wrong frames,
  missing scans and stale inputs are never treated as empty space.
- The measured free arc limits slot capacity, including one-sided or empty slots.
  Open slots require three distinct consecutive fresh scans and enough observed
  upstream/downstream extent for the predicted complete-merge time, using 8 m/s
  for an unseen upstream car. Thus empty track lists alone cannot approve entry.
- Final admission requires <=8 m path distance to the yield point, same vehicle
  pair, >=0.1 s uninterrupted feasibility and distinct tracker/observation scans.
  Current MPC geometry capacity and ramp limits are prediction inputs only.
- Predict until complete rear clearance +0.5 s, at 0.1 s steps. Rear cooperation
  starts after ego first occupies the circulating band, then 0.5 s response and
  2 m/s^2 deceleration. No reaction is credited before actual predicted entry.
- At the final command, latch the pair and enter COMMIT. The response delay does
  not permit cancellation. No gap reevaluation, rear TTC, slot switching or
  standalone collision guard runs after approval. Front ACC and existing
  localization/tracker protections remain. Front ACC can still brake for a leader.

## Diagnostics and rollback

The existing `/mission/roundabout/distance_debug` JSON adds observation state/time,
finite observed free length, confirmation duration, approval time/distance and
`approval_fixed`, plus ACC target ID/distance/speed. Entry-phase RViz labels distinguish CANDIDATE, READY and COMMITTED;
cyan cells are observed free, red cells occupied, absent cells unknown.
The approved gap band is purple. Approval-time entry/clear estimates are retained
for diagnostics, not reused as fresh collision predictions after COMMIT.

To return to the prior trajectory-aware mode, set `observed_entry_enabled: false`
and keep `spatial_only_admission: false`; its legacy rolling distance/speed gates
apply again. Complete pre-change files are backed up under
`backups/roundabout_observed_entry_20261008/`. No runtime parameter was changed and
no live node or controller was restarted during implementation.

## Verification boundary

Core tests cover observation, bounded/open slots, repeated cached scans, pair
changes, distance gates, response delay, rear intrusion, and independent front ACC.
Offline replay accepts `--sensor-bag HEAVY_BAG --route CSV`; it reconstructs the
same observation grid from recorded rays and non-ground data and publishes nothing.
Recorded ego/NPC trajectories remain fixed: changes in actual braking, new sensor
visibility while creeping, and NPC yielding require MORAI closed-loop validation.
