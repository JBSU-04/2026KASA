# M8 safety changes

Braking and obstacle ownership are separate. The planner still sees the reference
route and obstacle list; a clear generated path never deletes an avoidance target.
The stop node classifies objects before checking the driven path. Fresh `map`
local paths use segment/shape clearance; missing, stale (>0.5 s), invalid, or short
paths fall back to the reference decision. Body proximity and dynamic reference,
approach, and future driven-path conflicts remain stop conditions.

Measured static evidence requires fresh (0.3 s), independent position samples,
both 0.3/0.5 s windows below 0.3 m/s, spread below 0.25 m and established stillness.
A validated history can survive at most 0.3 s of non-coherent box wobble. Coherent
motion, stale measurements and missing windows cannot use this protection.
Raw tracker stationary_sec is unchanged. These decisions are M8-only.

The existing `/obstacle_stop/dynamic_state` JSON fields retain their meaning.
`brake_required` includes non-dynamic hold, handoff guard and avoidance failure;
`stop_decisions` reports each examined object ID, reason, local/reference fallback
selection and both edge distances. BG-only triggers still use their original
conservative safety branch. This is not a replacement for actuator feedback.

BG input now uses `velodyne_ground` with timestamp-interpolated odometry (endpoint
tolerance 0.08 s, interpolation gap limit 0.16 s, age limit 0.3 s). Regional 6 x 4 m tiles fit road planes using
low metre-cell seeds and neighbouring tiles. Minimum 12 seeds, slope <=0.12,
inlier band 0.12 m, RMS <=0.07 m and spatial span gates reject unsupported planes.
Far unsupported low candidates are not confirmed; <=8 m retains conservative
legacy proposals. The legacy background map is not regenerated. Differences
>0.15 m between height models are uncertainty, NOT positive object evidence.
Incompatible points are compared again using their legacy height. Legacy-novel
points remain candidates; other far points stay unverified without refreshing kick
memory. Within 8 m the legacy height band retains conservative proposals. Where
current direct walls exist, this fallback also retains the original measured-wall
inset; map-novel evidence is not disabled by that fallback-only gate. With missing
or stale direct walls, near fallback remains conservative.
Background rejection requires both legacy-map agreement and a directly measured
wall (age <=0.3 s, scan mismatch <=0.08 s) or valid curb-profile geometry, within
0.15 m. Predicted walls alone never reject points. Filtering is per point, not per
whole object cluster. Verified background clears old BG memory unless a foreground
candidate is nearby. Candidate labels
show per-cluster state, reason, confirmation count and height compatibility. Existing motion tracking
and 1.5 s candidate expiry remain in use.

`/tunnel/bg_candidates_debug` (JSON String, recorded by competition.launch) reports
candidate IDs, states, reasons, counts, distances and individual background checks.
Unverified observations do not contribute hits. Confirmed refers to the existing
memory-hit threshold, while `ahead` separately reports the existing two-frame kick
confirmation. Retained memory is marked `remembered` and includes observation age.
RViz cylinders are yellow for unverified, orange for candidate and red for confirmed.
IDs remain stable while a qualified candidate's existing merge-memory survives.

Verification: unit tests, builds, and limited direct bag-message inspection only;
no bag replay, live node restart, or road-test performance claim. Replay consumers
of XicpStatus must use embedded bag definitions for older recordings, since the
new optional stage timing fields change its ROS message MD5.

Limited inspection of the latest run's scan near 44.61 s reduced low-height
novel proposals from 282 to 32. There were 168 total novel points after conservative
height-compatibility protection, including 16 near the actual obstacle. This is a
single-scan check of the initial regional-ground implementation, not evidence of complete false-positive elimination or latency
performance on the vehicle.

After the background-state correction, direct inspection of four gravity scans
at 87.561, 93.064, 100.088 and 107.098 s in competition_2026-10-02-15-20-30.bag
produced zero eligible kick candidates. Unverified clusters remained visible.
The check used recorded nonground scans for direct wall fitting (fixed random seed,
without historical predicted-wall state), recorded odometry and route/profile
geometry. It did not replay the full candidate-memory/control pipeline and does
not establish full-drive false-positive elimination or real-obstacle recall.

## M8 re-stop consistency (2026-10-02, 16:03 run)

The stop node subscribes to `/tunnel/wall_limits` and uses measured curb
distances only while fresh (0.5 s), transformed through the original waypoint
tangent and current vehicle lateral position. Offset freshness is also 0.5 s.
The planner uses the same original-waypoint transform. Background/profile
classification remains available when live limits expire. Fresh inward normal
motion (at least 0.5 m/s) overrides wall exclusion, including wall-like shape
exclusion in the stop node. Along-wall centroid drift is not inward motion.

M8 predictive swept-collision checking follows a fresh local path with a fresh
ego pose, stepping 0.1 s across the configured 4 s horizon at current ego speed.
The existing vehicle rectangle, object radius and capped velocity uncertainty
are each applied once. Missing, stale, degenerate or horizon-short paths fall
back to the existing conservative model. Immediate approach, dynamic history,
body proximity, reverse protection and stop hold remain unchanged.

Small unknown fragments can inherit an independently measured static parent's
stop classification only after two distinct observations of both tracks and
0.3 m-expanded footprint overlap. Coherent fragment motion, dynamic parents and
recent dynamic parents prevent inheritance. Both original occupied geometries
remain available to the planner; association does not remove collision points.
Non-dynamic unknown observations older than 0.3 s cannot refresh a stop hold
unless there is immediate body proximity. Existing hold duration is unchanged.

M8 no longer retries search with a reduced near-pass radius. Clamp, postprocess,
original-candidate fallback and previous-path reuse are checked against the
latest obstacle geometry, normal radius, identical wall-offset shift, road
width, curb/body limits and curvature limit. If none passes, avoidance failure
still commands a stop. Reference-path obstacle ownership is unchanged: a clear
avoidance path does not itself promote the reference path.

`/obstacle_avoidance/validation_debug` reports validation stage, geometry stamp,
normal radius, collision/curb fractions, curvature and attributed obstacle IDs;
`attribution_complete` flags geometry not covered by per-ID attribution (e.g.
held or extended geometry). An empty search has no feasible candidate; its
sentinel fractions are not a measured collision fraction for a driven path.
The topic is recorded by competition.launch. Stop diagnostics include wall
exclusion reason, prediction path/time and static-parent ID.

Verification: 79 new_lane Python tests pass, including 12 new re-stop regressions.
The planner source-contract regression checks shared validation wiring but is
not a C++ runtime geometry test. obstacle_avoidance_node builds successfully.
Limited direct inspection of ID55 at 42.345 s in
competition_2026-10-02-16-03-41.bag, using recorded original waypoints, odometry,
wall offset and limits (age 0.068 s), classifies it as `live_limits` background.
This is not a full-pipeline replay or proof that all road-test stops are fixed.
No live node restart or complete bag replay was performed.

## Independent M8 avoidance corridor (1.30 m)

The planner launch now sets `robot_radius=1.30` (previous workspace setting:
1.25). The stop launch adds `tunnel_avoidance_corridor_radius=1.30`. This new
radius applies only to M8's actual local-path edge comparisons, predicted
positions compared to that path, and dynamic-to-static handoff on that path.
The reference-path and immediate-body guards retain
`tunnel_path_corridor_radius=1.65`; other missions retain their existing radius.
TTC rectangle width, uncertainty margins, classification, reverse protection,
and reference-path avoidance ownership are unchanged. The 0.9 m near-pass
retry remains disabled.

The edge distance already subtracts object half-width; the new corridor does
not add it again. M8 handoff uses the same segment-based edge distance and
finite-path-horizon protection as ordinary driven-path decisions. If a path
is missing, stale, or does not cover the object, existing reference/ego-frame
fallbacks remain active. An absent new parameter preserves the old tunnel
radius; a non-positive, non-finite or non-numeric value warns and falls back.

Verification: 90 new_lane tests pass, including 11 dedicated corridor tests
covering the 1.30 m boundary, independent tuning, unchanged proximity/dynamic
reference guards, predicted positions, handoff, path fallbacks, parameter
validation and matching launch values. This change does not unify the planner's
three-circle footprint with the stop node's path-edge geometry, nor resolve
the separately identified replan/handoff deadlock. No complete bag replay or
live node restart was performed; launch values apply on the next node start.
