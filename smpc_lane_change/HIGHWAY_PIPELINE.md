# Highway mandatory lane transitions

The mission order is CSV 0 -> 1 -> 2 -> 3. The existing target selector
projects ego and tracked objects into lane coordinates and reports remaining
source-lane distance. CSV indices are not distances.

## Decision and execution

1. The section gate enables highway decisions. The target selector provides
   the current/target front and rear vehicles plus nearby tracked vehicles.
2. SMPC evaluates immediate change and delayed longitudinal preparation
   candidates using its existing time-indexed collision, clearance and TTC gates.
3. Every candidate must finish before the source CSV endpoint, including
   post-merge settling distance and an ego-length margin. Invalid distances
   and transitions outside 0->1->2->3 reject the candidate.
4. Only enabled/executable preparation actions participate in selection.
   Feasible HOLD/YIELD/ACCEL candidates compete by cost; acceleration remains
   opt-in through the existing behavior_execute_accel setting.
5. waypoint_system creates a distance-based quintic blend using monotonic
   segment projection between CSVs. Length is max(minimum length,
   current speed * configured blend duration). Insufficient source length
   or a disconnected target produces no usable preview.
6. SMPC and waypoint_system use the shared spatial_blend.hpp generator.
   Execution freezes the preview snapshots and publishes a rolling window
   of that fixed path. Pose progress, not elapsed time, ends the spatial plan.
   Target CSV metadata and a target-path tail are retained.
7. The existing supervisor confirms physical target-lane occupancy. The
   /path_number handoff still starts execution; it is not a completion signal.
8. Existing ACC/end-of-path stopping and low-speed endpoint connectors remain.
   Missing previews cannot authorize fallback geometry.

## Parameters

- waypoint_system/config/base.yaml: path_switch_spatial_blend_enabled_,
  path_switch_spatial_min_length_m_, existing blend duration.
- smpc_lane_change/config/target_selector.yaml: lane-end preparation windows.
- smpc_lane_change/config/smpc_decision.yaml: collision gates, prediction horizon,
  preparation action enablement and costs.
- smpc_lane_change/config/supervisor.yaml: physical completion checks.

## 60-70 km/h profile

- Mission state 3 is capped at 19.44 m/s (70 km/h); state 4 remains 18.06 m/s
  (65 km/h). The SMPC/ACC stale-speed fallback is also 18.06 m/s.
- A normal lane change uses a 3.5 s, 60 m minimum spatial blend. Its length is
  about 60 m at 60 km/h and 68 m at 70 km/h. The 1.0 s post-merge buffer keeps
  the complete maneuver feasible in the first 118.5 m mandatory-change zone.
- ACC searches 100 m ahead and uses 0.8 s headway. The target-lane rear gate
  keeps 1.2 s headway and at least 3.5 s TTC.
- Normal SMPC yield preparation may reduce the episode entry speed by at most
  6 m/s and never requests less than 12 m/s. Independent collision/emergency
  braking remains able to command stronger deceleration.

MPC tuning and its global curvature profile are unchanged. Curvature of the
new spatial path is retained for tracking and speed planning.

## Verification and limits

Geometry tests cover unequal sampling, curved roads, reproducibility,
insufficient source length and disconnected targets. These do not establish
collision-free mission completion with live NPCs. Replay must check all three
transitions, fast rear approach, adjacent traffic, blocked deadlines, stopping
during a transition, and target-path completion.

This retains the existing representative-gap candidate planner; it does not
implement an exhaustive search over every pair of NPCs or a new in-motion
abort trajectory generator. The existing supervisor recovery policy remains.
PathSwitchPreview gained a field: restart its publishers/subscribers together
after rebuilding. Do not mix old and new message binaries.
