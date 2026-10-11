# Original upstream components

- Apollo: https://github.com/ApolloAuto/apollo
  commit `d53aa3da47a06a08e6d0cd175d5623a34fa0d6aa`, Apache-2.0.
  Four original Piecewise Jerk problem/path headers and sources are under
  `vendor/occupancy_apollo/modules/planning/planning_base/math/piecewise_jerk`.
  Original LICENSE and SHA-256 file manifest are preserved in that vendor directory.
- OSQP: https://github.com/osqp/osqp
  tag `v0.6.3`, commit `0dd00a578cf1c2691c5c379965d504c75bf6cfad`, Apache-2.0.
  Original repository is under `vendor/occupancy_osqp`, with its original LICENSE.
  Its QDLDL submodule is pinned to `7d16b70a10a152682204d745d814b6eb63dc5cd2`.
- OctoMap and grid_map are linked from the workspace/system packages.
  Their upstream libraries are not patched by this package.

`adapters/` contains our standalone integration code: Cyber logging/checks,
minimal optional path-constraint types, planning flags and OSQP API compatibility.
`adapters/solver.hpp` reuses Apollo's original matrix formulation, handles OSQP
resource ownership, and exports optimization results without Apollo's full runtime.
These adapters are not presented as upstream code. Optional Apollo vehicle/corner
constraints are empty; the ROS-independent engine checks the vehicle footprint
and curb constraints explicitly. This package is not the complete Apollo planner.

M8 integrated escape additions (2026-10-09):
- Apollo original `reeds_shepp_path.{h,cc}` is compiled unchanged. Original
  `hybrid_a_star.{h,cc}`, `node3d.h`, and `grid_search.{h,cc}` are retained with
  SHA-256 in the same upstream manifest. Revision/license match the Apollo pin above.
- `src/joint_escape.cpp` is our ROS1 search adapter, using Apollo's bicycle
  expansion equations and original Reeds-Shepp connection. It replaces Cyber,
  protobuf runtime and broad open-space APIs with the existing measured occupancy,
  body/boundary checks, one R-to-D cusp, observation gates and bounded worker budget.
  The full original HybridAStar node is not linked and is not claimed to be.
- Minimal vehicle/warm-start/node/math types and logging in `adapters/` support the
  unmodified RS source. Existing QP formulation and 25 ms OSQP solver limit remain.
- Background distance comparison is a separate Python ROS1 adapter in `new_lane`.
  Its Autoware original files, pin, license and SHA manifest are preserved there.
