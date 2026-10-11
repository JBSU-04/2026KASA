# Pinned upstream sources

Only the modified libpointmatcher backend from Perfectly Constrained is compiled.
The node enables official X-ICP detection and OptimizedEqualityConstraints and
refuses to start a different solver. No Open3D SLAM node is launched.

| Source | Revision | Retained license |
| --- | --- | --- |
| https://github.com/leggedrobotics/perfectlyconstrained | 0fbe4175ea205a271f85287abfd8048e5f7dd32a | third_party/perfectlyconstrained/LICENSE and libpointmatcher source headers |
| https://github.com/norlab-ulaval/libnabo | 0ce6ecbf1d845b9e1238f8e845a74a50cec04432 | third_party/libnabo/LICENSE |
| https://github.com/asherikov/qpmad | 11602ec9f87a62a97ba8318ebd81da6646f3c631 | third_party/qpmad/LICENSE |
| https://github.com/ANYbotics/message_logger | 1ea41e4829580848c005b437068d0a1f68f7bdcf | third_party/message_logger/LICENSE |
| https://github.com/ceres-solver/ceres-solver | f68321e7de8929fbcdb95dd42877531e64f72f66 (2.1.0) | third_party/ceres/LICENSE |

Local upstream safety patch: `calculatePointCloudCenter()` in
`libpointmatcher/pointmatcher/ICP.cpp` used inclusive bounds on a three-element
vector. Both bounds are now exclusive (`< featDim - 1`), eliminating an
out-of-bounds access. The X-ICP detection and constraint algorithm is unchanged.
QPMAD/logger headers are generated in the build directory, not patched upstream.
Ceres uses an isolated prefix and miniglog; no system packages are replaced.

Diagnostic-only patches: the equality branch now fills the existing equality/total
constraint counters (upstream filled them only in the inequality branch).
Unconditional equality-solver matrix dumps are debug logs, and requesting disabled
printing is no longer logged as an error. Solver matrices and results are unchanged.
