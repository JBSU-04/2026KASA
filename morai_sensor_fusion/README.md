# MORAI Autoware EKF integration

This package keeps the existing MORAI ROS topic contract while adapting GPS,
IMU and Competition vehicle speed to the pinned Autoware.AI `ekf_localizer`.
The filter state is `[x, y, yaw, yaw_bias, vx, wz]`, predicts at 50 Hz, supports
delayed measurements, and uses innovation-covariance Mahalanobis gates.

Important behavior:

- IMU linear acceleration is not integrated.
- Fresh IMU and vehicle speed continue predicting during a short GPS gap.
- GPS fixes are timestamp-aligned with buffered IMU yaw and corrected for the
  antenna lever arm.
- Position covariance grows with `v * timestamp_stddev`, which matters at
  100 km/h.
- `/odom_fused` and compatibility `/odom` are published at 50 Hz.
- `/localization/valid` becomes false on stale IMU/speed, prolonged GPS loss,
  stale EKF output, or excessive covariance.

Run standalone:

```bash
roslaunch morai_sensor_fusion autoware_ekf_morai.launch
```

The vendored EKF sources and exact upstream commits are recorded in the
`NOTICE` files of `ekf_localizer` and `amathutils_lib`.

## Reproducible report graphs

Collect a live stationary or driving interval while the EKF is running:

```bash
rosrun morai_sensor_fusion ekf_report_graphs.py \
  --live-seconds 20 --output-dir ~/ekf_report_graphs
```

Analyze the same topics from a rosbag instead:

```bash
rosrun morai_sensor_fusion ekf_report_graphs.py \
  --bag run.bag --output-dir ~/ekf_report_graphs
```

The output contains aligned CSV data, `summary.json`, trajectory and position
time-series plots, metric bars and a yaw-stability plot. If `/global_path` is
available during driving, metrics are reference-path CTE. A stationary capture
uses center-relative position stability; a moving capture without a reference
uses one-second high-frequency residuals. Competition status contributes speed
only and is never used as localization ground truth.
