# Non-GPS driving stack

`non_gps` is an ordinary source grouping directory, not a ROS package. Catkin
discovers the six ROS packages beneath it.

## Packages

- `lane_perception_pkg`: TwinLiteNet lane mask and sliding-window steering angle.
- `lidar_perception_pkg`: ground removal, 3D cluster-box tracking, and motion classification.
- `mission_pkg`: separate static- and dynamic-obstacle behavior policies.
- `control_pkg`: mission priority mux and the only `/ctrl_cmd` publisher in this stack.
- `non_gps_msgs`: messages shared across perception, mission, and control.
- `non_gps_bringup`: perception-only and complete-system launch files.

## Intended data flow

```text
camera -> TwinLiteNet -> lane mask -> sliding window -> steering angle ---+
                                                                    control mux -> vehicle control -> /ctrl_cmd
LiDAR -> ground grid -> clustering -> object tracking/classification     |
                                      |                                  |
                                      +-> static mission ----------------+
                                      +-> dynamic mission ---------------+
```

The object tracker associates complete cluster boxes using center distance, box-size
similarity, and BEV IoU. It estimates a filtered velocity and publishes persistent IDs,
box dimensions, and `UNKNOWN`/`STATIC`/`DYNAMIC` state. In RViz, tracked boxes are
yellow while unconfirmed, green when static, and red when dynamic.

The current velocity is relative to the LiDAR frame. Before using motion classification
while the ego vehicle is moving, compensate ego motion or transform boxes into a fixed
frame; otherwise a world-static object can appear to move relative to the vehicle.

## Launch

```bash
roslaunch non_gps_bringup perception.launch
roslaunch non_gps_bringup non_gps_system.launch
```

Set `run_twinlitenet:=false` to test downstream nodes without loading the model,
or `use_rviz:=true` to start the LiDAR RViz configuration.
