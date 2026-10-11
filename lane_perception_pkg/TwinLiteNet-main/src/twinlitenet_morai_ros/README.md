# TwinLiteNet MORAI ROS Node

ROS1 node for running TwinLiteNet inference from a MORAI camera image topic.

## Build

This repository now has a catkin-style `src/` layout:

```bash
cd /home/kuuve/TwinLiteNet-main
catkin_make
source devel/setup.bash
```

Install Python dependencies in the same Python environment used by ROS:

```bash
pip3 install torch torchvision opencv-python numpy
sudo apt install ros-noetic-cv-bridge
```

## Run

Default launch subscribes to `/image_jpeg/compressed` and publishes an overlay image to
`/twinlitenet/overlay`.

```bash
roslaunch twinlitenet_morai_ros twinlitenet_morai.launch
```

For a raw `sensor_msgs/Image` topic:

```bash
roslaunch twinlitenet_morai_ros twinlitenet_morai.launch \
  image_topic:=/camera/image_raw compressed:=false
```

For a different TwinLiteNet repository path:

```bash
roslaunch twinlitenet_morai_ros twinlitenet_morai.launch \
  repo_root:=/home/kuuve/TwinLiteNet-main
```

To view the result:

```bash
rqt_image_view /twinlitenet/overlay
```
