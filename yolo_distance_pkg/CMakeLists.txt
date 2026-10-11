cmake_minimum_required(VERSION 3.0.2)
project(yolo_distance_pkg)

find_package(catkin REQUIRED)

catkin_package()

catkin_install_python(PROGRAMS
  distance_estimator.py
  yolo_ttc_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Keep the large, local model files beside the installed executable so the
# estimator can find them automatically via __file__.
install(FILES
  depth_unet.pt
  yolov8n-seg.pt
  scale.json
  README.md
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY models/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/models
)
