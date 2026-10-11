#!/bin/bash
# 회전교차로 트래커 오프라인 재생 (2026-10-06).
#   일반 백(/odom, /mission/active_section) + heavy 백(/nonground)을 sim time 으로 함께 재생하고
#   트래커를 별도 출력 토픽(/replay/tracked_objects)으로 띄워 결과 백을 남긴다.
#   별도 ROS 마스터 포트를 써서 실행 중인 시스템과 섞이지 않는다.
# 사용: replay_tracker.sh MAIN.bag HEAVY.bag OUT.bag [추가 rosparam yaml]
set -e
MAIN=$1; HEAVY=$2; OUT=$3; EXTRA=$4
PORT=${REPLAY_PORT:-11399}
export ROS_MASTER_URI=http://localhost:$PORT
source ~/catkin_ws/devel/setup.bash
roscore -p $PORT >/dev/null 2>&1 & CORE=$!
trap 'kill $REC $NODE $CORE 2>/dev/null; wait 2>/dev/null' EXIT
for i in $(seq 1 50); do rosparam list >/dev/null 2>&1 && break; sleep 0.2; done
rosparam set use_sim_time true
SITE=$(rospack find mission_planner)/config/sites/kcity_roundabout_utm.yaml
rosparam load $SITE /roundabout_lidar_tracker
rosparam load $(rospack find total_bringup)/config/tuning_perception.yaml /
[ -n "$EXTRA" ] && rosparam load $EXTRA /roundabout_lidar_tracker
rosparam set /roundabout_lidar_tracker/lidar_tracker/topics/output /replay/tracked_objects
rosparam set /roundabout_lidar_tracker/lidar_tracker/expected_cloud_frame velodyne_ground
rosrun roundabout_lidar_tracker roundabout_lidar_tracker_node __name:=roundabout_lidar_tracker >/tmp/replay_tracker_node_$PORT.log 2>&1 & NODE=$!
rosbag record -O $OUT /replay/tracked_objects >/dev/null 2>&1 & REC=$!
sleep 2
rosbag play -q --clock $PLAY_ARGS $MAIN $HEAVY --topics /odom /nonground /mission/active_section >/dev/null 2>&1
sleep 1
kill -INT $REC; wait $REC 2>/dev/null || true
