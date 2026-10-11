#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Temporarily force every mapped MORAI traffic light to green."""

import json

import rospy
from morai_msgs.msg import SetTrafficLight


def load_ids(path):
    with open(path, encoding="utf-8") as stream:
        nodes = json.load(stream)
    return sorted({str(node.get("traffic_light_id"))
                   for node in nodes
                   if node.get("traffic_light_id")})


def main():
    rospy.init_node("force_green_traffic_lights")
    path = rospy.get_param("~node_set", "")
    ids = rospy.get_param("~traffic_light_ids", [])
    if isinstance(ids, str):
        ids = [item.strip() for item in ids.split(",") if item.strip()]
    if not ids:
        if not path:
            rospy.logfatal("~node_set or ~traffic_light_ids is required")
            return
        try:
            ids = load_ids(path)
        except (OSError, ValueError, TypeError) as exc:
            rospy.logfatal("cannot load traffic-light IDs: %s", exc)
            return

    pub = rospy.Publisher("/SetTrafficLight", SetTrafficLight, queue_size=1)
    rate = rospy.Rate(rospy.get_param("~rate", 2.0))
    rospy.logwarn("forcing %d traffic lights to GREEN; stop this node to restore control",
                  len(ids))
    while not rospy.is_shutdown():
        for traffic_light_id in ids:
            pub.publish(SetTrafficLight(trafficLightIndex=str(traffic_light_id),
                                        trafficLightStatus=16))
        rate.sleep()


if __name__ == "__main__":
    main()
