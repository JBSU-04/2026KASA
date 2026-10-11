"""Re-evaluate original COMMIT snapshots without ROS playback or publishing.

This is not closed-loop simulation: ego/NPC trajectories remain the recorded
ones. Conflict distance and MPC curvature cap are rounded debug values.
"""
import argparse
from collections import defaultdict, deque
import math
from pathlib import Path
import re
import subprocess

import rosbag
import yaml


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag")
    parser.add_argument("--config", required=True)
    parser.add_argument("--evaluator", required=True)
    parser.add_argument("--accel", type=float, default=2.2,
                        help="MPC ramp acceleration used during this recording")
    args = parser.parse_args()
    with open(args.config) as source:
        config = yaml.safe_load(source)
    rb = config["roundabout"]
    gs = rb["gap_select"]
    center = rb["center"]
    conflict = rb["conflict_point"]
    radius = rb["radius"]["nominal"]
    conflict_angle = math.atan2(conflict["y"] - center["y"], conflict["x"] - center["x"])
    direction = 1.0 if rb.get("counter_clockwise", True) else -1.0
    values = [gs["front_headway"], gs["seq_rear_headway"], gs["standstill_gap"],
              gs["check_horizon"], gs["speed_margin"], gs["entry_zone"], gs["max_wait"],
              gs["ttc_rear"], gs["late_margin"], gs["leader_start_distance"],
              int(gs["adaptive_leader_start"]), gs["leader_start_max_distance"],
              gs["confirm_seconds"], rb["safety"]["comfortable_decel"], gs["abort_margin"],
              gs["prediction_follow_standstill_gap"],
              gs["prediction_follow_time_gap"] if gs["use_mpc_prediction"] else 0.0,
              min(gs["ego_accel"], args.accel), gs["ego_start_delay"], gs["ego_length"]]
    rows = [" ".join(map(str, values))]
    histories = defaultdict(deque)
    vehicles = []
    previous_state = None
    speed = 0.0
    cap = rb["speed"]["circulation"]
    topics = ["/odom", "/perception/roundabout/tracked_objects",
              "/mission/roundabout/state", "/mission/roundabout/gap_markers"]
    with rosbag.Bag(args.bag) as bag:
        start = bag.get_start_time()
        for topic, msg, stamp in bag.read_messages(topics=topics):
            if topic == "/odom":
                speed = math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y)
            elif topic.endswith("gap_markers"):
                for marker in msg.markers:
                    match = re.search(r"at cap ([\d.]+)m/s", marker.text)
                    if match:
                        cap = min(float(match[1]), gs["ego_max_speed"] or rb["speed"]["circulation"])
            elif topic.endswith("tracked_objects"):
                vehicles = []
                sensor_time = msg.header.stamp.to_sec()
                for obj in msg.objects:
                    if not obj.velocity_valid or not obj.has_moved:
                        continue
                    dx, dy = obj.position.x - center["x"], obj.position.y - center["y"]
                    observed = math.hypot(dx, dy)
                    if not rb["radius"]["inner"] <= observed <= rb["radius"]["outer"]:
                        continue
                    angle = direction * (math.atan2(dy, dx) - conflict_angle)
                    pos = ((angle + math.pi) % (2.0 * math.pi) - math.pi) * radius
                    velocity = max(0.0, direction * (-dy * obj.velocity.x + dx * obj.velocity.y)
                                   * radius / (observed * observed))
                    history = histories[obj.id]
                    if not history or sensor_time > history[-1][0]:
                        history.append((sensor_time, velocity))
                    while len(history) > 1 and history[0][0] < sensor_time - 0.6:
                        history.popleft()
                    low = velocity
                    if len(history) > 1 and history[-1][0] - history[0][0] >= 0.2:
                        slowdown = max(0.0, (history[0][1] - history[-1][1]) /
                                       (history[-1][0] - history[0][0]))
                        low = max(0.0, velocity - min(slowdown, gs["leader_decel_limit"])
                                  * gs["leader_decel_horizon"])
                    vehicles.append([obj.id, pos, velocity,
                                     max(gs["min_vehicle_length"], obj.size.x), low, velocity,
                                     math.hypot(obj.velocity.x, obj.velocity.y)])
            else:
                if msg.state == 3 and previous_state != 3:
                    pair = re.search(r"seq L(\d+)/F(\d+)", msg.reason)
                    distance = re.search(r"conflict ([\d.]+)m", msg.reason)
                    if pair and distance:
                        row = [stamp.to_sec() - start, distance[1], speed, cap,
                               pair[1], pair[2], len(vehicles)]
                        rows.append(" ".join(map(str, row)))
                        rows.extend(" ".join(map(str, vehicle)) for vehicle in vehicles)
                previous_state = msg.state
    result = subprocess.run([str(Path(args.evaluator).resolve())],
                            input="\n".join(rows) + "\n", text=True,
                            capture_output=True, check=True)
    print("time_s ready feasible wait_s arrival_s rear_margin_m rear_ttc_s")
    print(result.stdout, end="")
    outputs = [line.split() for line in result.stdout.splitlines()]
    ready = sum(int(row[1]) for row in outputs)
    print("Original COMMIT snapshots: {}; now ready: {}; held/rejected: {}".format(
        len(outputs), ready, len(outputs) - ready))


if __name__ == "__main__":
    main()
