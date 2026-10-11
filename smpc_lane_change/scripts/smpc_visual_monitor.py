#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
SMPC Safe Zone & ACC Target Real-Time Visual Monitor
Displays real-time ASCII HUD in terminal for:
 1. Lane Change Safe Zone ([Front +10m / Rear -15m])
 2. Active ACC Target Vehicle (~15m gap tracking, TTC, closing speed)
 3. Speed & Longitudinal arbitration status
"""

import rospy
import math
import os
import sys
from std_msgs.msg import Float64, String
from visualization_msgs.msg import MarkerArray, Marker
try:
    from smpc_lane_change.msg import LaneTargets, BehaviorLongitudinalStatus
except ImportError:
    LaneTargets = None
    BehaviorLongitudinalStatus = None

class SmpcVisualMonitor:
    def __init__(self):
        rospy.init_node("smpc_visual_monitor", anonymous=True)

        self.acc_marker_text = "No ACC Target"
        self.acc_marker_dist = 0.0
        self.acc_has_target = False

        self.safe_zone_text = "Safe Zone: Waiting..."
        self.safe_zone_occupied = False
        self.side_veh_info = ""

        self.ego_speed_mps = 0.0
        self.cmd_speed_mps = 0.0
        self.active_lead_reason = "none"
        self.current_lane = -1
        self.target_lane = -1
        self.decision_mode = "WAIT"

        # Subscribers
        rospy.Subscriber("/smpc/acc_target_marker", MarkerArray, self.acc_marker_cb)
        rospy.Subscriber("/smpc/yield_zone_markers", MarkerArray, self.yield_zone_cb)
        rospy.Subscriber("/smpc/acc_speed", Float64, self.acc_speed_cb)
        rospy.Subscriber("/smpc/acc_active_lead", String, self.active_lead_cb)
        rospy.Subscriber("/smpc/decision_debug_marker", Marker, self.decision_cb)
        if LaneTargets is not None:
            rospy.Subscriber("/smpc/targets", LaneTargets, self.targets_cb)
        if BehaviorLongitudinalStatus is not None:
            rospy.Subscriber("/smpc/behavior_longitudinal_status", BehaviorLongitudinalStatus, self.behavior_status_cb)

        self.timer = rospy.Timer(rospy.Duration(0.1), self.render_hud)

    def acc_marker_cb(self, msg):
        has_text = False
        for m in msg.markers:
            if m.action == Marker.DELETEALL:
                self.acc_has_target = False
                self.acc_marker_text = "None (Cruising / Path Clear)"
                return
            if m.ns == "acc_target_text":
                self.acc_marker_text = m.text
                has_text = True
                self.acc_has_target = True
        if not has_text and len(msg.markers) == 0:
            self.acc_has_target = False
            self.acc_marker_text = "None (Cruising / Path Clear)"

    def yield_zone_cb(self, msg):
        for m in msg.markers:
            if m.ns == "yield_zone_text":
                self.safe_zone_text = m.text
                if "HOLD" in m.text or "YIELD" in m.text or "WAIT" in m.text:
                    self.safe_zone_occupied = True
                else:
                    self.safe_zone_occupied = False

    def acc_speed_cb(self, msg):
        self.cmd_speed_mps = msg.data

    def active_lead_cb(self, msg):
        self.active_lead_reason = msg.data

    def decision_cb(self, msg):
        self.decision_mode = msg.text.replace("\n", " | ")

    def targets_cb(self, msg):
        self.ego_speed_mps = msg.ego_v
        self.current_lane = msg.current_lane_id
        self.target_lane = msg.target_lane_id

    def behavior_status_cb(self, msg):
        self.current_lane = msg.current_lane_id
        self.target_lane = msg.target_lane_id

    def render_hud(self, event):
        # ANSI color codes
        RESET = "\033[0m"
        BOLD = "\033[1m"
        RED = "\033[31m"
        GREEN = "\033[32m"
        YELLOW = "\033[33m"
        BLUE = "\033[34m"
        CYAN = "\033[36m"
        WHITE = "\033[37m"

        ego_kmh = self.ego_speed_mps * 3.6
        cmd_kmh = self.cmd_speed_mps * 3.6

        hud = []
        hud.append(f"{BOLD}{BLUE}======================================================================{RESET}")
        hud.append(f"{BOLD}{WHITE}   [SMPC HIGHWAY] SAFE ZONE & ACC 15m TARGET VISUAL MONITOR{RESET}")
        hud.append(f"{BOLD}{BLUE}======================================================================{RESET}")
        hud.append(f" {BOLD}Ego Lane:{RESET} Lane {self.current_lane} -> Target: Lane {self.target_lane}  |  "
                   f"{BOLD}Speed:{RESET} Current: {ego_kmh:4.1f} km/h  |  Target: {cmd_kmh:4.1f} km/h")
        hud.append(f" {BOLD}Decision:{RESET} {CYAN}{self.decision_mode[:55]}{RESET}")
        hud.append(f" {BOLD}Arbiter Reason:{RESET} {YELLOW}{self.active_lead_reason}{RESET}")
        hud.append(f"{BLUE}----------------------------------------------------------------------{RESET}")

        # Section 1: Safe Zone
        hud.append(f"{BOLD}[1] Lane Change Safe Zone Status [Front +10.0m / Rear -15.0m]{RESET}")
        if self.safe_zone_occupied:
            zone_color = RED
            zone_badge = f"{RED}[ OCCUPIED / YIELDING ]{RESET}"
        else:
            zone_color = GREEN
            zone_badge = f"{GREEN}[ CLEAR / SAFE TO MERGE ]{RESET}"

        hud.append(f"  Safe Zone State: {zone_badge}")
        hud.append(f"  Safe Zone Detail:\n    {zone_color}{self.safe_zone_text.replace(chr(10), chr(10) + '    ')}{RESET}")
        hud.append(f"{BLUE}----------------------------------------------------------------------{RESET}")

        # Section 2: ACC Target Follow
        hud.append(f"{BOLD}[2] ACC Active Lead Target Tracking (~15m Nominal Gap / TTC){RESET}")
        if self.acc_has_target:
            hud.append(f"  Target Status: {GREEN}[ ACTIVE TRACKING ]{RESET}")
            target_lines = self.acc_marker_text.split("\n")
            for line in target_lines:
                if "Target: ~15m" in line:
                    hud.append(f"    {CYAN}{BOLD}{line}{RESET}")
                elif "TTC" in line:
                    if "Safe" in line:
                        hud.append(f"    {GREEN}{line}{RESET}")
                    else:
                        hud.append(f"    {YELLOW}{line}{RESET}")
                else:
                    hud.append(f"    {WHITE}{line}{RESET}")
        else:
            hud.append(f"  Target Status: {CYAN}[ NO LEAD VEHICLE IN RANGE - CRUISE MODE ]{RESET}")
            hud.append(f"    {self.acc_marker_text}")

        # Visual Gap Meter
        hud.append(f"{BLUE}----------------------------------------------------------------------{RESET}")
        hud.append(f"{BOLD}[3] Spatial Gap Scale Visualizer{RESET}")
        meter = "  -15m [Rear Safe] -------- [EGO (0m)] -------- +10m [Front Safe] ---- +15m [Target Gap] -- +"
        hud.append(meter)
        hud.append(f"{BOLD}{BLUE}======================================================================{RESET}")

        # Print to terminal
        sys.stdout.write("\033[H\033[J" + "\n".join(hud) + "\n")
        sys.stdout.flush()

if __name__ == "__main__":
    try:
        monitor = SmpcVisualMonitor()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
