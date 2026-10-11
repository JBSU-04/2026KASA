#!/usr/bin/env python3
import rospy
import sys, select, termios, tty
from morai_msgs.msg import CtrlCmd, EventInfo

# K-City Checkpoints (Approximated based on typical K-City coordinates)
CHECKPOINTS = {
    '1': [9355.0, 19160.0, 0.0], # Start / End
    '2': [9450.0, 19190.0, 0.0], # Traffic Light 1
    '3': [9550.0, 19190.0, 0.0], # Obstacles
    '4': [9550.0, 19000.0, 0.0], # Traffic Light 2
    '5': [9500.0, 19000.0, 0.0], # Roundabout
    '6': [9350.0, 18900.0, 0.0], # High Speed Entry
    '7': [9300.0, 19000.0, 0.0], # Vehicle Recog
    '8': [9300.0, 19100.0, 0.0], # GPS Shadow
}

msg = """
Control Mode Active:
---------------------------
[SPACE] : Start Auto Mode (Gear D, Auto Ctrl)
[ESC/M] : Emergency Stop (Gear P, Keyboard Ctrl)
[1~8]   : Teleport to Mission Checkpoints
[Q]     : Quit
"""

def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist: key = sys.stdin.read(1)
    else: key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key
    
def send_teleport(idx):
    # Requires Morai MultiEgoSetting
    from morai_msgs.msg import MultiEgoSetting
    pub = rospy.Publisher('/Ego_Setting', MultiEgoSetting, queue_size=1)
    if idx in CHECKPOINTS:
        pos = CHECKPOINTS[idx]
        m = MultiEgoSetting()
        m.ego_index = 0
        m.pose_x = pos[0]
        m.pose_y = pos[1]
        m.pose_z = pos[2]
        pub.publish(m)
        rospy.loginfo(f"Teleported to Checkpoint {idx}")

if __name__ == '__main__':
    settings = termios.tcgetattr(sys.stdin)
    rospy.init_node('keyboard_override_node')
    event_pub = rospy.Publisher('/EventCmd', EventInfo, queue_size=1)
    
    print(msg)
    try:
        while not rospy.is_shutdown():
            key = getKey()
            if key == ' ':
                ev = EventInfo()
                ev.option = 7 # Gear + Ctrl
                ev.ctrl_mode = 2 # Auto
                ev.gear = 4 # D
                event_pub.publish(ev)
                print("==> Triggered START (Auto, D Gear) - Good luck!")
            elif key.lower() == 'm' or key == '\x1b': # ESC
                ev = EventInfo()
                ev.option = 7
                ev.ctrl_mode = 1 # Keyboard mode
                ev.gear = 1 # P
                event_pub.publish(ev)
                print("==> E-STOP (Keyboard Mode, P Gear)")
            elif key in ['1', '2', '3', '4', '5', '6', '7', '8']:
                send_teleport(key)
                print(f"==> Teleported to Section {key}")
            elif key.lower() == 'q':
                break
    except Exception as e:
        print(e)
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
