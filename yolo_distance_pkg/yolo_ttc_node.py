#!/usr/bin/env python3
import json, math
from collections import defaultdict, deque
import rospy
from std_msgs.msg import String
from morai_msgs.msg import EgoVehicleStatus

class TtcNode:
    def __init__(self):
        rospy.init_node('yolo_ttc_node')
        self.hist = defaultdict(lambda: deque(maxlen=8))
        self.ego_speed = 0.0
        self.pub = rospy.Publisher('/yolo_distance/ttc', String, queue_size=1)
        rospy.Subscriber('/yolo_distance/detections', String, self.det_cb, queue_size=1)
        rospy.Subscriber('/Competition_topic', EgoVehicleStatus, self.status_cb, queue_size=1)
        rospy.loginfo('YOLO TTC node started')
        rospy.spin()

    def status_cb(self, msg):
        if math.isfinite(msg.velocity.x):
            self.ego_speed = max(0.0, float(msg.velocity.x))

    def det_cb(self, msg):
        try:
            data = json.loads(msg.data)
            stamp = float(data.get('header', {}).get('stamp', rospy.Time.now().to_sec()))
            detections = data.get('detections', [])
        except Exception as exc:
            rospy.logwarn_throttle(5.0, 'Invalid detection JSON: %s', exc)
            return
        out = []
        for i, det in enumerate(detections):
            try:
                distance = float(det['dist'])
                if not math.isfinite(distance) or distance <= 0.0:
                    continue
                box = det.get('box', [])
                # Stable association for this detector: class plus image-side.
                side = int((float(box[0]) + float(box[2])) * 0.5 / 160.0) if len(box) >= 4 else i
                key = '%s:%s' % (det.get('cls', 'object'), side)
                self.hist[key].append((stamp, distance))
                track = list(self.hist[key])
                closing = 0.0
                if len(track) >= 3:
                    t0, d0 = track[0]; t1, d1 = track[-1]
                    dt = t1 - t0
                    if dt > 0.05:
                        closing = max(0.0, (d0 - d1) / dt)
                ttc = distance / closing if closing > 0.15 else float('inf')
                out.append({'id': key, 'cls': det.get('cls', 'object'),
                            'distance': round(distance, 2),
                            'closing_speed': round(closing, 2),
                            'ttc': None if not math.isfinite(ttc) else round(ttc, 2),
                            'ego_speed': round(self.ego_speed, 2)})
            except (TypeError, ValueError, IndexError):
                continue
        self.pub.publish(String(data=json.dumps({'stamp': stamp, 'objects': out})))

if __name__ == '__main__':
    TtcNode()
