#!/usr/bin/env python3
"""Offline callbacks only: no ROS master, publishers, simulator or vehicle IO.
Uses recorded GroundGrid output, not a re-evaluation of ground removal or GT.
"""
import argparse
import heapq
import importlib.util
import json
from pathlib import Path
import time
import numpy as np
import rosbag
import rospy
import yaml

parser=argparse.ArgumentParser()
parser.add_argument('--params', required=True, help='roslaunch --dump-params YAML')
parser.add_argument('--regular',required=True)
parser.add_argument('--heavy',required=True)
parser.add_argument('--backend',default='apollo_measurement',choices=['legacy','apollo_measurement'])
parser.add_argument('--output',required=True)
a=parser.parse_args()
params=yaml.safe_load(Path(a.params).read_text())
params['/motion_tracking/backend']=a.backend
callbacks={}; now=[0.0]; frames=[]
def get_param(key,default=None):
    key='/obstacle_perception_node/'+key[1:] if key.startswith('~') else key
    if key in params: return params[key]
    nested={k[len(key)+1:]:v for k,v in params.items() if k.startswith(key+'/')}
    return nested or default
class Publisher:
    def __init__(self,topic,*args,**kwargs): self.topic=topic
    def publish(self,msg):
        if self.topic=='/obstacle_info': frames.append(json.loads(msg.data))
rospy.spin=lambda: None
rospy.init_node=lambda *args,**kwargs: None
rospy.get_param=get_param
rospy.Publisher=Publisher
rospy.Subscriber=lambda topic,typ,cb,**kw: callbacks.setdefault(topic,cb)
rospy.Time.now=staticmethod(lambda: rospy.Time.from_sec(now[0]))
for key in ['loginfo','logwarn','logerr','logdebug','loginfo_throttle','logwarn_throttle','logerr_throttle']:
    setattr(rospy,key,lambda *args,**kw: None)
root=Path(__file__).resolve().parents[2]
spec=importlib.util.spec_from_file_location('perception_replay',root/'yolo_test/obstacle_perception_node.py')
mod=importlib.util.module_from_spec(spec); spec.loader.exec_module(mod)
node=mod.ObstaclePerceptionNode()
node.publish_markers=lambda *args,**kwargs: None
start=time.monotonic()
with rosbag.Bag(a.regular) as regular, rosbag.Bag(a.heavy) as heavy:
    def events(bag):
        for topic,msg,t in bag.read_messages(topics=list(callbacks)):
            yield (t.to_sec(), topic,msg)
    for stamp,topic,msg in heapq.merge(events(regular),events(heavy),key=lambda e:e[0]):
        now[0]=stamp;callbacks[topic](msg)
entries=[o for f in frames for o in f.get('collision_objects',[])]
valid=[o for o in entries if o.get('velocity_valid') and o.get('observation_age',0)<=.3]
report=dict(backend=a.backend,frames=len(frames),objects=len(entries),fresh_valid_motion=len(valid),
    track_count=len(set(o['id'] for o in entries)),runtime_sec=time.monotonic()-start,
    velocity_std_percentiles=(np.percentile([o['vel_std'] for o in valid],[50,90,99]).tolist() if valid else []),
    note='No ground truth; these are execution/confidence statistics, not accuracy scores.')
Path(a.output).write_text(json.dumps(report,indent=2)+'\n')
Path(a.output+'.frames.json').write_text(json.dumps(frames))
print(json.dumps(report,indent=2))
