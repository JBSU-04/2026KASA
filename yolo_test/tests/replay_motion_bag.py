import time
import rospy,rosbag,yaml,importlib.util,xml.etree.ElementTree as ET,json,collections,sys
from pathlib import Path
clock=[0.]; params={}; output=[]
time.time=lambda:clock[0]
rospy.Time.now=staticmethod(lambda:rospy.Time.from_sec(clock[0]))
for k in ['init_node','spin','Subscriber','Timer','loginfo','logwarn','loginfo_throttle','logwarn_throttle','logdebug']:
 setattr(rospy,k,lambda *a,**kw:None)
rospy.get_param=lambda k,d=None:params.get(k.lstrip('~'),d)
class Pub:
 def __init__(self,*a,**kw):pass
 def publish(self,m):pass
rospy.Publisher=Pub

def load(path,name):
 sp=importlib.util.spec_from_file_location(name,path);m=importlib.util.module_from_spec(sp);sp.loader.exec_module(m);return m

def config(path):
 for q in ET.parse(path).iter('param'):
  v=q.get('value','')
  if '$(' not in v:params[q.get('name')]=yaml.safe_load(v)
params.update(yaml.safe_load(Path('src/total_bringup/config/tuning_perception.yaml').read_text())['obstacle_perception_node'])
config('src/yolo_test/launch/obstacle_perception.launch')
baseline='--baseline' in sys.argv
# Baseline mode requires snapshots in /tmp/perception_before. Default uses workspace code.
p=load('/tmp/perception_before/obstacle_perception_node.py' if baseline else 'src/yolo_test/obstacle_perception_node.py','perception'); node=p.ObstaclePerceptionNode()
node.publish_markers=lambda:None;node.publish_empty_markers=lambda:None
params.clear();config('src/new_lane/launch/obstacle_stop.launch');params['static_dynamic_split_idx']=3700;params['dynamic_zone_max_idx']=4900
s=load('/tmp/perception_before/obstacle_stop_node.py' if baseline else 'src/new_lane/scripts/obstacle_stop_node.py','stop');stop=s.ObstacleStopNode();stop.publish_visualization=lambda:None
# Freeze configured perception fields during deterministic replay.
node._sync_params=lambda:None
class Capture:
 def publish(self,m):
  d=json.loads(m.data); stop.obstacle_info_callback(m)
  output.append({'t':clock[0]-t0,'stop':stop.dynamic_obstacle_near,**d})
node.info_pub=Capture()
run=sys.argv[1] if len(sys.argv)>1 else '16-30-49'
g=f'bags/competition_2026-09-10-{run}.bag';h=f'bags/competition_heavy_2026-09-10-{run}.bag'
events=[]
with rosbag.Bag(g) as b:
 t0=b.get_start_time()
 for topic,m,t in b.read_messages(topics=['/odom','/final_waypoint','/mission/active_section']):events.append((t.to_sec(),topic,m))
with rosbag.Bag(h) as b:
 for topic,m,t in b.read_messages(topics=['/nonground']):events.append((t.to_sec(),topic,m))
for ts,topic,m in sorted(events,key=lambda q:q[0]):
 clock[0]=ts
 if topic=='/odom':node.odom_callback(m);stop.odom_callback(m)
 elif topic=='/final_waypoint':node.final_waypoint_callback(m);stop.waypoint_callback(m)
 elif topic=='/mission/active_section':node.active_section_callback(m)
 else:node.lidar_callback(m)
Path(f'/tmp/replay_{run}{"_baseline" if baseline else ""}.json').write_text(json.dumps(output))
prev=False
for d in output:
 if d['stop']!=prev:print('STOP',round(d['t'],3),d['stop']);prev=d['stop']
tracks=collections.defaultdict(list)
for d in output:
 for o in d['collision_objects']:tracks[o['id']].append((d['t'],o))
for tid,a in tracks.items():
 fresh=[(t,o) for t,o in a if o.get('observation_age',0)<.3]
 dyn=[t for t,o in fresh if o['motion_state']=='dynamic'];pro=[t for t,o in fresh if o.get('observed_motion_valid',False)]
 print(tid,'t',round(a[0][0],2),round(a[-1][0],2),'y',round(min(o['position_ego'][1] for _,o in a),2),round(max(o['position_ego'][1] for _,o in a),2),'dyn', [round(t,2) for t in dyn[:3]],'pro',[round(t,2) for t in pro[:3]])
