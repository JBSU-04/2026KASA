#!/usr/bin/env python3
"""Private ROS master; detector fixture + real merger/owner/stop/escape/preslow.
No UDP, MPC controller, bag playback, or modification of the user's ROS master.
"""
import csv,json,math,os,subprocess,sys,time,xmlrpc.client
from pathlib import Path
import yaml,numpy as np
root=Path(__file__).resolve().parents[3]
report=Path(sys.argv[2]).resolve();report.parent.mkdir(parents=True,exist_ok=True)
os.environ.update(ROS_MASTER_URI='http://127.0.0.1:11563',ROS_IP='127.0.0.1',ROS_LOG_DIR=str(report.parent/'ros_logs'))
processes=[];files=[];checks={};latencies=[]
def start(args,name):
 f=open(report.parent/(name+'.log'),'w');files.append(f);p=subprocess.Popen(args,stdout=f,stderr=subprocess.STDOUT);processes.append(p);return p
try:
 start(['roscore','-p','11563'],'master');master=xmlrpc.client.ServerProxy(os.environ['ROS_MASTER_URI'])
 for _ in range(100):
  try:
   if master.getUri('/m8_scene_test')[0]==1:break
  except OSError:pass
  time.sleep(.1)
 import rospy,rosparam
 from nav_msgs.msg import Odometry,Path as RosPath
 from std_msgs.msg import String,Bool,Header
 from sensor_msgs import point_cloud2 as pc
 from sensor_msgs.msg import PointCloud2
 from occupancy_avoidance.msg import TunnelScene,M8ApprovedPath
 from mission_supervisor.msg import SectionState
 from waypoint_maker.msg import Lane,Waypoint
 from morai_msgs.msg import EgoVehicleStatus
 sys.path.insert(0,str(root/'src/new_lane/scripts'))
 from tunnel_native_scene import packet_from_detector,scene_dict
 params=yaml.safe_load(Path(sys.argv[1]).read_text())
 prefixes=('/tunnel_recovery/','/lidar_extrinsic/','/section_router/','/obstacle_info_merger/','/obstacle_stop_node/','/reverse_escape_node/','/obstacle_avoidance_node/','/tunnel_wall_detector/','/speed_limit/')
 selected={k:v for k,v in params.items() if k.startswith(prefixes)}
 tree={}
 for key,value in selected.items():
  parts=key.strip('/').split('/');node=tree
  for part in parts[:-1]:node=node.setdefault(part,{})
  node[parts[-1]]=value
 rosparam.upload_params('/',tree)
 assert rospy.get_param('/tunnel_recovery/object_scene_enabled') and not rospy.get_param('/tunnel_recovery/common_scene_enabled')
 rospy.init_node('m8_scene_fixture',anonymous=True)
 for name in ('obstacle_info_merger','obstacle_avoidance_node','obstacle_stop_node','reverse_escape_node','tunnel_wall_detector'):
  package='new_lane';executable=name+'.py' if name!='obstacle_avoidance_node' else name
  start(['rosrun',package,executable,'__name:='+name],name)
 pub={k:rospy.Publisher(topic,typ,queue_size=1) for k,topic,typ in [('route','/final_waypoint',Lane),('odom','/odom',Odometry),('ground','/groundgrid/ground',PointCloud2),('native','/tunnel/detections_native',TunnelScene),('json_near','/obstacle_info_near',String),('section','/mission/active_section',SectionState),('ego','/Competition_topic',EgoVehicleStatus),('random','/tunnel/random_obstacle_active',Bool)]}
 scenes=[];approvals=[];selections=[];stops=[]
 def receive(m):
  scenes.append(m);latencies.append(1000*(rospy.Time.now()-m.header.stamp).to_sec())
 rospy.Subscriber('/tunnel/obstacle_scene',TunnelScene,receive,queue_size=1)
 rospy.Subscriber('/obstacle_avoidance/m8_approved_path',M8ApprovedPath,approvals.append,queue_size=1)
 rospy.Subscriber('/obstacle_avoidance/selection_state',String,lambda m:selections.append(json.loads(m.data)),queue_size=1)
 rospy.Subscriber('/obstacle_stop/dynamic_state',String,lambda m:stops.append(json.loads(m.data)),queue_size=1)
 time.sleep(3.)
 with open(root/'src/waypoint_system/data/default/3.csv') as f:rows=list(csv.reader(f))
 lane=Lane();index=7600
 road=[]
 for i in range(index,index+500):
  row=rows[i];xy=(float(row[1]),float(row[2]));road.append(xy)
  w=Waypoint();w.lane_number=3;w.waypoint_index=i;w.pose.pose.position.x,w.pose.pose.position.y=xy;w.pose.pose.orientation.w=1.;lane.waypoints.append(w)
 p=np.asarray(road[0]);yaw=math.atan2(road[5][1]-p[1],road[5][0]-p[0]);c,s=math.cos(yaw),math.sin(yaw);rotation=np.array([[c,s],[-s,c]])
 section=SectionState();section.section_id='M8';section.csv=3;section.index=index;section.index_valid=True
 ego=EgoVehicleStatus();ego.velocity.x=3.
 actual=np.array([[12.,dy,z] for dy in (-.4,0.,.4) for z in (.2,.4)])
 world=actual.copy();world[:,:2]=actual[:,:2]@rotation+p
 for sequence in range(1,66):
  stamp=rospy.Time.now();od=Odometry();od.header.stamp=stamp;od.header.frame_id='map';od.pose.pose.position.x,od.pose.pose.position.y=p;od.pose.pose.orientation.z=math.sin(yaw/2);od.pose.pose.orientation.w=math.cos(yaw/2);od.twist.twist.linear.x=3.
  pub['odom'].publish(od);pub['route'].publish(lane);pub['section'].publish(section);pub['ego'].publish(ego);pub['random'].publish(Bool(data=True))
  ground=[[x-1.1,y,-1.33] for x in np.arange(-12.,36.,.25) for y in np.arange(-5.,5.1,.25)]
  h=Header(stamp=stamp,frame_id='velodyne_ground');pub['ground'].publish(pc.create_cloud_xyz32(h,ground));time.sleep(.01)
  if sequence<15:objects=[]
  else:objects=[dict(id=37,confirmed=True,static_certified=True,motion_state='static',m8_surface_reason='repeated_stationary_surface',support_points_world=world.tolist(),position_world=world.mean(axis=0)[:2].tolist(),position_ego=[12.,0.,-.9],length=.8,width=.8,radius=.6,top_elevation=.4,last_seen=stamp.to_sec(),in_trigger_corridor=True,observed_motion_valid=False,observed_velocity=[0.,0.])]
  payload=dict(mission_mode='tunnel',collision_objects=objects,geometry_scan=dict(stamp=stamp.to_sec(),origin=[p[0],p[1],1.33],endpoints=world.tolist()))
  m=packet_from_detector(payload,payload,{},(*p,yaw,0.),stamp.to_sec(),sequence,source_packet=True);m.speed=3.;pub['native'].publish(m)
  poison=dict(payload,stamp=stamp.to_sec()+.005,geometry_scan=dict(payload['geometry_scan'],stamp=stamp.to_sec()+.005),collision_objects=[dict(objects[0],id=9999,last_seen=stamp.to_sec()+.005)] if objects else [])
  pub['json_near'].publish(String(data=json.dumps(poison)))
  time.sleep(.10)
 _,_,system=master.getSystemState('/m8_scene_fixture');publishers=dict(system[0]);subscribers=dict(system[1]);topic='/tunnel/obstacle_scene'
 checks['producer']=publishers.get(topic,[]);checks['consumers']=subscribers.get(topic,[])
 required={'/obstacle_avoidance_node','/obstacle_stop_node','/reverse_escape_node','/tunnel_wall_detector'}
 assert '/obstacle_info_merger' in checks['producer'] and required.issubset(checks['consumers']),checks
 assert scenes and any(m.objects for m in scenes),'native scene was never populated'
 all_ids={o.id for m in scenes for o in m.objects};assert len(all_ids)==1,all_ids
 latest=next(m for m in reversed(scenes)if m.objects);cells=scene_dict(latest)
 assert len(latest.cells)==len(world) and all(abs(q.z-.2)<1e-6 or abs(q.z-.4)<1e-6 for q in latest.cells)
 assert latest.header.stamp.to_sec()==latest.seen[0],'source stamp was rewritten'
 checks['same_owner']=list(all_ids);checks['actual_cells']=len(latest.cells);checks['scenes']=len(scenes)
 checks['source_to_scene_p95_ms']=float(np.percentile(latencies,95))
 checks['owner_received_native']=any(x.get('scene_id',0)>0 for x in selections)
 assert checks['owner_received_native'],'path owner never consumed the direct scene'
 checks['approved_paths']=len(approvals);checks['approved_scene_ids']=sorted({m.scene_id for m in approvals})
 assert approvals and any(m.scene_id>0 for m in approvals),'no approved path receipt for native scene'
 checks['consumer_processes_alive']=all(p.poll() is None for p in processes)
 assert checks['consumer_processes_alive'],'consumer exited; inspect isolated logs'
 checks['result']='passed'
except Exception as exc:
 checks['result']='failed';checks['error']=repr(exc);raise
finally:
 report.write_text(json.dumps(checks,indent=2))
 for p in reversed(processes):
  if p.poll() is None:p.terminate()
 for p in reversed(processes):
  try:p.wait(timeout=5)
  except subprocess.TimeoutExpired:p.kill();p.wait()
 for f in files:f.close()
