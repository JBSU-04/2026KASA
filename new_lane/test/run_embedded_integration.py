#!/usr/bin/env python3
"""Isolated ROS integration checks. Starts only the path owner, never a controller/UDP bridge.
Usage (after sourcing devel): python3 .../run_embedded_integration.py params.yaml report.json
"""
import json, math, os, subprocess, sys, time, xmlrpc.client
from pathlib import Path
import yaml

root=Path(__file__).resolve().parents[3]
report=Path(sys.argv[2]).resolve();report.parent.mkdir(parents=True,exist_ok=True)
os.environ['ROS_MASTER_URI']='http://127.0.0.1:11557'
os.environ['ROS_IP']='127.0.0.1'
os.environ['ROS_LOG_DIR']=str(report.parent/'ros_logs')
logs=[];processes=[];checks=[]
try:
    log=open(report.parent/'integration_master.log','w');logs.append(log)
    processes.append(subprocess.Popen(['roscore','-p','11557'],stdout=log,stderr=subprocess.STDOUT))
    master=xmlrpc.client.ServerProxy(os.environ['ROS_MASTER_URI'])
    deadline=time.monotonic()+10
    while True:
        try:
            if master.getUri('/embedded_test')[0]==1:break
        except OSError:pass
        if time.monotonic()>deadline:raise RuntimeError('isolated master failed to start')
        time.sleep(.1)
    import rospy
    from nav_msgs.msg import Odometry,Path as RosPath
    from sensor_msgs.msg import PointCloud2
    from sensor_msgs import point_cloud2
    from std_msgs.msg import Header,String,Bool,Float32MultiArray,Float64
    from waypoint_maker.msg import Lane,Waypoint
    from morai_msgs.msg import EgoVehicleStatus
    params=yaml.safe_load(Path(sys.argv[1]).read_text())
    for k,v in params.items():
        if k.startswith(('/obstacle_avoidance_node/','/lidar_extrinsic/','/speed_limit/')):rospy.set_param(k,v)
    csv=report.parent/'integration_road.csv'
    csv.write_text(''.join('%d,%.3f,4124000\n'%(i,302500+i*.1) for i in range(10001)))
    ns='/obstacle_avoidance_node/embedded_engine/'
    rospy.set_param(ns+'csv0',str(csv));rospy.set_param(ns+'csv3',str(csv))
    rospy.set_param(ns+'curb_edges',[[0,10000,5.,'test',5.]])
    rospy.set_param(ns+'tunnel_wall_profile',[7000,-5.,5.,9000,-5.,5.])
    rospy.set_param('/obstacle_avoidance_node/road_width_profile',[])
    log=open(report.parent/'integration_owner.log','w');logs.append(log)
    processes.append(subprocess.Popen([str(root/'devel/lib/new_lane/obstacle_avoidance_node')],stdout=log,stderr=subprocess.STDOUT))
    rospy.init_node('embedded_integration_test',anonymous=False)
    states=[];paths=[];failures=[];acks=[];caps=[]
    rospy.Subscriber('/obstacle_avoidance/selection_state',String,lambda m:states.append(json.loads(m.data)))
    rospy.Subscriber('/local_path',RosPath,lambda m:paths.append(m))
    rospy.Subscriber('/avoidance_failed',Bool,lambda m:failures.append(m.data))
    rospy.Subscriber('/occupancy_avoidance/escape_handoff_state',String,lambda m:acks.append(json.loads(m.data)))
    rospy.Subscriber('/obstacle_avoidance/target_speed_mps',Float64,lambda m:caps.append(m.data))
    pub={topic:rospy.Publisher(topic,typ,queue_size=1) for topic,typ in [
        ('/odom',Odometry),('/final_waypoint',Lane),('/Competition_topic',EgoVehicleStatus),
        ('/perception/points_gravity',PointCloud2),('/obstacle_info',String),
        ('/obstacle_info_near',String),('/obstacle_stop/dynamic_state',String),
        ('/localization/stop_ahead',Float32MultiArray),('/localization/stop_required',Bool),
        ('/reverse_escape/handoff',String)]}
    time.sleep(.5)
    idx=2500;lane=0;obstacle=False;cloud=True;red=False;hold=False;certificate=False
    def feed(seconds):
        until=time.monotonic()+seconds;count=0
        while time.monotonic()<until:
            now=rospy.Time.now();x=302500+idx*.1
            od=Odometry();od.header.stamp=now;od.pose.pose.position.x=x;od.pose.pose.position.y=4124000.;od.pose.pose.orientation.w=1
            pub['/odom'].publish(od)
            ego=EgoVehicleStatus();ego.velocity.x=8.;pub['/Competition_topic'].publish(ego)
            msg=Lane()
            for i in range(idx,idx+600):
                w=Waypoint();w.waypoint_index=i;w.lane_number=lane;w.ryaw=0
                w.pose.pose.position.x=302500+i*.1;w.pose.pose.position.y=4124000.;w.pose.pose.orientation.w=1;msg.waypoints.append(w)
            pub['/final_waypoint'].publish(msg)
            objects={'stamp':now.to_sec(),'mission_mode':'tunnel' if lane==3 else 'static','obstacles':[],'collision_objects':[]}
            pub['/obstacle_info_near'].publish(String(data=json.dumps(objects)))
            if certificate:
                objects['obstacles']=[{'id':7001,'static_occupancy':True,'motion_state':'static','last_seen':now.to_sec(),
                    'footprint_world':[[x+21.5,4123999.6],[x+22.5,4123999.6],[x+22.5,4124000.4],[x+21.5,4124000.4]]}]
            pub['/obstacle_info'].publish(String(data=json.dumps(objects)))
            pub['/obstacle_stop/dynamic_state'].publish(String(data=json.dumps({'hold':hold,'static_ids':[],'dynamic_ids':[]})))
            pub['/localization/stop_ahead'].publish(Float32MultiArray(data=[8. if red else -1.]))
            pub['/localization/stop_required'].publish(Bool(data=red))
            if cloud and count%2==0:
                pts=[(i*.5,j*.5,-1.765) for i in range(8,81) for j in range(-6,7)]
                if certificate:pts.append((20.5+(count%20)*.13,0.,-1.3))
                if obstacle:pts.extend((i*.15,j*.15,-1.3) for i in range(80,93) for j in range(-3,4))
                header=Header(stamp=now,frame_id='velodyne_ground')
                pub['/perception/points_gravity'].publish(point_cloud2.create_cloud_xyz32(header,pts))
            count+=1;time.sleep(.05)
    def check(name,value):
        checks.append({'name':name,'passed':bool(value)})
        if not value:raise AssertionError(name)
    feed(1.5)
    check('clear road uses embedded owner',any(s.get('valid') and s['source']=='embedded_apollo' for s in states))
    publishers=master.getSystemState('/embedded_test')[2][0]
    check('sole final path publisher',dict(publishers).get('/local_path')==['/obstacle_avoidance_node'])
    certificate=True;begin=len(states);feed(1.8)
    check('full far certificate admits sparse raw without near objects',any(s.get('valid') and s.get('trigger_reason')=='certified_static_geometry' for s in states[begin:]))
    certificate=False;idx=2800;feed(.7)
    obstacle=True;begin=len(states);feed(2.)
    selected=[s for s in states[begin:] if s.get('valid')]
    check('raw occupancy triggers without legacy cluster ids',any(s.get('trigger_count',0)>0 for s in selected))
    check('avoidance path actually reaches controller topic',any(max(abs(p.pose.position.y-4124000.) for p in m.poses)>1. for m in paths[-10:] if m.poses))
    # Removing detections/returns does not delete a certified obstacle without free-ray evidence.
    begin=len(states);obstacle=False;feed(.5)
    check('occluded occupancy stays certified',any(s.get('trigger_count',0)>0 for s in states[begin:] if s.get('valid')))
    red=True;begin=len(states);feed(.7)
    # This observer test does not launch the brake executor. The owner must
    # reject new avoidance on a red signal; target_speed alone is not a brake.
    check('red stop blocks new avoidance initiation',any('traffic_' in s.get('reason','') and not s.get('valid',False) for s in states[begin:]))
    red=False;cloud=False;feed(2.0)
    check('stale raw geometry stops rather than falling back',failures and all(failures[-3:]))
    check('never selected automatic Frenet fallback',not any(s.get('source')=='frenet' for s in states))
    # M8 owner still obeys original dynamic stop handoff; the stop executor owns the actual brake.
    cloud=True;lane=3;idx=7600;obstacle=False;feed(1.5)
    hold=True;begin=len(states);feed(.5)
    check('original dynamic hold suppresses engine avoidance',any(s.get('source')=='reference' for s in states[begin:]))
    blocked={'id':41,'stamp':rospy.Time.now().to_sec(),'forward':[[303260+i*.1,4124000.,0.] for i in range(401)]}
    pub['/reverse_escape/handoff'].publish(String(data=json.dumps(blocked)));feed(.2)
    check('reverse handoff cannot bypass original dynamic hold',not any(a.get('id')==41 and a.get('valid') for a in acks))
    hold=False;feed(.5)
    # A moving M8 vehicle must replan without a forced complete stop. Repeated
    # feedback for one rejected ID must not invalidate every subsequent snapshot.
    last=next(s for s in reversed(states) if s.get('valid') and s.get('source')=='embedded_apollo')
    rejected_id=last['plan_id'];begin=len(states)
    feedback=rospy.Publisher('/obstacle_avoidance/selection_state',String,queue_size=1)
    time.sleep(.1)
    for _ in range(16):
        feedback.publish(String(data=json.dumps(dict(plan_id=rejected_id,reason='occupancy',source='test_rejection'))))
        feed(.05)
    new=[s for s in states[begin:] if s.get('source')=='embedded_apollo']
    check('moving M8 rejection replans without wait-stop',not any(s.get('reason')=='escape_replan_wait_stop' for s in new))
    check('duplicate rejection cannot starve new plans',any(s.get('valid') and s['plan_id']>rejected_id for s in new))
    feedback.unregister()
    handoff={'id':42,'stamp':rospy.Time.now().to_sec(),'forward':[[303260+i*.1,4124000.,0.] for i in range(401)]}
    pub['/reverse_escape/handoff'].publish(String(data=json.dumps(handoff)));feed(.4)
    ready=[a for a in acks if a.get('id')==42 and a.get('valid')]
    check('reverse handoff acknowledged by actual final owner',ready and any(m.header.seq==ready[-1]['plan_id'] and m.poses for m in paths))
    # The legacy tunnel mode has a hold at the end; its few retained frames must not STOP
    # solely because the engine's nominal end index was crossed.
    idx=9001;feed(1.5)
    check('tunnel end mode hold keeps a valid engine path',any(s.get('valid') for s in states[-10:]) and not any(failures[-3:]))
    # New M8 integrated handoff retains the validated forward trajectory, rather
    # than requiring the same failing endpoint QP to succeed again.
    idx=7600;feed(.7);begin=len(states)
    handoff={'id':43,'stamp':rospy.Time.now().to_sec(),'integrated':True,'speed_cap':2.,
             'forward':[[303260+i*.1,4124000.,0.] for i in range(401)]}
    pub['/reverse_escape/handoff'].publish(String(data=json.dumps(handoff)));feed(.5)
    check('integrated forward handoff acknowledged',any(a.get('id')==43 and a.get('valid') for a in acks))
    check('integrated trajectory latched without endpoint QP',any(s.get('valid') and s.get('reason')=='escape_forward_latched' for s in states[begin:]))
    check('integrated forward cap remains 2mps',caps and max(caps[-3:])<=2.01)
    report.write_text(json.dumps({'passed':True,'checks':checks,'selection_count':len(states),'path_count':len(paths)},indent=2))
    print(json.dumps({'passed':True,'checks':len(checks)}))
except Exception as e:
    report.write_text(json.dumps({'passed':False,'checks':checks,'error':str(e)},indent=2));raise
finally:
    for p in reversed(processes):
        p.terminate()
        try:p.wait(timeout=5)
        except subprocess.TimeoutExpired:p.kill();p.wait()
    for log in logs:log.close()
