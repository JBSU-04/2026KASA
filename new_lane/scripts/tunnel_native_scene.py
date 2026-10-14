"""Native cell transport for M8. No classification or invented point heights."""
import copy,json,math

def packet_from_detector(root,near,parents,pose,stamp,scene_id,version=0,offset=(0.,0.),source_packet=False,preserve_certificate=True):
    from occupancy_avoidance.msg import TunnelScene,TunnelObject
    from geometry_msgs.msg import Point
    from rospy import Time
    m=TunnelScene();m.header.stamp=Time.from_sec(stamp);m.header.frame_id='map';m.scene_id=scene_id;m.coordinate_version=version;m.offset_x,m.offset_y=offset
    m.ego_pose.position.x,m.ego_pose.position.y=pose[:2];m.ego_pose.position.z=float(pose[3]) if len(pose)>3 else 0.;yaw=pose[2];m.ego_pose.orientation.z=math.sin(yaw/2);m.ego_pose.orientation.w=math.cos(yaw/2)
    sources={int(o.get('id',-1)):o for o in near.get('collision_objects',near.get('obstacles',[]))}
    objects=[];dedup=set()
    for source in root.get('collision_objects',[]):
        if source.get('background_excluded') and not source_packet:continue
        o=copy.deepcopy({k:v for k,v in source.items() if k not in ('support_points_world','geometry_seed_footprint_world','occupied_cells_world','observed_cells_world')});oid=int(o['id']);observed=sources.get(int(o.get('observed_track_id',oid)),{})
        pts=observed.get('support_points_world',[])
        parent=parents.get(oid,{})
        held=parent.get('support',[]) if o.get('static_occupancy') else []
        confirmed=bool(o.get('confirmed',False))
        # A certified retained surface is independent of the instantaneous
        # motion label. Never manufacture this certificate for unknown tracks.
        static=bool((preserve_certificate and o.get('static_certified',False)) or (confirmed and o.get('motion_state')=='static'))
        if o.get('dynamic_confirmed',False) or o.get('motion_state')=='dynamic':static=False
        o['dynamic_confirmed']=confirmed and o.get('motion_state')=='dynamic'
        o['confirmed_obstacle']=confirmed;o['static_certified']=static
        o['observed_surface_count']=len(pts);o['last_surface_count']=max(len(pts),len(held));o['bbox_is_diagnostic_only']=True
        fresh_surface=abs(float(observed.get('last_seen',stamp))-stamp)<1e-6
        o['observed_surface_count']=len(pts) if fresh_surface else 0
        for observed_now,points in [(fresh_surface,pts),(False,held)]:
            for q in points:
                if not observed_now and len(q)>=2 and parent.get('box'):
                    c,s=math.cos(parent['yaw']),math.sin(parent['yaw']);a,d=c*q[0]+s*q[1],-s*q[0]+c*q[1];a0,a1,d0,d1=parent['box']
                    if a<a0-.05 or a>a1+.05 or d<d0-.05 or d>d1+.05:continue
                if len(q)<3 or not all(math.isfinite(float(v)) for v in q[:3]):continue
                key=(oid,)+tuple(int(math.floor(float(v)/.1+1e-8)) for v in q[:3])
                if not source_packet and key in dedup:continue
                dedup.add(key);m.cells.append(Point(*map(float,q[:3])));m.owner_ids.append(oid);m.fixed.append(static);m.seen.append(float(observed.get('last_seen',stamp) if points is pts else parent.get('stamp',stamp)));m.observed.append(observed_now)
        for k in ['support_points_world','geometry_seed_footprint_world','occupied_cells_world','observed_cells_world']:o.pop(k,None)
        objects.append(o)
        obj=TunnelObject();obj.id=oid;obj.observed_track_id=int(o.get('observed_track_id',oid));obj.confirmed=confirmed;obj.motion_state={'static':obj.STATIC,'dynamic':obj.DYNAMIC}.get(o.get('motion_state'),obj.UNKNOWN)
        obj.static_certified=static;obj.dynamic_confirmed=o['dynamic_confirmed'];obj.top_elevation=float(o.get('top_elevation',float('nan')));pw=o.get('position_world',[0.,0.]);obj.position_world.x,obj.position_world.y=pw[:2];vel=o.get('velocity',[o.get('vx',0.),o.get('vy',0.)]);obj.velocity_world.x=float(vel[0]);obj.velocity_world.y=float(vel[1]);o['velocity_world']=[obj.velocity_world.x,obj.velocity_world.y];obj.last_seen=float(o.get('last_seen',stamp));obj.in_trigger_corridor=bool(o.get('in_trigger_corridor',False));obj.metadata_json=json.dumps(o,separators=(',',':'));m.objects.append(obj)
    m.mission_mode=root.get('mission_mode','tunnel')
    if source_packet:
        scan=root.get('geometry_scan',{});origin=scan.get('origin',[0.,0.,0.]);m.ray_origin=Point(*map(float,origin))
        m.ray_endpoints=[Point(*map(float,q)) for q in scan.get('endpoints',[])]
    m.metadata_json=json.dumps(dict(state='valid',perception_source='shared_nonground_detector'),separators=(',',':'))
    return m

def scene_dict(m):
    d=json.loads(m.metadata_json);d['obstacles']=[]
    for obj in m.objects:
        o=json.loads(obj.metadata_json);o.update(id=obj.id,observed_track_id=obj.observed_track_id,confirmed=obj.confirmed,confirmed_obstacle=obj.confirmed,motion_state={obj.STATIC:'static',obj.DYNAMIC:'dynamic'}.get(obj.motion_state,'unknown'),static_certified=obj.static_certified,dynamic_confirmed=obj.dynamic_confirmed,position_world=[obj.position_world.x,obj.position_world.y],vx=obj.velocity_world.x,vy=obj.velocity_world.y,velocity_world=[obj.velocity_world.x,obj.velocity_world.y],last_seen=obj.last_seen,in_trigger_corridor=obj.in_trigger_corridor)
        if math.isfinite(obj.top_elevation):o['top_elevation']=obj.top_elevation
        else:o.pop('top_elevation',None)
        d['obstacles'].append(o)
    stamp=m.header.stamp.to_sec();q=m.ego_pose.orientation;p=m.ego_pose.position;yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
    d['mission_mode']=m.mission_mode
    d.update(schema_version=2,geometry_stamp=stamp,stamp=stamp,scene_id=m.scene_id,coordinate_version=m.coordinate_version,map_version=m.map_version,offset_x=m.offset_x,offset_y=m.offset_y,scan_pose=[p.x,p.y,p.z,yaw,m.speed,m.curvature,0.],pose_world=[p.x,p.y,yaw],cell_resolution=.1,collision_cells_world=[],voxels=[],visibility=[list(m.visibility[i:i+3]) for i in range(0,len(m.visibility),3)],measured_ray_cleared_world=[])
    owners={int(o['id']):o for o in d.get('obstacles',[])}
    for o in owners.values():o['occupied_cells_world']=[];o['observed_cells_world']=[]
    n=len(m.cells)
    if not all(len(a)==n for a in [m.owner_ids,m.fixed,m.seen,m.observed]):raise ValueError('native scene arrays differ')
    for p,oid,fixed,seen,observed in zip(m.cells,m.owner_ids,m.fixed,m.seen,m.observed):
        o=owners.get(oid)
        if o is None:raise ValueError('native scene has unknown owner')
        xy=[p.x,p.y];o['occupied_cells_world'].append(xy)
        if observed:o['observed_cells_world'].append(xy)
        # Confirmed unknown surfaces constrain planning transiently, without
        # certifying static admission; consumers use the same observed occupancy.
        if fixed or o.get('dynamic_confirmed',False) or (o.get('confirmed_obstacle',False) and -.05<=stamp-seen<=.35):
            d['collision_cells_world'].append(xy);d['voxels'].append([p.x,p.y,p.z,bool(fixed),seen])
    return d

def final_root(scene):
    root={k:scene.get(k) for k in ['scene_id','coordinate_version','map_version','geometry_digest']};root.update(stamp=scene['geometry_stamp'],mission_mode=scene.get('mission_mode','tunnel'),perception_source='common_scene',final_detector_source='shared_nonground_detector',collision_geometry_source='native_owned_cells',scene_state=scene.get('state','valid'))
    objects=copy.deepcopy(scene.get('obstacles',[]))
    for o in objects:
        o.update(perception_source='common_scene',final_detector_source='shared_nonground_detector',collision_geometry_valid=bool(o.get('occupied_cells_world')),collision_geometry_owner_id=int(o['id']),collision_geometry_coordinate_version=scene['coordinate_version'],collision_geometry_stamp=scene['geometry_stamp'],collision_geometry_reason='native_owned_cells',scene_id=scene['scene_id'],coordinate_version=scene['coordinate_version'])
    root['collision_objects']=objects;root['obstacles']=[o for o in objects if o.get('in_trigger_corridor',False)];return root

def detector_root(m):
    """Native detector message -> existing merger algorithms, no large JSON parse."""
    s=scene_dict(m);objects=s['obstacles'];points={int(o['id']):[] for o in objects}
    for p,oid in zip(m.cells,m.owner_ids):points[oid].append([p.x,p.y,p.z])
    for o in objects:o['support_points_world']=points[o['id']]
    return dict(stamp=m.header.stamp.to_sec(),mission_mode=m.mission_mode,collision_objects=objects,obstacles=[o for o in objects if o.get('in_trigger_corridor')],geometry_scan=dict(stamp=m.header.stamp.to_sec(),frame='map',origin=[m.ray_origin.x,m.ray_origin.y,m.ray_origin.z],endpoints=[[p.x,p.y,p.z]for p in m.ray_endpoints]))

def rebase_detector_tracks(tracks,dx,dy):
    """Translate cached world surfaces and tracking history; preserve velocity/size."""
    import numpy as np
    shift=np.array([dx,dy],float)
    for t in tracks:
        t.position=t.position+shift;t.anchor_pos=t.anchor_pos+shift
        t.history=[(stamp,p+shift)for stamp,p in t.history]
        if getattr(t,'kf',None) is not None:t.kf.x[:2]+=shift
        for key in ('support_points_world','footprint_world'):
            if getattr(t,key,None) is not None:setattr(t,key,[[q[0]+dx,q[1]+dy,*q[2:]]for q in getattr(t,key)])
        surface=getattr(t,'surface_motion',None)
        if surface is not None:
            surface.history=[(stamp,p+shift)for stamp,p in surface.history]
        motion=getattr(t,'motion_observation',None)
        if motion is not None:
            for stamp,points,yaw in motion.history:points[:,:2]+=shift
