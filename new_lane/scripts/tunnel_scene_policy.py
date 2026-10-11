"""Shared M8 scene adapters and object-owned pre-slow policy; no ROS imports."""
import copy
import math
import numpy as np
from tunnel_occupancy_geometry import swept_owned_cells,body_cells_risk

def adapt_scene(scene, pose, now):
    stamp=float(scene.get('geometry_stamp',-1e9))
    valid=scene.get('state')=='valid' and -.05<=now-stamp<=.35
    root={k:scene.get(k) for k in ('scene_id','coordinate_version','map_version','geometry_digest')}
    root.update(stamp=stamp,mission_mode='tunnel',perception_source='common_scene',collision_geometry_source='common_scene',obstacles=[],collision_objects=[],scene_state='valid' if valid else 'expired')
    if not valid:return root
    x,y,yaw=pose;c,s=math.cos(yaw),math.sin(yaw)
    for source in scene.get('obstacles',[]):
        o=copy.deepcopy(source);pw=o.get('center',[])
        if len(pw)!=2:continue
        dx,dy=pw[0]-x,pw[1]-y;bx=c*dx+s*dy;by=-s*dx+c*dy
        vel=o.get('velocity_world',[0.,0.]);motion=o.get('motion_state','unknown')
        o.update(position_world=pw,position=[bx,by,0.],position_ego=[bx,by,0.],distance=math.hypot(bx,by),source='common_scene',perception_source='common_scene',
                 scene_id=scene.get('scene_id'),coordinate_version=scene.get('coordinate_version'),bbox_is_diagnostic_only=True,
                 collision_geometry_valid=True,collision_geometry_stamp=stamp,collision_geometry_reason='common_scene_cells',
                 observation_age=max(0.,now-float(o.get('last_seen',stamp))),footprint_world=o.get('footprint_world',[]),
                 observed_footprint_world=o.get('footprint_world',[]),static_occupancy=motion=='static' and o.get('confirmed_obstacle',False),static_certified=motion=='static' and o.get('confirmed_obstacle',False),
                 is_dynamic=motion=='dynamic',dynamic_confirmed=motion=='dynamic',velocity_valid=motion=='dynamic',
                 speed=math.hypot(*vel),vx=vel[0],vy=vel[1],velocity=vel,observed_velocity=vel,observed_motion_valid=motion=="dynamic",dynamic_streak=2 if motion=="dynamic" else 0,vel_std=.3,in_trigger_corridor=True,passed=False,
                 occupancy_state='confirmed' if motion=='static' else motion)
        root['collision_objects'].append(o);root['obstacles'].append(o)
    return root

def preslow(scene, pose, path, path_stamp, now, distance=20., reference=None,object_risk=True):
    stamp=float(scene.get('geometry_stamp',-1e9))
    if scene.get('state')!='valid' or not -.05<=now-stamp<=.35:
        return dict(present=False,distance=-1.,inpath=-1.,state='expired',objects=[],scene_id=scene.get('scene_id'))
    x,y,yaw=pose;c,s=math.cos(yaw),math.sin(yaw);targets=[];nearest=-1.;inpath=-1.
    fresh_path=path is not None and len(path)>=2 and -.05<=now-path_stamp<=.5
    if fresh_path:
        xy=np.asarray(path,float);v=np.diff(xy,axis=0);l2=np.maximum(np.sum(v*v,axis=1),1e-9);u=np.clip(np.sum((np.asarray([x,y])-xy[:-1])*v,axis=1)/l2,0,1)
        dist=np.linalg.norm(xy[:-1]+u[:,None]*v-[x,y],axis=1);j=int(np.argmin(dist));angle=math.atan2(v[j,1],v[j,0]);heading=abs(math.atan2(math.sin(angle-yaw),math.cos(angle-yaw)))
        fresh_path=dist[j]<=.3 and heading<=math.radians(10.)
    for o in scene.get('obstacles',[]):
        cells=np.asarray(o.get('occupied_cells_world',[]),float).reshape((-1,2))
        if not len(cells):continue
        d=cells-np.asarray([x,y]);bx=c*d[:,0]+s*d[:,1];by=-s*d[:,0]+c*d[:,1]
        reason='outside_monitoring'
        if o.get('observed_surface_count',3)<3 and o.get('last_surface_count',3)<3:
            targets.append(dict(id=o['id'],reason='insufficient_points_near_risk_only',active=False));continue
        if bx.max()<-.79-.2:reason='passed_body'
        elif bx.max()>=0 and bx.min()<=distance and np.any(abs(by)<=12.):
            risk=swept_owned_cells(cells,path,[x,y],yaw,horizon=max(20.,distance)) if fresh_path else None
            certified=bool(o.get('static_certified',False) or (o.get('motion_state')=='static' and o.get('confirmed_obstacle',True)))
            monitor=path if fresh_path else reference
            monitor_risk=swept_owned_cells(cells,monitor,[x,y],yaw,horizon=max(20.,distance)) if monitor is not None else None
            if body_cells_risk(cells,[x,y],yaw):monitor_risk=True
            crossing=False
            if o.get('dynamic_confirmed',False) or (o.get('observed_motion_valid',False) and int(o.get('dynamic_streak',0))>=2):
                vel=np.asarray(o.get('velocity_world',[0.,0.]),float)
                if vel.shape==(2,) and np.isfinite(vel).all() and monitor is not None:
                    # Match the finite planned driving time, capped at four seconds.
                    horizon=min(4.,distance/max(.5,float(scene.get('scan_pose',[0,0,0,0,0])[4])))
                    for t in np.arange(.1,horizon+.05,.1):
                        if swept_owned_cells(cells+vel*t,monitor,[x,y],yaw,horizon=distance) is True:crossing=True;break
            if certified and risk is False:reason='accepted_path_body_clear'
            elif object_risk and monitor_risk is False and not crossing:reason='outside_future_body_region'
            elif object_risk and monitor_risk is None and not crossing and not o.get('in_trigger_corridor',True):reason='outside_reference_corridor'
            else:
                reason='unconfirmed_or_crossing' if not certified else ('body_conflict' if risk else 'path_unknown')
                # Far side objects cannot become the distance cap of an already
                # cleared close object. Distances belong to active owners only.
                positive=bx[bx>=0];dist=float(positive.min()) if len(positive) else 0.
                nearest=dist if nearest<0 else min(nearest,dist)
                # Before an avoidance path exists, the fresh reference sweep
                # must also drive stopping-distance deceleration.
                if monitor_risk is True and dist>=4.35:inpath=dist if inpath<0 else min(inpath,dist)
        targets.append(dict(id=o['id'],reason=reason,active=reason in ('unconfirmed_or_crossing','body_conflict','path_unknown')))
    return dict(present=any(o['active'] for o in targets),distance=nearest,inpath=inpath,state='valid',objects=targets,scene_id=scene.get('scene_id'),coordinate_version=scene.get('coordinate_version'))

def tag_reference_corridor(root,route,radius=1.3,resolution=.1):
    """Relevance metadata uses real cells against reference segments, not a box."""
    xy=np.asarray([r[:2]for r in route],float)
    if len(xy)<2:return root
    from m8_geometry_native import available,Route
    native=None
    if available():
        key=xy.tobytes();cached=getattr(tag_reference_corridor,'_route',None)
        if cached is None or cached[0]!=key:cached=(key,Route(xy));tag_reference_corridor._route=cached
        native=cached[1]
    v=np.diff(xy,axis=0);l2=np.maximum(np.sum(v*v,axis=1),1e-9)
    for o in root.get('collision_objects',[]):
        cells=np.asarray(o.get('occupied_cells_world',[]),float).reshape((-1,2))
        if not len(cells):o['in_trigger_corridor']=False;continue
        if native is not None:distance=native.distance(cells)
        else:
            delta=cells[:,None,:]-xy[None,:-1,:]
            u=np.clip(np.sum(delta*v[None,:,:],axis=2)/l2[None,:],0.,1.)
            distance=float(np.linalg.norm(delta-u[:,:,None]*v[None,:,:],axis=2).min())
        o['in_trigger_corridor']=distance-resolution*.7072<=radius
        o['reference_edge_distance_m']=max(0.,distance-resolution*.7072)
    return root
