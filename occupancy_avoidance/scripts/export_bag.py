#!/usr/bin/env python3
"""Export synchronized raw LiDAR/pose/reference for the exact C++ engine.

Does not replay a bag, publish ROS topics or open a simulator control socket.
"""
import argparse
import bisect
import math
import struct
from pathlib import Path
import numpy as np
import rosbag
import rospy
import yaml
import json


def yaw(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


def cloud_xyz(m):
    fields = {f.name: f.offset for f in m.fields}
    endian = '>' if m.is_bigendian else '<'
    dtype = np.dtype({'names': ['x','y','z'], 'formats': [endian+'f4']*3,
                      'offsets': [fields[x] for x in ['x','y','z']], 'itemsize': m.point_step})
    arr = np.ndarray((m.height, m.width), dtype=dtype, buffer=m.data,
                     strides=(m.row_step, m.point_step))
    xyz = np.stack([arr[x].ravel() for x in ['x','y','z']], axis=-1)
    return xyz[np.isfinite(xyz).all(axis=1)].astype('<f4')


def main():
    args = argparse.ArgumentParser()
    args.add_argument('regular'); args.add_argument('heavy'); args.add_argument('output')
    args.add_argument('--root', default='/home/kuuve/catkin_ws')
    args.add_argument('--tunnel-map',help='M8 shared 3D classifier; output OCC5 annotated returns')
    args.add_argument('--map-registration',help='Read-only VALID X-ICP scan registration JSON')
    args.add_argument('--wall-observations',help='Read-only verified-background wall comparison JSON')
    args.add_argument('--lane', type=int, choices=[0, 3], default=0)
    args.add_argument('--static-planning-start-idx', type=int, default=2400,
                      help='static ownership start, including optional early approach')
    args.add_argument('--start-sec', type=float)
    args.add_argument('--end-sec', type=float)
    a = args.parse_args(); root = Path(a.root)
    static_map=confirmation=None
    if a.tunnel_map:
        import sys
        sys.path.insert(0,str(root/'src/new_lane/scripts'))
        from tunnel_map_geometry import StaticMap3D,ScanConfirmation,classify_scan,registered_lookup_pose
        static_map=StaticMap3D(a.tunnel_map);confirmation=ScanConfirmation(3,2)
    with rosbag.Bag(a.regular) as bag:
        origin = bag.get_start_time()
    window = {}
    if a.start_sec is not None: window['start_time'] = rospy.Time.from_sec(origin+a.start_sec-2.)
    if a.end_sec is not None: window['end_time'] = rospy.Time.from_sec(origin+a.end_sec)
    csv = np.loadtxt(root / ('src/waypoint_system/data/default/%d.csv' % a.lane), delimiter=',')
    road = csv[:, 1:3]; angles = np.arctan2(np.gradient(road[:, 1]), np.gradient(road[:, 0]))
    if a.lane == 0:
        edges = yaml.safe_load((root/'src/new_lane/config/curb_edges.yaml').read_text())['curb_edges']
        edges = [(int(r[0]), int(r[1]), float(r[2]), float(r[4]) if len(r)>4 else -1.) for r in edges]
        lo, hi = a.static_planning_start_idx, 3700
    else:
        profile = yaml.safe_load((root/'src/new_lane/config/tunnel_wall_profile.yaml').read_text())['tunnel_wall_profile']
        edges = [(int(profile[i]), int(profile[i+3])-1 if i+3<len(profile) else 9000,
                  -float(profile[i+1]), float(profile[i+2])) for i in range(0, len(profile), 3)]
        lo, hi = 7000, 9000
    poses, routes, walls, objects, speeds, admissions = [], [], [], [], [], []
    stop_distances, stop_requirements, dynamic_states = [], [], []
    with rosbag.Bag(a.regular) as b:
        for topic, m, t in b.read_messages(topics=['/Competition_topic', '/odom', '/final_waypoint', '/tunnel/wall_limits', '/obstacle_info_near', '/obstacle_info', '/localization/stop_ahead', '/localization/stop_required', '/obstacle_stop/dynamic_state']):
            if topic == '/Competition_topic':
                speeds.append((t.to_sec(), math.hypot(m.velocity.x,m.velocity.y)))
            elif topic == '/odom':
                p=m.pose.pose.position; v=m.twist.twist.linear
                poses.append((m.header.stamp.to_sec() or t.to_sec(), p.x,p.y,p.z,yaw(m.pose.pose.orientation),math.hypot(v.x,v.y)))
            elif topic == '/tunnel/wall_limits':
                walls.append((t.to_sec(), list(m.data)))
            elif topic == '/obstacle_info':
                try: admissions.append((t.to_sec(), json.loads(m.data)))
                except (ValueError, TypeError): pass
            elif topic == '/obstacle_info_near':
                try: objects.append((t.to_sec(), json.loads(m.data)))
                except (ValueError, TypeError): pass
            elif topic == '/localization/stop_ahead':
                stop_distances.append((t.to_sec(), float(m.data[0]) if m.data else -1.))
            elif topic == '/localization/stop_required':
                stop_requirements.append((t.to_sec(), bool(m.data)))
            elif topic == '/obstacle_stop/dynamic_state':
                try: dynamic_states.append((t.to_sec(), json.loads(m.data)))
                except (ValueError, TypeError): pass
            elif m.waypoints:
                routes.append((t.to_sec(), m.waypoints[0].lane_number,
                               [(w.pose.pose.position.x,w.pose.pose.position.y,w.ryaw,w.waypoint_index) for w in m.waypoints]))
    # Same speed source as the final owner/embedded adapter; MORAI odom twist may be zero.
    speeds.sort();speed_times=[v[0] for v in speeds]
    for i,p in enumerate(poses):
        j=bisect.bisect_right(speed_times,p[0])-1
        if j>=0 and 0<=p[0]-speeds[j][0]<=.5:poses[i]=(*p[:5],speeds[j][1])
    poses.sort(); routes.sort(key=lambda r:r[0]); pts=[p[0] for p in poses]; rts=[r[0] for r in routes]
    if not poses or not routes:
        raise RuntimeError('regular bag is missing odom or final_waypoint')
    def pose_at(stamp):
        j = bisect.bisect_left(pts, stamp)
        if j <= 0 or j >= len(poses) or poses[j][0]-poses[j-1][0] > .3:
            return None
        p0, p1 = poses[j-1], poses[j]
        u = (stamp-p0[0])/(p1[0]-p0[0])
        p = np.array(p0[1:], dtype=np.float64)
        p += u*(np.array(p1[1:], dtype=np.float64)-p)
        p[3] = p0[4]+u*math.atan2(math.sin(p1[4]-p0[4]),math.cos(p1[4]-p0[4]))
        return p
    registrations=[];registration_times=[]
    if a.map_registration:
        data=json.load(open(a.map_registration))['800']['rows']
        registrations=sorted([v for v in data if v['source_bag']==Path(a.regular).name and v['state']==2],key=lambda v:v['stamp'])
        registration_times=[v['stamp'] for v in registrations]
    if a.wall_observations:
        walls=[(v['stamp'],v['limits']) for v in json.load(open(a.wall_observations)) if len(v['limits'])>=5]
    ground_samples = []
    with rosbag.Bag(a.heavy) as b:
        for _, m, t in b.read_messages(topics=['/groundgrid/ground'], **window):
            if m.header.frame_id in ('velodyne_ground', 'velodyne'):
                ground_samples.append((t.to_sec(),m.header.stamp.to_sec() or t.to_sec(),cloud_xyz(m)))
    ground_times = [v[0] for v in ground_samples]
    wall_times = [v[0] for v in walls]
    object_times = [v[0] for v in objects]
    admission_times = [v[0] for v in admissions]
    old_regions = {}
    distance_times=[v[0] for v in stop_distances]; required_times=[v[0] for v in stop_requirements]
    dynamic_times=[v[0] for v in dynamic_states]
    mission=yaml.safe_load((root/'src/total_bringup/config/missions.yaml').read_text()).get('traffic_light_mission',{})
    axle=float(mission.get('front_axle_offset_m',3.845));buffer=float(mission.get('stop_buffer_m',3.0))
    with open(a.output,'wb') as out:
        out.write(struct.pack('<II',0x4f434335 if static_map is not None else 0x4f434334,len(road)))
        for i, xy in enumerate(road):out.write(struct.pack('<dddi',xy[0],xy[1],angles[i],i))
        out.write(struct.pack('<I',len(edges)))
        for row in edges:out.write(struct.pack('<iidd',*row))
        frames=0
        with rosbag.Bag(a.heavy) as b:
            for _,m,t in b.read_messages(topics=['/perception/points_gravity'], **window):
                stamp=m.header.stamp.to_sec() or t.to_sec(); j=bisect.bisect_left(pts,stamp)
                if j<=0 or j>=len(poses) or poses[j][0]-poses[j-1][0]>.3:continue
                k=bisect.bisect_right(rts,t.to_sec())-1
                if k<0 or t.to_sec()-routes[k][0]>.5:continue
                _, lane, route=routes[k]; idx=route[0][3]
                if lane!=a.lane or not lo-200<=idx<=hi+100:continue
                pose=pose_at(stamp)
                xyz=cloud_xyz(m)
                if static_map is not None:
                    lookup=pose[:4].copy();reg=bisect.bisect_left(registration_times,stamp)-1
                    if reg>=0 and 0<=stamp-registration_times[reg]<=.3:lookup=registered_lookup_pose(xyz,lookup,static_map,registrations[reg]['delta_xy'])
                    world,bg,height,valid,inside,candidate=classify_scan(xyz,lookup,static_map,confirmation)
                    confirmed=confirmation.update(world,candidate,stamp)
                    label=np.where(bg,1.,np.where(confirmed,np.where(valid,3.,4.),np.where(candidate,2.,0.)))
                    xyz=np.column_stack([xyz,xyz[:,2]-height,label]).astype('<f4')
                # Recompute reference yaw exactly as the online node does.
                rr=np.array(route); dy=np.diff(rr[:,1]);dx=np.diff(rr[:,0]);rr[:-1,2]=np.arctan2(dy,dx);rr[-1,2]=rr[-2,2]
                out.write(struct.pack('<6dIII',stamp,*pose,int(idx),int(lo<=idx<=hi),len(xyz)));out.write(xyz.tobytes())
                out.write(struct.pack('<I',len(rr)))
                for row in rr:out.write(struct.pack('<dddi',float(row[0]),float(row[1]),float(row[2]),int(row[3])))
                gi=bisect.bisect_right(ground_times,t.to_sec())-1
                ground=np.empty((0,3),dtype='<f4')
                if gi>=0 and abs(ground_samples[gi][1]-stamp)<=.2:
                    gp=pose_at(ground_samples[gi][1])
                    if gp is not None:
                        ground=ground_samples[gi][2].astype(np.float64)
                        cg,sg,cp,sp=math.cos(gp[3]),math.sin(gp[3]),math.cos(pose[3]),math.sin(pose[3])
                        dx=gp[0]+cg*(ground[:,0]+1.1)-sg*ground[:,1]-pose[0]
                        dy=gp[1]+sg*(ground[:,0]+1.1)+cg*ground[:,1]-pose[1]
                        ground=np.column_stack([cp*dx+sp*dy-1.1,-sp*dx+cp*dy,ground[:,2]+gp[2]-pose[2]]).astype('<f4')
                out.write(struct.pack('<I',len(ground)));out.write(ground.tobytes())
                wi=bisect.bisect_right(wall_times,t.to_sec())-1
                wall_valid=0;wall_data=[0.]*10
                if a.lane==3 and wi>=0 and t.to_sec()-walls[wi][0]<=.5:
                    wt,wd=walls[wi];wp=pose_at(wt)
                    if len(wd)>=5 and wd[0]>0 and wd[1]>0 and wd[2]>0 and wp is not None:
                        wall_valid=1;wall_data=[wt,*wp,*wd[1:5]]
                out.write(struct.pack('<I10d',wall_valid,*wall_data))
                oi=bisect.bisect_right(object_times,t.to_sec())-1;dynamic=[]
                di=bisect.bisect_right(dynamic_times,t.to_sec())-1
                authority=(a.lane==3 and di>=0 and 0<=t.to_sec()-dynamic_states[di][0]<=.5)
                dyn_ids=set(dynamic_states[di][1].get('dynamic_ids',[])) if authority else set()
                static_ids=set(dynamic_states[di][1].get('static_ids',[])) if authority else set()
                if a.lane==3 and oi>=0 and abs(objects[oi][1].get('stamp',objects[oi][0])-stamp)<=.35:
                    for o in objects[oi][1].get('collision_objects',objects[oi][1].get('obstacles',[])):
                        pw=o.get('position_world',[])
                        if len(pw)<2:continue
                        region=(pw[0],pw[1],o.get('geometry_reference_yaw',pose[3]),o.get('length',.35),o.get('width',.35))
                        oid=o.get('id',-1)
                        moving=(oid in dyn_ids and oid not in static_ids) if authority else (o.get('motion_state')=='dynamic' and o.get('dynamic_streak',0)>=2 and (o.get('velocity_valid') or o.get('observed_motion_valid')))
                        if moving:
                            dynamic.append(region)
                            if oid in old_regions:dynamic.append(old_regions[oid])
                        old_regions[oid]=region
                out.write(struct.pack('<I',len(dynamic)))
                for region in dynamic:out.write(struct.pack('<5d',*region))
                si=bisect.bisect_right(distance_times,t.to_sec())-1;ri=bisect.bisect_right(required_times,t.to_sec())-1
                sd=stop_distances[si] if si>=0 else (-1.,-1.)
                sr=stop_requirements[ri] if ri>=0 else (-1.,False)
                out.write(struct.pack('<III5d',int(a.lane==0),int(si>=0 or ri>=0),int(sr[1]),sd[1],sd[1]-axle-buffer,sd[0],sr[0],t.to_sec()))
                ai=bisect.bisect_right(admission_times,t.to_sec())-1;certified=[]
                if ai>=0 and 0<=t.to_sec()-admissions[ai][0]<=.5:
                    for o in admissions[ai][1].get('obstacles',[]):
                        if not o.get('static_occupancy') or o.get('passed') or o.get('background_excluded') or o.get('motion_state')=='dynamic' or o.get('id') in dyn_ids:continue
                        if a.lane==3 and not (o.get('id') in static_ids or o.get('motion_state')=='static' or o.get('static_certified')):continue
                        seen=float(o.get('last_seen',admissions[ai][1].get('stamp',t.to_sec())))
                        if t.to_sec()-seen>3. or seen>t.to_sec()+.05:continue
                        poly=o.get('planning_footprint_world',o.get('footprint_world',[]))
                        poly=[(float(q[0]),float(q[1]),0.) for q in poly if len(q)>=2 and math.isfinite(q[0]) and math.isfinite(q[1])]
                        if len(poly)>=3:certified.append((seen,poly))
                out.write(struct.pack('<I',len(certified)))
                for seen,poly in certified:
                    out.write(struct.pack('<dI',seen,len(poly)))
                    for q in poly:out.write(struct.pack('<3d',*q))
                frames+=1
        print('exported',frames,'sensor frames to',a.output)


if __name__ == '__main__':
    main()
