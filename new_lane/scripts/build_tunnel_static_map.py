#!/usr/bin/env python3
"""Augment existing measured wall map with only reviewed source-window road points.
No current/random obstacle drive is learned. Unsupported cells stay unknown.
"""
import os,sys,json,argparse,math
from pathlib import Path
sys.path.insert(0,str(Path(__file__).parents[2]/'tunnel_xicp/scripts'))
import rospkg
sys.path.insert(0,str(Path(rospkg.RosPack().get_path('tunnel_xicp'))/'scripts'))
from build_wall_map import xyz,interpolate_pose,to_local_world,mask_obstacles
import numpy as np
import rosbag

def build(metadata,wall_pcd,out):
    import yaml
    height=float(yaml.safe_load((Path(__file__).parents[2]/'total_bringup/config/lidar_extrinsic.yaml').read_text())['lidar_extrinsic']['z_from_ground'])
    m=json.load(open(metadata));origin=np.asarray(m['origin']);start=m['source_start']
    poses=[];obstacles=[]
    with rosbag.Bag(m['source_pose_bag']) as b:
        for topic,msg,t in b.read_messages(topics=['/odom','/obstacle_info']):
            if topic=='/odom':
                p=msg.pose.pose.position;q=msg.pose.pose.orientation
                h=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
                poses.append([msg.header.stamp.to_sec(),p.x,p.y,h])
            else:
                try:
                    v=json.loads(msg.data);obstacles.append((t.to_sec(),v.get('collision_objects',v.get('obstacles',[]))))
                except ValueError:pass
    poses=np.asarray(poses);poses[:,3]=np.unwrap(poses[:,3]);cells={};scan_count=0
    with rosbag.Bag(m['source_cloud_bag']) as b:
        for _,msg,t in b.read_messages(topics=['/perception/points_gravity']):
            stamp=msg.header.stamp.to_sec();dt=stamp-start
            if not m['selected_window'][0]<=dt<=m['selected_window'][1]:continue
            if any(a<=dt<=z for a,z in m['excluded_windows']):continue
            pose=interpolate_pose(poses,stamp)
            if pose is None:continue
            p=xyz(msg);p=p[(np.abs(p[:,2]+height)<.055)&(p[:,0]>-12)&(p[:,0]<40)&(np.abs(p[:,1])<12)]
            w=to_local_world(p,pose,origin,m['sensor_offset'])
            near=min(obstacles,key=lambda o:abs(o[0]-stamp)) if obstacles else None
            if near and abs(near[0]-stamp)<.3:w=w[mask_obstacles(w[:,:2]+origin[:2],near[1],.7)]
            # Repeat observations in >=3 different scans, not >=3 points/one scan.
            k=np.floor(w/.2).astype(int)
            for key in set(map(tuple,k)):
                q=w[(k==key).all(axis=1)].mean(axis=0)
                old=cells.get(key);cells[key]=(q,1) if old is None else ((old[0]*old[1]+q)/(old[1]+1),old[1]+1)
            scan_count+=1
    floor=np.asarray([p for p,n in cells.values() if n>=3]).reshape((-1,3))
    with open(wall_pcd,'rb') as f:
        while True:
            line=f.readline()
            if line.startswith(b'DATA '):break
        if line.strip()!=b'DATA binary':raise ValueError('expected binary PointNormal PCD')
        # Existing PCD has fields x y z normal_x normal_y normal_z curvature.
        names=m.get('pcd_fields',None)
        raw=f.read()
    # Parse PCD header offsets rather than assume PCL padding.
    header=Path(wall_pcd).read_bytes().split(b'DATA binary\n')[0].decode()
    fields=next(l.split()[1:] for l in header.splitlines() if l.startswith('FIELDS'))
    sizes=list(map(int,next(l.split()[1:] for l in header.splitlines() if l.startswith('SIZE'))))
    counts=list(map(int,next(l.split()[1:] for l in header.splitlines() if l.startswith('COUNT'))))
    offsets=np.cumsum([0]+[s*c for s,c in zip(sizes,counts)])
    dtype=np.dtype({'names':['x','y','z'],'formats':['<f4']*3,'offsets':[int(offsets[fields.index(k)]) for k in ['x','y','z']],'itemsize':int(offsets[-1])})
    a=np.frombuffer(raw,dtype=dtype);walls=np.column_stack([a[k] for k in ['x','y','z']])
    np.savez_compressed(out,origin=origin,points=np.vstack([walls,floor]).astype('f4'),floor=floor.astype('f4'))
    report=dict(source_metadata=str(metadata),included_scans=scan_count,floor_points=len(floor),wall_points=len(walls),notes='Floor only in reviewed old windows; repeated observed points, no new foreground learning. Map validation remains geometric and incomplete cells UNKNOWN.')
    Path(str(out)+'.json').write_text(json.dumps(report,indent=2)+'\n');print(report)

def build_empty_pairs(pairs, metadata, out, extrinsic=None,wall_out=None,sections=None):
    """GPS NORMAL empty drives, independent maps and measured cross-drive checks."""
    import yaml,hashlib
    from datetime import datetime
    from scipy.spatial import cKDTree
    from tunnel_map_boundary import physical_profile,common_profile
    if len(pairs)!=2:raise ValueError('Exactly two user-verified GPS-normal empty drives required')
    stack=Path(__file__).parents[2]
    sections=Path(sections) if sections else stack/'total_bringup/config/missions.yaml'
    cfg=yaml.safe_load(sections.read_text());router=cfg.get('section_router',cfg)
    m8=next(v for v in router['sections'] if v['id']=='M8')
    first,last=map(int,m8['idx']);map_first,map_last=first-100,last+100
    road=np.loadtxt(stack/'waypoint_system/data/default/3.csv',delimiter=',')[:,1:3]
    m=json.load(open(metadata));origin=np.asarray(m['origin'],dtype=float)
    config=Path(extrinsic) if extrinsic else Path(__file__).parents[2]/'total_bringup/config/lidar_extrinsic.yaml'
    ex=yaml.safe_load(config.read_text())['lidar_extrinsic']
    offset=[float(ex[k]) for k in ('x','y','z')];height=float(ex['z_from_ground'])
    drive_maps=[];drive_floors=[];observed_free=[];stats=[]
    for drive,(regular,heavy) in enumerate(pairs):
        poses=[];routes=[];gps=[]
        with rosbag.Bag(regular) as bag:
            for topic,msg,t in bag.read_messages(topics=['/odom','/final_waypoint','/localization/gps_mode']):
                if topic=='/odom':
                    q=msg.pose.pose.orientation;v=msg.pose.pose.position
                    yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
                    poses.append([msg.header.stamp.to_sec(),v.x,v.y,yaw,v.z])
                elif topic=='/final_waypoint' and msg.waypoints:routes.append((t.to_sec(),msg.waypoints[0].lane_number,msg.waypoints[0].waypoint_index))
                elif topic=='/localization/gps_mode':gps.append((t.to_sec(),msg.data))
        poses=np.asarray(poses);poses[:,3]=np.unwrap(poses[:,3]);rt=np.array([x[0]for x in routes]);gt=np.array([x[0]for x in gps])
        all_cells={};floor_cells={};supports={};floor_supports={};free=set();scans=0;skipped=0
        with rosbag.Bag(heavy) as bag:
            for topic,msg,t in bag.read_messages(topics=['/perception/points_gravity','/groundgrid/ground']):
                stamp=msg.header.stamp.to_sec();ri=np.searchsorted(rt,stamp,side='right')-1;gi=np.searchsorted(gt,stamp,side='right')-1
                if ri<0 or routes[ri][1]!=3 or not map_first<=routes[ri][2]<=map_last:continue
                if gi<0 or gps[gi][1]!='normal' or stamp-gps[gi][0]>.3:skipped+=1;continue
                pose=interpolate_pose(poses,stamp)
                if pose is None or msg.header.frame_id!='velodyne_ground':skipped+=1;continue
                p=xyz(msg);bx=p[:,0]+offset[0];by=p[:,1]+offset[1]
                body=(bx>-.79)&(bx<3.85)&(np.abs(by)<.95)
                p=p[~body&(p[:,0]>-15)&(p[:,0]<40)&(np.abs(p[:,1])<12)&(p[:,2]>-height-.45)&(p[:,2]<1.5)]
                if topic.endswith('ground'):p=p[np.abs(p[:,2]+height)<.15]
                w=to_local_world(p,pose[:3],origin,offset);w[:,2]+=pose[3]
                cells=all_cells if topic.endswith('gravity') else floor_cells
                support=supports if topic.endswith('gravity') else floor_supports
                # Repeat support belongs to a physical surface patch. Retain
                # measured 5cm detail inside a patch seen in 3 distinct scans;
                # sparse 16-channel returns need not hit the identical 5cm voxel.
                for patch in set(map(tuple,np.floor(w/.2).astype(np.int64))):
                    count,last=support.get(patch,(0,-1.))
                    if abs(stamp-last)>1e-6:support[patch]=(count+1,stamp)
                keys,inv,counts=np.unique(np.floor(w/.05).astype(np.int64),axis=0,return_inverse=True,return_counts=True)
                sums=np.zeros((len(keys),3));np.add.at(sums,inv,w);means=sums/counts[:,None]
                for key,q in zip(map(tuple,keys),means):
                    old=cells.get(key)
                    if old is None:cells[key]=(q,1,stamp)
                    elif abs(stamp-old[2])>1e-6:cells[key]=((old[0]*old[1]+q)/(old[1]+1),old[1]+1,stamp)
                if topic.endswith('gravity'):
                    scans+=1
                    # Only actual measured ray interiors establish visibility;
                    # never manufacture free space from absent returns.
                    origin_scan=np.array([pose[0]-origin[0],pose[1]-origin[1],pose[3]+offset[2]-origin[2]])
                    c,ss=np.cos(pose[2]),np.sin(pose[2]);origin_scan[:2]+=[c*offset[0]-ss*offset[1],ss*offset[0]+c*offset[1]]
                    for end in w[::12]:
                        vector=end-origin_scan;length=np.linalg.norm(vector)
                        if length<1:continue
                        ray=origin_scan+np.arange(.4,max(.4,length-.35),.2)[:,None]*vector/length
                        free.update(map(tuple,np.floor(ray/.2).astype(np.int64)))
        points=np.array([p for p,n,last in all_cells.values() if supports.get(tuple(np.floor(p/.2).astype(np.int64)),(0,0))[0]>=3],dtype=float).reshape((-1,3))
        floor=np.array([p for p,n,last in floor_cells.values() if floor_supports.get(tuple(np.floor(p/.2).astype(np.int64)),(0,0))[0]>=3],dtype=float).reshape((-1,3))
        if not len(points) or not len(floor):raise ValueError('No verified GPS-normal repeated map/floor')
        np.savez_compressed(str(out)+'.drive'+str(drive)+'.npz',origin=origin,points=points.astype('f4'),floor=floor.astype('f4'))
        drive_maps.append(points);drive_floors.append(floor);observed_free.append(free);stats.append(dict(scans=scans,skipped=skipped,points=len(points),floor=len(floor)))
        print('drive',drive,stats[-1],flush=True)
    def checked_merge(maps):
        accepted=[];comparisons=[]
        for i,points in enumerate(maps):
            keep=np.ones(len(points),bool)
            for j,other in enumerate(maps):
                if i==j:continue
                distances,_=cKDTree(other).query(points,k=1,distance_upper_bound=.12)
                visible=np.fromiter((tuple(k) in observed_free[j] for k in np.floor(points/.2).astype(np.int64)),bool,count=len(points))
                conflict=visible&~np.isfinite(distances);keep&=np.isfinite(distances)
                comparisons.append(dict(drive=i,other=j,matched=int(np.isfinite(distances).sum()),visible_conflicts=int(conflict.sum())))
            accepted.append(points[keep])
        return np.vstack(accepted),comparisons
    points,comparison=checked_merge(drive_maps);floor,floor_comparison=checked_merge(drive_floors)
    np.savez_compressed(out,origin=origin,points=points.astype('f4'),floor=floor.astype('f4'))
    report=dict(format_version=2,empty_drives=pairs,gps_required='normal',sensor_offset=offset,sensor_height=height,origin=origin.tolist(),resolution=.05,minimum_distinct_scans=3,support_patch_m=.2,points=len(points),floor_points=len(floor),drives=stats,cross_drive=comparison,floor_comparison=floor_comparison,version='gps_empty_'+datetime.now().strftime('%Y%m%d_%H%M%S'),sensor_config_sha256=hashlib.sha256(config.read_bytes()).hexdigest(),notes='Each accepted background point matches both independently repeated empty-drive maps within 0.12m. Unsupported/conflicting coverage remains unknown; physical boundaries contain no vehicle inset.')
    if wall_out:
        profiles=[physical_profile(p,f,origin,road,map_first,map_last) for p,f in zip(drive_maps,drive_floors)]
        profile=common_profile(profiles)
        # A zero end knot prevents the runtime step profile extending a wall beyond observations.
        if profile:profile.append([profile[-1][0]+10,0.,0.])
        wall=dict(tunnel_wall_profile=[v for row in profile for v in row],physical_boundary=True,center_inset_m=1.05,map_version=report['version'],sensor_config_sha256=report['sensor_config_sha256'],unknown_boundary_value=0.,source_empty_drives=pairs)
        Path(wall_out).write_text(yaml.safe_dump(wall,sort_keys=False))
        report['physical_wall_profile']=str(wall_out)
    Path(str(out)+'.json').write_text(json.dumps(report,indent=2)+'\n');print(report,flush=True)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--metadata',required=True);p.add_argument('--extrinsic');p.add_argument('--walls');p.add_argument('--wall-out');p.add_argument('--sections');p.add_argument('--out',required=True);p.add_argument('--empty-pair',nargs=2,action='append');a=p.parse_args()
    if a.empty_pair:build_empty_pairs(a.empty_pair,a.metadata,a.out,a.extrinsic,a.wall_out,a.sections)
    else:build(a.metadata,a.walls,a.out)
