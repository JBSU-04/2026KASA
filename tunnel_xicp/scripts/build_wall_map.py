#!/usr/bin/env python3
"""Build a LOCAL-origin, normal-bearing wall PCD. Reads bags, never replays ROS.

The old 2.5D map is only a wall-position prior; it is NOT extruded into a 3D map.
Output geometry consists exclusively of measured gravity-aligned LiDAR points.
"""
import argparse
import bisect
import hashlib
import json
import math
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree


def xyz(msg):
    fields = {f.name: f.offset for f in msg.fields}
    if any(k not in fields for k in ('x', 'y', 'z')):
        raise ValueError('missing xyz fields')
    endian = '>' if msg.is_bigendian else '<'
    dtype = np.dtype({'names': ['x', 'y', 'z'], 'formats': [endian+'f4']*3,
                      'offsets': [fields[k] for k in ('x', 'y', 'z')],
                      'itemsize': msg.point_step})
    # Honour row padding instead of assuming an unorganized, tightly packed cloud.
    rows = [np.frombuffer(msg.data, dtype=dtype, count=msg.width,
                          offset=i*msg.row_step) for i in range(msg.height)]
    a = np.concatenate(rows)
    p = np.column_stack([a[k] for k in ('x', 'y', 'z')])
    return p[np.isfinite(p).all(axis=1)]


def interpolate_pose(poses, stamp, max_gap=.08):
    i = np.searchsorted(poses[:, 0], stamp)
    if i < len(poses) and abs(poses[i, 0]-stamp) < 1e-6:
        return poses[i, 1:]
    if i == 0 or i == len(poses):
        return None
    a, b = poses[i-1], poses[i]
    if b[0]-a[0] > 2*max_gap or min(stamp-a[0], b[0]-stamp) > max_gap:
        return None
    u = (stamp-a[0])/(b[0]-a[0])
    return a[1:]*(1-u)+b[1:]*u


def to_local_world(points, pose, origin, sensor_offset):
    p = points.astype(np.float64)+np.asarray(sensor_offset)
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    return np.column_stack([x-origin[0]+c*p[:, 0]-s*p[:, 1],
                            y-origin[1]+s*p[:, 0]+c*p[:, 1],
                            p[:, 2]-origin[2]])


def mask_obstacles(world_xy, obstacles, margin=.5):
    keep = np.ones(len(world_xy), dtype=bool)
    for o in obstacles:
        w = o.get('position_world', [])
        if len(w) < 2:
            continue
        radius = .5*math.hypot(float(o.get('length', 0) or 0),
                                float(o.get('width', 0) or 0))+margin
        keep &= np.sum((world_xy-np.asarray(w[:2]))**2, axis=1) > radius**2
    return keep


def voxelize(points, resolution=.2):
    if not len(points):
        return points
    keys = np.floor(points/resolution).astype(np.int64)
    _, inv, counts = np.unique(keys, axis=0, return_inverse=True, return_counts=True)
    sums = np.zeros((len(counts), 3))
    np.add.at(sums, inv, points)
    return sums/counts[:, None]


def wall_normals(points, radius=.6, min_neighbors=6):
    """Retain planar VERTICAL surfaces; do not assign synthetic wall normals."""
    tree = cKDTree(points)
    out = []
    for p, inds in zip(points, tree.query_ball_point(points, radius)):
        if len(inds) < min_neighbors:
            continue
        q = points[inds]
        vals, vectors = np.linalg.eigh(np.cov(q.T))
        if vals[1] < 1e-5 or vals[0]/vals[1] > .15:
            continue
        normal = vectors[:, 0]
        if abs(normal[2]) > .3:
            continue
        out.append(np.concatenate([p, normal]))
    return np.asarray(out, dtype=np.float32).reshape(-1, 6)


def write_pcd(path, points):
    header = ('# .PCD v0.7\nVERSION 0.7\nFIELDS x y z normal_x normal_y normal_z curvature\n'
              'SIZE 4 4 4 4 4 4 4\nTYPE F F F F F F F\nCOUNT 1 1 1 1 1 1 1\n'
              'WIDTH %d\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS %d\nDATA binary\n'
              % (len(points), len(points)))
    with open(path, 'wb') as f:
        f.write(header.encode('ascii'))
        f.write(np.column_stack([points,np.zeros(len(points))]).astype('<f4').tobytes())


def file_sha256(path):
    h = hashlib.sha256()
    with open(path, 'rb') as f:
        for block in iter(lambda: f.read(1024*1024), b''):
            h.update(block)
    return h.hexdigest()


def build(args):
    import rosbag
    import rospy
    poses, sections, objects = [], [], []
    with rosbag.Bag(args.pose_bag) as bag:
        start = bag.get_start_time()
        for topic, m, t in bag.read_messages(topics=['/odom', '/mission/active_section', '/obstacle_info']):
            if topic == '/odom':
                if m.header.frame_id != 'map':
                    raise ValueError('pose bag must use map frame')
                q = m.pose.pose.orientation
                yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
                poses.append((m.header.stamp.to_sec(), m.pose.pose.position.x,
                              m.pose.pose.position.y, yaw))
            elif topic == '/mission/active_section':
                sections.append((t.to_sec(), m.section_id))
            else:
                data = json.loads(m.data)
                obs = data.get('collision_objects', data.get('obstacles', []))
                objects.append((t.to_sec(), obs))
    if not poses or not sections:
        raise ValueError('missing recorded odometry or mission section')
    poses = np.asarray(sorted(set(poses)))
    poses[:, 3] = np.unwrap(poses[:, 3])
    stimes = [x[0] for x in sections]
    otimes = [x[0] for x in objects]
    prior = np.load(args.background_map)
    hraw = prior['hmax_raw']
    heading = float(prior['theta'])
    c, s = math.cos(heading), math.sin(heading)
    origin = np.array([poses[0, 1], poses[0, 2], 0.0])
    collected, included_stamps = [], []
    scan_count = 0
    with rosbag.Bag(args.cloud_bag) as bag:
        for _, m, t in bag.read_messages(topics=['/perception/points_gravity'],
              start_time=rospy.Time.from_sec(start+args.start_sec),
              end_time=rospy.Time.from_sec(start+args.end_sec)):
            stamp = m.header.stamp.to_sec()
            relative = stamp-start
            if any(a <= relative <= b for a, b in args.exclude):
                continue
            j = bisect.bisect_right(stimes, stamp)-1
            if j < 0 or sections[j][1] != 'M8':
                continue
            if m.header.frame_id != 'velodyne_ground':
                raise ValueError('expected yaw-only gravity frame velodyne_ground')
            pose = interpolate_pose(poses, stamp)
            if pose is None:
                continue
            p = xyz(m)
            p = p[(p[:, 0]>=1)&(p[:, 0]<=35)&(abs(p[:, 1])>=1.5)&
                  (abs(p[:, 1])<=12)&(p[:, 2]>=0)&(p[:, 2]<=1.5)]
            q = to_local_world(p, pose, origin, args.sensor_offset)
            wx, wy = q[:, 0]+origin[0], q[:, 1]+origin[1]
            dx, dy = wx-prior['origin'][0], wy-prior['origin'][1]
            iu = np.floor((dx*c+dy*s)/float(prior['res'])).astype(int)-int(prior['i0'][0])
            iv = np.floor((-dx*s+dy*c)/float(prior['res'])).astype(int)-int(prior['i0'][1])
            valid = (iu>=0)&(iu<hraw.shape[0])&(iv>=0)&(iv<hraw.shape[1])
            # Only cells actually observed as high structures in the RAW map;
            # the dilated height map is deliberately not used for this selection.
            keep = np.zeros(len(q), dtype=bool)
            keep[valid] = hraw[iu[valid], iv[valid]] >= 1.5
            j = bisect.bisect_right(otimes, stamp+.1)-1
            if j >= 0 and abs(objects[j][0]-stamp) <= .3:
                keep &= mask_obstacles(np.column_stack([wx, wy]), objects[j][1])
            q = q[keep]
            if len(q):
                collected.append(q)
                included_stamps.append(stamp)
                scan_count += 1
    if not collected:
        raise ValueError('no measured walls available outside held-out windows')
    cloud = wall_normals(voxelize(np.concatenate(collected), args.voxel))
    if len(cloud) < 100:
        raise ValueError('wall map has fewer than 100 valid planar points')
    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    metadata = out.with_suffix('.json')
    if not args.overwrite and (out.exists() or metadata.exists()):
        raise FileExistsError('map already exists; choose a new output or explicit --overwrite')
    write_pcd(out, cloud)
    meta = dict(format_version=1, frame='map', source_frame='velodyne_ground',
                origin=origin.tolist(), road_heading_rad=heading,
                sensor_offset=list(args.sensor_offset), voxel_m=args.voxel,
                wall_height_sensor_m=[0.0, 1.5], wall_normal_max_abs_z=.3,
                source_pose_bag=str(Path(args.pose_bag).resolve()),
                source_cloud_bag=str(Path(args.cloud_bag).resolve()),
                source_start=start, selected_window=[args.start_sec, args.end_sec],
                excluded_windows=args.exclude, included_scan_count=scan_count,
                included_header_stamps=included_stamps, points=len(cloud),
                background_sha256=file_sha256(args.background_map),
                pcd_sha256=file_sha256(out), bounds_local=[cloud[:, :3].min(axis=0).tolist(),
                                                        cloud[:, :3].max(axis=0).tolist()],
                limitations='Same-run reference. Not independently surveyed or validated. No control use.')
    metadata.write_text(json.dumps(meta, indent=2)+'\n')
    print(json.dumps({k:meta[k] for k in ('points','included_scan_count','excluded_windows','origin')}))


def main():
    a = argparse.ArgumentParser(description=__doc__)
    a.add_argument('--pose-bag', required=True)
    a.add_argument('--cloud-bag', required=True)
    a.add_argument('--background-map', required=True)
    a.add_argument('--out', required=True)
    a.add_argument('--start-sec', type=float, default=16)
    a.add_argument('--end-sec', type=float, default=58)
    a.add_argument('--exclude', nargs=2, type=float, action='append')
    a.add_argument('--sensor-offset', nargs=3, type=float, default=[1.10, 0, 1.33],
                   help='yaw-only sensor->base translation, matching kcity_ground_pipeline TF')
    a.add_argument('--voxel', type=float, default=.2)
    a.add_argument('--overwrite', action='store_true')
    args = a.parse_args()
    if args.voxel<=0 or args.end_sec<=args.start_sec:
        a.error('invalid voxel or time window')
    if args.exclude is None:
        args.exclude = [[24,26], [42,53]]
    if any(b<a for a,b in args.exclude):
        a.error('invalid exclusion window')
    build(args)


if __name__ == '__main__':
    main()
