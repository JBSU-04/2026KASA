#!/usr/bin/env python3
"""ROS1 adapter of Autoware distance-based compare-map segmentation.

Original files/revision/license: ../vendor/autoware_compare_map. The ROS2 filter
kernel (3D nearest-map distance) is preserved using scipy's cached KD tree.
Added: independent verified floor lookup, unknown state, scan-stamped evidence.
Coordinates remain float64 local to the map origin (never float32 absolute UTM).
"""
import numpy as np
from scipy.spatial import cKDTree

UNKNOWN, BACKGROUND, CANDIDATE, CONFIRMED = 0, 1, 2, 3

class StaticMap3D:
    def __init__(self, filename, background_distance=.12, floor_distance=.055):
        a = np.load(filename, allow_pickle=False)
        self.origin = np.asarray(a['origin'], dtype=float)
        self.points = np.asarray(a['points'], dtype=float)
        self.floor = np.asarray(a['floor'], dtype=float)
        self.tree = cKDTree(self.points) if len(self.points) else None
        self.floor_tree = cKDTree(self.floor[:, :2]) if len(self.floor) else None
        self.distance = background_distance
        self.floor_distance = floor_distance

    def ground(self, world):
        out = np.full(len(world), np.nan)
        if self.floor_tree is None or not len(world):
            return out
        d, j = self.floor_tree.query(world[:, :2]-self.origin[:2], k=3,
                                    distance_upper_bound=1.2)
        valid = np.isfinite(d).all(axis=1)
        rows = np.nonzero(valid)[0]
        z = self.floor[j[valid], 2]
        valid_rows = np.ptp(z, axis=1) < .12
        out[rows[valid_rows]] = np.median(z[valid_rows], axis=1)+self.origin[2]
        return out

    def classify(self, world):
        ground = self.ground(world)
        height = world[:, 2]-ground
        known = np.isfinite(ground)
        if self.tree is None:
            return np.zeros(len(world), bool), height, known
        d, _ = self.tree.query(world-self.origin, k=1,
                               distance_upper_bound=self.distance)
        threshold = np.where(known & (height < .25), self.floor_distance, self.distance)
        # A 3D match is required. XY coincidence with a tall wall is insufficient.
        background = np.isfinite(d) & (d <= threshold)
        return background, height, known

class ScanConfirmation:
    def __init__(self, min_points=3, scans=2, memory=.35):
        self.min_points, self.scans, self.memory = min_points, scans, memory
        self.history = {}
    def update(self, world, candidate, stamp):
        result = np.zeros(len(world), bool)
        self.history = {k:v for k,v in self.history.items() if 0 <= stamp-v[0] <= self.memory}
        keys=np.floor(world[:,:2]/.5).astype(np.int64)
        groups={}
        for i in np.flatnonzero(candidate):groups.setdefault(tuple(keys[i]),[]).append(i)
        occupied=set(groups);visited=set();used=set()
        for seed in occupied:
            if seed in visited:continue
            stack=[seed];component=set();visited.add(seed)
            while stack:
                k=stack.pop();component.add(k)
                for dx in (-1,0,1):
                    for dy in (-1,0,1):
                        q=(k[0]+dx,k[1]+dy)
                        if q in occupied and q not in visited:visited.add(q);stack.append(q)
            indices=np.asarray([i for k in component for i in groups[k]],dtype=int)
            if len(indices)<self.min_points:continue
            center=world[indices,:2].mean(axis=0);radius=max(.6,float(np.linalg.norm(np.ptp(world[indices,:2],axis=0)))*.5)
            old_key=None;best=radius
            for k,value in self.history.items():
                if k in used:continue
                distance=float(np.linalg.norm(center-value[2]))
                if distance<=best:best=distance;old_key=k
            if old_key is None:
                old_key=tuple(np.floor(center/.5).astype(int));previous=(-1.,0,center)
            else:previous=self.history[old_key]
            old_stamp,hits,_=previous
            if stamp>old_stamp+1e-6:hits+=1
            self.history[old_key]=(stamp,hits,center);used.add(old_key)
            result[indices]=hits>=self.scans
        return result

def fields_array(msg, names):
    fields = {f.name:f for f in msg.fields}
    dtype = np.dtype({'names':names,'formats':[('>' if msg.is_bigendian else '<')+'f4']*len(names),
                      'offsets':[fields[k].offset for k in names],'itemsize':msg.point_step})
    rows = [np.frombuffer(msg.data, dtype=dtype, count=msg.width, offset=i*msg.row_step)
            for i in range(msg.height)]
    a = np.concatenate(rows)
    return np.column_stack([a[k] for k in names])

def regional_ground_fast(p, ground_z):
    """Same robust regional plane tests; batch the cell percentile/median work."""
    keys=np.floor(p[:,:2]/[6.,4.]).astype(int)
    height=np.full(len(p),np.nan);valid=np.zeros(len(p),bool)
    low=p[(p[:,2]>ground_z-1.)&(p[:,2]<ground_z+.8)]
    if not len(low):return height,valid
    cells=np.floor(low[:,:2]).astype(int)
    _,group,counts=np.unique(cells,axis=0,return_inverse=True,return_counts=True)
    order=np.lexsort((low[:,2],group));ordered=low[order];groups=group[order]
    bounds=np.r_[0,np.cumsum(counts)];rank=.3*(counts-1);lo=np.floor(rank).astype(int);hi=np.ceil(rank).astype(int)
    cutoff=ordered[bounds[:-1]+lo,2]*(hi-rank)+ordered[bounds[:-1]+hi,2]*(rank-lo)
    # Integer ranks need their exact value (the interpolation weights sum to 0).
    exact=lo==hi;cutoff[exact]=ordered[bounds[:-1][exact]+lo[exact],2]
    selected=(ordered[:,2]<=cutoff[groups])&(counts[groups]>=3)
    if not selected.any():return height,valid
    selected_groups=groups[selected];offset=np.arange(len(ordered))-bounds[groups]
    matrix=np.full((len(counts),int(counts.max()),3),np.nan)
    matrix[selected_groups,offset[selected]]=ordered[selected]
    import warnings
    with warnings.catch_warnings():
        warnings.simplefilter('ignore',RuntimeWarning);seeds=np.nanmedian(matrix,axis=1)
    seeds=seeds[np.isfinite(seeds).all(axis=1)]
    for cell in np.unique(keys,axis=0):
        center=(cell+.5)*[6.,4.];near=seeds[(abs(seeds[:,0]-center[0])<=9)&(abs(seeds[:,1]-center[1])<=6)]
        if len(near)<12:continue
        A=np.column_stack([near[:,:2]-center,np.ones(len(near))]);keep=np.ones(len(near),bool)
        for _ in range(4):
            if keep.sum()<12:break
            coef=np.linalg.lstsq(A[keep],near[keep,2],rcond=None)[0];keep=abs(near[:,2]-A@coef)<=.12
        if keep.sum()<12 or np.linalg.norm(coef[:2])>.12 or np.ptp(near[keep,0])<2 or np.ptp(near[keep,1])<1 or np.sqrt(np.mean((near[keep,2]-A[keep]@coef)**2))>.07:continue
        target=np.all(keys==cell,axis=1);z=(p[target,:2]-center)@coef[:2]+coef[2]
        height[target]=(p[target,2]-z)/np.sqrt(1+np.sum(coef[:2]**2));valid[target]=True
    return height,valid

def classify_scan(xyz, pose, static_map, confirmation, sensor=(1.1,0.,1.33),
                  min_height=.08, max_height=2.8, sensor_height=1.765):
    """Shared online/offline kernel. pose=(world x,y,z,yaw), xyz gravity frame."""
    from obstacle_geometry import regional_plane_heights
    xyz=np.asarray(xyz,dtype=float);px,py,z,h=pose
    c,s=np.cos(h),np.sin(h);b=xyz+np.asarray(sensor)
    world=np.column_stack([px+c*b[:,0]-s*b[:,1],py+s*b[:,0]+c*b[:,1],z+b[:,2]])
    bg=np.zeros(len(xyz),bool);height=np.full(len(xyz),np.nan);valid=np.zeros(len(xyz),bool)
    if static_map is not None:bg,height,valid=static_map.classify(world)
    roi=(xyz[:,0]>=-15.)&(xyz[:,0]<=40.)&(np.abs(xyz[:,1])<=12.)
    local=np.full(len(xyz),np.nan);local_valid=np.zeros(len(xyz),bool)
    if roi.any():local[roi],local_valid[roi]=regional_ground_fast(xyz[roi],-sensor_height)
    floor_restore=local_valid & valid & (np.abs(local)<=.04) & (height>=.08)&(height<=.30)
    trusted=local_valid & (~valid | (np.abs(local-height)<=.10) | floor_restore)
    if static_map is not None and floor_restore.any():
        # Only distributed, near-zero local road support can restore a bounded
        # vertical road-registration mismatch. Never undo a verified BG match or
        # shift a positive object return onto the floor.
        query=world[floor_restore].copy()
        query[:,2]+=local[floor_restore]-height[floor_restore]
        # Ground support/threshold are already known here. Avoid repeating the
        # floor kNN query for the same restored points.
        distance,_=static_map.tree.query(query-static_map.origin,k=1,
                                         distance_upper_bound=static_map.floor_distance)
        bg[floor_restore] |= np.isfinite(distance)
    height[trusted]=local[trusted];valid|=trusted
    inside=(b[:,0]>-.79)&(b[:,0]<3.85)&(np.abs(b[:,1])<.95)
    proposal_height=np.where(valid,height,xyz[:,2]+sensor_height)
    candidate=roi & ~bg & ~inside & (proposal_height>=min_height)&(proposal_height<=max_height)
    return world,bg,height,valid,inside,candidate


def registered_lookup_pose(xyz,pose,static_map,delta,sensor=(1.1,0.,1.33)):
    """Apply VALID X-ICP lateral shift only when measured WALL support improves.

    The foreground itself never chooses registration or enters the static map.
    Sparse/incomplete walls retain the original lookup and UNKNOWN evidence.
    """
    out=np.asarray(pose,dtype=float).copy()
    if static_map is None or static_map.tree is None:return out
    delta=np.asarray(delta,dtype=float)
    if delta.shape!=(2,) or not np.isfinite(delta).all() or np.linalg.norm(delta)>.6:return out
    p=np.asarray(xyz);wall=(p[:,2]>=0)&(p[:,2]<=1.5)&(np.abs(p[:,1])>=1.5)&(np.abs(p[:,1])<=12)&(p[:,0]>=1)&(p[:,0]<=35)
    p=p[wall]
    if len(p)<100:return out
    if len(p)>500:p=p[np.linspace(0,len(p)-1,500,dtype=int)]
    c,s=np.cos(out[3]),np.sin(out[3]);b=p+np.asarray(sensor)
    world=np.c_[out[0]+c*b[:,0]-s*b[:,1],out[1]+s*b[:,0]+c*b[:,1],out[2]+b[:,2]]-static_map.origin
    d0,_=static_map.tree.query(world,k=1);world[:,:2]+=delta;d1,_=static_map.tree.query(world,k=1)
    if np.mean(d1<=static_map.distance)>np.mean(d0<=static_map.distance)+.02 and np.median(d1)<np.median(d0):out[:2]+=delta
    return out
