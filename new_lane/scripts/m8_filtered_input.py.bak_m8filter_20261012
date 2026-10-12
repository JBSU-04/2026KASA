"""Shared M8 point contract. Map matching never supplies mission evidence.

XYZ stays in the original gravity-aligned sensor frame. ground_relative is
measured by the M2 near/far producer, never recomputed from the background map.
"""
import math
from pathlib import Path
import numpy as np
import yaml
from scipy.spatial import cKDTree


def cloud_rows(msg):
    fields={f.name:f for f in msg.fields}
    names=['x','y','z','ground_relative']
    if not all(n in fields for n in names):
        return np.empty((0,4),np.float32)
    dtype=np.dtype({'names':names,'formats':[('>' if msg.is_bigendian else '<')+'f4']*4,
                    'offsets':[fields[n].offset for n in names],'itemsize':msg.point_step})
    a=np.ndarray((msg.height,msg.width),dtype=dtype,buffer=msg.data,
                 strides=(msg.row_step,msg.point_step))
    p=np.column_stack([a[n].ravel() for n in names]).astype(np.float32)
    return p[np.isfinite(p).all(axis=1)]


def filtered_cloud(header, rows, sensor_xy=(0.,0.)):
    from sensor_msgs.msg import PointCloud2,PointField
    p=np.asarray(rows,dtype=np.float32).reshape((-1,4)).copy()
    p[:,:2]-=np.asarray(sensor_xy,dtype=np.float32)
    # Producer tag distinguishes filtered sensor data from Raw/map residuals.
    p=np.column_stack([p,np.ones(len(p),np.float32)]).astype('<f4')
    msg=PointCloud2();msg.header=header;msg.height=1;msg.width=len(p);msg.point_step=20
    msg.row_step=20*len(p);msg.is_dense=True;msg.is_bigendian=False
    msg.fields=[PointField(name=n,offset=i*4,datatype=PointField.FLOAT32,count=1)
                for i,n in enumerate(['x','y','z','ground_relative','m2_filtered'])]
    msg.data=p.tobytes();return msg


class WallSurface:
    """Remove supported longitudinal wall surfaces, never an entire distance band."""
    def __init__(self, road, profile, band=.4):
        self.road=np.asarray(road,dtype=float)[:,:2]
        self.tree=cKDTree(self.road)
        tangent=np.gradient(self.road,axis=0)
        self.tangent=tangent/np.maximum(1e-9,np.linalg.norm(tangent,axis=1))[:,None]
        self.profile=np.asarray(profile,dtype=float).reshape((-1,3));self.band=band

    @classmethod
    def load(cls, profile_path=None):
        root=Path(__file__).resolve().parents[2]
        cfg=yaml.safe_load(Path(profile_path or root/'new_lane/config/tunnel_wall_profile.yaml').read_text())
        road=np.loadtxt(root/'waypoint_system/data/default/3.csv',delimiter=',')[:,1:3]
        return cls(road,cfg['tunnel_wall_profile'])

    def keep(self, sensor_rows, pose, sensor_xy=(1.1,0.)):
        p=np.asarray(sensor_rows);keep=np.ones(len(p),bool)
        if not len(p) or len(self.profile)<2:return keep
        c,s=math.cos(pose[2]),math.sin(pose[2])
        b=p[:,:2]+np.asarray(sensor_xy)
        w=b@np.array([[c,s],[-s,c]])+np.asarray(pose[:2])
        _,idx=self.tree.query(w);delta=w-self.road[idx];tan=self.tangent[idx]
        lat=-tan[:,1]*delta[:,0]+tan[:,0]*delta[:,1]
        knots=self.profile[:,0];inside=(idx>=knots[0])&(idx<=knots[-1])
        # Do not interpolate walls through open/unknown knots.
        k=np.clip(np.searchsorted(knots,idx,side='right')-1,0,len(knots)-1)
        for side,col in [(-1,1),(1,2)]:
            boundary=self.profile[k,col]
            candidates=inside&(side*boundary>0)&(abs(lat-boundary)<=self.band)
            for key in np.unique(idx[candidates]//50):
                ids=np.flatnonzero(candidates&(idx//50==key))
                high=ids[p[ids,3]>=.30]
                if len(high)<6:continue
                span=np.ptp(w[high]@tan[high].mean(axis=0))
                median=np.median(lat[high]);inlier=abs(lat[high]-median)<=.12
                if span<2. or np.mean(inlier)<.8:continue
                # Preserve low objects and protrusions, including within the band.
                remove=ids[(p[ids,3]>=.30)&(abs(lat[ids]-median)<=.12)]
                keep[remove]=False
        return keep


def safe_static_ids(root, path, pose, margin=1.15):
    """Per-owner suppression; no path, unknown/dynamic or stale => no release."""
    if path is None or len(path)<2:return set()
    xy=np.asarray(path,float)[:,:2];released=set()
    for o in root.get('collision_objects',root.get('obstacles',[])):
        if o.get('motion_state')!='static' or not (o.get('static_certified') or o.get('confirmed')):continue
        pts=np.asarray(o.get('support_points_world',o.get('observed_cells_world',o.get('planning_footprint_world',o.get('footprint_world',[])))),float)
        if pts.ndim!=2 or len(pts)==0:continue
        a=xy[:-1];v=xy[1:]-a;l2=np.sum(v*v,axis=1)
        distance=np.full(len(pts),np.inf)
        for i,p in enumerate(pts[:,:2]):
            u=np.clip(np.sum((p-a)*v,axis=1)/np.maximum(l2,1e-12),0,1)
            distance[i]=np.min(np.linalg.norm(p-a-u[:,None]*v,axis=1))
        # A safe prefix ending before this object is not an approved pass.
        end_tangent=xy[-1]-xy[-2]
        if np.any((pts[:,:2]-xy[-1])@end_tangent>=0):continue
        if np.all(distance>margin):released.add(int(o['id']))
    return released


def preslow_speed_cap(distance, start=25., minimum=4., initial=6.):
    if not math.isfinite(distance) or distance<0 or distance>start:return 9999.
    return float(minimum+(initial-minimum)*np.clip((distance-5.)/max(1.,start-5.),0.,1.))


def far_ground_working_set(points,x_min,x_max,y_max,plane_x,plane_y=None):
    """Keep full seed cells for every detection ROI neighborhood and plane fit.

    No detection point is downsampled. Whole one-meter seed cells at the bounds
    are retained so percentile/median fitting cannot change at a crop edge.
    """
    p=np.asarray(points)
    lo_x=min(float(plane_x[0]),(np.floor(x_min/6.)+.5)*6.-9.)
    hi_x=max(float(plane_x[1]),(np.floor(x_max/6.)+.5)*6.+9.)
    lo_y=(np.floor(-y_max/4.)+.5)*4.-6.
    hi_y=(np.floor(y_max/4.)+.5)*4.+6.
    if plane_y is not None:
        lo_y=min(lo_y,float(plane_y[0]));hi_y=max(hi_y,float(plane_y[1]))
    lo_x,lo_y=np.floor([lo_x,lo_y])-1.
    hi_x,hi_y=np.ceil([hi_x,hi_y])+1.
    return p[(p[:,0]>=lo_x)&(p[:,0]<=hi_x)&(p[:,1]>=lo_y)&(p[:,1]<=hi_y)]
