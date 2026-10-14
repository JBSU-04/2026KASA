"""M8-only evidence and risk helpers. Unknown is never an empty cell set."""
import math
import numpy as np
from scipy.spatial import cKDTree
from tunnel_occupancy_geometry import swept_owned_cells, body_cells_risk

def fresh(stamp, now, timeout=.35):
    return stamp is not None and math.isfinite(float(stamp)) and -.05 <= now-float(stamp) <= timeout

def verified_motion(o, now, coordinate_version=0):
    e=o.get('motion_evidence') or {}
    return (fresh(o.get('last_seen'),now) and
            e.get('coordinate_version',coordinate_version)==coordinate_version and
            abs(float(e.get('observation_stamp',-1e9))-float(o.get('last_seen',0)))<1e-4 and
            bool(e.get('surface_translation_valid')) and not e.get('common_pose_shift') and
            bool(o.get('observed_motion_valid')))

def static_certificate(o,now,coordinate_version=0):
    # Classification and collision geometry are independent contracts.
    certificate=o.get('static_certified') or o.get('motion_state')=='static' or o.get('m8_restatic')
    return bool(certificate and o.get('confirmed_obstacle',o.get('confirmed',False)) and
        (fresh(o.get('last_seen'),now) or (o.get('static_occupancy') and o.get('collision_geometry_source','native_owned_cells')=='native_owned_cells' and fresh(o.get('collision_geometry_stamp'),now) and bool(o.get('occupied_cells_world')))) and o.get('coordinate_version',coordinate_version)==coordinate_version and
        not verified_motion(o,now,coordinate_version))

def owned_cells(o,now,coordinate_version=0):
    if (not o.get('collision_geometry_valid') or not fresh(o.get('collision_geometry_stamp'),now) or
        o.get('coordinate_version',coordinate_version)!=coordinate_version or
        o.get('collision_geometry_coordinate_version',coordinate_version)!=coordinate_version or
        o.get('collision_geometry_owner_id',o.get('id'))!=o.get('id')):return None
    cells=np.asarray(o.get('occupied_cells_world',[]),float).reshape((-1,2))
    if not np.isfinite(cells).all():return None
    if not len(cells) and o.get('collision_geometry_reason')!='previous_cells_cleared_by_engine':return None
    return cells

def owned_risk(o,path,pose,now,coordinate_version=0,horizon=30.):
    cells=owned_cells(o,now,coordinate_version)
    if cells is None:return None
    return swept_owned_cells(cells,path,pose[:2],pose[2],horizon=horizon)

def point_path_distance(points,path,ego):
    """Finite remaining segments, rather than an old origin or corridor box."""
    xy=np.asarray(path,float);p=np.asarray(points,float).reshape((-1,2))
    if len(xy)<2 or not np.isfinite(xy).all():return None
    v=np.diff(xy,axis=0);l2=np.sum(v*v,axis=1)
    if np.any(l2<1e-9):return None
    u=np.clip(np.sum((np.asarray(ego)-xy[:-1])*v,axis=1)/l2,0,1)
    j=int(np.argmin(np.linalg.norm(xy[:-1]+u[:,None]*v-ego,axis=1)))
    xy=np.vstack((xy[j]+u[j]*v[j],xy[j+1:]));v=np.diff(xy,axis=0);l2=np.maximum(np.sum(v*v,axis=1),1e-9)
    if not len(p):return np.empty(0)
    _,vertices=cKDTree(xy).query(p,k=min(2,len(xy)))
    vertices=np.asarray(vertices).reshape((len(p),-1));best=np.full(len(p),np.inf)
    for column in range(vertices.shape[1]):
        for offset in (-1,0):
            k=np.clip(vertices[:,column]+offset,0,len(v)-1);a=xy[k];step=v[k];ll=l2[k]
            t=np.clip(np.sum((p-a)*step,axis=1)/ll,0,1)
            best=np.minimum(best,np.linalg.norm(p-a-t[:,None]*step,axis=1))
    return best

def early_risk(points,path,pose,speed,forward,side_speed=3.,crossing_window=1.5):
    """Unknown compact surface: can it reach the driven body before we pass?

    side_speed is a configurable monitoring envelope, never measured velocity.
    Confirmed moving objects use their measured prediction in the stop node.
    """
    distance=point_path_distance(points,path,pose[:2])
    if distance is None:return None
    travel=np.minimum(crossing_window,np.maximum(0.,np.asarray(forward))/max(4.,abs(speed)))
    return distance <= .95+.2+side_speed*travel


def inspection_required(o,path,pose,speed):
    """Missing shape: request stop only inside the current stopping region.

    This is not a clearance certification and cannot release a previous hold.
    Final planning/coverage still enforces braking beyond the finite path.
    """
    points=o.get('observed_footprint_world') or [o.get('position_world',[])]
    try:p=np.asarray(points,float).reshape((-1,2))
    except (ValueError,TypeError):return True
    if not len(p) or not np.isfinite(p).all():return True
    d=point_path_distance(p,path,pose[:2]) if path is not None else None
    if d is None:return True
    c,s=math.cos(pose[2]),math.sin(pose[2]);delta=p-np.asarray(pose[:2]);forward=c*delta[:,0]+s*delta[:,1]
    stopping=3.85+.3*abs(speed)+speed*speed/5.+.2
    return bool(np.any((forward>=-.99)&(forward<=stopping)&(d<=1.15)))
