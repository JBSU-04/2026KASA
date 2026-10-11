"""M8 measured occupancy association. Boxes identify a track; never fill a box.
Independent of ROS. Unknown geometry is not an empty/clear object.
"""
import math
import numpy as np
from scipy.spatial import cKDTree

def observed_polygon(obs):
    return np.asarray(obs.get('observed_footprint_world') or obs.get('footprint_world') or [],float).reshape((-1,2))

def associate(scene,objects,now,previous=None):
    previous={} if previous is None else previous
    stamp=float(scene.get('geometry_stamp',-1e9));resolution=float(scene.get('cell_resolution',.1))
    if not -.05<=now-stamp<=.35:return {},previous
    cells=np.asarray(scene.get('collision_cells_world',[]),float).reshape((-1,2))
    if not np.isfinite(cells).all():return {},previous
    tree=cKDTree(cells) if len(cells) else None
    owned=np.full(len(cells),-1,int);scores=np.full(len(cells),np.inf)
    ids=[];polygons=[]
    for obs in objects:
        oid=int(obs.get('id',-1));poly=observed_polygon(obs)
        if oid in ids or len(poly)<3:continue
        ids.append(oid);polygons.append(poly)
    # Each occupied cell belongs to at most one observed object. Keep holes and
    # disconnected components; no convex hull or rectangle becomes occupancy.
    for idx,poly in enumerate(polygons):
        center=poly.mean(axis=0);radius=np.linalg.norm(poly-center,axis=1).max()+.2
        candidates=np.asarray(tree.query_ball_point(center,radius),int) if tree is not None else np.empty(0,int)
        if not len(candidates):continue
        q=cells[candidates];edge=np.full(len(q),np.inf);signed=np.zeros(len(q),int)
        area=np.sum(poly[:,0]*np.roll(poly[:,1],-1)-poly[:,1]*np.roll(poly[:,0],-1));orientation=1 if area>=0 else -1
        inside=np.ones(len(q),bool)
        for a,b in zip(poly,np.roll(poly,-1,axis=0)):
            v=b-a;l2=float(v@v)
            if l2<1e-12:continue
            u=np.clip(((q-a)@v)/l2,0,1);edge=np.minimum(edge,np.linalg.norm(q-a-u[:,None]*v,axis=1))
            inside &= orientation*(v[0]*(q[:,1]-a[1])-v[1]*(q[:,0]-a[0]))>=-1e-6
        eligible=inside|(edge<=resolution*.7072+.1)
        # Nearest observed center resolves overlapping diagnostic rectangles.
        score=np.linalg.norm(q-center,axis=1)
        use=eligible&(score<scores[candidates]);owned[candidates[use]]=idx;scores[candidates[use]]=score[use]
    result={}
    pose=scene.get('pose_world',[])
    cleared=np.asarray(scene.get('measured_ray_cleared_world',[]),float).reshape((-1,2))
    cleared_tree=cKDTree(cleared) if len(cleared) and np.isfinite(cleared).all() else None
    for idx,oid in enumerate(ids):
        selected=cells[owned==idx];old=previous.get(oid,{});valid=bool(len(selected));reason='measured_occupied_cells'
        if not valid and old.get('cells') and len(pose)>=3:
            center=polygons[idx].mean(axis=0);d=center-np.asarray(pose[:2]);c,s=math.cos(pose[2]),math.sin(pose[2]);x=c*d[0]+s*d[1];y=-s*d[0]+c*d[1]
            # Only explicit three-scan ray-clear receipts can release retained
            # occupancy. Transient expiry or map ROI departure are not clearance.
            old_points=np.asarray(old['cells']);present=False
            if tree is not None:present=np.any(tree.query(old_points,k=1)[0]<resolution*.8)
            ray_cleared=cleared_tree is not None and np.all(cleared_tree.query(old_points,k=1)[0]<=resolution*1.5)
            if not present and ray_cleared and -12<x<35 and abs(y)<12:
                valid=True;reason='previous_cells_cleared_by_engine'
        result[oid]=dict(cells=selected.tolist(),stamp=stamp,valid=valid,reason=reason)
        if len(selected):previous[oid]=dict(cells=selected.tolist(),stamp=stamp)
    previous={k:v for k,v in previous.items() if 0<=now-v['stamp']<=10}
    return result,previous

def body_cells_risk(cells,ego,yaw,resolution=.1):
    p=np.asarray(cells,float).reshape((-1,2))
    if not len(p):return False
    delta=p-np.asarray(ego);c,s=math.cos(yaw),math.sin(yaw);x=c*delta[:,0]+s*delta[:,1];y=-s*delta[:,0]+c*delta[:,1];pad=.2+resolution*.7072
    return bool(np.any((x>=-.79-pad)&(x<=3.85+pad)&(abs(y)<=.95+pad)))

def swept_owned_cells(cells,path,ego,ego_yaw,resolution=.1,horizon=10.):
    from m8_geometry_native import available,sweep
    if available() and resolution==.1 and path is not None:
        return sweep(cells,path,ego,ego_yaw,horizon)
    p=np.asarray(cells,float).reshape((-1,2));xy=np.asarray(path,float)
    if len(xy)<2:return None
    v=np.diff(xy,axis=0);length=np.linalg.norm(v,axis=1)
    if np.any(length<1e-6):return None
    if not len(p):return False
    if body_cells_risk(p,ego,ego_yaw,resolution):return True
    u=np.clip(np.sum((np.asarray(ego)-xy[:-1])*v,axis=1)/(length*length),0,1)
    j=int(np.argmin(np.linalg.norm(xy[:-1]+u[:,None]*v-ego,axis=1)))
    # Object beyond the finite forward horizon cannot be declared clear.
    tangent=v[-1]/length[-1]
    if np.all((p-xy[-1])@tangent>3.85):return None
    tree=cKDTree(p);pad=.2+resolution*.7072;radius=math.hypot(3.85+pad,.95+pad);travelled=0.
    for k in range(j,len(v)):
        tangent=v[k]/length[k];start=u[k]*length[k]if k==j else 0.
        distances=np.arange(start,length[k]+1e-6,.1)
        for d in distances:
            center=xy[k]+d*tangent;indices=tree.query_ball_point(center,radius)
            if not indices:continue
            delta=p[indices]-center;x=delta@tangent;y=-delta[:,0]*tangent[1]+delta[:,1]*tangent[0]
            if np.any((x>=-.79-pad)&(x<=3.85+pad)&(abs(y)<=.95+pad)):return True
        travelled+=length[k]-start
        if travelled>=horizon:break
    return False
