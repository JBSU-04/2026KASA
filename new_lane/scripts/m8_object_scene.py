"""Detector-owned M8 scene. No planner input and no synthetic box occupancy."""
import copy,math
import numpy as np
from scipy.spatial import cKDTree
from obstacle_geometry import RayEvidence
from tunnel_native_scene import packet_from_detector


def shared_surface(a,b,distance=.12):
    a=np.asarray(a,float);b=np.asarray(b,float)
    if a.ndim!=2 or b.ndim!=2 or min(len(a),len(b))<3:return False
    # Repeated vertical returns are not distinct correspondence features.
    a=np.unique(np.round(a[:,:2]/.02),axis=0)*.02;b=np.unique(np.round(b[:,:2]/.02),axis=0)*.02
    if min(len(a),len(b))<3:return False
    da,ia=cKDTree(b).query(a);db,ib=cKDTree(a).query(b)
    ga=da<=distance;gb=db<=distance
    return bool(ga.sum()>=3 and gb.sum()>=3 and ga.mean()>=.5 and gb.mean()>=.5 and
                len(np.unique(ia[ga]))>=3 and len(np.unique(ib[gb]))>=3)


class ObjectSceneBuilder:
    def __init__(self):
        self.records={};self.aliases={};self.parent_aliases={};self.version=None;self.last_stamp=-1.;self.sequence=0;self.handoff=None

    def rebase(self,version,dx,dy):
        for r in self.records.values():
            r['cells']={tuple(np.floor((np.asarray(p)+[dx,dy,0])/.1+1e-8).astype(np.int64)):(np.asarray(p)+[dx,dy,0],seen) for p,seen in r['cells'].values()}
            r['free']=[]
        self.version=version

    def build(self,root,sources,pose,stamp,now,version=0,offset=(0.,0.),visibility=(),route=None):
        if not -.05<=now-stamp<=.35 or stamp<=self.last_stamp:return None
        if self.version!=version:
            self.records={};self.version=version
        # Roots are current detections + the existing retained parents.
        objects=copy.deepcopy(root.get('collision_objects',[]));src={int(o['id']):o for o in sources}
        resolved={}
        for o in objects:
            internal=int(o['id']);track=int(o.get('observed_track_id',internal));key=(o.get('source','lidar_only'),track)
            owner=self.parent_aliases.get(internal,self.aliases.get(key))
            points=src.get(track,{}).get('support_points_world',[])
            if owner is None:
                matches=[oid for oid,r in self.records.items() if shared_surface(points,[v[0]for v in r['cells'].values()])]
                owner=matches[0] if len(matches)==1 else internal
            prior=self.aliases.get(key)
            if prior is not None and prior!=owner and prior in self.records:
                old=self.records.pop(prior);target=self.records.setdefault(owner,dict(cells={},free=[],certified=False))
                for cell,value in old['cells'].items():
                    if cell not in target['cells'] or target['cells'][cell][1]<value[1]:target['cells'][cell]=value
                target['certified']|=old['certified'];target['free']=[]
                self.aliases={k:(owner if v==prior else v)for k,v in self.aliases.items()}
                self.parent_aliases={k:(owner if v==prior else v)for k,v in self.parent_aliases.items()}
            self.parent_aliases[internal]=owner;self.aliases[key]=owner
            o['id']=owner;o['observed_track_id']=track
            previous=resolved.get(owner)
            rank=lambda q:(bool(q.get('dynamic_candidate') and q.get('observed_motion_valid')) or q.get('motion_state')=='dynamic',q.get('source')!='raw_temporal_far',float(q.get('last_seen',-1e9)))
            if previous is None or rank(o)>rank(previous):resolved[owner]=o
        objects=list(resolved.values());active=set(resolved)
        if self.handoff is not None and route is not None:
            for o in objects:
                measured=src.get(int(o['observed_track_id']),{})
                o['support_points_world']=measured.get('support_points_world',[])
                for key in ('motion_support_xy','motion_epoch','motion_dr_valid'):
                    if key in measured:o[key]=measured[key]
            self.handoff.update({'collision_objects':objects},stamp,route,0)

        self.records={k:v for k,v in self.records.items()if k in active}
        src={int(o['id']):o for o in sources};owned_sources=[];parents={}
        rays=None
        for o in objects:
            oid=int(o['id']);track=int(o.get('observed_track_id',oid));observed=src.get(track,{})
            points=np.asarray(observed.get('support_points_world',[]),float)
            if points.ndim!=2 or points.shape[1]<3:points=np.empty((0,3))
            else:points=points[np.isfinite(points[:,:3]).all(1),:3]
            seen=float(observed.get('last_seen',-1e9));valid=-.05<=stamp-seen<=.35
            moving=bool(o.get('motion_state')=='dynamic' or (o.get('dynamic_candidate') and o.get('observed_motion_valid')))
            r=self.records.setdefault(oid,dict(cells={},free=[],certified=False))
            if moving:r['cells'].clear();r['free']=[];r['certified']=False
            if o.get('ever_dynamic') and not o.get('m8_restatic') and not moving:
                r['certified']=False
            r['certified']=bool(not moving and not (o.get('ever_dynamic') and not o.get('m8_restatic')) and (r['certified'] or o.get('static_certified') or o.get('motion_state')=='static' or o.get('m8_restatic')))
            if valid:
                for q in points:
                    key=tuple(np.floor(q/.1+1e-8).astype(np.int64));r['cells'][key]=(q.tolist(),seen)
            present={tuple(np.floor(q/.1+1e-8).astype(np.int64))for q in points} if valid else set()
            old=[key for key in r['cells']if key not in present]
            # Whole-component clearance is deliberately conservative; unseen
            # rays cannot clear cells. Distinct returned scans are required.
            if old and rays is None:rays=RayEvidence(dict(root.get('geometry_scan',{}),native_geometry=True),now)
            if old and rays.clear([r['cells'][k][0]for k in old]):
                if not r['free'] or stamp>r['free'][-1]+.07:r['free'].append(stamp)
                r['free']=[t for t in r['free']if stamp-t<=.5][-3:]
                if len(r['free'])>=3:
                    for k in old:r['cells'].pop(k,None)
                    o['collision_geometry_reason']='three_returned_scans_clear_old_surface'
            else:r['free']=[]
            if not r['certified']:
                r['cells']={k:v for k,v in r['cells'].items()if stamp-v[1]<=.35}
            # Never certify an unknown track solely because its cells persist.
            if r['certified']:
                o['static_certified']=True;o['static_occupancy']=True
            o['bbox_is_diagnostic_only']=True
            o['observed_surface_stamp']=seen if valid else o.get('last_seen',-1e9)
            o['observed_track_id']=track;o['observation_stamp']=seen
            o.pop('motion_support_xy',None)
            # Keep each actual surface; no clipping against the display box.
            parents[oid]=dict(support=[v[0]for v in r['cells'].values()],stamp=max((v[1]for v in r['cells'].values()),default=seen))
            owned_sources.append(dict(observed,id=track,support_points_world=points.tolist()))
        prepared=dict(root,collision_objects=objects)
        self.sequence+=1
        packet=packet_from_detector(prepared,dict(collision_objects=owned_sources),parents,pose,stamp,self.sequence,version,offset)
        # Preserve each surface's observation time; a new scene is not a new observation.
        for i,(q,oid) in enumerate(zip(packet.cells,packet.owner_ids)):
            key=tuple(np.floor(np.asarray([q.x,q.y,q.z])/.1+1e-8).astype(np.int64))
            held=self.records[oid]['cells'].get(key)
            if held is not None:
                packet.seen[i]=held[1];packet.observed[i]=abs(stamp-held[1])<1e-6
        packet.visibility=list(visibility)
        packet.speed=float(pose[4]) if len(pose)>4 else 0.
        packet.curvature=float(pose[5]) if len(pose)>5 else 0.
        packet.mission_mode=root.get('mission_mode','tunnel')
        self.last_stamp=stamp
        return packet
