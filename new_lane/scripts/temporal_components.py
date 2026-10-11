"""Shared sparse-component connectivity for M2 proposals and M8 tracking.
M8 emits current measured returns only: old returns connect evidence but cannot
be relabelled as a new observation or used as current motion measurements.
"""
from collections import deque
import numpy as np
from scipy.spatial import cKDTree


def connected_components(xy, eps):
    if not len(xy): return []
    tree = cKDTree(xy)
    parent = np.arange(len(xy))
    def root(i):
        while parent[i] != i:
            parent[i] = parent[parent[i]]; i = parent[i]
        return i
    for i, j in tree.query_pairs(eps):
        ri, rj = root(i), root(j)
        if ri != rj: parent[rj] = ri
    groups = {}
    for i in range(len(xy)): groups.setdefault(root(i), []).append(i)
    return list(groups.values())


class TemporalFarInput:
    def __init__(self, window=.55, eps=.65, points=5, scans=2):
        self.window,self.eps,self.points,self.scans=window,eps,points,scans
        self.frames=deque();self.version=None

    def groups(self, current, pose, stamp, version):
        if version != self.version or (self.frames and (stamp <= self.frames[-1][0] or stamp-self.frames[-1][0]>.35)):
            self.frames.clear();self.version=version
        c,s=np.cos(pose[2]),np.sin(pose[2])
        current=np.asarray(current,float)
        ids=np.nonzero((current[:,0]>=12.)&(current[:,0]<=40.)&np.isfinite(current).all(axis=1)&(current[:,3]>=.08))[0]
        world=current[ids,:2]@np.array([[c,s],[-s,c]])+np.asarray(pose[:2])
        while self.frames and stamp-self.frames[0][0]>self.window:self.frames.popleft()
        old=list(self.frames);self.frames.append((stamp,world.copy()))
        if not len(world):return []
        arrays=[q for _,q in old]+[world]
        xy=np.vstack(arrays);labels=np.concatenate([np.full(len(q),i)for i,q in enumerate(arrays)])
        offset=len(xy)-len(world);out=[]
        for group in connected_components(xy,self.eps):
            group=np.asarray(group,int);fresh=group[group>=offset]-offset
            if len(group)<self.points or len(np.unique(labels[group]))<self.scans or len(fresh)<3:continue
            # Do not connect a long series of separate scene objects.
            if np.linalg.norm(np.ptp(xy[group],axis=0))>8.:continue
            out.append(ids[fresh])
        return out
