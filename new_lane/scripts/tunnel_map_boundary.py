"""Physical inside wall/curb profiles. No vehicle inset or missing-wall filling."""
import numpy as np
from scipy.spatial import cKDTree

def physical_profile(points,floor,origin,road,first,last,stride=10):
    points=np.asarray(points,float).reshape((-1,3));floor=np.asarray(floor,float).reshape((-1,3));road=np.asarray(road,float)
    if len(road)<2 or not len(points) or len(floor)<3:raise ValueError('missing observed map, floor or reference')
    # road rows are world x/y; map points are local xyz, retain float64 offsets.
    path=road[:,:2]-np.asarray(origin)[:2]
    tangents=np.diff(path,axis=0);tangents=np.vstack([tangents,tangents[-1]]);tangents/=np.maximum(np.linalg.norm(tangents,axis=1)[:,None],1e-9)
    distance,idx=cKDTree(path).query(points[:,:2],k=1)
    floor_distance,fi=cKDTree(floor[:,:2]).query(points[:,:2],k=3,distance_upper_bound=1.2)
    valid=np.isfinite(floor_distance).all(axis=1)
    height=np.full(len(points),np.nan)
    if valid.any():
        z=floor[fi[valid],2];good=np.ptp(z,axis=1)<.12;select=np.flatnonzero(valid)[good];height[select]=points[select,2]-np.median(z[good],axis=1)
    delta=points[:,:2]-path[idx];lat=-tangents[idx,1]*delta[:,0]+tangents[idx,0]*delta[:,1]
    candidate=(height>=.08)&(height<=2.)&(distance<12)&(abs(lat)>=1.5)
    result=[]
    for j in range(max(0,first),min(last,len(path)-1)+1,stride):
        local=candidate&(abs(idx-j)<=stride//2)
        values=[]
        for side in [-1,1]:
            q=lat[local&(side*lat>0)]*side
            if len(q)<3:values.append(0.);continue
            bins,counts=np.unique(np.floor(q/.1).astype(int),return_counts=True)
            dense=bins[(counts>=3)&(counts>=.25*counts.max())]
            values.append(float(side*(dense.min()+.5)*.1) if len(dense) else 0.)
        result.append([j,*values])
    # A physical line needs support in a neighbouring measured slice. Never fill gaps.
    original=[r[:] for r in result]
    for k,r in enumerate(result):
        for side in (1,2):
            if not r[side]:continue
            neighbours=[original[t][side] for t in (k-1,k+1) if 0<=t<len(original)]
            if not any(v and abs(v-r[side])<=.4 for v in neighbours):r[side]=0.
    return result

def common_profile(profiles,tolerance=.12):
    if len(profiles)!=2 or [r[0] for r in profiles[0]]!=[r[0]for r in profiles[1]]:raise ValueError('two aligned profiles required')
    result=[]
    for a,b in zip(*profiles):
        r=[a[0]]
        for side in (1,2):
            # Missing or inconsistent observation stays unknown; no extrapolation.
            r.append(float((a[side]+b[side])/2) if a[side]*b[side]>0 and abs(a[side]-b[side])<=tolerance else 0.)
        result.append(r)
    return result
