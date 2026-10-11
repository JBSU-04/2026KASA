#!/usr/bin/env python3
"""Lossless packaging of existing local-coordinate NPZ; no map learning/rebuild."""
import argparse,json,hashlib,struct
from pathlib import Path
import numpy as np

def export(source,target):
    with np.load(source,allow_pickle=False)as a:
        origin=np.asarray(a['origin'],dtype='<f8');p=np.asarray(a['points'],dtype='<f4');f=np.asarray(a['floor'],dtype='<f4')
        if origin.shape!=(3,) or p.ndim!=2 or f.ndim!=2 or p.shape[1]!=3 or f.shape[1]!=3 or not all(np.isfinite(x).all()for x in (origin,p,f)):raise ValueError('invalid map')
        with open(target,'wb')as out:out.write(b'TMAP3D1\0');out.write(origin.tobytes());out.write(struct.pack('<QQ',len(p),len(f)));out.write(p.tobytes());out.write(f.tobytes())
    Path(str(target)+'.json').write_text(json.dumps(dict(source=str(source),source_sha256=hashlib.sha256(Path(source).read_bytes()).hexdigest(),points=len(p),floor_points=len(f),origin=origin.tolist(),format='TMAP3D1',geometry_changed=False),indent=2))
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('source');p.add_argument('target');a=p.parse_args();export(a.source,a.target)
