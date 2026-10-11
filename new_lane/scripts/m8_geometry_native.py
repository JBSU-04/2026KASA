"""Compiled M8 numeric kernels; original Python policies remain available."""
import ctypes as C
import os
from pathlib import Path
import numpy as np
_lib=None
for prefix in [Path(__file__).resolve().parents[3]/'devel',Path(__file__).resolve().parents[3],*[Path(p)for p in os.environ.get('CMAKE_PREFIX_PATH','').split(':')if p]]:
    try:_lib=C.CDLL(str(prefix/'lib/libm8_geometry.so'));break
    except OSError:pass
if _lib is not None:
    D=C.POINTER(C.c_double)
    for name,args,result in [('m8_rays_create',[D,D,C.c_int],C.c_void_p),('m8_rays_destroy',[C.c_void_p],None),('m8_rays_clear',[C.c_void_p,D,C.c_int],C.c_int),('m8_route_create',[D,C.c_int],C.c_void_p),('m8_route_destroy',[C.c_void_p],None),('m8_route_distance',[C.c_void_p,D,C.c_int],C.c_double)]:
        f=getattr(_lib,name);f.argtypes=args;f.restype=result
if _lib is not None:
    _lib.m8_components.argtypes=[D,C.c_int,C.c_double,C.POINTER(C.c_int)];_lib.m8_components.restype=C.c_int
    _lib.m8_sweep.argtypes=[D,C.c_int,D,C.c_int,C.c_double,C.c_double,C.c_double,C.c_double];_lib.m8_sweep.restype=C.c_int

def components(points,eps):
    p=_array(points,2);out=np.empty(len(p),dtype=np.int32)
    if not _lib.m8_components(_ptr(p),len(p),eps,out.ctypes.data_as(C.POINTER(C.c_int))):raise ValueError('native connectivity failed')
    return out

def sweep(cells,path,ego,yaw,horizon):
    p=_array(cells,2);xy=_array(path,2)
    value=_lib.m8_sweep(_ptr(p),len(p),_ptr(xy),len(xy),ego[0],ego[1],yaw,horizon)
    return None if value<0 else bool(value)

enabled=True
def available():return enabled and _lib is not None
def _array(a,cols):return np.ascontiguousarray(a,dtype=np.float64).reshape((-1,cols))
def _ptr(a):return a.ctypes.data_as(C.POINTER(C.c_double))
class Rays:
    def __init__(self,origin,ends):
        o=_array(origin,3);p=_array(ends,3);self.handle=_lib.m8_rays_create(_ptr(o),_ptr(p),len(p))
        if not self.handle:raise ValueError('invalid native rays')
    def clear(self,points):
        p=_array(points,3)
        return bool(_lib.m8_rays_clear(self.handle,_ptr(p),len(p)))
    def __del__(self):
        if getattr(self,'handle',None):_lib.m8_rays_destroy(self.handle)
class Route:
    def __init__(self,xy):
        p=_array(xy,2);self.handle=_lib.m8_route_create(_ptr(p),len(p))
        if not self.handle:raise ValueError('invalid native route')
    def distance(self,points):
        p=_array(points,2)
        return _lib.m8_route_distance(self.handle,_ptr(p),len(p))
    def __del__(self):
        if getattr(self,'handle',None):_lib.m8_route_destroy(self.handle)
