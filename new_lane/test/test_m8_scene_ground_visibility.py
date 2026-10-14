# 2026-10-14(18:16 백): 장면 시각과 맞는 지면 스캔을 최근 몇 개 중에서 고른다(최신 1개만 보면 장면이 늦어 늘 비었다).
import os,sys,types
import numpy as np
sys.path.insert(0,os.path.join(os.path.dirname(__file__),'..','scripts'))
import obstacle_info_merger as M


class Msg:
    def __init__(self,stamp,pts):
        self.header=types.SimpleNamespace(frame_id='velodyne_ground',stamp=types.SimpleNamespace(to_sec=lambda:stamp))
        self.pts=pts


def node(monkeypatch):
    n=M.Merger.__new__(M.Merger);n.ground_packet=None
    import tunnel_map_geometry
    monkeypatch.setattr(tunnel_map_geometry,'fields_array',lambda msg,names:np.asarray(msg.pts,float))
    monkeypatch.setattr(M.rospy,'get_param',lambda k,d=None:d)
    return n


def test_scene_uses_ground_scan_with_matching_stamp(monkeypatch):
    n=node(monkeypatch)
    n.ground_cb(Msg(1.0,[[5.,0.,-1.3]]))
    n.ground_cb(Msg(1.1,[[9.,0.,-1.3]]))
    n.ground_cb(Msg(1.2,[[13.,0.,-1.3]]))
    vis=np.asarray(n.ground_visibility(1.0,(0.,0.,0.))).reshape(-1,3)
    assert len(vis)==1 and vis[0][2]==1.0 and vis[0][0]==int(np.floor((5.+1.1)/.5))


def test_no_matching_ground_scan_gives_no_visibility(monkeypatch):
    n=node(monkeypatch)
    n.ground_cb(Msg(1.0,[[5.,0.,-1.3]]))
    assert n.ground_visibility(1.5,(0.,0.,0.))==[]
