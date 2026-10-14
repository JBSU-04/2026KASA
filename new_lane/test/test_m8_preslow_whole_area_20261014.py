"""2026-10-14(18:56, 18:57 백) 선서행 = 회피 가능 영역(벽선 안쪽 0.6 m) 안 확정 물체 전부. ROS master 없음."""
import json
import numpy as np
import pytest
from test_m8_preslow_prewall_20261013 import n as base,cloud,state  # noqa: F401


@pytest.fixture
def n(base):
    base.preslow_whole_area=True;base.preslow_area_margin=.6;base.object_scene_enabled=True
    return base


def test_wall_base_strip_never_preslows(n):
    # 앞쪽 오탐: 벽선(우 -3.6, 좌 7.3)에서 0.45 m 안쪽 벽 밑동·아치 조각.
    n.repaired_preslow_cb(cloud([[12.,-3.3,3],[15.,-3.2,3],[18.,6.9,3]]))
    assert n.preslow_cap_pub.values[-1]==9999.


def test_far_side_static_object_inside_area_preslows(n):
    # 좌측 벽 옆 정지 물체(횡 6.0, 벽선 1.3 m 안쪽): 경로를 막지 않아도 선서행.
    n.repaired_preslow_cb(cloud([[25.,6.,3],[25.2,6.1,3]]))
    s=state(n);assert s['present'] and n.preslow_cap_pub.values[-1]<7.


def test_approved_path_clearing_does_not_release_wall_side_object(n):
    target=dict(id=7,motion_state='static',confirmed=True,last_seen=10.,
                footprint_world=[[20.5,-2.5],[21.5,-2.5],[21.5,-1.6],[20.5,-1.6]])
    n.preslow_root=dict(stamp=10.,obstacles=[target]);n.preslow_scene=dict(stamp=10.,collision_objects=[target])
    n.preslow_approved=(np.c_[np.arange(0.,45.,.2),np.zeros(225)],10.,11,0)
    n.repaired_preslow_cb(cloud([[20.,-2.,3],[20.2,-1.9,3]]))
    assert state(n)['present'] and n.preslow_cap_pub.values[-1]<7.


def test_switch_off_keeps_previous_envelope(base):
    base.preslow_whole_area=False
    base.repaired_preslow_cb(cloud([[20.,6.6,3]]));assert base.preslow_cap_pub.values[-1]==9999.
