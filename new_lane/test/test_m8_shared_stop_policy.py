"""Shared classification must survive tunnel-specific geometry and waiting."""
import test_tunnel_driven_stop as fixture


def setup():
    f=fixture.DrivenStopTests();f.setUp();n,o=f.n,dict(f.o)
    n.shared_detector_policy=True;n._ever_dyn={}
    n._world_fast_motion=lambda *a: (_ for _ in ()).throw(AssertionError('legacy centroid classifier called'))
    n._far_static_match=lambda *a: (_ for _ in ()).throw(AssertionError('legacy far classifier called'))
    return n,o


def test_static_parent_box_velocity_does_not_override_common_static():
    n,o=setup();o.update(motion_state='static',confirmed=True,speed=3.,velocity_valid=True)
    result=n._dynamic_zone_stop(o,True,True,True,10.,tunnel=True)
    assert result[0] is False
    assert o['id'] in n._frame_static and o['id'] not in n._frame_dyn


def test_unknown_stationary_duration_alone_does_not_certify_static():
    n,o=setup();o.update(motion_state='unknown',speed=.1,stationary_sec=20.)
    n._dynamic_zone_stop(o,True,True,True,10.,tunnel=True)
    assert o['id'] not in n._frame_static


def test_static_and_dynamic_objects_keep_independent_simultaneous_roles():
    n,o=setup();o.update(motion_state='static',speed=.1,confirmed=True)
    n._dynamic_zone_stop(o,True,True,True,10.,tunnel=True)
    moving=dict(o,id=49,motion_state='dynamic',speed=2.,velocity_valid=True,velocity=[0.,2.],stationary_sec=0.)
    n._driven_predictive_conflict=lambda *a,**k:(True,'driven_swept_t1.00')
    result=n._dynamic_zone_stop(moving,False,False,True,10.,tunnel=True)
    assert result[0] is True
    assert 48 in n._frame_static and 49 in n._frame_dyn


def test_restatic_handoff_clears_only_this_dynamic_hold():
    n,o=setup();o.update(motion_state='static',m8_restatic=True,confirmed=True,speed=.1)
    n._tunnel_dynamic_seen={48:9.9,49:9.9};n._ever_dyn={48:9.9,49:9.9}
    n._dynamic_zone_stop(o,True,True,True,10.,tunnel=True)
    assert 48 not in n._tunnel_dynamic_seen and 49 in n._tunnel_dynamic_seen
    assert 48 in n._frame_static
