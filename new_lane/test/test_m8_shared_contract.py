from m8_object_policy import detector_profile
from test_static_occupancy import StaticOccupancyTests
from obstacle_geometry import measured_geometry
import numpy as np


def test_m8_uses_shared_height_and_shape_filters_without_touching_other_missions():
    profiles=dict(static=dict(min_obstacle_elevation=.12,low_obstacle_max_length=4.,roi_y_max=8.),
                  dynamic=dict(min_obstacle_elevation=.12,roi_y_max=8.),
                  tunnel=dict(min_obstacle_elevation=.08,low_obstacle_max_length=100.,roi_y_max=6.))
    tunnel=detector_profile(profiles,'tunnel')
    assert tunnel==dict(min_obstacle_elevation=.12,low_obstacle_max_length=4.,roi_y_max=8.)
    assert detector_profile(profiles,'dynamic')==profiles['dynamic']
    assert detector_profile(profiles,'static_preview')==profiles['static']
    assert detector_profile(profiles,'tunnel',False)==profiles['tunnel']


def test_unknown_map_proposal_cannot_become_static_parent_by_elapsed_time():
    fixture=StaticOccupancyTests();fixture.setUp();fixture.cache.shared_detector_policy=True
    for t in (9.8,9.9,10.):
        fixture.update([fixture.obj(stationary_sec=20.,source='raw_temporal_far',last_seen=t)],t,mode='tunnel')
    assert not fixture.cache.parents


def test_shared_confirmed_static_surface_still_enters_avoidance():
    fixture=StaticOccupancyTests();fixture.setUp();fixture.cache.shared_detector_policy=True
    points=np.array([[15.,-.3,.2],[15.,.3,.2],[15.4,-.3,.2],[15.4,.3,.2]])
    for t in (9.8,9.9,10.,10.1):
        obj=fixture.obj(motion_state='static',last_seen=t,support_points_world=points.tolist(),
                        **measured_geometry(points,0.))
        result=fixture.update([obj],t,mode='tunnel')
    assert fixture.cache.parents and result['obstacles']
