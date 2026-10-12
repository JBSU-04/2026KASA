"""Distance handoff, physical expansion and returned-beam-only contraction."""
import copy
import importlib.util
import math
from pathlib import Path
import sys
import unittest
from types import SimpleNamespace
from unittest.mock import patch
from collections import deque
import numpy as np
import test_static_occupancy as fixture
from obstacle_geometry import measured_geometry, projection, RayEvidence, pose_at, sensor_to_base_xy

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'yolo_test'))
import obstacle_perception_node as near
spec=importlib.util.spec_from_file_location('far_shape',ROOT/'new_lane/scripts/far_static_obstacle_detector.py')
far=importlib.util.module_from_spec(spec);spec.loader.exec_module(far)


class MeasuredGeometryTests(unittest.TestCase):
    def test_sparse_and_dense_same_outer_returns_same_size(self):
        corners=np.array([[10.,-1.,1.],[14.,-1.,1.],[14.,1.,1.],[10.,1.,1.]])
        a=measured_geometry(corners,0.);b=measured_geometry(np.repeat(corners,8,axis=0),0.)
        for key in ('length','width','position_world','footprint_world'):self.assertEqual(a[key],b[key])
    def test_large_vehicle_not_clipped(self):
        g=measured_geometry([[0.,0.],[12.,0.],[12.,3.],[0.,3.]],0.)
        self.assertEqual(g['length'],12.);self.assertEqual(g['width'],3.)
    def test_sensor_origin_conversion_once_and_z_unchanged(self):
        p=np.array([[10.,1.,-1.]])
        q=sensor_to_base_xy(p,'velodyne_ground',(1.1,0.,1.33))
        np.testing.assert_allclose(q,[[11.1,1.,-1.]])
        np.testing.assert_allclose(sensor_to_base_xy(q,'base_link',(1.1,0.,1.33)),q)
        np.testing.assert_allclose(p,[[10.,1.,-1.]])
    def test_unknown_frame_not_silently_transformed(self):
        with self.assertRaises(ValueError):sensor_to_base_xy(np.zeros((1,3)),'map',(1.1,0.,1.33))
    def test_pose_interpolation_at_30kmh_not_latest_pose(self):
        p=pose_at([(10.,(0.,0.,0.,0.)),(10.1,(.833,0.,0.,0.))],10.05)
        self.assertAlmostEqual(p[0],.4165)
    def test_pose_gap_and_stale_scan_rejected(self):
        self.assertIsNone(pose_at([(10.,(0.,0.,0.))],10.2))
        self.assertIsNone(pose_at([(10.,(0.,0.,0.)),(10.5,(1.,0.,0.))],10.2))
    def test_yaw_wrap_interpolates_short_way(self):
        p=pose_at([(10.,(0.,0.,math.radians(179))),(10.1,(0.,0.,math.radians(-179)))],10.05)
        self.assertAlmostEqual(abs(p[2]),math.pi)


class RayEvidenceTests(unittest.TestCase):
    def scan(self,ends,stamp=10.):return dict(stamp=stamp,frame='map',origin=[0.,0.,1.],endpoints=ends)
    def test_return_beyond_old_boundary_proves_cell_clear(self):
        self.assertTrue(RayEvidence(self.scan([[20.,0.,1.]]),10.).clear([[10.,0.,1.]]))
    def test_occluded_or_missing_beam_is_unknown(self):
        self.assertFalse(RayEvidence(self.scan([[5.,0.,1.]]),10.).clear([[10.,0.,1.]]))
        self.assertFalse(RayEvidence(self.scan([]),10.).clear([[10.,0.,1.]]))
    def test_different_height_cannot_clear_hidden_vehicle(self):
        self.assertFalse(RayEvidence(self.scan([[20.,0.,3.]]),10.).clear([[10.,0.,1.]]))
    def test_stale_or_wrong_frame_cannot_clear(self):
        self.assertFalse(RayEvidence(self.scan([[20.,0.,1.]],9.5),10.).clear([[10.,0.,1.]]))
        scan=self.scan([[20.,0.,1.]]);scan['frame']='base_link'
        self.assertFalse(RayEvidence(scan,10.).clear([[10.,0.,1.]]))
    def test_every_old_supported_cell_must_be_clear(self):
        self.assertFalse(RayEvidence(self.scan([[20.,0.,1.]]),10.).clear([[10.,0.,1.],[10.,1.,1.]]))


class StableParentTests(fixture.StaticOccupancyTests):
    # Base tests also exercise compatibility of old JSON producers.
    def measured(self,width=2.,length=1.,source='raw_temporal_far',stamp=10.):
        pts=[[15.-length/2,-width/2,1.],[15.+length/2,-width/2,1.],
             [15.+length/2,width/2,1.],[15.-length/2,width/2,1.]]
        g=measured_geometry(pts,0.)
        return self.obj(x=15.,y=0.,source=source,last_seen=stamp,support_points_world=pts,**g)
    def test_near_fragment_does_not_shrink_hidden_extent(self):
        for t in (9.8,9.9,10.):self.update([self.measured(stamp=t)],t)
        for t in (10.1,10.2,10.3,10.4,10.5):r=self.update([self.measured(width=.5,source='lidar_only',stamp=t)],t)
        self.assertAlmostEqual(r['obstacles'][0]['width'],2.)
    def test_new_hazard_immediate_but_planning_shape_requires_three_votes(self):
        for t in (9.8,9.9,10.):self.update([self.measured(width=1.,stamp=t)],t)
        r=self.update([self.measured(width=3.,source='lidar_only',stamp=10.1)],10.1)['obstacles'][0]
        self.assertAlmostEqual(r['width'],1.)
        b=projection(r,0.);self.assertAlmostEqual(b[3]-b[2],3.)
        for t in (10.2,10.3):r=self.update([self.measured(width=3.,source='lidar_only',stamp=t)],t)['obstacles'][0]
        self.assertAlmostEqual(r['width'],3.)
    def test_returned_beams_can_correct_far_excess_after_three_scans(self):
        o=self.measured();old=copy.deepcopy(o['support_points_world'])
        for t in (9.8,9.9,10.):self.update([self.measured(stamp=t)],t)
        for t in (10.1,10.2,10.3,10.4,10.5):
            n=self.measured(width=1.,source='lidar_only',stamp=t)
            rays=dict(stamp=t,frame='map',origin=[0.,0.,1.],endpoints=[[2*x,2*y,z] for x,y,z in old])
            r=self.cache.update(dict(mission_mode='static',obstacles=[n],collision_objects=[n],geometry_scan=rays),(0.,0.,0.),t)
        self.assertAlmostEqual(r['obstacles'][0]['width'],1.)
        self.assertEqual(r['obstacles'][0]['geometry_reason'],'three_returned_beams_clear_old_boundary')
    def test_duplicate_far_near_publications_do_not_confirm_shape(self):
        o=self.measured();self.update([o]);n=self.measured(source='lidar_only')
        for t in (10.,10.02,10.04):self.update([copy.deepcopy(n)],t)
        p=next(iter(self.cache.parents.values()));self.assertEqual(len(p['history']),1)
        self.assertEqual(p['geometry_state'],'provisional')
    def test_long_vehicle_partial_face_keeps_parent_id(self):
        first=self.update([self.measured(length=12.)])['obstacles'][0]
        n=self.obj(oid=20,x=10.5,y=0.,length=1.,width=.5,last_seen=10.1)
        r=self.update([n],10.1)['obstacles']
        self.assertEqual(len(r),1);self.assertEqual(r[0]['id'],first['id'])
        self.assertAlmostEqual(r[0]['length'],12.)
    def test_diagonal_vehicle_does_not_fill_empty_box_corners(self):
        from obstacle_geometry import rectangle,overlap
        points=rectangle(15.,0.,math.pi/4,6.,1.)
        g=measured_geometry(points,0.)
        o=self.obj(**g);r=self.update([o])['obstacles'][0]
        empty_corner=rectangle(12.6,2.4,0.,.1,.1)
        self.assertFalse(overlap(r['planning_footprint_world'],empty_corner))
        self.assertFalse(overlap(r['footprint_world'],empty_corner))

    def test_two_adjacent_separate_detections_do_not_merge(self):
        a=self.obj(oid=8,x=15.,y=0.,length=1.,width=.5)
        b=self.obj(oid=9,x=16.2,y=0.,length=1.,width=.5)
        r=self.update([a,b])['obstacles']
        self.assertEqual(len(r),2)
        self.assertNotEqual(r[0]['static_parent_id'],r[1]['static_parent_id'])

    def test_far_single_scan_extremum_not_seeding_planning_envelope(self):
        o=self.measured(width=4.)
        narrow=self.measured(width=2.)
        o['geometry_seed_footprint_world']=narrow['footprint_world']
        o['support_points_world']=narrow['support_points_world']
        r=self.update([o])['obstacles'][0]
        self.assertAlmostEqual(r['width'],2.)
        b=projection(r,0.);self.assertAlmostEqual(b[3]-b[2],4.)

    def test_distance_and_yaw_change_keeps_confirmed_geometry(self):
        for t in (9.8,9.9,10.):self.update([self.measured(length=4.,stamp=t)],t)
        first=next(iter(self.cache.parents));shapes=[]
        self.assertEqual(self.cache.parents[first]['geometry_state'],'confirmed')
        for j,(distance,yaw) in enumerate([(30.,0.),(20.,.2),(10.,-.3),(5.,.5)]):
            t=10.1+j*.1;o=self.measured(length=4.,source='lidar_only',stamp=t);o['footprint_yaw']=yaw
            r=self.update([o],t,(15.-distance,0.,yaw))['obstacles'][0]
            self.assertEqual(r['id'],first);shapes.append(r['planning_footprint_world'])
        self.assertTrue(all(shape==shapes[0] for shape in shapes))


class ActualDetectorHandoffTests(unittest.TestCase):
    def near_node(self,pose,stamp):
        n=near.ObstaclePerceptionNode.__new__(near.ObstaclePerceptionNode)
        n.scan_pose=pose;n.latest_ego_pose=pose;n.tracks=[];n.mission_zone='static'
        n._near_known_infra=lambda p:False;n._age_out_tracks=lambda t:None
        n.geometry_pose_history=deque([(stamp,pose+(0.,))]);n.pose_max_age=.15
        n.sensor_offset=(1.1,0.,1.33);n.geometry_z_offset=1.33
        n.path_world=np.array([[0.,0.],[100.,0.]])
        defaults=dict(ground_z=-1.765,use_ground_relative=False,init_confirm_elevation=.3,
            static_speed_threshold=.3,min_motion_baseline_sec=.2,dynamic_confirm_count=3,
            motion_speed_window_sec=1.,max_plausible_speed=20.,motion_z_threshold=3.,motion_z_var_floor=.05,
            kf_enabled=False,kf_q_accel=1.,kf_r_base=.1,kf_r_slope=.01,kf_vel_std_max=2.,ego_rate_window_sec=1.,
            motion_backend='legacy',track_timeout=1.,max_association_dist=1.5)
        for k,v in defaults.items():setattr(n,k,v)
        return n
    def test_kalman_enabled_keeps_id_across_second_real_callback(self):
        n=self.near_node((0.,0.,0.),10.);n.kf_enabled=True
        pts=np.array([[14.,-.5,-1.],[16.,-.5,-1.],[16.,.5,-1.],[14.,.5,-1.]])
        n.associate_and_update_tracks([(pts.mean(axis=0),pts,True)],10.)
        first=n.tracks[0].id
        n.geometry_pose_history.append((10.1,(0.,0.,0.,0.)))
        n.associate_and_update_tracks([(pts.mean(axis=0),pts,True)],10.1)
        self.assertEqual(len(n.tracks),1);self.assertEqual(n.tracks[0].id,first)

    def test_actual_near_and_far_use_identical_world_shape(self):
        world=np.array([[28.,-.95,-1.],[32.,-.95,-1.],[32.,.95,-1.],[28.,.95,-1.]])
        # Far needs connected returns and two distinct scans, with actual points
        # on all four boundaries, including sparse ring returns.
        grid=np.array([[x,y,-1.] for x in np.linspace(28.,32.,17) for y in np.linspace(-.95,.95,9)])
        n=self.near_node((25.,0.,.3),10.)
        c,s=math.cos(.3),math.sin(.3);pts=world.copy();pts[:,:2]=(world[:,:2]-[25.,0.])@np.array([[c,-s],[s,c]])
        n.associate_and_update_tracks([(pts.mean(axis=0),pts,True)],10.)
        track=n.tracks[0]
        f=far.FarStaticDetector.__new__(far.FarStaticDetector)
        f.frames=deque();f.min_points=5;f.min_scans=2;f.eps=.65;f.path_half=2.;f.max_extent=20.
        f.x_min=10.;f.x_max=50.;f.ground_z=-1.765
        f.tracked=[];f.id_timeout=1.4;f.match_dist=1.6;f.next_id=1;f.hold_sec=1.25
        for stamp,seq in [(9.9,1),(10.,2)]:
            q=np.c_[grid[:,:2],grid[:,2]+1.33,np.full(len(grid),seq),np.full(len(grid),stamp),grid[:,1]]
            f.frames.append((stamp,q))
        from scipy.spatial import cKDTree
        proposal=f._cluster(10.,0.,0.,0.,np.array([[1.,0.],[1.,0.]]),cKDTree([[0.,0.],[100.,0.]]),2,1.33)[0]
        extra=np.array([[27.6,-.95,.33,1.,10.,-.95]])
        f.frames[0]=(9.9,np.vstack([f.frames[0][1],extra]))
        noisy=f._cluster(10.,0.,0.,0.,np.array([[1.,0.],[1.,0.]]),cKDTree([[0.,0.],[100.,0.]]),2,1.33)[0]
        seed=projection(dict(noisy,footprint_world=noisy['geometry_seed_footprint_world']),0.)
        self.assertAlmostEqual(seed[1]-seed[0],4.)
        self.assertGreater(noisy['length'],4.)
        np.testing.assert_allclose(proposal['position_world'],track.position,atol=.001)
        self.assertAlmostEqual(proposal['length'],track.length,places=6)
        self.assertAlmostEqual(proposal['width'],track.width,places=6)
        for yaw in (0.,.6,-.4):
            a=projection(proposal,yaw);b=projection(dict(position_world=track.position.tolist(),footprint_world=track.footprint_world),yaw)
            np.testing.assert_allclose(a,b,atol=.001)


if __name__=='__main__':unittest.main()


class DriftRecenterTests(fixture.StaticOccupancyTests):
    """2026-10-07: 위치 밀림은 평행 이동, 실제로 커진 물체와 부분 관측은 기존 동작."""
    def measured_at(self,x,width=1.,length=2.,stamp=10.):
        pts=[[x-length/2,-width/2,1.],[x+length/2,-width/2,1.],[x+length/2,width/2,1.],[x-length/2,width/2,1.]]
        g=measured_geometry(pts,0.)
        return self.obj(x=x,y=0.,source='lidar_only',last_seen=stamp,support_points_world=pts,**g)
    def test_drifted_observation_translates_box_without_growth(self):
        for t in (9.8,9.9,10.):self.update([self.measured_at(15.,stamp=t)],t)
        for t in (10.1,10.2,10.3,10.4):r=self.update([self.measured_at(15.8,stamp=t)],t)['obstacles'][0]
        self.assertLess(r['length'],2.3)
        b=projection(r,0.)
        self.assertGreater(b[1],16.6)
    def test_really_larger_object_still_grows(self):
        for t in (9.8,9.9,10.):self.update([self.measured_at(15.,stamp=t)],t)
        for t in (10.1,10.2,10.3,10.4):r=self.update([self.measured_at(15.5,length=3.,stamp=t)],t)['obstacles'][0]
        self.assertGreater(r['length'],2.9)
    def test_disabled_keeps_previous_behavior(self):
        self.cache.drift_recenter=False
        for t in (9.8,9.9,10.):self.update([self.measured_at(15.,stamp=t)],t)
        for t in (10.1,10.2,10.3,10.4):r=self.update([self.measured_at(15.8,stamp=t)],t)['obstacles'][0]
        self.assertGreater(r['length'],2.6)


class TunnelSizeCapTests(fixture.StaticOccupancyTests):
    """2026-10-08: 터널에서 벽+턱 군집(21 m)이 누적 부모(25 m, RViz 큰 원)가 되지 않는다."""
    def measured(self,width=2.,length=1.,stamp=10.):
        pts=[[15.-length/2,-width/2,1.],[15.+length/2,-width/2,1.],
             [15.+length/2,width/2,1.],[15.-length/2,width/2,1.]]
        g=measured_geometry(pts,0.)
        return self.obj(x=15.,y=0.,source='raw_temporal_far',last_seen=stamp,support_points_world=pts,**g)
    def test_wall_sized_cluster_never_becomes_parent_in_tunnel(self):
        for t in (9.8,9.9,10.):r=self.update([self.measured(length=21.,width=2.8,stamp=t)],t,mode='tunnel')
        self.assertFalse(self.cache.parents)
    def test_vehicle_sized_object_still_retained_in_tunnel(self):
        for t in (9.8,9.9,10.):r=self.update([self.measured(length=4.5,width=1.9,stamp=t)],t,mode='tunnel')
        self.assertEqual(len(self.cache.parents),1)
    def test_long_vehicle_allowed_outside_tunnel(self):
        for t in (9.8,9.9,10.):self.update([self.measured(length=12.,width=2.5,stamp=t)],t)
        self.assertEqual(len(self.cache.parents),1)


class TunnelWallStripTests(unittest.TestCase):
    """2026-10-08: 군집 전에 지도(idx 별 벽 경계) 기준 벽/턱 띠 점을 지운다."""
    def setUp(self):
        n = near.ObstaclePerceptionNode.__new__(near.ObstaclePerceptionNode)
        xs = np.arange(0., 50., .1)
        n.path_world = np.c_[xs, np.zeros_like(xs)]
        n.path_idx = (8000 + np.arange(len(xs))).astype(int)
        n.path_lane = 3
        n.scan_pose = (0., 0., 0.)
        n.tunnel_wall_strip_enabled = True
        n.tunnel_wall_strip_m = .5
        n.tunnel_wall_profile = (np.array([7990., 8600.]), np.array([-3.8, -3.8]), np.array([6.9, 6.9]))
        self.n = n
    def test_wall_and_curb_points_removed_lane_points_kept(self):
        pts = np.array([[10., -3.5], [10., -3.0], [10., 0.], [10., 6.6], [10., 6.0]])
        with patch.object(near.rospy,'get_param',return_value=False):
            np.testing.assert_array_equal(self.n._tunnel_wall_strip_keep(pts), [False, True, True, False, True])
    def test_novel_wall_adjacent_points_cannot_be_erased_by_band_alone(self):
        pts=np.array([[10.,-3.5,.1],[10.,6.6,.1]])
        with patch.object(near.rospy,'get_param',return_value=True):
            self.assertTrue(self.n._tunnel_wall_strip_keep(pts).all())
    def test_verified_background_is_required_and_removed(self):
        from scipy.spatial import cKDTree
        pts=np.array([[10.,-3.5,.1],[10.,6.6,.1]])
        self.n.tunnel_background_tree=cKDTree(pts)
        self.n.tunnel_background_stamp=10.
        with patch.object(near.rospy,'get_param',return_value=True),patch.object(near.rospy.Time,'now',return_value=SimpleNamespace(to_sec=lambda:10.)):
            self.assertFalse(self.n._tunnel_wall_strip_keep(pts).any())
    def test_no_profile_or_other_lane_keeps_everything(self):
        self.n.path_lane = 0
        self.assertTrue(self.n._tunnel_wall_strip_keep(np.array([[10., -3.7]])).all())


class FarOnlyTunnelTests(fixture.StaticOccupancyTests):
    """2026-10-09: 원거리 단독 노면 제안(윗면 0.17 m)이 누적돼 정지를 일으키지 않는다."""
    def far(self,x,top,stamp):
        pts=[[x-.3,-1.,1.],[x+.3,-1.,1.],[x+.3,1.,1.],[x-.3,1.,1.]]
        g=measured_geometry(pts,0.)
        return self.obj(x=x,y=0.,source='raw_temporal_far',confirmed=True,top_elevation=top,
                        last_seen=stamp,support_points_world=pts,**g)
    def test_low_far_only_proposal_not_accumulated(self):
        for t in (9.8,9.9,10.):self.update([self.far(25.,.17,t)],t,mode='tunnel')
        self.assertFalse(self.cache.parents)
    def test_close_far_only_parent_kept_for_near_handoff(self):
        # 2026-10-11 M8 최종: GroundGrid 근거리는 터널 저상 물체를 6 m 에서야 본다(21:24:39 백).
        #   원거리 시야(13 m) 안쪽으로 들어온 원거리 단독 부모는 근거리 확인 없이도 유지한다.
        for t in (9.8,9.9,10.):self.update([self.far(12.,.4,t)],t,mode='tunnel')
        for t in (10.2,10.6,11.2):self.update([],t,mode='tunnel')
        self.assertEqual(len(self.cache.parents),1)
    def test_far_only_parent_dropped_when_far_loses_it(self):
        # 원거리가 볼 수 있는 거리(13 m 이상)에서 0.6 s 넘게 다시 안 보이면 원거리 오검출로 지운다.
        for t in (9.8,9.9,10.):self.update([self.far(25.,.4,t)],t,mode='tunnel')
        for t in (10.3,10.6,11.2):self.update([],t,mode='tunnel')
        self.assertFalse(self.cache.parents)
