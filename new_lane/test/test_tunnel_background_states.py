"""Background uncertainty must not refresh speed kick; no ROS master required."""
import unittest
from unittest.mock import Mock, patch
from types import SimpleNamespace
import numpy as np
import rospy
from test_tunnel_regional_ground import wall


class BackgroundStatesTests(unittest.TestCase):
    def setUp(self):
        self.n = wall.TunnelWallDetector.__new__(wall.TunnelWallDetector)
        for k,v in dict(kick_min_x=3.,sensor_x=1.1,kick_range=35.,kick_detect_range=35.,
                        kick_ground_z=-1.75,bg_min_h=.25,bg_max_h=2.5,bg_margin_h=.3,
                        bg_cell_m=1.,bg_cell_min_points=4,kick_min_height=.2,
                        kick_max_height=2.2,route=None,wprof=None,bg_direct_walls=[],
                        kick_memory=[],kick_memory_merge_m=1.,kick_memory_min_hits=2,
                        kick_confirm_frames=2,kick_hits=0,filtered_shift=0.).items():
            setattr(self.n,k,v)
        self.n.bg_lateral_ok = lambda *a, **k: True
        self.n.bgmap = Mock()
        self.n.bgmap.novel.side_effect = lambda x,y,h,m: (np.asarray(h) > .9, np.ones(len(h),bool))
        self.now = patch.object(wall.rospy.Time,'now',return_value=rospy.Time.from_sec(10.))
        self.now.start();self.addCleanup(self.now.stop)

    def scan(self,x=20.,y=7.3,h=1.2,old=.5,compatible=False):
        q=np.array([[x+i*.02,y,h] for i in range(6)])
        with patch.object(wall,'regional_ground_points',return_value=(
                q,np.full(6,compatible),np.full(6,old))):
            return self.n.bg_candidates(np.empty((0,3)),0.,0.,1.,0.)

    def test_far_height_disagreement_stays_unverified(self):
        self.assertEqual(self.scan(),[])
        self.assertEqual(self.n.bg_observations[0]['state'],'unverified')
        self.assertEqual(self.n.bg_observations[0]['hits'],0)

    def test_compatible_map_novel_retains_original_detection(self):
        self.assertEqual(len(self.scan(compatible=True,old=1.2)),1)

    def test_legacy_novel_protects_height_disagreement_object(self):
        self.assertEqual(len(self.scan(old=1.2)),1)
        self.assertEqual(self.n.bg_observations[0]['reason'],'legacy_map_novel')

    def test_near_uncertainty_retains_conservative_proposal(self):
        self.assertEqual(len(self.scan(x=5.)),1)
        self.assertEqual(self.n.bg_observations[0]['reason'],'near_fallback')

    def test_current_wall_and_map_reject_near_background(self):
        self.n.bg_direct_walls=[(10.,(0.,7.3,1.,0.))]
        self.assertEqual(self.scan(x=5.),[])
        self.assertEqual(self.n.bg_observations[0]['state'],'background')

    def test_stale_or_predicted_wall_cannot_reject_near_candidate(self):
        for walls in [[],[(9.,(0.,7.3,1.,0.))]]:
            self.n.bg_direct_walls=walls
            self.assertEqual(len(self.scan(x=5.)),1)

    def test_real_object_not_erased_even_if_adjacent_to_wall(self):
        self.n.bg_direct_walls=[(10.,(0.,7.3,1.,0.))]
        self.assertEqual(len(self.scan(x=5.,old=1.2)),1)

    def test_near_fallback_at_profile_wall_is_rejected_without_live_wall(self):
        # 2026-10-08(11:34:37): 실시간 벽선이 없을 때 벽/턱 점이 근거리 후보 -> 빈 터널 선감속.
        self.n.bg_lateral_ok = lambda *a, **k: (k.get('margin') or 0.) >= 0.
        self.assertEqual(self.scan(x=5.),[])

    def test_map_novel_object_near_profile_wall_still_kicks(self):
        self.n.bg_lateral_ok = lambda *a, **k: (k.get('margin') or 0.) >= 0.
        self.assertEqual(len(self.scan(x=5.,old=1.2)),1)

    def test_near_fallback_keeps_measured_wall_inset(self):
        self.n.bg_direct_walls=[(10.,(0.,7.3,1.,0.))]
        self.assertEqual(self.scan(x=5.,y=7.),[])
        self.assertEqual(self.n.bg_observations[0]['state'],'unverified')
        self.assertEqual(len(self.scan(x=5.,y=5.)),1)

    def test_wall_from_different_scan_cannot_reject(self):
        self.n.bg_direct_walls=[(9.9,(0.,7.3,1.,0.))]
        self.n.bg_scan_stamp=10.
        self.assertEqual(len(self.scan(x=5.)),1)

    def test_unobserved_map_cannot_verify_background(self):
        self.n.bg_direct_walls=[(10.,(0.,7.3,1.,0.))]
        self.n.bgmap.novel.side_effect=lambda x,y,h,m:(np.zeros(len(h),bool),np.zeros(len(h),bool))
        self.assertEqual(self.scan(x=5.),[])
        self.assertEqual(self.n.bg_observations[0]['state'],'unverified')
        self.assertEqual(len(self.scan(x=5.,y=5.)),1)

    def test_valid_curb_requires_background_map_agreement(self):
        self.n.route=(np.array([[0.,0.],[40.,0.]]),np.array([7300.,7700.]),
                      np.array([[1.,0.],[1.,0.]]))
        self.n.wprof=(np.array([7300.,7700.]),np.array([-4.,-4.]),np.array([7.3,7.3]))
        self.assertEqual(self.scan(),[])
        self.assertEqual(self.n.bg_observations[0]['state'],'background')
        self.assertEqual(len(self.scan(old=1.2)),1)

    def test_point_filter_does_not_erase_mixed_foreground_cluster(self):
        self.n.bg_direct_walls=[(10.,(0.,7.3,1.,0.))]
        q=np.array([[5.+i*.02,7.3,1.2] for i in range(6)]+
                   [[5.+i*.02,7.,.35] for i in range(6)])
        with patch.object(wall,'regional_ground_points',return_value=(
                q,np.zeros(12,bool),np.r_[np.full(6,.5),np.full(6,1.2)])):
            centers=self.n.bg_candidates(np.empty((0,3)),0.,0.,1.,0.)
        self.assertEqual(len(centers),1)
        self.assertAlmostEqual(centers[0][1],7.)

    def test_verified_background_clears_memory_not_mixed_object(self):
        self.n.kick_memory=[[10.,7.,10.,5,True],[10.,7.,10.,5,False]]
        self.n.bg_observations=[dict(state='background',position_world=[10.,7.])]
        self.n.clear_background_memory()
        self.assertEqual(len(self.n.kick_memory),1);self.assertFalse(self.n.kick_memory[0][4])
        self.n.kick_memory=[[10.,7.,10.,5,True]]
        self.n.bg_observations.append(dict(state='obstacle_candidate',position_world=[10.,7.1]))
        self.n.clear_background_memory();self.assertEqual(len(self.n.kick_memory),1)

    def test_kick_confirmation_frames_unchanged(self):
        self.n.kick_pub=Mock();self.n.kick_dist_pub=Mock()
        with patch.object(wall.rospy,'loginfo_throttle'):
            self.n.publish_kick(True,5.);self.assertFalse(self.n.last_kick_confirmed)
            self.n.publish_kick(True,5.);self.assertTrue(self.n.last_kick_confirmed)

    def test_raw_pipeline_unknown_does_not_increment_memory(self):
        n=self.n
        for k,v in dict(kick_enabled=True,pose=(0.,0.,0.),pose_hist=[(10.,0.,0.,0.)],
                        kick_lines_state=None,bg_motion_enabled=False,bg_memory_sec=1.5,
                        kick_memory_sec=.3).items():setattr(n,k,v)
        n.is_tunnel_zone=lambda:True;n.bgmap.covers.return_value=True
        n.kick_pub=Mock();n.kick_dist_pub=Mock();n.bg_candidates_debug_pub=Mock()
        n.publish_bg_markers=lambda stamp:None
        msg=SimpleNamespace(header=SimpleNamespace(frame_id='velodyne_ground',stamp=rospy.Time.from_sec(10.)))
        q=np.array([[20.+i*.02,7.3,1.2] for i in range(6)])
        with patch.object(wall,'pointcloud2_to_xyz_array',return_value=np.empty((0,3))), \
             patch.object(wall,'regional_ground_points',return_value=(q,np.zeros(6,bool),np.full(6,.5))), \
             patch.object(wall.rospy,'loginfo_throttle'):
            for _ in range(4):n.raw_kick_cb(msg)
        self.assertEqual(n.kick_memory,[]);self.assertFalse(n.last_kick_confirmed)
        self.assertEqual(n.bg_debug_rows[0]['state'],'unverified')
        # New real evidence starts at hit 1; uncertainty never contributes hits.
        # Existing two memory hits + two kick confirmations require three scans.
        with patch.object(wall,'pointcloud2_to_xyz_array',return_value=np.empty((0,3))), \
             patch.object(wall,'regional_ground_points',return_value=(q,np.zeros(6,bool),np.full(6,1.2))), \
             patch.object(wall.rospy,'loginfo_throttle'):
            for hits,confirmed in [(1,False),(2,False),(3,True)]:
                n.raw_kick_cb(msg)
                self.assertEqual(n.kick_memory[0][3],hits)
                self.assertEqual(n.last_kick_confirmed,confirmed)
        self.assertEqual(n.bg_debug_rows[0]['state'],'confirmed')

    def test_debug_retains_id_and_reason_for_remembered_trigger(self):
        n=self.n
        n.bg_observations=[];n.bg_debug_pose=(0.,0.,1.,0.)
        n.bg_candidates_debug_pub=Mock();n.last_kick_confirmed=True
        metadata=dict(id='stable',state='obstacle_candidate',reason='legacy_map_novel',
                      position_world=[5.,0.],distance_m=5.,hits=0,height_compatible=False)
        n.kick_memory=[[5.,0.,9.9,4,True,metadata]]
        with patch.object(wall.rospy,'loginfo_throttle'):
            n.publish_bg_candidate_debug(rospy.Time.from_sec(10.),5.)
        row=n.bg_debug_rows[0]
        self.assertEqual(row['id'],'stable');self.assertEqual(row['state'],'confirmed')
        self.assertTrue(row['remembered']);self.assertAlmostEqual(row['observation_age'],.1)


if __name__=='__main__':unittest.main()


class KickStaticExclusionTests(unittest.TestCase):
    """2026-10-08: 정적으로 인지된 물체(일부가 코리도에 걸쳐도)는 선감속 대상에서 뺀다."""
    def setUp(self):
        import numpy as np
        self.n = wall.TunnelWallDetector.__new__(wall.TunnelWallDetector)
        poly = np.array([[10., 2.], [12., 2.], [12., 4.], [10., 4.]])
        self.n.kick_static_footprints = [(poly, poly.min(axis=0), poly.max(axis=0))]
        self.n.kick_static_margin = .6
        self.n.kick_static_t = 100.
        p = patch.object(wall.rospy.Time, 'now', return_value=SimpleNamespace(to_sec=lambda: 100.2))
        p.start(); self.addCleanup(p.stop)

    def test_point_inside_or_near_static_object_is_excluded(self):
        self.assertTrue(self.n.near_static_obstacle(11., 3.))
        self.assertTrue(self.n.near_static_obstacle(11., 1.5))   # 코리도 쪽으로 0.5 m 삐져나온 점

    def test_unrelated_point_still_kicks(self):
        self.assertFalse(self.n.near_static_obstacle(11., 0.))

    def test_stale_static_list_is_ignored(self):
        self.n.kick_static_t -= 5.
        self.assertFalse(self.n.near_static_obstacle(11., 3.))


class KickAvoidanceHandledTests(unittest.TestCase):
    """2026-10-08: 회피 경로가 피해 가는 장애물은 선감속하지 않는다."""
    def setUp(self):
        self.n = wall.TunnelWallDetector.__new__(wall.TunnelWallDetector)
        self.n.kick_avoid_path = np.array([[0., 3.], [20., 3.]])   # 왼쪽 3 m 로 회피 중
        self.n.kick_avoid_path_t = 100.
        self.n.kick_avoiding_t = 100.
        self.n.kick_avoid_clear_m = 1.25
        p = patch.object(wall.rospy.Time, 'now', return_value=SimpleNamespace(to_sec=lambda: 100.2))
        p.start(); self.addCleanup(p.stop)

    def test_obstacle_beside_avoidance_path_not_kicked(self):
        self.assertTrue(self.n.handled_by_avoidance(10., 0.))      # 경로에서 3 m: 회피가 처리
    def test_point_on_avoidance_path_still_kicks(self):
        self.assertFalse(self.n.handled_by_avoidance(10., 2.5))    # 회피 경로 위로 들어온 물체
    def test_not_avoiding_means_no_exclusion(self):
        self.n.kick_avoiding_t = 90.
        self.assertFalse(self.n.handled_by_avoidance(10., 0.))
