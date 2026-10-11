"""Scan-confirmed release of preslow memory; no ROS master or controller."""
import importlib.util
import unittest
from pathlib import Path

spec = importlib.util.spec_from_file_location('candidate_release_detector',
    Path(__file__).parents[1]/'scripts/tunnel_wall_detector.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class CandidateRelease(unittest.TestCase):
    def setUp(self):
        self.node = module.TunnelWallDetector.__new__(module.TunnelWallDetector)
        self.node.map_raw_active = True
        self.node.kick_memory_merge_m = .4
        self.node.kick_memory = [[10.,0.,0.,2,True,{'id':1}]]
        self.node.bg_observations = [dict(state='background',position_world=[10.,0.])]
        self.node.bg_scan_stamp = 1.

    def test_duplicate_scan_does_not_release(self):
        self.node.clear_background_memory()
        self.node.clear_background_memory()
        self.assertEqual(self.node.kick_memory[0][5]['background_clear_hits'],1)

    def test_three_independent_background_scans_release(self):
        for stamp in [1.,1.1]:
            self.node.bg_scan_stamp=stamp
            self.node.clear_background_memory()
            self.assertEqual(len(self.node.kick_memory),1)
        self.node.bg_scan_stamp=1.2
        self.node.clear_background_memory()
        self.assertEqual(self.node.kick_memory,[])

    def test_unobserved_and_mixed_foreground_keep_memory(self):
        for state in ['unknown','obstacle_candidate']:
            self.node.bg_observations.append(dict(state=state,position_world=[10.,0.]))
            self.node.clear_background_memory()
            self.assertEqual(len(self.node.kick_memory),1)
            self.assertEqual(self.node.kick_memory[0][5]['background_clear_hits'],0)

    def test_legacy_policy_unchanged(self):
        self.node.map_raw_active=False
        self.node.clear_background_memory()
        self.assertEqual(self.node.kick_memory,[])


if __name__=='__main__': unittest.main()
