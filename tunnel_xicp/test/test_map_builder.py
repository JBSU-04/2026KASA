#!/usr/bin/env python3
import importlib.util
from pathlib import Path
import unittest
import numpy as np

spec=importlib.util.spec_from_file_location('builder',Path(__file__).resolve().parents[1]/'scripts/build_wall_map.py')
builder=importlib.util.module_from_spec(spec);spec.loader.exec_module(builder)

class MapBuilderTests(unittest.TestCase):
    def test_extrinsic_applied_exactly_once(self):
        p=builder.to_local_world(np.array([[2.,3.,.5]]),[100,200,0],[100,200,0],[1.1,0,1.33])
        np.testing.assert_allclose(p,[[3.1,3,1.83]])
    def test_rotated_sensor(self):
        p=builder.to_local_world(np.array([[2.,3.,.5]]),[100,200,np.pi/2],[100,200,0],[1.1,0,1.33])
        np.testing.assert_allclose(p,[[-3,3.1,1.83]])
    def test_stale_pose_rejected(self):
        poses=np.array([[0,0,0,0],[1,0,0,0]])
        self.assertIsNone(builder.interpolate_pose(poses,.5))
        poses=np.array([[0,0,0,0],[.02,1,0,0]])
        np.testing.assert_allclose(builder.interpolate_pose(poses,.01),[.5,0,0])
    def test_body_mask_retains_separate_wall(self):
        q=np.array([[10,10],[10,12],[10,15]])
        keep=builder.mask_obstacles(q,[dict(position_world=[10,10],length=.4,width=.4)])
        np.testing.assert_array_equal(keep,[False,True,True])
    def test_voxel_is_three_dimensional(self):
        p=builder.voxelize(np.array([[0,0,0],[.1,.1,.1],[0,0,1]]),.2)
        self.assertEqual(len(p),2)
    def test_normals_keep_wall_but_reject_floor(self):
        x,z=np.meshgrid(np.arange(0,2,.2),np.arange(0,1.6,.2))
        wall=np.column_stack([x.ravel(),np.ones(x.size)*5,z.ravel()])
        n=builder.wall_normals(wall)
        self.assertGreater(len(n),50)
        self.assertTrue(np.all(np.abs(n[:,4])>.99))
        floor=np.column_stack([x.ravel(),z.ravel(),np.zeros(x.size)])
        self.assertEqual(len(builder.wall_normals(floor)),0)
    def test_output_publishers_observation_only(self):
        source=(Path(__file__).resolve().parents[1]/'src/node.cpp').read_text()
        publishers=[line for line in source.splitlines() if '.advertise<' in line]
        self.assertEqual(len(publishers),4)
        self.assertTrue(all('/tunnel/xicp/' in line for line in publishers))
        self.assertNotIn('sendTransform',source)
        self.assertIn('r.state==tx::State::VALID&&r.yaw&&r.left&&r.right',source)
        self.assertIn('ros::Time::now()-stamp<=ros::Duration(max_age_)',source)

if __name__=='__main__':unittest.main()
