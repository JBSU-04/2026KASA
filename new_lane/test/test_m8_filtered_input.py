import sys,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
import numpy as np
from m8_filtered_input import WallSurface,preslow_speed_cap,safe_static_ids

class FilteredContract(unittest.TestCase):
    def test_speed_curve(self):
        self.assertEqual(preslow_speed_cap(26),9999.)
        self.assertEqual(preslow_speed_cap(25),6.)
        self.assertEqual(preslow_speed_cap(15),5.)
        self.assertEqual(preslow_speed_cap(5),4.)
        self.assertEqual(preslow_speed_cap(1),4.)
        self.assertEqual(preslow_speed_cap(float('nan')),9999.)
    def wall(self):
        return WallSurface(np.c_[np.arange(100)*.1,np.zeros(100)],[[0,-3,4],[99,-3,4]])
    def test_wall_surface_not_distance_band(self):
        x=np.linspace(.2,3.8,20);wall=np.c_[x,np.full(20,-3.),np.zeros(20),np.ones(20)]
        object_=np.array([[2.,-2.68,0.,.5],[2.1,-2.68,0.,.5],[2.2,-3,0.,.12]])
        keep=self.wall().keep(np.vstack([wall,object_]),[0,0,0],(0,0))
        self.assertFalse(keep[:20].any());self.assertTrue(keep[20:].all())
    def test_unknown_wall_is_not_extended(self):
        w=WallSurface(np.c_[np.arange(100)*.1,np.zeros(100)],[[0,0,0],[50,-3,4],[99,0,0]])
        p=np.c_[np.linspace(.2,3.8,20),np.full(20,-3.),np.zeros(20),np.ones(20)]
        self.assertTrue(w.keep(p,[0,0,0],(0,0)).all())
    def obj(self,motion='static'):
        return {'id':7,'motion_state':motion,'static_certified':True,'support_points_world':[[5,2],[5.2,2.1],[5.1,2.2]]}
    def test_release_only_this_static_owner(self):
        path=np.c_[np.linspace(0,10,101),np.zeros(101)]
        root={'collision_objects':[self.obj(),dict(self.obj('dynamic'),id=8)]}
        self.assertEqual(safe_static_ids(root,path,[0,0,0]),{7})
    def test_collision_or_short_prefix_does_not_release(self):
        o=self.obj();o['support_points_world']=[[5,0],[5,.2],[5.1,.1]]
        self.assertFalse(safe_static_ids({'collision_objects':[o]},[[0,0],[10,0]],[0,0,0]))
        self.assertFalse(safe_static_ids({'collision_objects':[self.obj()]},[[0,0],[3,0]],[0,0,0]))
    def test_original_stamp_xyz_height_preserved(self):
        from std_msgs.msg import Header
        from m8_filtered_input import filtered_cloud,cloud_rows
        import rospy
        h=Header(stamp=rospy.Time(100,23),frame_id='velodyne_ground')
        p=np.array([[1.,2.,-1.4,.12],[10,3,0,1.7]])
        msg=filtered_cloud(h,p);np.testing.assert_allclose(cloud_rows(msg),p,atol=1e-7)
        self.assertEqual(msg.header.stamp,h.stamp)
        self.assertEqual(cloud_rows(filtered_cloud(h,np.empty((0,4)))).shape,(0,4))

if __name__=='__main__':unittest.main()
