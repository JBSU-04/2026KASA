import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).parents[1]/'scripts'))
from tunnel_scene_policy import preslow,adapt_scene
import numpy as np

def scene(objects,stamp=10.):return dict(state='valid',scene_id=7,coordinate_version=2,geometry_stamp=stamp,obstacles=objects)
def obj(oid,x,y,state='static'):
 return dict(id=oid,center=[x,y],motion_state=state,occupied_cells_world=[[x,y],[x+.1,y],[x,y+.1]],footprint_world=[[x-.1,y-.1],[x+.2,y-.1],[x+.2,y+.2],[x-.1,y+.2]],last_seen=10.)
PATH=np.array([[x,0.]for x in np.arange(0.,41.,.5)])
def decide(s,path=PATH,stamp=10.):return preslow(s,(0.,0.,0.),path,stamp,10.)
def test_static_cleared_side_owner_releases():
 d=decide(scene([obj(1,8,2.)]));assert not d['present'];assert d['objects'][0]['reason']=='accepted_path_body_clear'
def test_unconfirmed_side_structure_does_not_preslow():assert not decide(scene([obj(1,15,3,'unknown')]))['present']
def test_new_dynamic_cannot_be_released_by_static_clear():
 crossing=obj(2,16,3,'dynamic');crossing.update(dynamic_confirmed=True,velocity_world=[0.,-1.])
 d=decide(scene([obj(1,5,3),crossing]));assert d['present'];assert d['distance']==16
 assert not d['objects'][0]['active'] and d['objects'][1]['active']
def test_actual_body_risk_cannot_clear():assert decide(scene([obj(1,2.,.5)]))['present']
def test_twenty_meter_window():assert not decide(scene([obj(1,21.,0.)]))['present']
def test_passed_body_releases_dynamic():assert not decide(scene([obj(1,-2.,0.,'dynamic')]))['present']
def test_unknown_path_does_not_clear_static():assert decide(scene([obj(1,12.,2.)]),None)['present']
def test_expired_map_is_expired_not_free_space():
 d=decide(scene([],9.));assert d['state']=='expired';assert d['distance']==-1
 root=adapt_scene(scene([obj(1,12,0)],9.),(0,0,0),10.);assert root['scene_state']=='expired' and not root['obstacles']
def test_repeated_safe_frames_do_not_rearm():
 for _ in range(30):assert not decide(scene([obj(1,10,2)]))['present']
def test_disconnected_cells_keep_s_gap():
 o=obj(1,12,0);o['occupied_cells_world']=[[12,-2],[12,2],[14,2]]
 assert not decide(scene([o]))['present']
def test_adapter_keeps_identity_and_cells():
 o=obj(123,12,0);root=adapt_scene(scene([o]),(0,0,0),10.)
 out=root['obstacles'][0];assert out['id']==123 and out['scene_id']==7 and out['coordinate_version']==2
 assert out['occupied_cells_world']==o['occupied_cells_world'];assert out['position_ego']==[12.,0.,0.]
def test_short_path_cannot_release_unseen_far_object():
 assert decide(scene([obj(1,15,2)]),np.array([[0.,0.],[5.,0.]]))['present']
def test_single_noise_point_only_keeps_near_risk_evidence():
 o=obj(1,12,0,'unknown');o['observed_surface_count']=1;o['last_surface_count']=1
 d=decide(scene([o]));assert not d['present'];assert d['objects'][0]['reason']=='insufficient_points_near_risk_only'
def test_three_point_unconfirmed_owner_still_preslows():
 o=obj(1,19,.8,'unknown');o['observed_surface_count']=3;o['last_surface_count']=3
 assert decide(scene([o]))['present']
def test_reference_corridor_uses_cells_not_large_diagnostic_box():
 from tunnel_scene_policy import tag_reference_corridor
 o=obj(1,12,0);o['occupied_cells_world']=[[12,2.],[12,-2.]]
 root=adapt_scene(scene([o]),(0,0,0),10.)
 tag_reference_corridor(root,[[0,0],[30,0]],1.3)
 assert not root['collision_objects'][0]['in_trigger_corridor']
 assert root['collision_objects'][0]['occupied_cells_world']==o['occupied_cells_world']
def test_stationary_unknown_ground_is_not_certified_by_motion_history():
 o=obj(1,12,.8,'static');o['confirmed_obstacle']=False
 assert decide(scene([o]))['present']
 root=adapt_scene(scene([o]),(0,0,0),10.)
 assert not root['obstacles'][0]['static_certified']
