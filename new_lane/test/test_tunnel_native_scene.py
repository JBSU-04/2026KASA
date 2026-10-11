import sys,io,math
from pathlib import Path
sys.path.insert(0,str(Path(__file__).parents[1]/'scripts'))
import numpy as np
from tunnel_native_scene import packet_from_detector,scene_dict,final_root
from tunnel_scene_policy import tag_reference_corridor,preslow
from tunnel_map_boundary import physical_profile,common_profile
from occupancy_avoidance.msg import TunnelScene

def make(state='unknown',confirmed=False,points=None):
    points=points or [[302500.123456789,4124000.125,.05],[302500.3,4124000.125,.06],[302500.4,4124000.125,.07]]
    o=dict(id=14,motion_state=state,confirmed=confirmed,last_seen=10.,top_elevation=.05,position_world=points[0][:2],support_points_world=points,in_trigger_corridor=True)
    m=packet_from_detector(dict(collision_objects=[o]),dict(collision_objects=[o]),{},(302500.,4124000.,0.),10.,42,7)
    return m

def test_uncertain_not_forced_confirmed_or_given_height():
    m=make();assert m.objects[0].motion_state==0 and not m.objects[0].confirmed
    assert not any(m.fixed);s=scene_dict(m);assert not s['collision_cells_world']
    o=final_root(s)['collision_objects'][0];assert not o['confirmed_obstacle'];assert o['top_elevation']==.05

def test_world_double_precision_survives_wire():
    m=make('static',True);buf=io.BytesIO();m.serialize(buf);out=TunnelScene();out.deserialize(buf.getvalue())
    assert out.cells[0].x==302500.123456789;assert out.scene_id==42 and out.coordinate_version==7
    assert scene_dict(out)['obstacles'][0]['id']==14

def test_actual_disconnected_surfaces_do_not_fill_box_or_s_gap():
    pts=[[302510.,4123998.,.2],[302510.1,4123998.,.2],[302510.,4124002.,.2],[302510.1,4124002.,.2]]
    m=make('static',True,pts);s=scene_dict(m);assert len(s['collision_cells_world'])==4
    r=tag_reference_corridor(final_root(s),[[302500.,4124000.],[302530.,4124000.]],1.3)
    assert not r['collision_objects'][0]['in_trigger_corridor']

def test_dynamic_and_static_both_present_candidates_separate():
    m=make('static',True);d=make('dynamic',True);d.objects[0].id=15
    for i in range(len(d.owner_ids)):d.owner_ids[i]=15
    m.objects.extend(d.objects);m.cells.extend(d.cells);m.owner_ids.extend(d.owner_ids);m.fixed.extend(d.fixed);m.seen.extend(d.seen);m.observed.extend(d.observed)
    s=scene_dict(m);assert {o['motion_state'] for o in s['obstacles']}=={'static','dynamic'}
    assert s['obstacles'][1]['dynamic_confirmed'];assert len(s['voxels'])==len(m.cells)

def test_empty_native_scene_is_fresh_but_expiry_is_not_free():
    m=packet_from_detector(dict(collision_objects=[]),{}, {},(0.,0.,0.),10.,43)
    s=scene_dict(m);assert s['geometry_stamp']==10. and s['state']=='valid' and not s['obstacles']
    assert preslow(s,(0,0,0),None,10.,10.4)['state']=='expired'

def test_actual_support_excludes_unrelated_point_inside_search_box():
    m=make('static',True);assert [p.z for p in m.cells]==[.05,.06,.07]
    assert len(m.cells)==3 # metadata box is never sampled or filled

def test_cleared_parent_points_do_not_return_after_three_scan_shrink():
    o=dict(id=1000001,observed_track_id=14,motion_state='static',confirmed=True,static_occupancy=True,position_world=[0.,0.],last_seen=10.)
    parents={1000001:dict(yaw=0.,box=[0.,1.,-.5,.5],support=[[.5,0.,.2],[2.,0.,.2]],stamp=10.)}
    m=packet_from_detector(dict(collision_objects=[o]),dict(collision_objects=[]),parents,(0.,0.,0.),10.,43)
    assert [(p.x,p.y)for p in m.cells]==[(.5,0.)];assert m.observed==[False]

def test_two_drives_missing_wall_stays_unknown():
    a=[[0,-3.,5.],[10,-3.,5.],[20,0.,0.]];b=[[0,-3.1,5.],[10,0.,5.3],[20,0.,0.]]
    p=common_profile([a,b]);assert p[0][1]==-3.05;assert p[1][1:]==[0.,0.];assert p[2][1:]==[0.,0.]

def test_physical_boundary_no_inset_and_open_entry_no_extension():
    road=np.c_[np.arange(0,10,.1),np.zeros(100)]
    floor=np.array([[x,y,0.]for x in np.arange(0,10,.2)for y in np.arange(-4,5,.2)])
    pts=np.array([[x,y,z]for x in np.arange(3.,7.,.1)for y in [-3.,4.]for z in [.15,.3,.5]])
    p=physical_profile(pts,floor,[0,0,0],road,0,90)
    assert p[0][1:]==[0.,0.] and p[-1][1:]==[0.,0.]
    wall=[r for r in p if r[1] and r[2]];assert wall
    assert abs(wall[0][1]+3)<.11 and abs(wall[0][2]-4)<.11


def test_native_source_preserves_real_beams_and_all_support_points():
    from tunnel_native_scene import detector_root
    points=[[12.,0.,.1],[12.01,0.,.1],[12.02,0.,.1]]
    o=dict(id=9,position_world=[12.,0.],motion_state='dynamic',confirmed=True,last_seen=10.,velocity=[1.,-2.],support_points_world=points)
    root=dict(mission_mode='tunnel',collision_objects=[o],geometry_scan=dict(origin=[1.,0.,1.33],endpoints=[[15.,0.,.1]]))
    m=packet_from_detector(root,root,{},(0.,0.,0.),10.,4,source_packet=True)
    out=detector_root(m);obj=out['collision_objects'][0]
    assert obj['support_points_world']==points and obj['velocity_world']==[1.,-2.]
    assert out['geometry_scan']['endpoints']==[[15.,0.,.1]]
    assert len(m.cells)==3

def test_held_surface_not_labeled_as_new_observation():
    m=make('static',True);m.objects[0].last_seen=9.8
    # Explicit source time remains old on its original support.
    root=__import__('tunnel_native_scene').detector_root(m);root['collision_objects'][0]['last_seen']=9.8
    out=packet_from_detector(root,root,{},(302500.,4124000.,0.),10.,44)
    assert not any(out.observed) and all(s==9.8 for s in out.seen)

def test_common_adapter_uses_same_dynamic_vector_and_near_cells():
    m=make('dynamic',True);m.objects[0].velocity_world.x=3.;m.objects[0].velocity_world.y=-1.
    out=final_root(scene_dict(m))['collision_objects'][0]
    assert out['perception_source']=='common_scene' and out['velocity_world']==[3.,-1.]
    assert len(out['occupied_cells_world'])==3


def test_coordinate_update_preserves_size_velocity_and_surface_motion():
    from types import SimpleNamespace
    from collections import deque
    from tunnel_native_scene import rebase_detector_tracks
    points=np.array([[4.,1.,.2],[4.1,1.,.2]])
    track=SimpleNamespace(position=np.array([4.,1.]),anchor_pos=np.array([4.,1.]),history=[(1.,np.array([4.,1.]))],kf=SimpleNamespace(x=np.array([4.,1.,2.,-1.])),footprint_world=[[4.,1.],[5.,1.],[5.,2.]],support_points_world=points.tolist(),motion_observation=SimpleNamespace(history=deque([(1.,points.copy(),0.)])))
    rebase_detector_tracks([track],.3,-.2)
    assert np.allclose(track.kf.x,[4.3,.8,2.,-1.])
    assert np.allclose(track.motion_observation.history[0][1],points+[.3,-.2,0.])
    assert np.allclose(track.history[0][1],track.position)
    assert np.allclose(np.ptp(track.footprint_world,axis=0),[1.,1.])

def retained_parent_with_real_support():
    from obstacle_info_merger import StaticOccupancy
    c=StaticOccupancy();c.native_clear_enabled=True
    o=dict(id=17,position_world=[6.,0.],position_ego=[6.,0.,-1.],length=.5,width=.5,source='lidar_only',confirmed=True,in_trigger_corridor=True,motion_state='static',observation_age=0.,last_seen=10.,support_points_world=[[6.,0.,.4]])
    for stamp in [9.8,9.9,10.]:
        o['last_seen']=stamp;c.update(dict(mission_mode='tunnel',collision_objects=[dict(o)],obstacles=[dict(o)]),(0.,0.,0.),stamp)
    return c,o

def test_three_distinct_returned_raw_scans_clear_missing_surface():
    c,o=retained_parent_with_real_support();assert c.parents
    for stamp in [10.1,10.2]:
        out=c.update(dict(mission_mode='tunnel',collision_objects=[],obstacles=[],geometry_scan=dict(stamp=stamp,frame='map',origin=[0.,0.,1.],endpoints=[[10.,0.,0.]])),(0.,0.,0.),stamp)
        assert c.parents
    c.update(dict(mission_mode='tunnel',collision_objects=[],obstacles=[],geometry_scan=dict(stamp=10.3,frame='map',origin=[0.,0.,1.],endpoints=[[10.,0.,0.]])),(0.,0.,0.),10.3)
    assert not c.parents
    # Re-publication of the old detector object cannot resurrect cleared occupancy.
    o['last_seen']=10.;o['observation_age']=.4
    out=c.update(dict(mission_mode='tunnel',collision_objects=[o],obstacles=[o]),(0.,0.,0.),10.4)
    assert not out['collision_objects']

def test_missing_or_occluded_beam_is_not_clearance():
    c,o=retained_parent_with_real_support()
    for stamp in [10.1,10.2,10.3,10.4]:
        c.update(dict(mission_mode='tunnel',collision_objects=[],obstacles=[],geometry_scan=dict(stamp=stamp,frame='map',origin=[0.,0.,1.],endpoints=[[4.,0.,.6]])),(0.,0.,0.),stamp)
    assert c.parents

def test_same_ray_scan_does_not_count_three_times():
    c,o=retained_parent_with_real_support()
    for now in [10.1,10.15,10.2]:
        c.update(dict(mission_mode='tunnel',collision_objects=[],obstacles=[],geometry_scan=dict(stamp=10.1,frame='map',origin=[0.,0.,1.],endpoints=[[10.,0.,0.]])),(0.,0.,0.),now)
    assert c.parents
