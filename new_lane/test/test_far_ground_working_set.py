import numpy as np
from m8_filtered_input import far_ground_working_set
from obstacle_geometry import regional_plane_heights

def test_crop_preserves_every_detection_point_and_ground_fit():
    rng=np.random.RandomState(42)
    floor=np.array([[x,y,-1.765+.012*x-.008*y+rng.uniform(-.004,.004)] for x in np.arange(-10,90,.25) for y in np.arange(-25,25,.25)])
    obstacles=np.array([[15,.2,-1.3],[39.9,-7.4,-1.5],[25,6,-1.6]])
    p=np.vstack([floor,obstacles]);h,v,_=regional_plane_heights(p,-1.765)
    crop=far_ground_working_set(p,12,40,7.5,[3,35]);hc,vc,_=regional_plane_heights(crop,-1.765)
    roi=lambda a:(a[:,0]>=12)&(a[:,0]<=40)&(abs(a[:,1])<=7.5)
    np.testing.assert_array_equal(p[roi(p)],crop[roi(crop)])
    np.testing.assert_array_equal(v[roi(p)],vc[roi(crop)])
    np.testing.assert_allclose(h[roi(p)],hc[roi(crop)],atol=1e-12,equal_nan=True)
    assert len(crop)<len(p)*.5
