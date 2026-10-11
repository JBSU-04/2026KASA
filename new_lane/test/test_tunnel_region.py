import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from tunnel_region import read_region,contains

def region(first=7201,last=8850):
    params={'/section_router/sections':[{'id':'M7B','csv':3,'idx':[6197,first-1]},{'id':'M8','csv':3,'idx':[first,last]}]}
    return read_region(lambda k,d=None:params.get(k,d))

def test_preparation_does_not_arm_control():
    r=region()
    for idx in [7101,7119,7200]:
        assert contains(r,3,idx,True)
        assert not contains(r,3,idx)
    assert not contains(r,3,7100,True)

def test_boundaries_and_lane():
    r=region()
    assert contains(r,3,7201) and contains(r,3,8850)
    assert not contains(r,3,8851) and not contains(r,0,7201)

def test_router_change_updates_policy():
    r=region(7400,8900)
    assert not contains(r,3,7201)
    assert contains(r,3,7300,True) and contains(r,3,7400)

def test_bad_configuration_rejected():
    import pytest
    with pytest.raises(ValueError):region(8000,7000)
    with pytest.raises(ValueError):read_region(lambda k,d=None:[] if k.endswith('sections') else d)
