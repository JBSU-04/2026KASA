"""Lead profile conversion without a master or control publication."""
import importlib.util
import math
from pathlib import Path
from types import SimpleNamespace
import pytest
spec = importlib.util.spec_from_file_location('mission_speed_cap', Path(__file__).parents[1] / 'scripts/constraint_to_speed_cap.py')
cap = importlib.util.module_from_spec(spec)
spec.loader.exec_module(cap)

def constraint(override=False, gap=2.5, time=.5):
    return SimpleNamespace(lead_distance=7., lead_speed=5., comfortable_decel=2.5,
                           lead_profile_override=override, lead_standstill_distance=gap, lead_time_gap=time)

def test_other_mission_retains_default_profile():
    assert cap.lead_speed_cap(constraint(),5.,4.,1.5,2.5)==pytest.approx(2.)

def test_roundabout_uses_selected_profile_and_braking_cap():
    assert cap.lead_speed_cap(constraint(True),5.,4.,1.5,2.5)==pytest.approx(math.sqrt(47.5))

@pytest.mark.parametrize('gap,time',[(float('nan'),.5),(2.5,float('inf')),(-1,.5),(2.5,0)])
def test_invalid_override_preserves_defaults(gap,time):
    assert cap.lead_speed_cap(constraint(True,gap,time),5.,4.,1.5,2.5)==pytest.approx(2.)
