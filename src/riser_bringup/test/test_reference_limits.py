"""Verify derived stroke limits against authoritative geometry and URDF ranges."""
import importlib.util
import json
import math
from pathlib import Path

# This test is source-only: canonical evidence is intentionally not duplicated.
ROOT = Path(__file__).resolve().parents[3]


def test_actuator_bounds_and_angular_speed_envelope():
    path = ROOT / 'src/riser_bringup/scripts/actuator_mapping.py'
    spec = importlib.util.spec_from_file_location('mapping_reference', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    maps = module.load_maps(ROOT / 'evidence/model_reference/actuator_mapping.json')
    config = json.loads((ROOT / 'src/riser_bringup/urdf/model/kinematics.yaml').read_text())
    k, neutral = config['joints'], config['neutral_cad']
    for coordinate, actuator in [('frame_pitch','top_actuator'), ('foot_pitch','bottom_actuator'),
                                 ('torso_lateral','torso_actuator'), ('hip_lateral','hip_actuator')]:
        physical, stroke = k[coordinate], k[actuator]
        mapping = maps[coordinate]
        zero = mapping.forward(neutral[coordinate])
        endpoints = [mapping.forward(physical[b] + neutral[coordinate]) - zero
                     for b in ('lower', 'upper')]
        assert math.isclose(stroke['lower'], min(endpoints), abs_tol=1e-12)
        assert math.isclose(stroke['upper'], max(endpoints), abs_tol=1e-12)
        # Sampled regression check, not certification of hardware travel/speed.
        for i in range(1001):
            q = physical['lower'] + (physical['upper']-physical['lower'])*i/1000
            q += neutral[coordinate]
            s = mapping.forward(q) - zero
            assert stroke['lower']-1e-12 <= s <= stroke['upper']+1e-12
            assert abs(mapping.derivative(q)*physical['velocity']) <= stroke['velocity']+1e-12
    c = json.loads((ROOT / 'src/riser_bringup/config/control_coordinates.json').read_text())['ml']
    for name, h in c['height_m'].items():
        physical = k[name + '_lateral']
        assert h*math.tan(c['limit_rad']) <= physical['upper']+1e-12
        assert -h*math.tan(c['limit_rad']) >= physical['lower']-1e-12
        assert h*(1+math.tan(c['limit_rad'])**2)*c['velocity_rad_s'] <= physical['velocity']+1e-12
