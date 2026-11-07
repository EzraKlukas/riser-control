"""Independent Python oracle and source-frame/provenance checks; no ROS needed."""
import importlib.util
import json
import math
from pathlib import Path
import random
import subprocess
import sys
import xml.etree.ElementTree as ET

binary, root = sys.argv[1], Path(sys.argv[2])
spec = importlib.util.spec_from_file_location("oracle", root / "src/riser_bringup/scripts/actuator_mapping.py")
oracle = importlib.util.module_from_spec(spec)
spec.loader.exec_module(oracle)
data = json.loads((root / "evidence/model_reference/actuator_mapping.json").read_text())
linkage = json.loads((root / "evidence/linkage_geometry.json").read_text())
maps = oracle.load_maps(root / "evidence/model_reference/actuator_mapping.json")
rng = random.Random(73)
config = json.loads((root / 'src/riser_bringup/urdf/model/kinematics.yaml').read_text())
domains = [(config['joints'][name]['lower'] + config['neutral_cad'][name],
            config['joints'][name]['upper'] + config['neutral_cad'][name])
           for name in data['joint_order']]
samples = []
for i, name in enumerate(data["joint_order"]):
    lower, upper = domains[i]
    for q in [lower, 0., upper] + [rng.uniform(lower, upper) for _ in range(300)]:
        samples.append((i, name, q, rng.uniform(-.05, .05), lower, upper))
request = ''.join(f'{lo:.17g} {hi:.17g}\n' for lo,hi in domains)
request += ''.join(f'{i} {q:.17g} {v:.17g}\n' for i, _, q, v, _, _ in samples)
result = subprocess.run([binary, '--samples'], input=request, text=True, capture_output=True, check=True)
rows = result.stdout.splitlines()
assert len(rows) == len(samples)
for (i, name, q, v, lower, upper), row in zip(samples, rows):
    s, J, sv, iq, iv = map(float, row.split())
    ref = maps[name]
    assert math.isclose(s, ref.forward(q), abs_tol=2e-14)
    assert math.isclose(J, ref.derivative(q), abs_tol=2e-13)
    assert math.isclose(sv, J*v, abs_tol=1e-14)
    assert math.isclose(iq, ref.inverse(s, lower, upper), abs_tol=1e-10)
    assert math.isclose(iq, q, abs_tol=1e-10)
    assert math.isclose(iv, v, abs_tol=1e-10)

# CAD vectors use the permutation ROS(x,y,z)=CAD(z,x,y). Linkage evidence is
# relative to the pitch pivot, so no world floor-height translation is applied.
def ros(v):
    return [v[2], v[0], v[1]]
def close_vector(a, b):
    assert all(abs(x-y) < 1e-8 for x, y in zip(a, b)), (a,b)

# Read only the complete named axis objects: the user's ongoing actuator edits
# elsewhere in kinematics.yaml need not be parseable for this frame audit.
kinematics = (root / "src/riser_bringup/urdf/model/kinematics.yaml").read_text()
urdf = ET.parse(root / "src/riser_bringup/urdf/riser.urdf.xacro").getroot()
for name, evidence, joint in [('frame_pitch', 'top', 'backboard_pitch'),
                              ('foot_pitch', 'bottom', 'foot_plate_pitch')]:
    g = data['axes'][name]
    r, J, B, C = maps[name]._raw(0.)
    close_vector(ros(B), linkage[evidence]['B'])
    close_vector(ros(g['body_point_cad_m']), linkage[evidence]['body_local'])
    close_vector(ros(g['slider_reference_cad_m']), linkage[evidence]['C'])
    close_vector(ros(g['slider_axis_cad']), linkage[evidence]['axis'])
    assert abs(g['rod_length_m'] - linkage[evidence]['rod_length_m']) < 1e-9
    assert abs(maps[name].closure_residual(0.)) < 1e-12
    axis_text = kinematics.split('"' + name + '"', 1)[1].split(':', 1)[1].lstrip()
    axis_config, _ = json.JSONDecoder().raw_decode(axis_text)
    close_vector(axis_config['axis'], [0.,1.,0.])
    assert abs(axis_config['rpy_rad'][0]) < 1e-12
    assert abs(axis_config['rpy_rad'][1] - g['angle_offset_rad'] - config['neutral_cad'][name]) < 1e-9
    close_vector(axis_config['xyz_m'], [0.,0.,0.])
    element = urdf.find(f"joint[@name='{joint}']")
    assert f"k['joints']['{name}']['axis']" in element.find('axis').get('xyz')
for name in ('torso_lateral', 'hip_lateral'):
    assert data['axes'][name]['ratio'] == -1.
print(f'{len(samples)} Python parity samples and CAD/URDF frame cross-checks passed')
