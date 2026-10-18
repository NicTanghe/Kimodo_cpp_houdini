"""End-to-end manual generation and constraint-input regression; run with hython.
Usage: hython tests/houdini_constraints.py --models /path/to/kimodo.cpp
"""
import argparse
import math
from pathlib import Path
import hou

parser = argparse.ArgumentParser()
parser.add_argument('--models', type=Path, required=True)
parser.add_argument('--device', choices=('cpu', 'vulkan'), default='vulkan')
parser.add_argument('--steps', type=int, default=20)
parser.add_argument('--save', type=Path)
args = parser.parse_args()
hou.setFrame(1)
geo = hou.node('/obj').createNode('geo', 'kimodo_constraints_example')
skeleton = geo.createNode('sop_kimodo_skeleton', 'kimodo_rest_skeleton')
assert len(skeleton.geometry().points()) == 30
assert len(skeleton.geometry().prims()) == 29
assert skeleton.geometry().findPointAttrib('transform').size() == 9
animated = geo.createNode('xform', 'animate_skeleton_with_native_tools')
animated.setInput(0, skeleton)
animated.parm('tx').setExpression('($F-1)/29')
metadata = geo.createNode('python', 'capture_frames')
metadata.setInput(0, animated)
metadata.parm('python').set('''import hou
g = hou.pwd().geometry()
g.addArrayAttrib(hou.attribType.Global, "kimodo_capture_frames", hou.attribData.Int)
g.setGlobalAttribValue("kimodo_capture_frames", [15])
''')
waypoints = geo.createNode('python', 'timed_waypoints')
waypoints.parm('python').set('''import hou
g = hou.pwd().geometry()
g.addAttrib(hou.attribType.Point, "frame", 0)
for frame, x in [(1, 0.), (30, 1.)]:
    p = g.createPoint()
    p.setPosition((x, 0, 0))
    p.setAttribValue("frame", frame)
''')
node = geo.createNode('sop_kimodo_native', 'generate_motion')
node.setParms({'motion_model': str(args.models/'models/kimodo-soma-rp-v1.1-f32.gguf'),
               'text_model': '', 'device': 1 if args.device == 'cpu' else 2, 'frames': 30, 'steps': args.steps})
node.setInput(0, skeleton)  # Pose locks default off, even with an input connected.
assert node.geometry().attribValue('kimodo_generation') == 0
node.parm('prompt').set('This edit must not generate')
hou.setFrame(12)
assert node.geometry().attribValue('kimodo_generation') == 0

def snapshot():
    return tuple(tuple(p.position()) for p in node.geometry().points())

def generate(expected):
    node.parm('generate').pressButton()
    assert not node.errors(), node.errors()
    assert not node.warnings(), node.warnings()
    assert node.geometry().attribValue('kimodo_generation') == expected
    assert len(node.geometry().points()) == 30
    assert all(math.isfinite(v) for p in node.geometry().points() for v in p.position())

generate(1)
baseline = snapshot()
node.setInput(0, metadata)
node.setInput(1, waypoints)
node.setParms({'enable_pose_locks': 1, 'enable_waypoints': 1, 'capture_frames': '1 30', 'capture_source': 2})
assert snapshot() == baseline
assert node.geometry().attribValue('kimodo_generation') == 1
generate(2)
conditioned = snapshot()
assert conditioned != baseline, 'Constraints must affect inference'
node.parm('seed').set(123)
assert snapshot() == conditioned
assert node.geometry().attribValue('kimodo_generation') == 2
node.setParms({'capture_mode': 1, 'capture_group': '@name=LeftHand @name=RightHand', 'capture_source': 1})
generate(3)
selected = snapshot()
assert selected != conditioned
node.setParms({'waypoint_mode': 1})
generate(4)
dense = snapshot()
node.parm('capture_group').set('missing_group')
node.parm('generate').pressButton()
assert node.geometry().attribValue('kimodo_generation') == 4
assert snapshot() == dense
assert any('Locked Joints' in warning for warning in node.warnings()), node.warnings()
node.parm('capture_group').set('@name=LeftHand @name=RightHand')
node.setParms({'capture_source': 0, 'capture_frames': '0'})
node.parm('generate').pressButton()
assert node.geometry().attribValue('kimodo_generation') == 4
assert snapshot() == dense
assert any('Capture frames' in warning for warning in node.warnings()), node.warnings()
# UI ranges plus detail array; generate again with the exact same settings also runs.
node.setParms({'capture_source': 2, 'capture_frames': '1-2 29-30', 'capture_mode': 0, 'waypoint_mode': 0})
generate(5)
generate(6)
# Later input edits and recooks must not replace the generated clip.
before = snapshot()
animated.parm('tx').set(10)
node.cook(force=True)
assert snapshot() == before
assert node.geometry().attribValue('kimodo_generation') == 6
animated.parm('tx').setExpression('($F-1)/29')
# String detail attributes are supported, as well as integer arrays.
metadata.parm('python').set(metadata.parm('python').eval().replace(
    'g.addArrayAttrib(hou.attribType.Global, "kimodo_capture_frames", hou.attribData.Int)',
    'g.addAttrib(hou.attribType.Global, "kimodo_capture_frames", "")').replace(
    '[15])', '"15")'))
node.setParms({'capture_source': 1})
generate(7)
# Exercise text encoding plus constrained motion through the extended C API.
node.setParms({'text_model': str(args.models/'Llama-3-Kimodo-Q8_0.gguf'),
               'prompt': 'A person walking forward', 'capture_source': 2})
generate(8)
# Invalid waypoint data must not silently become unconstrained inference.
old_code = waypoints.parm('python').eval()
waypoints.parm('python').set(old_code.replace('"frame", 0)', '"wrong_name", 0)').replace('"frame", frame)', '"wrong_name", frame)'))
previous = snapshot()
node.parm('generate').pressButton()
assert node.geometry().attribValue('kimodo_generation') == 8
assert snapshot() == previous
assert any('integer point attribute frame' in w for w in node.warnings()), node.warnings()
waypoints.parm('python').set(old_code)
generate(9)
# Save a useful authoring graph; generated memory is intentionally not serialized.
skeleton.setComment('Model-matched rest skeleton. Add Rig Pose or other native KineFX tools downstream.')
node.setComment('Input 0: animated skeleton, UI/detail capture frames. Input 1: points with integer frame. Press Generate after editing. Use File Cache to persist the clip.')
node.setDisplayFlag(True)
node.setRenderFlag(True)
hou.playbar.setFrameRange(1, 30)
hou.playbar.setPlaybackRange(1, 30)
geo.layoutChildren()
if args.save:
    hou.hipFile.save(str(args.save))
print('PASS: manual trigger, UI/detail capture, animated input sampling, selected joints, waypoints, dense path, failed-generation preservation')
