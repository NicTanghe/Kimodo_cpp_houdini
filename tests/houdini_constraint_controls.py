"""Validate conditional UI, the Rig Pose name picker, and disabled-input behavior.
Run with hython and --models /path/to/kimodo.cpp; defaults to Vulkan GPU.
"""
import argparse
from pathlib import Path
from unittest.mock import patch
import hou
import soputils

parser = argparse.ArgumentParser()
parser.add_argument('--models', type=Path, required=True)
parser.add_argument('--device', choices=('cpu', 'vulkan'), default='vulkan')
args = parser.parse_args()
geo = hou.node('/obj').createNode('geo', 'kimodo_controls_test')
node = geo.createNode('sop_kimodo_native')
node.setParms({'motion_model': str(args.models/'models/kimodo-soma-rp-v1.1-f32.gguf'),
               'text_model': '', 'device': 1 if args.device == 'cpu' else 2, 'frames': 8, 'steps': 2})

def visible(name, expected):
    node.updateParmStates()
    assert node.parm(name).isHidden() == (not expected), name
    assert node.parm(name).isDisabled() == (not expected), name

for name in ('capture_source', 'capture_frames', 'capture_attribute', 'capture_mode',
             'capture_group', 'capture_rotations', 'waypoint_mode', 'constraint_cfg'):
    visible(name, False)
node.parm('enable_pose_locks').set(1)
for name in ('capture_source', 'capture_frames', 'capture_attribute', 'capture_mode', 'constraint_cfg'):
    visible(name, True)
visible('capture_group', False)
node.parm('capture_mode').set(1)
visible('capture_group', True)
visible('capture_rotations', True)
node.parm('capture_source').set(0)
visible('capture_attribute', False)
visible('capture_frames', True)
node.parm('capture_source').set(1)
visible('capture_frames', False)
visible('capture_attribute', True)
node.parm('enable_waypoints').set(1)
visible('waypoint_mode', True)
node.parm('enable_pose_locks').set(0)
visible('capture_group', False)
visible('constraint_cfg', True)
node.parm('enable_waypoints').set(0)
visible('constraint_cfg', False)

# Validate the actual script-action wiring without opening a modal selector.
tags = node.parm('capture_group').parmTemplate().tags()
assert tags['script_action_icon'] == 'BUTTONS_reselect'
with patch.object(soputils, 'selectGroupParm') as select:
    exec(tags['script_action'], {'kwargs': {'node': node, 'parmtuple': node.parmTuple('capture_group')}})
    kw = select.call_args.args[0]
    assert kw['inputindex'] == 0 and kw['nameattr'] == 'name'
    assert kw['geometrytype'] == hou.geometryType.Points

# Disabled branches must not cook, validate, or condition on connected inputs.
bad = geo.createNode('python', 'must_not_cook_when_disabled')
bad.parm('python').set('raise hou.NodeError("Disabled constraint input was cooked")')
node.setInput(0, bad)
node.setInput(1, bad)
node.setParms({'capture_frames': 'nonsense', 'capture_source': 0, 'waypoint_mode': 1})
node.parm('generate').pressButton()
assert node.geometry().attribValue('kimodo_generation') == 1, node.warnings()
assert not node.errors() and not node.warnings(), (node.errors(), node.warnings())
original = tuple(tuple(p.position()) for p in node.geometry().points())
node.setInput(0, None)
node.setInput(1, None)
node.parm('enable_pose_locks').set(1)
assert node.geometry().attribValue('kimodo_generation') == 1
assert tuple(tuple(p.position()) for p in node.geometry().points()) == original
node.parm('generate').pressButton()
assert node.geometry().attribValue('kimodo_generation') == 1
assert any('input 0' in w for w in node.warnings()), node.warnings()
node.parm('enable_pose_locks').set(0)
node.parm('enable_waypoints').set(1)
node.parm('generate').pressButton()
assert node.geometry().attribValue('kimodo_generation') == 1
assert any('input 1' in w for w in node.warnings()), node.warnings()
node.parm('enable_waypoints').set(0)

skeleton = geo.createNode('sop_kimodo_skeleton')
assert skeleton.geometry().findPointGroup('hands') is None
node.setInput(0, skeleton)
node.setParms({'enable_pose_locks': 1, 'capture_frames': '1 8', 'capture_mode': 1,
               'capture_group': '@name=LeftHand @name=RightHand'})
node.parm('generate').pressButton()
assert node.geometry().attribValue('kimodo_generation') == 2, node.warnings()
assert not node.warnings(), node.warnings()
# Empty selections must never be interpreted as all joints.
node.parm('capture_group').set('')
node.parm('generate').pressButton()
assert node.geometry().attribValue('kimodo_generation') == 2
assert any('Select Locked Joints' in w for w in node.warnings()), node.warnings()
node.parm('enable_pose_locks').set(0)
node.parm('generate').pressButton()
assert node.geometry().attribValue('kimodo_generation') == 3, node.warnings()
assert not node.warnings(), node.warnings()
assert tuple(tuple(p.position()) for p in node.geometry().points()) == original
# Waypoints work independently while an invalid pose branch is disabled.
waypoints = geo.createNode('python', 'valid_waypoints')
waypoints.parm('python').set('import hou\ng=hou.pwd().geometry()\ng.addAttrib(hou.attribType.Point,"frame",0)\nfor f,x in [(1,0.),(8,0.5)]:\n p=g.createPoint()\n p.setPosition((x,0,0))\n p.setAttribValue("frame",f)')
node.setInput(0, bad)
node.setInput(1, waypoints)
node.parm('enable_waypoints').set(1)
node.parm('generate').pressButton()
assert node.geometry().attribValue('kimodo_generation') == 4, node.warnings()
assert not node.warnings(), node.warnings()
print('PASS: conditional visibility, Rig Pose selector wiring, name-based locking without groups, disabled inputs ignored, Generate-only updates')
