"""Run with hython to verify an installed DSO and real inference backends.

Example: KIMODO_TEXT_BACKEND=cpu KIMODO_PROFILE=1 hython tests/houdini_smoke.py \
    --models /path/to/kimodo.cpp --device vulkan --text
"""
import argparse
import math
from pathlib import Path

import hou

parser = argparse.ArgumentParser()
parser.add_argument('--models', type=Path, required=True)
parser.add_argument('--device', choices=('cpu', 'vulkan'), default='vulkan')
parser.add_argument('--text', action='store_true')
parser.add_argument('--save', type=Path)
args = parser.parse_args()
geo = hou.node('/obj').createNode('geo', 'kimodo_demo')
node = geo.createNode('sop_kimodo_native', 'kimodo_native')
node.setParms({
    'motion_model': str(args.models / 'models/kimodo-soma-rp-v1.1-f32.gguf'),
    'text_model': str(args.models / 'Llama-3-Kimodo-Q8_0.gguf') if args.text else '',
    'device': 2 if args.device == 'vulkan' else 1,
    'prompt': 'A person walking forward',
    'frames': 30,
    'steps': 4,
})
node.setDisplayFlag(True)
node.setRenderFlag(True)
hou.setFrame(1)
node.cook(force=True)
assert not node.errors(), node.errors()
g = node.geometry()
assert len(g.points()) == 30, len(g.points())
assert len(g.prims()) == 29, len(g.prims())
for name in ('name', 'transform', 'localtransform', 'rot', 'parent'):
    assert g.findPointAttrib(name) is not None, name
assert all(math.isfinite(v) for p in g.points() for v in p.position())
assert len({p.attribValue('name') for p in g.points()}) == 30
first_positions = [tuple(p.position()) for p in g.points()]
hou.setFrame(10)
node.cook(force=True)
assert not node.errors(), node.errors()
assert len(node.geometry().points()) == 30
assert [tuple(p.position()) for p in node.geometry().points()] != first_positions
hou.playbar.setFrameRange(1, 30)
hou.playbar.setPlaybackRange(1, 30)
hou.setFrame(1)
if args.save:
    hou.hipFile.save(str(args.save))
print('PASS: inference, 30 named joints, 29 bones, finite positions, KineFX attributes, timeline recook')
