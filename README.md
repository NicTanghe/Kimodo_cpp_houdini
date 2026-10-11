# kimodo_houdini — Native Houdini 21+ C++ HDK Plugin for `kimodo.cpp`

Pure C++23 Houdini 21+ HDK integration for [`kimodo.cpp`](../kimodo.cpp). Eliminates Python runtimes, garbage collection pauses, JSON serialization, and intermediate daemon processes by linking `kimodo.cpp` directly into a native Houdini SOP operator (`SOP_Kimodo` / `sop_kimodo_native`).

## Architecture

```
[ Houdini 21+ Viewport / Timeline ]
         │
         ▼
[ SOP_Kimodo ("Kimodo Native" HDK Node) ]
         │
         ├─ Evaluates Parms: prompt, frames, steps, seed, CFG weights, GGUF paths, device
         ├─ KimodoSessionCache: Lazy-loads GGUF weights once into RAM/VRAM
         ├─ Calls C ABI: kimodo_generate() via <kimodo/kimodo_capi.h>
         ├─ Caches motion clip across timeline frames (zero re-inference while scrubbing)
         ├─ Evaluates C++ Forward Kinematics (SOMA-30 / SMPL-X 22 / Unitree G1 34)
         │
         ▼
Direct GU_Detail KineFX Skeleton Output:
  - P                  : World-space joint positions
  - s@name             : Skeleton joint names (SOMA-30, SMPL-X 22, or G1-34)
  - p@rot              : Local joint quaternion [x, y, z, w]
  - 3@transform        : World-space 3x3 rotation matrix (KineFX standard)
  - 4@localtransform   : Parent-local 4x4 transform matrix (KineFX standard)
  - i@parent           : Parent joint index (-1 for root)
  - GEO_PrimPoly lines : Connected parent-child KineFX bone hierarchy
```

## Directory Layout

- `include/kimodo_houdini/fk_skeleton.hpp` — SOMA-30, SMPL-X 22, and Unitree G1 34 skeleton definitions, C++ Forward Kinematics solver, KineFX matrix builders, and native `.glb` / `.bvh` writers.
- `include/kimodo_houdini/motion_cache.hpp` — Thread-safe RAII wrapper around `kimodo_capi.h` (`kimodo_model_load`, `kimodo_generate`, `kimodo_generate_embedding`) with timeline-aware motion caching.
- `src/SOP_Kimodo.h` / `src/SOP_Kimodo.C` — Native Houdini 21+ HDK SOP node (`sop_kimodo_native`).
- `src/kimodo_cli.cpp` — Standalone pure-C++ CLI binary that exports animated `.glb` (glTF 2.0 binary) and `.bvh` files directly via `libkimodo.so`.
- `scripts/build_hcustom.sh` — Compiles `libkimodo.so` and installs `SOP_Kimodo.so` into `$HOME/houdini21.X/dso/` via Houdini's `hcustom`.
- `tests/test_pipeline.cpp` — C++ test suite verifying `libkimodo.so` C-API linkage, SOMA-30 Forward Kinematics, KineFX transform matrices, and binary `.glb` / `.bvh` generation.

## Building & Testing (Linux)

### 1. Build `libkimodo.so`, `kimodo_cli`, and C++ Tests via CMake

```bash
cmake -B build -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build
ctest --test-dir build --output-on-failure
```

If Houdini 21+ is installed in `/opt/hfs21.*` or `$HFS` is set in your environment, CMake automatically detects `$HFS/toolkit/cmake` and builds `SOP_Kimodo.so`.

### 2. Compile & Install `SOP_Kimodo.so` via `hcustom` (Houdini 21+)

```bash
source /opt/hfs21.0/houdini_setup
./scripts/build_hcustom.sh
```

Verify symbol linkage on the installed DSO:

```bash
ldd $HOME/houdini21*/dso/SOP_Kimodo.so | grep -i -E "kimodo|ggml"
```

### 3. Standalone C++ CLI (`kimodo_cli`)

You can also generate standalone `.glb` or `.bvh` files from C++ without launching Houdini:

```bash
./build/kimodo_cli \
  --model /path/to/kimodo-soma-rp-v1.1-f32.gguf \
  --text-model /path/to/llm2vec-q8_0.gguf \
  --prompt "A person walking forward and waving" \
  --frames 60 \
  --steps 50 \
  --seed 42 \
  --output-glb /tmp/animation.glb \
  --output-bvh /tmp/animation.bvh
```

## KineFX Downstream Graph in Houdini 21+

Drop **Kimodo Native** (`sop_kimodo_native`) into any `/obj/geo` network:

```
[ Kimodo Native (SOP_Kimodo) ]
         │  (Outputs KineFX skeleton: P, 3@transform, 4@localtransform, p@rot, s@name, bone lines)
         ▼
[ Skeleton Rename SOP ]   <-- Optional: map SOMA-30 / SMPL-X 22 / G1-34 joint names to target rig
         │
         ▼
[ Rig Match Pose SOP ]    <-- Align rest / T-pose with target production character
         │
         ▼
[ Character Retarget SOP ] <-- Solve animated motion onto target character rig
         │
         ▼
[ Bone Deform SOP ]       <-- Deform character mesh
```
