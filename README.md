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

---

## Installation Guide (Houdini 21+ on Linux)

### Step 1: Download the GGUF Model Weights

If you haven't downloaded the native GGUF weights yet, run `download_gguf_weights.sh` from your `kimodo.cpp` directory:

```bash
cd /home/dude/dev/Houdini_motionAI/kimodo.cpp
./scripts/download_gguf_weights.sh --output "$PWD" --model soma-rp-v1.1
```

This downloads:
- **Motion model**: `/home/dude/dev/Houdini_motionAI/kimodo.cpp/models/kimodo-soma-rp-v1.1-f32.gguf`
- **Text encoder bundle**: `q8_0` weights + `tokenizer.gguf` in `/home/dude/dev/Houdini_motionAI/kimodo.cpp/models/`

*(Optional)* You can also download `soma-seed-v1.1`, `g1-rp-v1`, or `g1-seed-v1` by passing additional `--model` flags.

---

### Step 2: Compile & Install the Houdini 21+ DSO (`SOP_Kimodo.so`)

You can build and install the plugin using either `build_hcustom.sh` or CMake + Ninja.

#### Option A: Using `scripts/build_hcustom.sh` (Recommended)

Point `HFS` to your Houdini 21+ installation (defaults to `/opt/hfs21*`) and run:

```bash
cd /home/dude/dev/Houdini_motionAI/kimodo_houdini
HFS=/opt/hfs21.0 ./scripts/build_hcustom.sh
```

What this script does:
1. Compiles `build/libkimodo.so` with `-fPIC` and embeds the runtime `RPATH` for `libggml.so` and `libggml-vulkan.so`.
2. Sources `$HFS/houdini_setup` and compiles `src/SOP_Kimodo.C` via `hcustom`.
3. Installs the compiled plugin directly into `~/houdini21.0/dso/SOP_Kimodo.so`.
4. Verifies dynamic library resolution using `ldd`.

#### Option B: Using CMake + Ninja

```bash
source /opt/hfs21.0/houdini_setup
cd /home/dude/dev/Houdini_motionAI/kimodo_houdini
cmake -B build -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build
ctest --test-dir build --output-on-failure
```

When `HFS` is active in your shell, `CMakeLists.txt` detects `$HFS/toolkit/cmake` and runs `houdini_configure_target(SOP_Kimodo)`, installing `SOP_Kimodo.so` into `~/houdini21.0/dso/`.

#### Verify DSO Linkage

```bash
ldd $HOME/houdini21*/dso/SOP_Kimodo.so | grep -i -E "kimodo|ggml"
```

---

### Step 3: Use Inside Houdini 21+

1. Launch `houdini`.
2. Create a **Geometry (`/obj/geo1`)** node and dive inside.
3. Press <kbd>Tab</kbd> and create **Kimodo Native** (`sop_kimodo_native`).
4. Configure the node parameters:
   - **Motion GGUF Path**: `/home/dude/dev/Houdini_motionAI/kimodo.cpp/models/kimodo-soma-rp-v1.1-f32.gguf`
   - **Text Encoder GGUF / Bundle**: Path to your downloaded LLM2Vec GGUF file (or leave empty to test denoiser-only zero-embedding mode)
   - **Compute Backend**: `Auto (Vulkan / CPU)`, `CPU`, or `Vulkan GPU`
   - **Text Prompt**: e.g., `A person walking forward and waving`
   - **Frame Count / Diffusion Steps / Seed**: Adjust as needed (motion is cached across timeline scrubbing and only re-runs when generation parameters change).
5. Connect the output into your KineFX retargeting graph:

```
[ Kimodo Native (SOP_Kimodo) ]
         │  (Outputs KineFX skeleton: P, 3@transform, 4@localtransform, p@rot, s@name, bone lines)
         ▼
[ Skeleton Rename SOP ]    <-- Optional: map SOMA-30 / SMPL-X 22 / G1-34 joint names to target rig
         │
         ▼
[ Rig Match Pose SOP ]     <-- Align rest / T-pose with target production character
         │
         ▼
[ Character Retarget SOP ] <-- Solve animated motion onto target character rig
         │
         ▼
[ Bone Deform SOP ]        <-- Deform character mesh
```

---

## Standalone C++ CLI (`kimodo_cli`)

You can also generate standalone `.glb` or `.bvh` files from C++ without launching Houdini:

```bash
./build/kimodo_cli \
  --model /home/dude/dev/Houdini_motionAI/kimodo.cpp/models/kimodo-soma-rp-v1.1-f32.gguf \
  --text-model /home/dude/dev/Houdini_motionAI/kimodo.cpp/models/llm2vec-q8_0.gguf \
  --prompt "A person walking forward and waving" \
  --frames 60 \
  --steps 50 \
  --seed 42 \
  --output-glb /tmp/animation.glb \
  --output-bvh /tmp/animation.bvh
```
