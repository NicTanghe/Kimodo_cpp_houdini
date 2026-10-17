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
- `scripts/build_hcustom.sh` — Compiles `libkimodo.so` and installs `SOP_Kimodo.so` into `$HOME/houdiniX.Y/dso/` via Houdini's `hcustom`.
- `tests/test_pipeline.cpp` — C++ test suite verifying `libkimodo.so` C-API linkage, SOMA-30 Forward Kinematics, KineFX transform matrices, and binary `.glb` / `.bvh` generation.

---

## Installation Guide (Houdini 21+ on Linux)

### Step 0: Set paths for this computer

Run these commands from the `Kimodo_cpp_houdini` folder:

```bash
export PLUGIN_DIR="$PWD"
export KIMODO_CPP_DIR="$(dirname "$PLUGIN_DIR")/kimodo.cpp"
export KIMODO_CPP_BUILD_DIR="$KIMODO_CPP_DIR/build"
export HFS=/opt/hfs22.0  # Change to the Houdini installation you want to use.
```

The plugin and [`kimodo.cpp`](https://github.com/localai-org/kimodo.cpp) are
separate checkouts. If the dependency is not already present, download it:

```bash
git clone --recurse-submodules https://github.com/localai-org/kimodo.cpp.git "$KIMODO_CPP_DIR"
git -C "$KIMODO_CPP_DIR" checkout 5679ff19ba0a522c0b0516e9a9d402fe1af2c027
git -C "$KIMODO_CPP_DIR" submodule update --init --recursive
git -C "$KIMODO_CPP_DIR" apply "$PLUGIN_DIR/patches/kimodo-split-backends.patch"
git -C "$KIMODO_CPP_DIR" apply "$PLUGIN_DIR/patches/kimodo-constraints.patch"
```

This integration was built against kimodo.cpp commit
`5679ff19ba0a522c0b0516e9a9d402fe1af2c027` with Houdini 22.0.473 on Linux.
CMake 3.25+, a C++23-capable compiler, Make, and Git are required. Ninja is optional.
The backend dependency patch makes the C API honor the motion-device selector,
adds `KIMODO_TEXT_BACKEND` independently of the motion backend, and fixes model
manifest verification matching unrelated tokenizer paths. Apply it once to the
pinned checkout above. The second patch exposes conditioned sampling through the
C API. If upgrading a checkout with the backend patch already applied, apply only
`patches/kimodo-constraints.patch`, then rebuild the plugin. Do not apply either
patch twice.

For **GPU motion + CPU text**, install `libvulkan-dev`, `glslc`, and
`spirv-headers` using your distribution's package manager, then build:

```bash
cmake -S "$KIMODO_CPP_DIR" -B "$KIMODO_CPP_BUILD_DIR" \
  -DCMAKE_BUILD_TYPE=Release -DBUILD_SHARED_LIBS=ON \
  -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
  -DKIMODO_ENABLE_VULKAN=ON -DKIMODO_BUILD_TESTS=OFF -DBUILD_TESTING=OFF
cmake --build "$KIMODO_CPP_BUILD_DIR" --target kimodo --parallel 6
```

For CPU-only use, configure with `-DKIMODO_ENABLE_VULKAN=OFF` instead.
This integration expects shared GGML libraries under
`$KIMODO_CPP_BUILD_DIR/ggml/src`.

Do not copy a `build/` directory from another computer: its CMake cache and
compiled library paths refer to the original checkout. Keep the local build and
dependency directories after installation, because the plugin loads libraries
from them at runtime. Rebuild the DSO for the Houdini version you launch.

### Step 1: Download the GGUF Model Weights

If you haven't downloaded the native GGUF weights yet, run `download_gguf_weights.sh` from your `kimodo.cpp` directory:

```bash
python3 -m venv "$(dirname "$PLUGIN_DIR")/.venv-download"
source "$(dirname "$PLUGIN_DIR")/.venv-download/bin/activate"
python -m pip install huggingface_hub
export HF_HOME="$(dirname "$PLUGIN_DIR")/.hf-cache"
cd "$KIMODO_CPP_DIR"
./scripts/download_gguf_weights.sh --output "$PWD" --model soma-rp-v1.1
```

This downloads:
- **Motion model**: `${KIMODO_CPP_DIR}/models/kimodo-soma-rp-v1.1-f32.gguf`
- **Text encoder**: `${KIMODO_CPP_DIR}/Llama-3-Kimodo-Q8_0.gguf` and `${KIMODO_CPP_DIR}/tokenizer.gguf` (keep these together)

*(Optional)* You can also download `soma-seed-v1.1`, `g1-rp-v1`, or `g1-seed-v1` by passing additional `--model` flags.

---

### Step 2: Compile & Install the Houdini 21+ DSO (`SOP_Kimodo.so`)

You can build and install the plugin using either `build_hcustom.sh` or CMake.

#### Option A: Using `scripts/build_hcustom.sh`

Point `HFS` to your Houdini 21+ installation (searches `/opt/hfs*` for version 21 or newer) and run:

```bash
cd "$PLUGIN_DIR"
./scripts/build_hcustom.sh
```

What this script does:
1. Compiles `build/libkimodo.so` with `-fPIC` and embeds the runtime `RPATH` for `libggml.so` and `libggml-vulkan.so`.
2. Sources `$HFS/houdini_setup` and compiles `src/SOP_Kimodo.C` via `hcustom`.
3. Installs the compiled plugin directly into `~/houdiniX.Y/dso/SOP_Kimodo.so`.
4. Verifies dynamic library resolution using `ldd`.

#### Option B: Using CMake (verified on Houdini 22.0.459)

```bash
cd "$HFS"
source ./houdini_setup
cd "$PLUGIN_DIR"
cmake -B build -DCMAKE_BUILD_TYPE=Release \
  -DKIMODO_CPP_DIR="$KIMODO_CPP_DIR" \
  -DKIMODO_CPP_BUILD_DIR="$KIMODO_CPP_BUILD_DIR"
cmake --build build
ctest --test-dir build --output-on-failure
```

`X.Y` is the selected Houdini version (for example, `22.0`).

When `HFS` is active in your shell, `CMakeLists.txt` detects `$HFS/toolkit/cmake` and runs `houdini_configure_target(SOP_Kimodo)`, installing `SOP_Kimodo.so` into `~/houdiniX.Y/dso/`.

To stage the plugin inside the build directory before installing, add
`-DKIMODO_DSO_DIR="$PLUGIN_DIR/build/dso"` to the configure command. After building:

```bash
install -D -m 755 "$PLUGIN_DIR/build/dso/SOP_Kimodo.so" \
  "$HOME/houdini22.0/dso/SOP_Kimodo.so"
```

Use the user directory matching the Houdini version you compiled against.

#### Verify DSO Linkage

```bash
ldd $HOME/houdini*/dso/SOP_Kimodo.so | grep -i -E "kimodo|ggml"
```

---

### Step 3: Configure CPU text and local model paths

Create `$HOME/houdini22.0/packages/kimodo.json` (adjust the Houdini version):

```json
{
  "env": [
    {"KIMODO_TEXT_BACKEND": "cpu"},
    {"KIMODO_MODEL_DIR": "/absolute/path/to/kimodo.cpp"}
  ]
}
```

Replace the model-directory value with your actual checkout. Restart Houdini after
installing the plugin or changing this package. New nodes use this directory for
their default model paths. The text encoder runs on CPU; the node's **Compute
Backend** controls only the motion model. Select **Vulkan GPU** to require GPU
execution (an unavailable GPU produces an error rather than a CPU fallback).
For CLI use, export `KIMODO_TEXT_BACKEND=cpu` before launching the CLI.

### Step 4: Use Inside Houdini 21+

1. Launch `houdini`.
2. Create a **Geometry (`/obj/geo1`)** node and dive inside.
3. Press <kbd>Tab</kbd> and create **Kimodo Native** (`sop_kimodo_native`).
4. Configure the node parameters:
   - Model paths default to `$KIMODO_MODEL_DIR` from your Houdini package; you can also browse to the files. `${KIMODO_CPP_DIR}` below refers to the shell variable set above; use its full resolved path in Houdini.
   - **Motion GGUF Path**: `${KIMODO_CPP_DIR}/models/kimodo-soma-rp-v1.1-f32.gguf`
   - **Text Encoder GGUF / Bundle**: `${KIMODO_CPP_DIR}/Llama-3-Kimodo-Q8_0.gguf` (leaving this empty runs denoiser-only zero-embedding mode and ignores the prompt)
   - **Compute Backend**: choose `Vulkan GPU` for GPU motion with CPU text; choose `CPU` for CPU-only builds.
   - **Text Prompt**: e.g., `A person walking forward and waving`
   - **Frame Count / Diffusion Steps / Seed**: Adjust as needed (edits do not regenerate motion; press Generate explicitly).
5. Press **Generate**. Scrubbing and parameter edits retain that clip until the next button click.
6. Connect the output into your KineFX retargeting graph:

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

## Pose capture, in-betweening, and paths

Create **Kimodo Skeleton** (`sop_kimodo_skeleton`) for a model-matched SOMA 30,
SMPL-X 22, or G1 34 rest skeleton. It requires no model and performs no inference.
It outputs named joints, parent-child bone lines, `transform`, `localtransform`,
`rot`, and `parent`, with matrix/quaternion type metadata. Adjust **Root Height**
and animate it using Rig Pose or other native KineFX tools.

Connect the animated skeleton to **input 0** of **Kimodo Native**, and waypoint
points to **input 1**. Choose the matching **Skeleton** on both nodes and a motion
model for that skeleton. Captures are matched by joint `name`, not point order.
All positions are SOP-space, Y-up, in meters. Use Object Merge transforms when
bringing inputs from different objects. The model produces 30 motion samples per
second; a 30 FPS Houdini scene plays those samples at their intended speed.

### Input 0: capture animation at chosen frames

**Start Frame** and **Frame Count** define the output clip interval. Capture and
waypoint frames are absolute Houdini frame numbers within that interval. For
example, Start Frame 101 and Frame Count 60 give frames 101 through 160.

Set **Capture Frames From** to the UI list, detail attribute, or their union:

- **Capture Frames** accepts a list such as `1 30 60`, inclusive ranges such as
  `1-10 51-60`, or stepped ranges such as `1-60:5`. Commas are also accepted.
- The default detail attribute is **`kimodo_capture_frames`**. It can be an
  integer array or a string using the same range syntax. For a Detail Wrangle:

  ```c
  i[]@kimodo_capture_frames = array(1, 30, 60);
  ```

The node cooks input 0 separately at every selected frame **when Generate is
pressed** and snapshots the poses. It does not use the currently displayed pose
for every capture. An empty list means no pose constraints; frame numbers without
input 0 are rejected. Duplicate capture frames are merged.

For in-betweening, capture the beginning and ending poses, or ranges at both ends
(e.g. `1-10 51-60`), then let Kimodo generate the motion between them.

**Full Body** constrains all joint positions and the root position/heading, as in
NVIDIA's FullBodyConstraintSet. **Selected Joints** uses a named **Joint Point
Group** on input 0; create that group with native Group tools. The full named
skeleton must remain present. Selected positions are constrained, and enabling
**Constrain Selected Joint Rotations** also reads their world-space KineFX
`transform` matrices. Root position/heading are constrained in this mode too, as
required by the smooth-root representation. Hand and foot constraints are the
principal trained use case; arbitrary joint selections are accepted but have not
been separately quality-benchmarked.

### Input 1: timed waypoints or a dense path

Each point requires:

| Attribute | Type | Meaning |
| --- | --- | --- |
| `P` | vector | Target smooth-root position; X and Z are used, Y is ignored |
| `frame` | scalar integer | Absolute Houdini frame at which to reach the waypoint |
| `heading` | optional scalar float | Facing angle in radians; zero is forward +Z |

Point order is irrelevant; timing comes from `frame`. Duplicate or out-of-range
waypoint frames are rejected. **Sparse Waypoints** only constrains those frames.
**Dense Path** linearly interpolates X/Z at every frame and requires waypoints on
the first and last clip frames. If heading is present it interpolates along the
shortest angular arc.

Pose and waypoint constraints can be combined. A pose capture constrains the
smooth root to its pelvis X/Z; a waypoint at the same frame must agree with that
position. Conflicting constraints produce a warning and keep the previous clip.

### Generate and retain results

Only **Generate** runs inference. Creating the node, changing prompts/settings,
editing either input, scrubbing, and force-recooking do not generate a new clip.
Pressing Generate again uses the current inputs/settings even with the same seed.
A failed generation keeps the last successful output and reports a warning.
Without a clip the node displays a rest skeleton and asks you to Generate.

`kimodo_generation` and `kimodo_clip_frames` detail attributes report the current
successful generation count and cached clip length. The clip is held in memory,
not serialized in the HIP file. Use a downstream **File Cache** to retain generated
animation across Houdini sessions; reopening an uncached graph requires pressing
Generate again. The separate skeleton and all authored keyframes remain normal
Houdini scene data.

Conditioning enters the model's constraint CFG branch at every diffusion step.
These are learned constraints, not hard IK locks: generated poses may deviate,
and smooth-root waypoints do not guarantee the pelvis hits the exact X/Z target.
There is no post-generation snapping disguised as model conditioning. See
[NVIDIA's constraint concepts](https://research.nvidia.com/labs/sil/projects/kimodo/docs/key_concepts/constraints.html).
Multi-prompt stitching, upstream postprocessing/IK, and 77-joint SOMA expansion
are not exposed by this node.


## Standalone C++ CLI (`kimodo_cli`)

You can also generate standalone `.glb` or `.bvh` files from C++ without launching Houdini:

```bash
KIMODO_TEXT_BACKEND=cpu ./build/kimodo_cli \
  --model "$KIMODO_CPP_DIR/models/kimodo-soma-rp-v1.1-f32.gguf" \
  --text-model "$KIMODO_CPP_DIR/Llama-3-Kimodo-Q8_0.gguf" \
  --device vulkan \
  --prompt "A person walking forward and waving" \
  --frames 60 \
  --steps 50 \
  --seed 42 \
  --output-glb /tmp/animation.glb \
  --output-bvh /tmp/animation.bvh
```


## Houdini integration check

With the plugin installed and model weights downloaded, run this from the plugin
folder (requires access to your GPU and Houdini license server):

```bash
KIMODO_TEXT_BACKEND=cpu KIMODO_PROFILE=1 "$HFS/bin/hython" tests/houdini_smoke.py \
  --models "$KIMODO_CPP_DIR" --device vulkan --text
```

The profile output should show `text.backend=CPU` and `motion.backend=Vulkan0`.
The check verifies real inference, skeleton topology, attributes, finite positions,
and timeline updates. It uses only four diffusion steps as a smoke test; increase
the node's diffusion steps for production animation.

Run the input-workflow regression (with CPU text and GPU motion):

```bash
KIMODO_TEXT_BACKEND=cpu "$HFS/bin/hython" tests/houdini_constraints.py \
  --models "$KIMODO_CPP_DIR" --save /tmp/kimodo_constraints_example.hip
```

It checks UI/array/string frame capture, full and selected-joint constraints,
waypoints and dense paths, button-only generation, and preservation of the last
clip after invalid input. The saved authoring network requires Generate after
opening because the inference cache is not serialized.
