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
export HFS=/opt/hfs22.0.459  # Change to the Houdini installation you want to use.
```

The plugin and [`kimodo.cpp`](https://github.com/localai-org/kimodo.cpp) are
separate checkouts. If the dependency is not already present, download it:

```bash
git clone --recurse-submodules https://github.com/localai-org/kimodo.cpp.git "$KIMODO_CPP_DIR"
git -C "$KIMODO_CPP_DIR" checkout 5679ff19ba0a522c0b0516e9a9d402fe1af2c027
git -C "$KIMODO_CPP_DIR" submodule update --init --recursive
git -C "$KIMODO_CPP_DIR" apply "$PLUGIN_DIR/patches/kimodo-split-backends.patch"
```

This integration was built against kimodo.cpp commit
`5679ff19ba0a522c0b0516e9a9d402fe1af2c027` with Houdini 22.0.459 on Linux.
CMake 3.25+, a C++23-capable compiler, Make, and Git are required. Ninja is optional.
The bundled dependency patch makes the C API honor the motion-device selector,
adds `KIMODO_TEXT_BACKEND` independently of the motion backend, and fixes model
manifest verification matching unrelated tokenizer paths. Apply it once to the
pinned checkout above.

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
