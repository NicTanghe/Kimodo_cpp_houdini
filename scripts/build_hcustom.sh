#!/usr/bin/env bash
# Build and install SOP_Kimodo.so into $HOME/houdini21.X/dso using Houdini 21+ hcustom.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
KIMODO_CPP_DIR="${KIMODO_CPP_DIR:-$(cd "${PROJECT_ROOT}/../kimodo.cpp" && pwd)}"
BUILD_DIR="${PROJECT_ROOT}/build"

# 1. Ensure libkimodo.so is compiled with -fPIC
if [[ ! -f "${BUILD_DIR}/libkimodo.so" ]]; then
    echo "[1/3] Building shared libkimodo.so..."
    cmake -B "${BUILD_DIR}" -S "${PROJECT_ROOT}" -G Ninja -DCMAKE_BUILD_TYPE=Release
    cmake --build "${BUILD_DIR}" --target kimodo_shared
fi

# 2. Source Houdini 21+ environment if not already active
if ! command -v hcustom >/dev/null 2>&1; then
    HFS_CANDIDATE="${HFS:-}"
    if [[ -z "${HFS_CANDIDATE}" ]]; then
        for dir in /opt/hfs21*; do
            if [[ -f "${dir}/houdini_setup" ]]; then
                HFS_CANDIDATE="${dir}"
            fi
        done
    fi
    if [[ -z "${HFS_CANDIDATE}" || ! -f "${HFS_CANDIDATE}/houdini_setup" ]]; then
        echo "Error: hcustom not found. Set HFS=/opt/hfs21.0 or source houdini_setup first." >&2
        exit 1
    fi
    pushd "${HFS_CANDIDATE}" >/dev/null
    # shellcheck disable=SC1091
    source ./houdini_setup
    popd >/dev/null
fi

GGML_LIB_DIR="${KIMODO_CPP_DIR}/build/ggml/src"
GGML_VK_LIB_DIR="${KIMODO_CPP_DIR}/build/ggml/src/ggml-vulkan"

echo "[2/3] Compiling SOP_Kimodo.C via hcustom..."
hcustom \
    -e \
    -I "${PROJECT_ROOT}/include" \
    -I "${KIMODO_CPP_DIR}/include" \
    -L "${BUILD_DIR}" \
    -lkimodo \
    -Wl,-rpath,"${BUILD_DIR}:${GGML_LIB_DIR}:${GGML_VK_LIB_DIR}" \
    "${PROJECT_ROOT}/src/SOP_Kimodo.C"

echo "[3/3] Verifying DSO linkage..."
for dso in "${HOME}"/houdini21*/dso/SOP_Kimodo.so; do
    if [[ -f "${dso}" ]]; then
        echo "Installed: ${dso}"
        ldd "${dso}" | grep -i -E "kimodo|ggml" || true
    fi
done
