#!/usr/bin/env bash
# Build and install SOP_Kimodo.so into $HOME/houdiniX.Y/dso using Houdini 21+ hcustom.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
KIMODO_CPP_DIR="${KIMODO_CPP_DIR:-${PROJECT_ROOT}/../kimodo.cpp}"
KIMODO_CPP_BUILD_DIR="${KIMODO_CPP_BUILD_DIR:-${KIMODO_CPP_DIR}/build}"
BUILD_DIR="${PROJECT_ROOT}/build"

if [[ ! -f "${KIMODO_CPP_DIR}/include/kimodo/kimodo_capi.h" ]]; then
    echo "Error: kimodo.cpp is missing at ${KIMODO_CPP_DIR}. Set KIMODO_CPP_DIR to its checkout." >&2
    exit 1
fi

# Source the selected Houdini environment before configuring CMake.
if [[ -n "${HFS:-}" ]] || ! command -v hcustom >/dev/null 2>&1; then
    HFS_CANDIDATE="${HFS:-}"
    if [[ -z "${HFS_CANDIDATE}" ]]; then
        for dir in /opt/hfs*; do
            if [[ "${dir##*/}" =~ ^hfs([0-9]+) && ${BASH_REMATCH[1]} -ge 21 && -f "${dir}/houdini_setup" ]]; then
                HFS_CANDIDATE="${dir}"
            fi
        done
    fi
    if [[ -z "${HFS_CANDIDATE}" || ! -f "${HFS_CANDIDATE}/houdini_setup" ]]; then
        echo "Error: hcustom not found. Set HFS to your Houdini installation or source houdini_setup first." >&2
        exit 1
    fi
    pushd "${HFS_CANDIDATE}" >/dev/null
    # shellcheck disable=SC1091
    set +u
    source ./houdini_setup
    set -u
    popd >/dev/null
fi

# Always configure and build to pick up dependency paths and changed sources.
echo "[1/3] Building shared libkimodo.so..."
cmake -B "${BUILD_DIR}" -S "${PROJECT_ROOT}" -DCMAKE_BUILD_TYPE=Release \
    -DKIMODO_CPP_DIR="${KIMODO_CPP_DIR}" \
    -DKIMODO_CPP_BUILD_DIR="${KIMODO_CPP_BUILD_DIR}"
cmake --build "${BUILD_DIR}" --target kimodo_shared

GGML_LIB_DIR="${KIMODO_CPP_BUILD_DIR}/ggml/src"
GGML_VK_LIB_DIR="${KIMODO_CPP_BUILD_DIR}/ggml/src/ggml-vulkan"

# hcustom accepts extra linker flags through its environment, not a -Wl option.
export HCUSTOM_LDFLAGS="${HCUSTOM_LDFLAGS:-} -Wl,-rpath,${BUILD_DIR}:${GGML_LIB_DIR}:${GGML_VK_LIB_DIR}"
export HCUSTOM_CFLAGS="${HCUSTOM_CFLAGS:-} -std=c++23"
cd "${BUILD_DIR}"
echo "[2/3] Compiling SOP_Kimodo.C via hcustom..."
hcustom \
    -e \
    -I "${PROJECT_ROOT}/include" \
    -I "${KIMODO_CPP_DIR}/include" \
    -L "${BUILD_DIR}" \
    -lkimodo \
    "${PROJECT_ROOT}/src/SOP_Kimodo.C"

echo "[3/3] Verifying DSO linkage..."
for dso in "${HOME}"/houdini*/dso/SOP_Kimodo.so; do
    if [[ -f "${dso}" ]]; then
        echo "Installed: ${dso}"
        ldd "${dso}" | grep -i -E "kimodo|ggml" || true
    fi
done
