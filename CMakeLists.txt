cmake_minimum_required(VERSION 3.25)
project(kimodo_houdini VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

set(KIMODO_CPP_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../kimodo.cpp"
    CACHE PATH "Path to localai-org/kimodo.cpp repository")
set(KIMODO_CPP_BUILD_DIR "${KIMODO_CPP_DIR}/build"
    CACHE PATH "Path to kimodo.cpp build directory containing ggml shared libraries")

if(NOT EXISTS "${KIMODO_CPP_DIR}/include/kimodo/kimodo_capi.h")
    message(FATAL_ERROR "kimodo.cpp headers not found at ${KIMODO_CPP_DIR}/include/kimodo/kimodo_capi.h")
endif()

set(GGML_LIB_DIR "${KIMODO_CPP_BUILD_DIR}/ggml/src")
set(GGML_VK_LIB_DIR "${KIMODO_CPP_BUILD_DIR}/ggml/src/ggml-vulkan")

find_library(GGML_LIB NAMES ggml PATHS "${GGML_LIB_DIR}" NO_DEFAULT_PATH REQUIRED)
find_library(GGML_BASE_LIB NAMES ggml-base PATHS "${GGML_LIB_DIR}" NO_DEFAULT_PATH REQUIRED)
find_library(GGML_CPU_LIB NAMES ggml-cpu PATHS "${GGML_LIB_DIR}" NO_DEFAULT_PATH REQUIRED)
find_library(GGML_VULKAN_LIB NAMES ggml-vulkan PATHS "${GGML_VK_LIB_DIR}" NO_DEFAULT_PATH)

# 1. Build position-independent shared library libkimodo.so for Houdini 21+ DSO linkage
add_library(kimodo_shared SHARED
    "${KIMODO_CPP_DIR}/src/capi.cpp"
    "${KIMODO_CPP_DIR}/src/model.cpp"
    "${KIMODO_CPP_DIR}/src/gguf.cpp"
    "${KIMODO_CPP_DIR}/src/diffusion.cpp"
    "${KIMODO_CPP_DIR}/src/motion_rep.cpp"
    "${KIMODO_CPP_DIR}/src/motion_decode.cpp"
    "${KIMODO_CPP_DIR}/src/ggml_weights.cpp"
    "${KIMODO_CPP_DIR}/src/denoiser.cpp"
    "${KIMODO_CPP_DIR}/src/sequence.cpp"
    "${KIMODO_CPP_DIR}/src/llm_tokenizer.cpp"
    "${KIMODO_CPP_DIR}/src/llm_text_encoder.cpp"
)
set_target_properties(kimodo_shared PROPERTIES
    OUTPUT_NAME "kimodo"
    CXX_VISIBILITY_PRESET hidden
    VISIBILITY_INLINES_HIDDEN YES
    BUILD_RPATH "${GGML_LIB_DIR};${GGML_VK_LIB_DIR}"
    INSTALL_RPATH "${GGML_LIB_DIR};${GGML_VK_LIB_DIR}"
)
target_include_directories(kimodo_shared
    PUBLIC
        "${KIMODO_CPP_DIR}/include"
    PRIVATE
        "${KIMODO_CPP_DIR}/src"
        "${KIMODO_CPP_DIR}/ggml/include"
)
target_compile_definitions(kimodo_shared PRIVATE
    KIMODO_BUILD
    KIMODO_SHARED
    KIMODO_HAVE_GGML=1
    GGML_SHARED
    GGML_BACKEND_SHARED
    GGML_USE_CPU
)
target_link_libraries(kimodo_shared PRIVATE
    "${GGML_LIB}"
    "${GGML_CPU_LIB}"
    "${GGML_BASE_LIB}"
)
if(GGML_VULKAN_LIB)
    target_compile_definitions(kimodo_shared PRIVATE
        KIMODO_HAVE_GGML_VULKAN=1
        GGML_USE_VULKAN
    )
    target_link_libraries(kimodo_shared PRIVATE "${GGML_VULKAN_LIB}")
endif()

# Header-only interface for FK solver, KineFX transforms, and session cache
add_library(kimodo_houdini_core INTERFACE)
target_include_directories(kimodo_houdini_core INTERFACE
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
    "${KIMODO_CPP_DIR}/include"
)
target_link_libraries(kimodo_houdini_core INTERFACE kimodo_shared)

# 2. Standalone C++ CLI executable (GLB & BVH export without Python)
add_executable(kimodo_cli src/kimodo_cli.cpp)
target_link_libraries(kimodo_cli PRIVATE kimodo_houdini_core)
set_target_properties(kimodo_cli PROPERTIES
    BUILD_RPATH "${CMAKE_CURRENT_BINARY_DIR};${GGML_LIB_DIR};${GGML_VK_LIB_DIR}"
)

# 3. Houdini 21+ HDK SOP Plugin (SOP_Kimodo.so)
if(DEFINED ENV{HFS} AND EXISTS "$ENV{HFS}/toolkit/cmake")
    list(APPEND CMAKE_PREFIX_PATH "$ENV{HFS}/toolkit/cmake")
else()
    file(GLOB HFS_DIRS "/opt/hfs*")
    list(SORT HFS_DIRS COMPARE NATURAL ORDER DESCENDING)
    foreach(HFS_CANDIDATE ${HFS_DIRS})
        get_filename_component(HFS_NAME "${HFS_CANDIDATE}" NAME)
        if(HFS_NAME MATCHES "^hfs([0-9]+)" AND CMAKE_MATCH_1 GREATER_EQUAL 21
           AND EXISTS "${HFS_CANDIDATE}/toolkit/cmake")
            list(APPEND CMAKE_PREFIX_PATH "${HFS_CANDIDATE}/toolkit/cmake")
        endif()
    endforeach()
endif()

find_package(Houdini QUIET)
if(Houdini_FOUND)
    message(STATUS "Found Houdini ${Houdini_VERSION} HDK -- configuring SOP_Kimodo DSO")
    add_library(SOP_Kimodo SHARED
        src/SOP_Kimodo.C
        src/SOP_Kimodo.h
    )
    target_link_libraries(SOP_Kimodo PRIVATE
        Houdini
        kimodo_houdini_core
    )
    set_target_properties(SOP_Kimodo PROPERTIES
        BUILD_RPATH "${CMAKE_CURRENT_BINARY_DIR};${GGML_LIB_DIR};${GGML_VK_LIB_DIR}"
        INSTALL_RPATH "${CMAKE_CURRENT_BINARY_DIR};${GGML_LIB_DIR};${GGML_VK_LIB_DIR}"
    )
    set(KIMODO_DSO_DIR "" CACHE PATH "Override Houdini DSO output directory (empty uses Houdini's user directory)")
    if(KIMODO_DSO_DIR)
        houdini_configure_target(SOP_Kimodo INSTDIR "${KIMODO_DSO_DIR}")
    else()
        houdini_configure_target(SOP_Kimodo)
    endif()
else()
    message(STATUS "Houdini 21+ HDK not found in environment (set HFS to your Houdini installation or run scripts/build_hcustom.sh)")
endif()

# 4. C++ Verification Tests
enable_testing()
add_executable(kimodo_houdini_test tests/test_pipeline.cpp)
target_link_libraries(kimodo_houdini_test PRIVATE kimodo_houdini_core)
# The tests use assert; keep checks enabled in Release and RelWithDebInfo builds.
target_compile_options(kimodo_houdini_test PRIVATE $<$<CXX_COMPILER_ID:GNU,Clang>:-UNDEBUG>)
set_target_properties(kimodo_houdini_test PROPERTIES
    BUILD_RPATH "${CMAKE_CURRENT_BINARY_DIR};${GGML_LIB_DIR};${GGML_VK_LIB_DIR}"
)
add_test(NAME kimodo_houdini_test COMMAND kimodo_houdini_test)
