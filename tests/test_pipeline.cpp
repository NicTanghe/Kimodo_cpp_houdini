#include <kimodo/kimodo_capi.h>
#include <kimodo_houdini/constraints.hpp>
#include <kimodo_houdini/fk_skeleton.hpp>
#include <kimodo_houdini/motion_cache.hpp>

#include <cassert>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

namespace {

bool approx_eq(float a, float b, float eps = 1e-4f) {
    return std::abs(a - b) <= eps;
}

void test_capi_linkage() {
    const int abi = kimodo_abi_version();
    assert(abi == KIMODO_CAPI_ABI_VERSION);

    kimodo_houdini::KimodoSessionCache cache;
    kimodo_houdini::ModelConfig cfg{};
    cfg.motion_gguf = "/nonexistent/model.gguf";
    kimodo_houdini::GenerationParams params{};
    std::string err;
    const kimodo_motion *m = cache.get_or_generate(cfg, params, err);
    assert(m == nullptr);
    assert(!err.empty());
    std::cout << "[PASS] C-API linkage & error firewall check (ABI v" << abi << ")\n";
}

void test_soma30_fk_and_exports() {
    const auto *skel = kimodo_houdini::find_skeleton_by_joints(30);
    assert(skel != nullptr);
    assert(skel->key == "soma30");
    assert(skel->joint_count() == 30);

    constexpr int kFrames = 10;
    constexpr int kJoints = 30;
    std::vector<float> roots(static_cast<std::size_t>(kFrames * 3), 0.0f);
    std::vector<float> rots(static_cast<std::size_t>(kFrames * kJoints * 4), 0.0f);

    for (int f = 0; f < kFrames; ++f) {
        roots[static_cast<std::size_t>(f * 3 + 0)] = 0.1f * static_cast<float>(f);
        roots[static_cast<std::size_t>(f * 3 + 1)] = 0.95f;
        roots[static_cast<std::size_t>(f * 3 + 2)] = 0.2f * static_cast<float>(f);
        for (int j = 0; j < kJoints; ++j) {
            rots[static_cast<std::size_t>((f * kJoints + j) * 4 + 3)] = 1.0f; // identity quat [0,0,0,1]
        }
    }

    const auto poses = kimodo_houdini::evaluate_frame_fk(
        5, kFrames, kJoints, roots.data(), rots.data(), skel);
    assert(poses.size() == 30);

    // Frame 5 root position should be (0.5, 0.95, 1.0)
    assert(approx_eq(poses[0].world_pos.x, 0.5f));
    assert(approx_eq(poses[0].world_pos.y, 0.95f));
    assert(approx_eq(poses[0].world_pos.z, 1.0f));

    // Spine1 (joint 1, parent 0) with identity rotation should be root + offset[1]
    assert(approx_eq(poses[1].world_pos.x, 0.5f + skel->offsets[1][0]));
    assert(approx_eq(poses[1].world_pos.y, 0.95f + skel->offsets[1][1]));
    assert(approx_eq(poses[1].world_pos.z, 1.0f + skel->offsets[1][2]));

    // Check KineFX 4x4 localtransform translation row (row 3)
    assert(approx_eq(poses[0].local_mat4[12], 0.5f));
    assert(approx_eq(poses[0].local_mat4[13], 0.95f));
    assert(approx_eq(poses[0].local_mat4[14], 1.0f));
    assert(approx_eq(poses[0].local_mat4[15], 1.0f));

    // Test GLB and BVH native C++ exporters
    const std::string glb_path = "/tmp/kimodo_houdini_test.glb";
    const std::string bvh_path = "/tmp/kimodo_houdini_test.bvh";

    const bool glb_ok = kimodo_houdini::write_skeleton_glb(
        glb_path, kFrames, kJoints, 30.0f, roots.data(), rots.data(), *skel);
    assert(glb_ok);
    assert(std::filesystem::exists(glb_path));
    assert(std::filesystem::file_size(glb_path) > 100);

    std::ifstream glb_in(glb_path, std::ios::binary);
    char magic[4]{};
    glb_in.read(magic, 4);
    assert(magic[0] == 'g' && magic[1] == 'l' && magic[2] == 'T' && magic[3] == 'F');
    glb_in.close();
    std::filesystem::remove(glb_path);

    const bool bvh_ok = kimodo_houdini::write_skeleton_bvh(
        bvh_path, kFrames, kJoints, 30.0f, roots.data(), rots.data(), *skel);
    assert(bvh_ok);
    assert(std::filesystem::exists(bvh_path));
    assert(std::filesystem::file_size(bvh_path) > 100);
    std::filesystem::remove(bvh_path);

    std::cout << "[PASS] SOMA-30 Forward Kinematics, KineFX matrix layout, GLB & BVH export\n";
}

void test_constraint_features() {
    using namespace kimodo_houdini;
    assert((parse_capture_frames("10 12-16:2, 20",10,11)==std::vector<int>{10,12,14,16,20}));
    auto rejects=[](auto fn) {bool failed=false;try{fn();}catch(const std::runtime_error&){failed=true;}assert(failed);};
    rejects([]{parse_capture_frames("0",1,30);});
    rejects([]{parse_capture_frames("1-10:0",1,30);});
    const auto *s=find_skeleton_by_joints(30);
    auto sparse=make_conditioning(*s,30,{},{{0,1,2},{29,4,8}});
    assert(sparse.mask[0]==1 && sparse.mask[2]==1 && sparse.mask[1]==0);
    assert(sparse.values[0]==1 && sparse.values[2]==2);
    auto dense=make_conditioning(*s,3,{},{{0,0,0},{2,2,4}},true);
    const size_t D=369;
    assert(dense.values[D]==1 && dense.values[D+2]==2 && dense.mask[D]==1);
    rejects([&]{make_conditioning(*s,3,{},{{1,0,0},{2,2,4}},true);});
    rejects([&]{make_conditioning(*s,3,{},{{0,0,0},{0,2,4}});});
    PoseCapture pose;pose.frame=0;pose.positions.resize(30);pose.selected.assign(30,true);
    pose.positions[0]={3,1,4};pose.positions[22]={3.1f,.9f,4};pose.positions[26]={2.9f,.9f,4};
    pose.positions[13]={4,2,5};
    auto full=make_conditioning(*s,3,{pose},{});
    assert(full.values[0]==3 && full.values[1]==1 && full.values[2]==4);
    assert(full.values[5+13*3]==1 && full.values[6+13*3]==2 && full.values[7+13*3]==1);
    assert(full.mask[5+30*3]==0); // Full-body conditions use positions, not joint rotations.
    pose.selected.assign(30,false);pose.selected[13]=true;pose.rotations=true;pose.transforms.resize(30);
    pose.transforms[13]={0,0,-1,0,1,0,1,0,0};
    auto partial=make_conditioning(*s,3,{pose},{});
    assert(partial.mask[5]==0 && partial.mask[5+13*3]==1);
    assert(partial.values[5+90+13*6+2]==-1 && partial.mask[5+90+13*6+2]==1);
    rejects([&]{make_conditioning(*s,3,{pose},{{0,0,0}});});
    std::cout << "[PASS] Frame ranges, waypoint timing, dense paths, full/partial pose features, and conflict checks\n";
}

} // namespace

int main() {
    test_constraint_features();
    test_capi_linkage();
    test_soma30_fk_and_exports();
    std::cout << "All kimodo_houdini C++ tests passed.\n";
    return 0;
}
