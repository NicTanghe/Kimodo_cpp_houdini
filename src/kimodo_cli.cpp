#include <kimodo/kimodo_capi.h>
#include <kimodo_houdini/fk_skeleton.hpp>
#include <kimodo_houdini/motion_cache.hpp>

#include <cstdlib>
#include <iostream>
#include <string>
#include <string_view>

namespace {

void print_usage(const char *prog) {
    std::cerr
        << "Usage: " << prog << " [options]\n"
        << "Options:\n"
        << "  --model <path>         Path to motion GGUF model (required)\n"
        << "  --text-model <path>    Path to LLM2Vec text GGUF or bundle (optional; uses zero embedding if omitted)\n"
        << "  --prompt <text>        UTF-8 text prompt (default: \"A person walking forward and waving\")\n"
        << "  --frames <int>         Number of frames to generate (default: 60)\n"
        << "  --steps <int>          Diffusion steps (default: 50)\n"
        << "  --seed <uint64>        Random seed (default: 42)\n"
        << "  --fps <float>          Frame rate for GLB/BVH export (default: 30.0)\n"
        << "  --device <auto|cpu|vulkan> Compute backend (default: auto)\n"
        << "  --output-glb <path>    Write animated skeleton hierarchy to glTF 2.0 binary (.glb)\n"
        << "  --output-bvh <path>    Write animated skeleton hierarchy to Biovision Hierarchy (.bvh)\n";
}

} // namespace

int main(int argc, char **argv) {
    kimodo_houdini::ModelConfig model_cfg{};
    kimodo_houdini::GenerationParams gen_params{};
    gen_params.prompt = "A person walking forward and waving";
    gen_params.frames = 60;
    gen_params.diffusion_steps = 50;
    gen_params.seed = 42;

    float fps = 30.0f;
    std::string out_glb;
    std::string out_bvh;

    for (int i = 1; i < argc; ++i) {
        const std::string_view arg(argv[i]);
        auto next_val = [&](const char *flag) -> std::string {
            if (i + 1 >= argc) {
                std::cerr << "Error: missing value for " << flag << "\n";
                print_usage(argv[0]);
                std::exit(2);
            }
            return argv[++i];
        };

        if (arg == "--model") {
            model_cfg.motion_gguf = next_val("--model");
        } else if (arg == "--text-model") {
            model_cfg.text_gguf = next_val("--text-model");
        } else if (arg == "--prompt") {
            gen_params.prompt = next_val("--prompt");
        } else if (arg == "--frames") {
            gen_params.frames = static_cast<std::uint32_t>(std::stoul(next_val("--frames")));
        } else if (arg == "--steps") {
            gen_params.diffusion_steps = static_cast<std::uint32_t>(std::stoul(next_val("--steps")));
        } else if (arg == "--seed") {
            gen_params.seed = static_cast<std::uint64_t>(std::stoull(next_val("--seed")));
        } else if (arg == "--fps") {
            fps = std::stof(next_val("--fps"));
        } else if (arg == "--device") {
            const std::string dev = next_val("--device");
            if (dev == "cpu") {
                model_cfg.device = KIMODO_DEVICE_CPU;
            } else if (dev == "vulkan") {
                model_cfg.device = KIMODO_DEVICE_VULKAN;
            } else {
                model_cfg.device = KIMODO_DEVICE_AUTO;
            }
        } else if (arg == "--output-glb") {
            out_glb = next_val("--output-glb");
        } else if (arg == "--output-bvh") {
            out_bvh = next_val("--output-bvh");
        } else if (arg == "--help" || arg == "-h") {
            print_usage(argv[0]);
            return 0;
        } else {
            std::cerr << "Unknown argument: " << arg << "\n";
            print_usage(argv[0]);
            return 2;
        }
    }

    if (model_cfg.motion_gguf.empty()) {
        std::cerr << "Error: --model <path> is required.\n";
        print_usage(argv[0]);
        return 2;
    }

    kimodo_houdini::KimodoSessionCache cache;
    std::string err_msg;
    const kimodo_motion *motion = cache.get_or_generate(model_cfg, gen_params, err_msg);
    if (!motion) {
        std::cerr << "Generation failed: " << err_msg << "\n";
        return 1;
    }

    const int frames = kimodo_motion_frames(motion);
    const int joints = kimodo_motion_joints(motion);
    const float *roots = kimodo_motion_root_positions(motion);
    const float *rots = kimodo_motion_local_rotations_xyzw(motion);

    const kimodo_houdini::SkeletonDef *skel = kimodo_houdini::find_skeleton_by_joints(joints);
    if (!skel) {
        std::cerr << "Unsupported skeleton joint count: " << joints << "\n";
        return 1;
    }

    if (!out_glb.empty()) {
        if (!kimodo_houdini::write_skeleton_glb(out_glb, frames, joints, fps, roots, rots, *skel)) {
            std::cerr << "Failed to write GLB file: " << out_glb << "\n";
            return 1;
        }
        std::cout << "Exported GLB (" << frames << " frames, " << joints << " joints) -> " << out_glb << "\n";
    }

    if (!out_bvh.empty()) {
        if (!kimodo_houdini::write_skeleton_bvh(out_bvh, frames, joints, fps, roots, rots, *skel)) {
            std::cerr << "Failed to write BVH file: " << out_bvh << "\n";
            return 1;
        }
        std::cout << "Exported BVH (" << frames << " frames, " << joints << " joints) -> " << out_bvh << "\n";
    }

    return 0;
}
