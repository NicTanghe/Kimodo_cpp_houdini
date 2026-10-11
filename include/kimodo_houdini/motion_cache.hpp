#pragma once

#include <kimodo/kimodo_capi.h>

#include <array>
#include <cmath>
#include <cstdint>
#include <mutex>
#include <string>

namespace kimodo_houdini {

struct ModelConfig {
    std::string motion_gguf;
    std::string text_gguf;
    kimodo_device device = KIMODO_DEVICE_AUTO;
    std::uint32_t threads = 0;

    bool operator==(const ModelConfig &other) const noexcept {
        return motion_gguf == other.motion_gguf &&
               text_gguf == other.text_gguf &&
               device == other.device &&
               threads == other.threads;
    }
    bool operator!=(const ModelConfig &other) const noexcept {
        return !(*this == other);
    }
};

struct GenerationParams {
    std::string prompt;
    std::uint32_t frames = 60;
    std::uint32_t diffusion_steps = 50;
    std::uint64_t seed = 42;
    float text_cfg_weight = 2.0f;
    float constraint_cfg_weight = 2.0f;

    bool operator==(const GenerationParams &other) const noexcept {
        return prompt == other.prompt &&
               frames == other.frames &&
               diffusion_steps == other.diffusion_steps &&
               seed == other.seed &&
               std::abs(text_cfg_weight - other.text_cfg_weight) <= 1e-6f &&
               std::abs(constraint_cfg_weight - other.constraint_cfg_weight) <= 1e-6f;
    }
    bool operator!=(const GenerationParams &other) const noexcept {
        return !(*this == other);
    }
};

// RAII session & motion cache for SOP_Kimodo.
// Ensures model weights are loaded once into RAM/VRAM and generated motion clips
// are cached across Houdini timeline frame cooks unless parameters change.
class KimodoSessionCache {
public:
    KimodoSessionCache() = default;

    ~KimodoSessionCache() {
        reset();
    }

    KimodoSessionCache(const KimodoSessionCache &) = delete;
    KimodoSessionCache &operator=(const KimodoSessionCache &) = delete;

    void reset() noexcept {
        std::lock_guard<std::mutex> lock(mutex_);
        free_motion_unlocked();
        free_model_unlocked();
    }

    void invalidate_motion() noexcept {
        std::lock_guard<std::mutex> lock(mutex_);
        free_motion_unlocked();
    }

    // Ensures the model is loaded and returns a cached or newly generated kimodo_motion*.
    // The returned pointer remains owned by KimodoSessionCache and valid until the next call.
    const kimodo_motion *get_or_generate(
        const ModelConfig &model_cfg,
        const GenerationParams &gen_params,
        std::string &out_error) {

        std::lock_guard<std::mutex> lock(mutex_);

        if (!ensure_model_loaded_unlocked(model_cfg, out_error)) {
            return nullptr;
        }

        if (motion_ && has_cached_params_ && cached_gen_params_ == gen_params) {
            return motion_;
        }

        free_motion_unlocked();

        kimodo_generation_options opts{};
        opts.size = sizeof(kimodo_generation_options);
        opts.seed = gen_params.seed;
        opts.frames = gen_params.frames;
        opts.diffusion_steps = gen_params.diffusion_steps;
        opts.text_cfg_weight = gen_params.text_cfg_weight;
        opts.constraint_cfg_weight = gen_params.constraint_cfg_weight;

        char err_buf[1024]{};
        if (!model_cfg.text_gguf.empty()) {
            motion_ = kimodo_generate(
                model_,
                gen_params.prompt.c_str(),
                &opts,
                err_buf,
                static_cast<int>(sizeof(err_buf)));
        } else {
            // Denoiser-only fallback with deterministic zero embedding when no text GGUF is set
            std::array<float, 4096> zero_emb{};
            kimodo_embedding emb{zero_emb.data(), static_cast<std::uint32_t>(zero_emb.size())};
            motion_ = kimodo_generate_embedding(
                model_,
                &emb,
                &opts,
                err_buf,
                static_cast<int>(sizeof(err_buf)));
        }

        if (!motion_) {
            out_error = err_buf[0] != '\0' ? std::string(err_buf) : "kimodo inference execution failed";
            return nullptr;
        }

        cached_gen_params_ = gen_params;
        has_cached_params_ = true;
        return motion_;
    }

private:
    bool ensure_model_loaded_unlocked(const ModelConfig &cfg, std::string &out_error) {
        if (model_ && has_loaded_cfg_ && loaded_model_cfg_ == cfg) {
            return true;
        }

        free_motion_unlocked();
        free_model_unlocked();

        kimodo_runtime_options rt_opts{};
        rt_opts.size = sizeof(kimodo_runtime_options);
        rt_opts.threads = cfg.threads;
        rt_opts.device = cfg.device;
        rt_opts.backend_dir = nullptr;

        char err_buf[1024]{};
        model_ = kimodo_model_load(
            cfg.motion_gguf.c_str(),
            cfg.text_gguf.empty() ? nullptr : cfg.text_gguf.c_str(),
            nullptr,
            &rt_opts,
            err_buf,
            static_cast<int>(sizeof(err_buf)));

        if (!model_) {
            out_error = err_buf[0] != '\0' ? std::string(err_buf) : "Failed to load kimodo GGUF model";
            return false;
        }

        loaded_model_cfg_ = cfg;
        has_loaded_cfg_ = true;
        return true;
    }

    void free_motion_unlocked() noexcept {
        if (motion_) {
            kimodo_motion_free(motion_);
            motion_ = nullptr;
        }
        has_cached_params_ = false;
    }

    void free_model_unlocked() noexcept {
        if (model_) {
            kimodo_model_free(model_);
            model_ = nullptr;
        }
        has_loaded_cfg_ = false;
    }

    std::mutex mutex_;
    kimodo_model *model_ = nullptr;
    kimodo_motion *motion_ = nullptr;
    ModelConfig loaded_model_cfg_{};
    GenerationParams cached_gen_params_{};
    bool has_loaded_cfg_ = false;
    bool has_cached_params_ = false;
};

} // namespace kimodo_houdini
