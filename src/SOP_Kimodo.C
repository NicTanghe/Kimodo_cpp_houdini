#include <UT/UT_DSOVersion.h>
#include <PRM/PRM_Include.h>
#include <PRM/PRM_SpareData.h>
#include <OP/OP_Operator.h>
#include <OP/OP_OperatorTable.h>
#include <SOP/SOP_Node.h>
#include <GU/GU_Detail.h>
#include <GEO/GEO_PrimPoly.h>
#include <GA/GA_Handle.h>
#include <UT/UT_Matrix3.h>
#include <UT/UT_Matrix4.h>
#include <UT/UT_Vector3.h>
#include <UT/UT_Vector4.h>
#include <SYS/SYS_Math.h>

#include "SOP_Kimodo.h"
#include <kimodo/kimodo_capi.h>
#include <kimodo_houdini/fk_skeleton.hpp>

#include <string>
#include <vector>

// Compatibility aliases matching the PDF reference snippet naming
using GA_RWHandle_S = GA_RWHandleS;
using GA_RWHandle_V4 = GA_RWHandleV4;

// Parameter UI definitions
static PRM_Name prmPrompt("prompt", "Text Prompt");
static PRM_Default prmPromptDef(0.0, "A person walking forward and waving");

static PRM_Name prmFrames("frames", "Frame Count");
static PRM_Default prmFramesDef(60);
static PRM_Range prmFramesRange(PRM_RANGE_RESTRICTED, 1, PRM_RANGE_UI, 300);

static PRM_Name prmSteps("steps", "Diffusion Steps");
static PRM_Default prmStepsDef(50);
static PRM_Range prmStepsRange(PRM_RANGE_RESTRICTED, 1, PRM_RANGE_UI, 100);

static PRM_Name prmSeed("seed", "Seed");
static PRM_Default prmSeedDef(42);

static PRM_Name prmTextCfg("text_cfg", "Text CFG Weight");
static PRM_Default prmTextCfgDef(2.0f);
static PRM_Range prmCfgRange(PRM_RANGE_RESTRICTED, 0.0f, PRM_RANGE_UI, 10.0f);

static PRM_Name prmConstraintCfg("constraint_cfg", "Constraint CFG Weight");
static PRM_Default prmConstraintCfgDef(2.0f);

static PRM_Name prmMotionModel("motion_model", "Motion GGUF Path");
static PRM_Default prmMotionModelDef(0.0, "/opt/kimodo.cpp/models/kimodo-soma-rp-v1.1-f32.gguf");

static PRM_Name prmTextModel("text_model", "Text Encoder GGUF / Bundle");
static PRM_Default prmTextModelDef(0.0, "/opt/kimodo.cpp/models/llm2vec-q8_0.gguf");

static PRM_Name prmDevice("device", "Compute Backend");
static PRM_Default prmDeviceDef(0); // 0 = Auto, 1 = CPU, 2 = Vulkan
static PRM_Name prmDeviceChoices[] = {
    PRM_Name("auto", "Auto (Vulkan / CPU)"),
    PRM_Name("cpu", "CPU"),
    PRM_Name("vulkan", "Vulkan GPU"),
    PRM_Name(nullptr)
};
static PRM_ChoiceList prmDeviceMenu(PRM_CHOICELIST_SINGLE, prmDeviceChoices);

static PRM_Name prmConnectBones("connect_bones", "Create KineFX Bone Lines");
static PRM_Default prmConnectBonesDef(1);

PRM_Template SOP_Kimodo::myTemplateList[] = {
    PRM_Template(PRM_STRING, 1, &prmPrompt, &prmPromptDef),
    PRM_Template(PRM_INT_J,  1, &prmFrames, &prmFramesDef, nullptr, &prmFramesRange),
    PRM_Template(PRM_INT_J,  1, &prmSteps,  &prmStepsDef,  nullptr, &prmStepsRange),
    PRM_Template(PRM_INT_J,  1, &prmSeed,   &prmSeedDef),
    PRM_Template(PRM_FLT_J,  1, &prmTextCfg, &prmTextCfgDef, nullptr, &prmCfgRange),
    PRM_Template(PRM_FLT_J,  1, &prmConstraintCfg, &prmConstraintCfgDef, nullptr, &prmCfgRange),
    PRM_Template(PRM_FILE,   1, &prmMotionModel, &prmMotionModelDef),
    PRM_Template(PRM_FILE,   1, &prmTextModel,   &prmTextModelDef),
    PRM_Template(PRM_ORD,    1, &prmDevice,      &prmDeviceDef, &prmDeviceMenu),
    PRM_Template(PRM_TOGGLE, 1, &prmConnectBones, &prmConnectBonesDef),
    PRM_Template()
};

OP_Node *SOP_Kimodo::myConstructor(OP_Network *net, const char *name, OP_Parameters *params) {
    return new SOP_Kimodo(net, name, params);
}

SOP_Kimodo::SOP_Kimodo(OP_Network *net, const char *name, OP_Parameters *params)
    : SOP_Node(net, name, params) {}

SOP_Kimodo::~SOP_Kimodo() = default;

OP_ERROR SOP_Kimodo::cookMySop(OP_Context &context) {
    // Flag node as time-dependent across the Houdini timeline
    flags().setTimeDep(true);
    gdp->clearAndDestroy();

    const fpreal t = context.getTime();
    const fpreal curr_frame = context.getFrame();

    UT_String prompt_ut;
    evalString(prompt_ut, "prompt", 0, t);

    UT_String motion_model_ut;
    evalString(motion_model_ut, "motion_model", 0, t);

    UT_String text_model_ut;
    evalString(text_model_ut, "text_model", 0, t);

    const int total_frames = SYSmax(1, static_cast<int>(evalInt("frames", 0, t)));
    const int steps = SYSmax(1, static_cast<int>(evalInt("steps", 0, t)));
    const int seed = static_cast<int>(evalInt("seed", 0, t));
    const float text_cfg = static_cast<float>(evalFloat("text_cfg", 0, t));
    const float constraint_cfg = static_cast<float>(evalFloat("constraint_cfg", 0, t));
    const int device_ord = static_cast<int>(evalInt("device", 0, t));
    const bool connect_bones = evalInt("connect_bones", 0, t) != 0;

    kimodo_houdini::ModelConfig model_cfg{};
    model_cfg.motion_gguf = motion_model_ut.c_str() ? motion_model_ut.c_str() : "";
    model_cfg.text_gguf = text_model_ut.c_str() ? text_model_ut.c_str() : "";
    if (device_ord == 1) {
        model_cfg.device = KIMODO_DEVICE_CPU;
    } else if (device_ord == 2) {
        model_cfg.device = KIMODO_DEVICE_VULKAN;
    } else {
        model_cfg.device = KIMODO_DEVICE_AUTO;
    }

    kimodo_houdini::GenerationParams gen_params{};
    gen_params.prompt = prompt_ut.c_str() ? prompt_ut.c_str() : "";
    gen_params.frames = static_cast<std::uint32_t>(total_frames);
    gen_params.diffusion_steps = static_cast<std::uint32_t>(steps);
    gen_params.seed = static_cast<std::uint64_t>(seed >= 0 ? seed : 0);
    gen_params.text_cfg_weight = text_cfg;
    gen_params.constraint_cfg_weight = constraint_cfg;

    // 1 & 2. Lazy-load GGUF weights and run/retrieve cached diffusion motion
    std::string err_msg;
    const kimodo_motion *motion = mySessionCache.get_or_generate(model_cfg, gen_params, err_msg);
    if (!motion) {
        addError(SOP_MESSAGE, err_msg.c_str());
        return error();
    }

    const int out_frames = kimodo_motion_frames(motion);
    const int num_joints = kimodo_motion_joints(motion);
    const float *root_positions = kimodo_motion_root_positions(motion);
    const float *local_rotations = kimodo_motion_local_rotations_xyzw(motion);

    if (out_frames <= 0 || num_joints <= 0 || !root_positions || !local_rotations) {
        addError(SOP_MESSAGE, "kimodo_motion returned empty frame or joint buffers");
        return error();
    }

    const int frame_idx = SYSclamp(static_cast<int>(curr_frame) - 1, 0, out_frames - 1);
    const kimodo_houdini::SkeletonDef *skel = kimodo_houdini::find_skeleton_by_joints(num_joints);

    const std::vector<kimodo_houdini::JointPose> poses = kimodo_houdini::evaluate_frame_fk(
        frame_idx, out_frames, num_joints, root_positions, local_rotations, skel);

    // 3. Register KineFX attributes on geometry (Houdini 21+ KineFX standard)
    GA_RWHandle_S name_handle(gdp->addStringTuple(GA_ATTRIB_POINT, "name", 1));
    GA_RWHandle_V4 rot_handle(gdp->addFloatTuple(GA_ATTRIB_POINT, "rot", 4)); // Quat [x, y, z, w]
    GA_RWHandleM3 transform_handle(gdp->addFloatTuple(GA_ATTRIB_POINT, "transform", 9)); // 3x3 world rotation
    GA_RWHandleM4 localtransform_handle(gdp->addFloatTuple(GA_ATTRIB_POINT, "localtransform", 16)); // 4x4 local transform
    GA_RWHandleI parent_handle(gdp->addIntTuple(GA_ATTRIB_POINT, "parent", 1));

    // 4. Construct skeleton points in native gdp (GU_Detail)
    std::vector<GA_Offset> joint_ptoffs(static_cast<std::size_t>(num_joints));
    for (int j = 0; j < num_joints; ++j) {
        const auto sj = static_cast<std::size_t>(j);
        const GA_Offset ptoff = gdp->appendPoint();
        joint_ptoffs[sj] = ptoff;

        const kimodo_houdini::JointPose &pose = poses[sj];

        // World-space joint position P
        gdp->setPos3(ptoff, UT_Vector3(pose.world_pos.x, pose.world_pos.y, pose.world_pos.z));

        // Joint name s@name (for KineFX identification & retargeting)
        if (skel && sj < skel->joint_count()) {
            std::string jname(skel->names[sj]);
            name_handle.set(ptoff, jname.c_str());
        } else {
            std::string fallback_name = "joint_" + std::to_string(j);
            name_handle.set(ptoff, fallback_name.c_str());
        }

        // Local joint rotation quaternion p@rot [x, y, z, w]
        rot_handle.set(
            ptoff,
            UT_Vector4(pose.local_rot.x, pose.local_rot.y, pose.local_rot.z, pose.local_rot.w));

        // KineFX 3@transform (world 3x3 orientation)
        UT_Matrix3F world_m3(
            pose.world_mat3[0], pose.world_mat3[1], pose.world_mat3[2],
            pose.world_mat3[3], pose.world_mat3[4], pose.world_mat3[5],
            pose.world_mat3[6], pose.world_mat3[7], pose.world_mat3[8]);
        transform_handle.set(ptoff, world_m3);

        // KineFX 4@localtransform (parent-relative 4x4 transform)
        UT_Matrix4F local_m4(
            pose.local_mat4[0],  pose.local_mat4[1],  pose.local_mat4[2],  pose.local_mat4[3],
            pose.local_mat4[4],  pose.local_mat4[5],  pose.local_mat4[6],  pose.local_mat4[7],
            pose.local_mat4[8],  pose.local_mat4[9],  pose.local_mat4[10], pose.local_mat4[11],
            pose.local_mat4[12], pose.local_mat4[13], pose.local_mat4[14], pose.local_mat4[15]);
        localtransform_handle.set(ptoff, local_m4);

        const int parent_idx = (skel && sj < skel->joint_count()) ? skel->parents[sj] : (j > 0 ? j - 1 : -1);
        parent_handle.set(ptoff, parent_idx);
    }

    // 5. Connect joints with primitive lines (Houdini standard KineFX bone hierarchy display)
    if (connect_bones && skel) {
        for (int j = 0; j < num_joints; ++j) {
            const auto sj = static_cast<std::size_t>(j);
            if (sj >= skel->joint_count()) continue;
            const int parent_idx = skel->parents[sj];
            if (parent_idx >= 0 && parent_idx < num_joints) {
                GEO_PrimPoly *bone_line = GEO_PrimPoly::build(gdp, 2, GU_POLY_OPEN);
                bone_line->setPointOffset(0, joint_ptoffs[static_cast<std::size_t>(parent_idx)]);
                bone_line->setPointOffset(1, joint_ptoffs[sj]);
            }
        }
    }

    return error();
}

// Houdini 21+ operator registration entry point
void newSopOperator(OP_OperatorTable *table) {
    table->addOperator(new OP_Operator(
        "sop_kimodo_native",
        "Kimodo Native",
        SOP_Kimodo::myConstructor,
        SOP_Kimodo::myTemplateList,
        0,
        0 // Pure text-to-motion generator SOP requires 0 geometric inputs
    ));
}
