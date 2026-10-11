#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <span>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace kimodo_houdini {

struct Vec3 {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
};

// Quaternion stored in XYZW order (matching kimodo.cpp and Houdini UT_Quaternion / p@rot)
struct QuatXYZW {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
    float w = 1.0f;
};

struct JointPose {
    Vec3 local_pos{};
    QuatXYZW local_rot{};
    Vec3 world_pos{};
    QuatXYZW world_rot{};
    // Row-vector convention (Houdini UT_Matrix3 / KineFX 3@transform): v_world = v_local * R
    std::array<float, 9> world_mat3{};
    // Row-vector convention (Houdini UT_Matrix4 / KineFX 4@localtransform)
    std::array<float, 16> local_mat4{};
};

struct SkeletonDef {
    std::string_view key;
    std::span<const std::string_view> names;
    std::span<const int> parents;
    std::span<const std::array<float, 3>> offsets;

    [[nodiscard]] constexpr std::size_t joint_count() const noexcept {
        return names.size();
    }
};

// 22-joint SMPL-X skeleton
inline constexpr std::array<std::string_view, 22> kSmplx22Names{
    "pelvis",         "left_hip",       "right_hip",     "spine1",
    "left_knee",      "right_knee",     "spine2",        "left_ankle",
    "right_ankle",    "spine3",         "left_foot",     "right_foot",
    "neck",           "left_collar",    "right_collar",  "head",
    "left_shoulder",  "right_shoulder", "left_elbow",    "right_elbow",
    "left_wrist",     "right_wrist"};

inline constexpr std::array<int, 22> kSmplx22Parents{
    -1, 0, 0, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 9, 9, 12, 13, 14, 16, 17, 18, 19};

inline constexpr std::array<std::array<float, 3>, 22> kSmplx22Offsets{{
    {0.0f, 0.0f, 0.0f},
    {0.052299179f, -0.093935639f, -0.027606763f},
    {-0.057192899f, -0.106548190f, -0.022217851f},
    {-0.001495834f, 0.112929940f, -0.024981268f},
    {0.058866613f, -0.416441321f, -0.006556974f},
    {-0.048074268f, -0.397559673f, -0.014061437f},
    {0.006900469f, 0.145636231f, -0.006858510f},
    {-0.041737989f, -0.437583506f, -0.029511765f},
    {0.014489345f, -0.446852267f, -0.018029511f},
    {-0.010334037f, 0.056081813f, 0.021115851f},
    {0.049293540f, -0.065279245f, 0.126259089f},
    {-0.040575184f, -0.065286517f, 0.127075911f},
    {-0.011025756f, 0.171365142f, -0.028827066f},
    {0.047724526f, 0.087643057f, -0.008375450f},
    {-0.046636276f, 0.086612143f, -0.014864366f},
    {0.024654359f, 0.175390735f, 0.024463326f},
    {0.126284808f, 0.057680372f, -0.013885141f},
    {-0.109341696f, 0.053674292f, -0.009117880f},
    {0.272907287f, -0.069853373f, -0.039094493f},
    {-0.292028785f, -0.035440356f, -0.024564851f},
    {0.276173830f, 0.021254137f, -0.002478220f},
    {-0.271878421f, -0.004834589f, -0.016445294f}}};

// 30-joint SOMA control skeleton (default for Kimodo SOMA-RP / SOMA-SEED v1.1)
inline constexpr std::array<std::string_view, 30> kSoma30Names{
    "Hips",              "Spine1",          "Spine2",
    "Chest",             "Neck1",           "Neck2",
    "Head",              "Jaw",             "LeftEye",
    "RightEye",          "LeftShoulder",    "LeftArm",
    "LeftForeArm",       "LeftHand",        "LeftHandThumbEnd",
    "LeftHandMiddleEnd", "RightShoulder",   "RightArm",
    "RightForeArm",      "RightHand",       "RightHandThumbEnd",
    "RightHandMiddleEnd","LeftLeg",         "LeftShin",
    "LeftFoot",          "LeftToeBase",     "RightLeg",
    "RightShin",         "RightFoot",       "RightToeBase"};

inline constexpr std::array<int, 30> kSoma30Parents{
    -1, 0,  1,  2,  3,  4,  5,  6,  6,  6,  3,  10, 11, 12, 13,
    13, 3,  16, 17, 18, 19, 19, 0,  22, 23, 24, 0,  26, 27, 28};

inline constexpr std::array<std::array<float, 3>, 30> kSoma30Offsets{{
    {0.0f, 0.0f, 0.0f},
    {-0.00013727f, 0.0500376256f, -0.00053726669f},
    {-1.86574103e-9f, 0.0712530139f, -0.000298248546f},
    {-5.75188398e-9f, 0.0755006305f, -0.00815970992f},
    {-0.00181676517f, 0.263112953f, -0.00553348292f},
    {-2.85102231e-8f, 0.0770939664f, 0.0230258546f},
    {-4.5975437e-8f, 0.0612891595f, 0.0195370861f},
    {2.63687901e-5f, 0.0047559225f, 0.0309494062f},
    {0.0320638079f, 0.0538020513f, 0.0758688308f},
    {-0.0322244017f, 0.05361869f, 0.0755823359f},
    {0.0162165175f, 0.232371641f, 0.0511341324f},
    {0.149198457f, 2.19397873e-8f, -0.0550232576f},
    {0.287393078f, 2.50268389e-9f, -2.58787737e-5f},
    {0.270939812f, -7.06625108e-9f, 2.60897248e-5f},
    {0.122686267f, -0.0322017573f, 0.0483306876f},
    {0.190119595f, -0.00312878387f, -0.000339570373f},
    {-0.0138011824f, 0.231803086f, 0.0521415786f},
    {-0.150371962f, 1.17387901e-7f, -0.0554560437f},
    {-0.287366393f, 1.87628082e-8f, -2.59709359e-5f},
    {-0.271336198f, -1.16767401e-9f, 2.61269368e-5f},
    {-0.122642483f, -0.0321145448f, 0.0480403904f},
    {-0.190005945f, -0.00306615542f, -0.0003157343f},
    {0.10043214f, -0.0843452671f, 0.0259565473f},
    {-1e-8f, -0.432217537f, -0.00802912805f},
    {1e-8f, -0.421550959f, -0.0348152298f},
    {0.0f, -0.0505947206f, 0.132315294f},
    {-0.10047278f, -0.0829525995f, 0.0262031695f},
    {1e-8f, -0.433622059f, -0.00805555828f},
    {2e-8f, -0.421173943f, -0.0347839785f},
    {-3.42907669e-9f, -0.0507960932f, 0.132841956f}}};

// 34-joint Unitree G1 humanoid robot skeleton
inline constexpr std::array<std::string_view, 34> kG1Skel34Names{
    "pelvis_skel",              "left_hip_pitch_skel",      "left_hip_roll_skel",
    "left_hip_yaw_skel",        "left_knee_skel",           "left_ankle_pitch_skel",
    "left_ankle_roll_skel",     "left_toe_base",            "right_hip_pitch_skel",
    "right_hip_roll_skel",      "right_hip_yaw_skel",       "right_knee_skel",
    "right_ankle_pitch_skel",   "right_ankle_roll_skel",    "right_toe_base",
    "waist_yaw_skel",           "waist_roll_skel",          "waist_pitch_skel",
    "left_shoulder_pitch_skel", "left_shoulder_roll_skel",  "left_shoulder_yaw_skel",
    "left_elbow_skel",          "left_wrist_roll_skel",     "left_wrist_pitch_skel",
    "left_wrist_yaw_skel",      "left_hand_roll_skel",      "right_shoulder_pitch_skel",
    "right_shoulder_roll_skel", "right_shoulder_yaw_skel",  "right_elbow_skel",
    "right_wrist_roll_skel",    "right_wrist_pitch_skel",   "right_wrist_yaw_skel",
    "right_hand_roll_skel"};

inline constexpr std::array<int, 34> kG1Skel34Parents{
    -1, 0,  1,  2,  3,  4,  5,  6,  0,  8,  9,  10, 11, 12, 13, 0,  15,
    16, 17, 18, 19, 20, 21, 22, 23, 24, 17, 26, 27, 28, 29, 30, 31, 32};

inline constexpr std::array<std::array<float, 3>, 34> kG1Skel34Offsets{{
    {0.0f, 0.0f, 0.0f},         {0.064452f, -0.1027f, 0.0f},
    {0.052f, -0.030465f, 0.0f}, {0.0f, -0.12412f, 0.025001f},
    {0.0021489f, -0.17734f, -0.078273f}, {-0.000094445f, -0.30001f, 0.0f},
    {0.0f, -0.017558f, 0.0f},   {0.0f, -0.035f, 0.14f},
    {-0.064452f, -0.1027f, 0.0f}, {-0.052f, -0.030465f, 0.0f},
    {0.0f, -0.12412f, 0.025001f}, {-0.0021489f, -0.17734f, -0.078273f},
    {0.000094445f, -0.30001f, 0.0f}, {0.0f, -0.017558f, 0.0f},
    {0.0f, -0.035f, 0.14f},     {0.0f, 0.0f, 0.0f},
    {0.0f, 0.044f, -0.0039635f},{0.0f, 0.0f, 0.0f},
    {0.10022f, 0.24778f, 0.0039563f}, {0.038f, -0.013831f, 0.0f},
    {0.00624f, -0.1032f, 0.0f}, {0.0f, -0.080518f, 0.015783f},
    {0.00188791f, -0.01f, 0.1f},{0.0f, 0.0f, 0.038f},
    {0.0f, 0.0f, 0.046f},       {0.0f, 0.0f, 0.1f},
    {-0.10021f, 0.24778f, 0.0039563f}, {-0.038f, -0.013831f, 0.0f},
    {-0.00624f, -0.1032f, 0.0f},{0.0f, -0.080518f, 0.015783f},
    {-0.00188791f, -0.01f, 0.1f},{0.0f, 0.0f, 0.038f},
    {0.0f, 0.0f, 0.046f},       {0.0f, 0.0f, 0.1f}}};

inline constexpr SkeletonDef kSmplx22Spec{"smplx22", kSmplx22Names, kSmplx22Parents, kSmplx22Offsets};
inline constexpr SkeletonDef kSoma30Spec{"soma30", kSoma30Names, kSoma30Parents, kSoma30Offsets};
inline constexpr SkeletonDef kG1Skel34Spec{"g1skel34", kG1Skel34Names, kG1Skel34Parents, kG1Skel34Offsets};

inline constexpr const SkeletonDef *find_skeleton_by_joints(int num_joints) noexcept {
    switch (num_joints) {
        case 22: return &kSmplx22Spec;
        case 30: return &kSoma30Spec;
        case 34: return &kG1Skel34Spec;
        default: return nullptr;
    }
}

inline constexpr const SkeletonDef *find_skeleton_by_key(std::string_view key) noexcept {
    if (key == kSoma30Spec.key) return &kSoma30Spec;
    if (key == kSmplx22Spec.key) return &kSmplx22Spec;
    if (key == kG1Skel34Spec.key) return &kG1Skel34Spec;
    return nullptr;
}

inline QuatXYZW quat_normalize(QuatXYZW q) noexcept {
    const float norm_sq = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
    if (norm_sq <= 1e-12f) {
        return {0.0f, 0.0f, 0.0f, 1.0f};
    }
    const float inv = 1.0f / std::sqrt(norm_sq);
    return {q.x * inv, q.y * inv, q.z * inv, q.w * inv};
}

// Hamilton product: a * b (applies b in local space, then a)
inline QuatXYZW quat_multiply(QuatXYZW a, QuatXYZW b) noexcept {
    return quat_normalize({
        a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
        a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
        a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
        a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z
    });
}

// Rotate vector v by unit quaternion q: q * v * q^-1
inline Vec3 quat_rotate(QuatXYZW q, Vec3 v) noexcept {
    const float tx = 2.0f * (q.y * v.z - q.z * v.y);
    const float ty = 2.0f * (q.z * v.x - q.x * v.z);
    const float tz = 2.0f * (q.x * v.y - q.y * v.x);
    return {
        v.x + q.w * tx + (q.y * tz - q.z * ty),
        v.y + q.w * ty + (q.z * tx - q.x * tz),
        v.z + q.w * tz + (q.x * ty - q.y * tx)
    };
}

// Convert unit quaternion to Houdini / KineFX 3x3 row-vector rotation matrix:
// Row 0 = rotated X axis, Row 1 = rotated Y axis, Row 2 = rotated Z axis.
inline std::array<float, 9> quat_to_houdini_mat3(QuatXYZW q) noexcept {
    const float xx = q.x * q.x;
    const float yy = q.y * q.y;
    const float zz = q.z * q.z;
    const float xy = q.x * q.y;
    const float xz = q.x * q.z;
    const float yz = q.y * q.z;
    const float wx = q.w * q.x;
    const float wy = q.w * q.y;
    const float wz = q.w * q.z;

    return {
        1.0f - 2.0f * (yy + zz), 2.0f * (xy + wz),        2.0f * (xz - wy),
        2.0f * (xy - wz),        1.0f - 2.0f * (xx + zz), 2.0f * (yz + wx),
        2.0f * (xz + wy),        2.0f * (yz - wx),        1.0f - 2.0f * (xx + yy)
    };
}

// Convert unit quaternion + translation to Houdini / KineFX 4x4 row-vector transform matrix
inline std::array<float, 16> make_houdini_mat4(QuatXYZW q, Vec3 t) noexcept {
    const auto m3 = quat_to_houdini_mat3(q);
    return {
        m3[0], m3[1], m3[2], 0.0f,
        m3[3], m3[4], m3[5], 0.0f,
        m3[6], m3[7], m3[8], 0.0f,
        t.x,   t.y,   t.z,   1.0f
    };
}

// Evaluates Forward Kinematics for a single frame from kimodo.cpp output buffers.
// root_positions: [T, 3]
// local_rotations_xyzw: [T, J, 4]
inline std::vector<JointPose> evaluate_frame_fk(
    int frame_idx,
    int total_frames,
    int num_joints,
    const float *root_positions,
    const float *local_rotations_xyzw,
    const SkeletonDef *skel) {

    if (total_frames <= 0 || num_joints <= 0 || !root_positions || !local_rotations_xyzw) {
        return {};
    }
    const int clamped_frame = frame_idx < 0 ? 0 : (frame_idx >= total_frames ? total_frames - 1 : frame_idx);
    std::vector<JointPose> poses(static_cast<std::size_t>(num_joints));

    const float *frame_root = root_positions + static_cast<std::size_t>(clamped_frame) * 3;
    const float *frame_rots = local_rotations_xyzw + static_cast<std::size_t>(clamped_frame) * static_cast<std::size_t>(num_joints) * 4;

    for (int j = 0; j < num_joints; ++j) {
        const auto sj = static_cast<std::size_t>(j);
        const float *q_ptr = frame_rots + sj * 4;
        const QuatXYZW local_q = quat_normalize({q_ptr[0], q_ptr[1], q_ptr[2], q_ptr[3]});

        Vec3 local_p{0.0f, 0.0f, 0.0f};
        int parent = -1;
        if (skel && sj < skel->joint_count()) {
            parent = skel->parents[sj];
            if (j == 0 || parent < 0) {
                local_p = {frame_root[0], frame_root[1], frame_root[2]};
            } else {
                const auto &off = skel->offsets[sj];
                local_p = {off[0], off[1], off[2]};
            }
        } else if (j == 0) {
            local_p = {frame_root[0], frame_root[1], frame_root[2]};
        }

        JointPose &pose = poses[sj];
        pose.local_pos = local_p;
        pose.local_rot = local_q;
        pose.local_mat4 = make_houdini_mat4(local_q, local_p);

        if (parent >= 0 && parent < j) {
            const JointPose &parent_pose = poses[static_cast<std::size_t>(parent)];
            const Vec3 rotated_offset = quat_rotate(parent_pose.world_rot, local_p);
            pose.world_pos = {
                parent_pose.world_pos.x + rotated_offset.x,
                parent_pose.world_pos.y + rotated_offset.y,
                parent_pose.world_pos.z + rotated_offset.z
            };
            pose.world_rot = quat_multiply(parent_pose.world_rot, local_q);
        } else {
            pose.world_pos = local_p;
            pose.world_rot = local_q;
        }
        pose.world_mat3 = quat_to_houdini_mat3(pose.world_rot);
    }

    return poses;
}

// Convert unit quaternion to ZYX intrinsic Euler angles (in degrees) for BVH export:
// R = Rz(z_deg) * Ry(y_deg) * Rx(x_deg)
inline Vec3 quat_to_euler_zyx_deg(QuatXYZW q) noexcept {
    q = quat_normalize(q);
    // Standard column-vector rotation matrix elements for R = Rz * Ry * Rx:
    const float r00 = 1.0f - 2.0f * (q.y * q.y + q.z * q.z);
    const float r10 = 2.0f * (q.x * q.y + q.w * q.z);
    const float r20 = 2.0f * (q.x * q.z - q.w * q.y);
    const float r21 = 2.0f * (q.y * q.z + q.w * q.x);
    const float r22 = 1.0f - 2.0f * (q.x * q.x + q.y * q.y);
    const float r01 = 2.0f * (q.x * q.y - q.w * q.z);
    const float r11 = 1.0f - 2.0f * (q.x * q.x + q.z * q.z);

    constexpr float kRadToDeg = 180.0f / 3.14159265358979323846f;
    float x_rad = 0.0f;
    float y_rad = 0.0f;
    float z_rad = 0.0f;

    if (std::abs(r20) < 0.99999f) {
        y_rad = std::asin(-r20);
        z_rad = std::atan2(r10, r00);
        x_rad = std::atan2(r21, r22);
    } else {
        // Gimbal lock
        y_rad = (r20 < 0.0f) ? (3.14159265358979323846f * 0.5f) : (-3.14159265358979323846f * 0.5f);
        z_rad = std::atan2(-r01, r11);
        x_rad = 0.0f;
    }
    return {x_rad * kRadToDeg, y_rad * kRadToDeg, z_rad * kRadToDeg};
}

namespace detail {

inline void write_bvh_node(
    std::ostream &os,
    int joint_idx,
    int indent,
    const SkeletonDef &skel,
    std::vector<int> &channel_joint_order) {

    const std::string pad(static_cast<std::size_t>(indent * 2), ' ');
    const auto sj = static_cast<std::size_t>(joint_idx);
    const bool is_root = (joint_idx == 0);

    os << pad << (is_root ? "ROOT " : "JOINT ") << skel.names[sj] << "\n";
    os << pad << "{\n";
    const auto &off = skel.offsets[sj];
    os << pad << "  OFFSET " << off[0] << " " << off[1] << " " << off[2] << "\n";
    if (is_root) {
        os << pad << "  CHANNELS 6 Xposition Yposition Zposition Zrotation Yrotation Xrotation\n";
    } else {
        os << pad << "  CHANNELS 3 Zrotation Yrotation Xrotation\n";
    }
    channel_joint_order.push_back(joint_idx);

    bool has_children = false;
    for (std::size_t child = 0; child < skel.joint_count(); ++child) {
        if (skel.parents[child] == joint_idx) {
            has_children = true;
            write_bvh_node(os, static_cast<int>(child), indent + 1, skel, channel_joint_order);
        }
    }

    if (!has_children) {
        os << pad << "  End Site\n";
        os << pad << "  {\n";
        os << pad << "    OFFSET 0.0 0.05 0.0\n";
        os << pad << "  }\n";
    }
    os << pad << "}\n";
}

inline void append_f32_le(std::vector<std::uint8_t> &buf, const float *vals, std::size_t count) {
    const std::size_t old_size = buf.size();
    buf.resize(old_size + count * sizeof(float));
    std::memcpy(buf.data() + old_size, vals, count * sizeof(float));
}

inline void append_u32_le(std::vector<std::uint8_t> &buf, std::uint32_t val) {
    buf.push_back(static_cast<std::uint8_t>(val & 0xFFu));
    buf.push_back(static_cast<std::uint8_t>((val >> 8) & 0xFFu));
    buf.push_back(static_cast<std::uint8_t>((val >> 16) & 0xFFu));
    buf.push_back(static_cast<std::uint8_t>((val >> 24) & 0xFFu));
}

} // namespace detail

// Pure C++ BVH exporter (consumable by Houdini Mocap Import / MotionClip SOP)
inline bool write_skeleton_bvh(
    const std::string &path,
    int frames,
    int joints,
    float fps,
    const float *root_positions,
    const float *local_rotations_xyzw,
    const SkeletonDef &skel) {

    if (frames <= 0 || joints <= 0 || static_cast<std::size_t>(joints) != skel.joint_count()) {
        return false;
    }
    std::ofstream out(path, std::ios::out | std::ios::trunc);
    if (!out) return false;

    out << std::fixed << std::setprecision(6);
    out << "HIERARCHY\n";
    std::vector<int> channel_joint_order;
    channel_joint_order.reserve(skel.joint_count());
    detail::write_bvh_node(out, 0, 0, skel, channel_joint_order);

    out << "MOTION\n";
    out << "Frames: " << frames << "\n";
    out << "Frame Time: " << (1.0f / (fps > 0.0f ? fps : 30.0f)) << "\n";

    for (int f = 0; f < frames; ++f) {
        const float *rpos = root_positions + static_cast<std::size_t>(f) * 3;
        const float *rots = local_rotations_xyzw + static_cast<std::size_t>(f * joints) * 4;
        for (std::size_t idx = 0; idx < channel_joint_order.size(); ++idx) {
            const int j = channel_joint_order[idx];
            if (j == 0) {
                out << rpos[0] << " " << rpos[1] << " " << rpos[2] << " ";
            }
            const float *q = rots + static_cast<std::size_t>(j) * 4;
            const Vec3 euler = quat_to_euler_zyx_deg({q[0], q[1], q[2], q[3]});
            out << euler.z << " " << euler.y << " " << euler.x;
            if (idx + 1 < channel_joint_order.size()) {
                out << " ";
            }
        }
        out << "\n";
    }
    return static_cast<bool>(out);
}

// Pure C++ standalone binary glTF 2.0 (.glb) skeleton animation exporter
// (consumable by Houdini glTF SOP / glTF Character Import SOP)
inline bool write_skeleton_glb(
    const std::string &path,
    int frames,
    int joints,
    float fps,
    const float *root_positions,
    const float *local_rotations_xyzw,
    const SkeletonDef &skel) {

    if (frames <= 0 || joints <= 0 || static_cast<std::size_t>(joints) != skel.joint_count()) {
        return false;
    }

    const float frame_dt = 1.0f / (fps > 0.0f ? fps : 30.0f);
    std::vector<float> times(static_cast<std::size_t>(frames));
    for (int i = 0; i < frames; ++i) {
        times[static_cast<std::size_t>(i)] = static_cast<float>(i) * frame_dt;
    }

    struct BufferView {
        std::size_t byte_offset;
        std::size_t byte_length;
    };
    std::vector<std::uint8_t> bin;
    bin.reserve(static_cast<std::size_t>(frames * (1 + 3 + joints * 4)) * sizeof(float));
    std::vector<BufferView> views;
    views.reserve(static_cast<std::size_t>(joints + 2));

    auto add_view = [&](const float *data, std::size_t count) -> int {
        const std::size_t offset = bin.size();
        detail::append_f32_le(bin, data, count);
        views.push_back({offset, bin.size() - offset});
        return static_cast<int>(views.size() - 1);
    };

    const int time_view = add_view(times.data(), times.size());
    const int root_view = add_view(root_positions, static_cast<std::size_t>(frames) * 3);

    std::vector<int> rot_views(static_cast<std::size_t>(joints));
    std::vector<float> track(static_cast<std::size_t>(frames) * 4);
    for (int j = 0; j < joints; ++j) {
        for (int f = 0; f < frames; ++f) {
            const float *src = local_rotations_xyzw + static_cast<std::size_t>(f * joints + j) * 4;
            const QuatXYZW q = quat_normalize({src[0], src[1], src[2], src[3]});
            track[static_cast<std::size_t>(f) * 4 + 0] = q.x;
            track[static_cast<std::size_t>(f) * 4 + 1] = q.y;
            track[static_cast<std::size_t>(f) * 4 + 2] = q.z;
            track[static_cast<std::size_t>(f) * 4 + 3] = q.w;
        }
        rot_views[static_cast<std::size_t>(j)] = add_view(track.data(), track.size());
    }

    std::ostringstream json;
    json << std::fixed << std::setprecision(7);
    json << "{\"asset\":{\"version\":\"2.0\",\"generator\":\"kimodo_houdini C++ GLB exporter\"},";
    json << "\"scene\":0,\"scenes\":[{\"nodes\":[0]}],";

    // Nodes
    json << "\"nodes\":[";
    for (int j = 0; j < joints; ++j) {
        if (j > 0) json << ",";
        const auto sj = static_cast<std::size_t>(j);
        json << "{\"name\":\"" << skel.names[sj] << "\"";
        if (j != 0) {
            const auto &off = skel.offsets[sj];
            json << ",\"translation\":[" << off[0] << "," << off[1] << "," << off[2] << "]";
        }
        std::vector<int> children;
        for (int c = 0; c < joints; ++c) {
            if (skel.parents[static_cast<std::size_t>(c)] == j) {
                children.push_back(c);
            }
        }
        if (!children.empty()) {
            json << ",\"children\":[";
            for (std::size_t k = 0; k < children.size(); ++k) {
                if (k > 0) json << ",";
                json << children[k];
            }
            json << "]";
        }
        json << "}";
    }
    json << "],";

    // Buffers & BufferViews
    json << "\"buffers\":[{\"byteLength\":" << bin.size() << "}],";
    json << "\"bufferViews\":[";
    for (std::size_t i = 0; i < views.size(); ++i) {
        if (i > 0) json << ",";
        json << "{\"buffer\":0,\"byteOffset\":" << views[i].byte_offset
             << ",\"byteLength\":" << views[i].byte_length << "}";
    }
    json << "],";

    // Accessors: 0 = time (SCALAR), 1 = root translation (VEC3), 2..J+1 = joint rotations (VEC4)
    json << "\"accessors\":[";
    json << "{\"bufferView\":" << time_view << ",\"componentType\":5126,\"count\":" << frames
         << ",\"type\":\"SCALAR\",\"min\":[0.0],\"max\":[" << times.back() << "]},";
    json << "{\"bufferView\":" << root_view << ",\"componentType\":5126,\"count\":" << frames
         << ",\"type\":\"VEC3\"}";
    for (int j = 0; j < joints; ++j) {
        json << ",{\"bufferView\":" << rot_views[static_cast<std::size_t>(j)]
             << ",\"componentType\":5126,\"count\":" << frames << ",\"type\":\"VEC4\"}";
    }
    json << "],";

    // Animations
    json << "\"animations\":[{\"name\":\"KimodoMotion\",\"samplers\":[";
    // Sampler 0: root translation -> accessor 1
    json << "{\"input\":0,\"output\":1,\"interpolation\":\"LINEAR\"}";
    for (int j = 0; j < joints; ++j) {
        json << ",{\"input\":0,\"output\":" << (j + 2) << ",\"interpolation\":\"LINEAR\"}";
    }
    json << "],\"channels\":[";
    json << "{\"sampler\":0,\"target\":{\"node\":0,\"path\":\"translation\"}}";
    for (int j = 0; j < joints; ++j) {
        json << ",{\"sampler\":" << (j + 1) << ",\"target\":{\"node\":" << j << ",\"path\":\"rotation\"}}";
    }
    json << "]}]}";

    std::string json_str = json.str();
    while (json_str.size() % 4 != 0) {
        json_str.push_back(' ');
    }
    while (bin.size() % 4 != 0) {
        bin.push_back(0);
    }

    const std::uint32_t total_len = static_cast<std::uint32_t>(12 + 8 + json_str.size() + 8 + bin.size());
    std::vector<std::uint8_t> glb;
    glb.reserve(total_len);

    // GLB Header
    detail::append_u32_le(glb, 0x46546C67u); // "glTF"
    detail::append_u32_le(glb, 2u);          // version 2
    detail::append_u32_le(glb, total_len);

    // JSON Chunk
    detail::append_u32_le(glb, static_cast<std::uint32_t>(json_str.size()));
    detail::append_u32_le(glb, 0x4E4F534Au); // "JSON"
    glb.insert(glb.end(), json_str.begin(), json_str.end());

    // BIN Chunk
    detail::append_u32_le(glb, static_cast<std::uint32_t>(bin.size()));
    detail::append_u32_le(glb, 0x004E4942u); // "BIN\0"
    glb.insert(glb.end(), bin.begin(), bin.end());

    std::ofstream out(path, std::ios::binary | std::ios::trunc);
    if (!out) return false;
    out.write(reinterpret_cast<const char *>(glb.data()), static_cast<std::streamsize>(glb.size()));
    return static_cast<bool>(out);
}

} // namespace kimodo_houdini
