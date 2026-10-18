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

#include <OP/OP_AutoLockInputs.h>
#include <CH/CH_Manager.h>
#include <GA/GA_ElementGroup.h>
#include <kimodo_houdini/constraints.hpp>
#include <map>
#include <set>
#include <utility>

namespace {
using namespace kimodo_houdini;
void write_skeleton(GU_Detail *gdp, const std::vector<JointPose>& poses, const SkeletonDef *skel, bool connect_bones) {
    gdp->clearAndDestroy();
    const int num_joints=static_cast<int>(poses.size());
    // 3. Register KineFX attributes on geometry (Houdini 21+ KineFX standard)
    GA_RWHandleS name_handle(gdp->addStringTuple(GA_ATTRIB_POINT, "name", 1));
    GA_RWHandleV4 rot_handle(gdp->addFloatTuple(GA_ATTRIB_POINT, "rot", 4)); // Quat [x, y, z, w]
    GA_RWHandleM3 transform_handle(gdp->addFloatTuple(GA_ATTRIB_POINT, "transform", 9)); // 3x3 world rotation
    GA_RWHandleM4 localtransform_handle(gdp->addFloatTuple(GA_ATTRIB_POINT, "localtransform", 16)); // 4x4 local transform
    GA_RWHandleI parent_handle(gdp->addIntTuple(GA_ATTRIB_POINT, "parent", 1));

    gdp->findPointAttribute("transform")->setTypeInfo(GA_TYPE_TRANSFORM);
    gdp->findPointAttribute("localtransform")->setTypeInfo(GA_TYPE_TRANSFORM);
    gdp->findPointAttribute("rot")->setTypeInfo(GA_TYPE_QUATERNION);
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
                GEO_PrimPoly *bone_line = GEO_PrimPoly::build(gdp, 2, GU_POLY_OPEN, false);
                bone_line->setPointOffset(0, joint_ptoffs[static_cast<std::size_t>(parent_idx)]);
                bone_line->setPointOffset(1, joint_ptoffs[sj]);
            }
        }
    }

}
const SkeletonDef *selected_skeleton(int selection) {
    return find_skeleton_by_joints(selection==1?22:selection==2?34:30);
}
std::vector<JointPose> rest_pose(const SkeletonDef *s, float height) {
    std::vector<float> q(s->joint_count()*4,0.f);
    for(size_t j=0;j<s->joint_count();++j) q[j*4+3]=1.f;
    float root[3]={0,height,0};
    return evaluate_frame_fk(0,1,static_cast<int>(s->joint_count()),root,q.data(),s);
}
static PRM_Name skeleton_name("skeleton","Skeleton");
static PRM_Name skeleton_choices[]={PRM_Name("soma30","SOMA 30"),PRM_Name("smplx22","SMPL-X 22"),PRM_Name("g1skel34","Unitree G1 34"),PRM_Name()};
static PRM_ChoiceList skeleton_menu(PRM_CHOICELIST_SINGLE,skeleton_choices);
static PRM_Default zero(0),one(1),height_default(1.0),frames_default(60),steps_default(50),seed_default(42),cfg_default(2);
static PRM_Name height_name("root_height","Root Height (m)");
class SOP_KimodoSkeleton final : public SOP_Node {
public:
    SOP_KimodoSkeleton(OP_Network *n,const char *s,OP_Operator *o):SOP_Node(n,s,o){}
    static OP_Node *create(OP_Network *n,const char *s,OP_Operator *o){return new SOP_KimodoSkeleton(n,s,o);}
    static PRM_Template parms[];
    OP_ERROR cookMySop(OP_Context &c) override {
        const auto *s=selected_skeleton(evalInt("skeleton",0,c.getTime()));
        write_skeleton(gdp,rest_pose(s,evalFloat("root_height",0,c.getTime())),s,true);
        return error();
    }
};
PRM_Template SOP_KimodoSkeleton::parms[]={
    PRM_Template(PRM_ORD,1,&skeleton_name,&zero,&skeleton_menu),
    PRM_Template(PRM_FLT,1,&height_name,&height_default),PRM_Template()};

static PRM_Name prompt_name("prompt","Text Prompt"),frames_name("frames","Frame Count"),start_name("start_frame","Start Frame"),steps_name("steps","Diffusion Steps"),seed_name("seed","Seed");
static PRM_Name text_cfg_name("text_cfg","Text CFG Weight"),constraint_cfg_name("constraint_cfg","Constraint CFG Weight");
static PRM_Name model_name("motion_model","Motion GGUF Path"),text_model_name("text_model","Text Encoder GGUF / Bundle"),device_name("device","Motion Compute Backend"),bones_name("connect_bones","Create KineFX Bone Lines");
static PRM_Default prompt_default(0,"A person walking forward and waving"),model_default(0,"$KIMODO_MODEL_DIR/models/kimodo-soma-rp-v1.1-f32.gguf"),text_model_default(0,"$KIMODO_MODEL_DIR/Llama-3-Kimodo-Q8_0.gguf");
static PRM_Name device_choices[]={PRM_Name("auto","Auto"),PRM_Name("cpu","CPU"),PRM_Name("vulkan","Vulkan GPU"),PRM_Name()};
static PRM_ChoiceList device_menu(PRM_CHOICELIST_SINGLE,device_choices);
static PRM_Name generate_name("generate","Generate"),capture_name("capture_frames","Capture Frames"),source_name("capture_source","Capture Frames From"),capture_attr_name("capture_attribute","Capture Frame Detail Attribute"),mode_name("capture_mode","Pose Capture Mode"),group_name("capture_group","Locked Joints"),rot_name("capture_rotations","Constrain Selected Joint Rotations"),path_name("waypoint_mode","Waypoint Mode");
static PRM_Name pose_enable_name("enable_pose_locks", "Enable Pose Locks");
static PRM_Name waypoint_enable_name("enable_waypoints", "Enable Waypoints");
// Same name-based viewport picker used by KineFX Rig Pose. Keep capture_group's
// internal token so existing selections in saved scenes continue to load.
static PRM_SpareData joint_picker(PRM_SpareArgs()
    << PRM_SpareToken("script_action", "import soputils\nsoputils.selectPointGroupParmByName(kwargs, inputindex=0)\n")
    << PRM_SpareToken("script_action_icon", "BUTTONS_reselect")
    << PRM_SpareToken("script_action_help", "Select joints on input 0 in the viewport, then press Enter. Selections are stored by joint name.")
    << PRM_SpareToken("script_callback_language", "python"));
static PRM_Default attr_default(0,"kimodo_capture_frames"),empty_default(0,""),merge_default(2);
static PRM_Name source_choices[]={PRM_Name("ui","UI Frame List"),PRM_Name("attribute","Detail Attribute"),PRM_Name("merge","UI + Detail Attribute"),PRM_Name()};
static PRM_Name mode_choices[]={PRM_Name("full","Full Body"),PRM_Name("selected","Selected Joints"),PRM_Name()};
static PRM_Name path_choices[]={PRM_Name("sparse","Sparse Waypoints"),PRM_Name("dense","Dense Path (Linear Interpolation)"),PRM_Name()};
static PRM_ChoiceList source_menu(PRM_CHOICELIST_SINGLE,source_choices),mode_menu(PRM_CHOICELIST_SINGLE,mode_choices),path_menu(PRM_CHOICELIST_SINGLE,path_choices);
static PRM_Range frame_range(PRM_RANGE_RESTRICTED,2,PRM_RANGE_UI,300),step_range(PRM_RANGE_RESTRICTED,1,PRM_RANGE_UI,100);
}

PRM_Template SOP_Kimodo::myTemplateList[]={
    PRM_Template(PRM_CALLBACK,1,&generate_name,nullptr,nullptr,nullptr,&SOP_Kimodo::generateCallback),
    PRM_Template(PRM_STRING,1,&prompt_name,&prompt_default),
    PRM_Template(PRM_INT,1,&start_name,&one),
    PRM_Template(PRM_INT,1,&frames_name,&frames_default,nullptr,&frame_range),
    PRM_Template(PRM_INT,1,&steps_name,&steps_default,nullptr,&step_range),
    PRM_Template(PRM_INT,1,&seed_name,&seed_default),
    PRM_Template(PRM_FLT,1,&text_cfg_name,&cfg_default),
    PRM_Template(PRM_FLT,1,&constraint_cfg_name,&cfg_default),
    PRM_Template(PRM_ORD,1,&skeleton_name,&zero,&skeleton_menu),
    PRM_Template(PRM_TOGGLE,1,&pose_enable_name,&zero),
    PRM_Template(PRM_ORD,1,&source_name,&merge_default,&source_menu),
    PRM_Template(PRM_STRING,1,&capture_name,&empty_default),
    PRM_Template(PRM_STRING,1,&capture_attr_name,&attr_default),
    PRM_Template(PRM_ORD,1,&mode_name,&zero,&mode_menu),
    PRM_Template(PRM_STRING,1,&group_name,&empty_default,nullptr,nullptr,nullptr,&joint_picker),
    PRM_Template(PRM_TOGGLE,1,&rot_name,&one),
    PRM_Template(PRM_TOGGLE,1,&waypoint_enable_name,&zero),
    PRM_Template(PRM_ORD,1,&path_name,&zero,&path_menu),
    PRM_Template(PRM_FILE,1,&model_name,&model_default),
    PRM_Template(PRM_FILE,1,&text_model_name,&text_model_default),
    PRM_Template(PRM_ORD,1,&device_name,&zero,&device_menu),
    PRM_Template(PRM_TOGGLE,1,&bones_name,&one),PRM_Template()};

OP_Node *SOP_Kimodo::myConstructor(OP_Network *n,const char *s,OP_Operator *o){return new SOP_Kimodo(n,s,o);}
SOP_Kimodo::SOP_Kimodo(OP_Network *n,const char *s,OP_Operator *o):SOP_Node(n,s,o){}
SOP_Kimodo::~SOP_Kimodo()=default;
const char *SOP_Kimodo::inputLabel(OP_InputIdx i) const {return i==0?"Animated KineFX Skeleton / Pose Captures":"Waypoints (P, integer frame; optional heading in radians)";}
bool SOP_Kimodo::updateParmsFlags() {
    bool changed = SOP_Node::updateParmsFlags();
    const fpreal time = CHgetEvalTime();
    const bool poses = evalInt("enable_pose_locks", 0, time) != 0;
    const bool waypoints = evalInt("enable_waypoints", 0, time) != 0;
    const bool selected = poses && evalInt("capture_mode", 0, time) == 1;
    const int source = evalInt("capture_source", 0, time);
    auto show = [&](const char *name, bool visible) {
        changed |= setVisibleState(name, visible);
        changed |= enableParm(name, visible);
    };
    show("capture_source", poses);
    show("capture_frames", poses && source != 1);
    show("capture_attribute", poses && source != 0);
    show("capture_mode", poses);
    show("capture_group", selected);
    show("capture_rotations", selected);
    show("waypoint_mode", waypoints);
    show("constraint_cfg", poses || waypoints);
    return changed;
}

int SOP_Kimodo::generateCallback(void *op,int,fpreal time,const PRM_Template *) {
    auto *node=static_cast<SOP_Kimodo *>(op);
    node->myGenerateRequested=true;
    node->forceRecook();
    OP_Context context(time);
    node->cook(context);
    return 1;
}

OP_ERROR SOP_Kimodo::cookMySop(OP_Context &context) {
    flags().setTimeDep(true);
    const fpreal t=context.getTime();
    auto text=[&](const char *name){UT_String s;evalString(s,name,0,t);return std::string(s.c_str()?s.c_str():"");};
    if(std::exchange(myGenerateRequested,false)) {
        try {
            const int frames=evalInt("frames",0,t),start=evalInt("start_frame",0,t);
            const auto *s=selected_skeleton(evalInt("skeleton",0,t));
            if(frames<2 || frames>10000) throw std::runtime_error("Frame count must be between 2 and 10000");
            const bool pose_enabled = evalInt("enable_pose_locks", 0, t) != 0;
            const bool waypoint_enabled = evalInt("enable_waypoints", 0, t) != 0;
            if (pose_enabled && !getInput(0))
                throw std::runtime_error("Pose locks require an animated skeleton on input 0");
            if (waypoint_enabled && !getInput(1))
                throw std::runtime_error("Waypoints require points on input 1");
            std::set<int> capture_frames;
            const int source=evalInt("capture_source",0,t);
            if(pose_enabled && source!=1) {auto list=parse_capture_frames(text("capture_frames"),start,frames);capture_frames.insert(list.begin(),list.end());}
            if(pose_enabled && getInput(0) && source!=0) {
                OP_AutoLockInputs lock(this);
                if(lock.lockInput(0,context)>=UT_ERROR_ABORT) throw std::runtime_error("Cannot cook pose input");
                const auto *geo=inputGeo(0);
                const std::string attr_name=text("capture_attribute");
                const auto *attr=geo->findGlobalAttribute(attr_name.c_str());
                if(!attr && source==1) throw std::runtime_error("Missing capture-frame detail attribute: "+attr_name);
                if(attr) {
                    GA_ROHandleIA ints(attr);GA_ROHandleS str(attr);
                    std::string list;
                    if(ints.isValid()) {UT_Int32Array a;ints.get(GA_Offset(0),a);for(auto f:a) list+=std::to_string(f)+" ";}
                    else if(str.isValid()) list=str.get(GA_Offset(0)).c_str();
                    else throw std::runtime_error("Capture-frame detail attribute must be an integer array or string");
                    auto parsed=parse_capture_frames(list,start,frames);capture_frames.insert(parsed.begin(),parsed.end());
                }
            }
            if(!getInput(0) && !capture_frames.empty()) throw std::runtime_error("Capture frames require a skeleton on input 0");
            if (pose_enabled && capture_frames.empty())
                throw std::runtime_error("Specify capture frames for the enabled pose locks");
            std::vector<PoseCapture> poses;
            for(int frame:capture_frames) {
                OP_Context capture_context=context;capture_context.setFrame(frame);
                OP_AutoLockInputs lock(this);
                if(lock.lockInput(0,capture_context)>=UT_ERROR_ABORT) throw std::runtime_error("Cannot cook pose input at frame "+std::to_string(frame));
                const auto *geo=inputGeo(0);
                GA_ROHandleS names(geo->findPointAttribute("name"));
                if(!names.isValid()) throw std::runtime_error("Input 0 requires string point attribute name");
                std::map<std::string,GA_Offset> points;
                for(GA_Iterator it(geo->getPointRange());!it.atEnd();++it)
                    if(!points.emplace(names.get(*it).c_str(),*it).second) throw std::runtime_error("Duplicate joint name on input 0");
                const bool selected=evalInt("capture_mode",0,t)==1;
                const GA_PointGroup *group=nullptr;
                if (selected) {
                    const std::string selection = text("capture_group");
                    // Empty Houdini group expressions mean all points; here an
                    // empty Locked Joints field must never accidentally lock all.
                    if (selection.find_first_not_of(" \t\r\n") == std::string::npos)
                        throw std::runtime_error("Select Locked Joints using the viewport picker");
                    group = parsePointGroups(selection.c_str(), GroupCreator(geo), true, false);
                    if (!group || group->isEmpty())
                        throw std::runtime_error("Locked Joints selection does not match input 0");
                }
                PoseCapture p;p.frame=frame-start;p.rotations=selected && evalInt("capture_rotations",0,t);
                GA_ROHandleM3 transforms(geo->findPointAttribute("transform"));
                if(p.rotations && !transforms.isValid()) throw std::runtime_error("Rotation capture requires KineFX point matrix3 transform");
                for(auto name:s->names) {
                    auto it=points.find(std::string(name));
                    if(it==points.end()) throw std::runtime_error("Input skeleton is missing joint: "+std::string(name));
                    const auto P=geo->getPos3(it->second);p.positions.push_back({float(P.x()),float(P.y()),float(P.z())});
                    p.selected.push_back(!selected || group->contains(it->second));
                    if(p.rotations) {
                        const UT_Matrix3F m=transforms.get(it->second);
                        std::array<float,9> a{};for(int r=0;r<3;++r)for(int c=0;c<3;++c)a[r*3+c]=m(r,c);
                        p.transforms.push_back(a);
                    }
                }
                if(std::none_of(p.selected.begin(),p.selected.end(),[](bool v){return v;})) throw std::runtime_error("Locked Joints selection contains no model joints");
                poses.push_back(std::move(p));
            }
            std::vector<Waypoint> waypoints;
            if(waypoint_enabled && getInput(1)) {
                OP_AutoLockInputs lock(this);
                if(lock.lockInput(1,context)>=UT_ERROR_ABORT) throw std::runtime_error("Cannot cook waypoint input");
                const auto *geo=inputGeo(1);const auto *attr=geo->findPointAttribute("frame");
                if(!attr || attr->getStorageClass()!=GA_STORECLASS_INT || attr->getTupleSize()!=1) throw std::runtime_error("Input 1 requires scalar integer point attribute frame");
                GA_ROHandleI frame_attr(attr);GA_ROHandleF heading(geo->findPointAttribute("heading"));
                for(GA_Iterator it(geo->getPointRange());!it.atEnd();++it) {
                    const auto P=geo->getPos3(*it);
                    waypoints.push_back({frame_attr.get(*it)-start,float(P.x()),float(P.z()),heading.isValid(),heading.isValid()?heading.get(*it):0.f});
                }
                if(waypoints.empty()) throw std::runtime_error("Waypoint input contains no points");
            }
            auto constraints=make_conditioning(*s,frames,poses,waypoints,evalInt("waypoint_mode",0,t)==1);
            auto view=constraints.view();
            ModelConfig model;model.motion_gguf=text("motion_model");model.text_gguf=text("text_model");model.device=static_cast<kimodo_device>(evalInt("device",0,t));
            GenerationParams params;params.prompt=text("prompt");params.frames=frames;params.diffusion_steps=evalInt("steps",0,t);params.seed=std::max<exint>(0,evalInt("seed",0,t));params.text_cfg_weight=evalFloat("text_cfg",0,t);params.constraint_cfg_weight=evalFloat("constraint_cfg",0,t);
            std::string err;
            const auto *motion=mySessionCache.get_or_generate(model,params,err,constraints.any?&view:nullptr,true);
            if(!motion) throw std::runtime_error(err);
            const int joints=kimodo_motion_joints(motion),actual_frames=kimodo_motion_frames(motion);
            if(joints!=static_cast<int>(s->joint_count())) throw std::runtime_error("Selected skeleton does not match the loaded motion model");
            std::vector<float> roots(kimodo_motion_root_positions(motion),kimodo_motion_root_positions(motion)+actual_frames*3);
            std::vector<float> rotations(kimodo_motion_local_rotations_xyzw(motion),kimodo_motion_local_rotations_xyzw(motion)+actual_frames*joints*4);
            myRoots=std::move(roots);myRotations=std::move(rotations);myFrames=actual_frames;myJoints=joints;myStart=start;++myGeneration;
            myLastError.clear();
        } catch(const std::exception &e) {myLastError=e.what();}
    }
    if(myFrames) {
        const int frame=SYSclamp(int(context.getFrame())-myStart,0,myFrames-1);
        const auto *s=find_skeleton_by_joints(myJoints);
        write_skeleton(gdp,evaluate_frame_fk(frame,myFrames,myJoints,myRoots.data(),myRotations.data(),s),s,evalInt("connect_bones",0,t));
    } else {
        const auto *s=selected_skeleton(evalInt("skeleton",0,t));
        write_skeleton(gdp,rest_pose(s,1.f),s,evalInt("connect_bones",0,t));
        addWarning(SOP_MESSAGE,"No generated clip. Configure inputs and press Generate.");
    }
    GA_RWHandleI(gdp->addIntTuple(GA_ATTRIB_DETAIL,"kimodo_generation",1)).set(GA_Offset(0),myGeneration);
    GA_RWHandleI(gdp->addIntTuple(GA_ATTRIB_DETAIL,"kimodo_clip_frames",1)).set(GA_Offset(0),myFrames);
    if(!myLastError.empty()) addWarning(SOP_MESSAGE,myLastError.c_str());
    return error();
}
void newSopOperator(OP_OperatorTable *table) {
    table->addOperator(new OP_Operator("sop_kimodo_native","Kimodo Native",SOP_Kimodo::myConstructor,SOP_Kimodo::myTemplateList,0u,2u));
    table->addOperator(new OP_Operator("sop_kimodo_skeleton","Kimodo Skeleton",SOP_KimodoSkeleton::create,SOP_KimodoSkeleton::parms,0u,0u));
}
