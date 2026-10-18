#pragma once
#include <SOP/SOP_Node.h>
#include <kimodo_houdini/motion_cache.hpp>
#include <vector>

class SOP_Kimodo final : public SOP_Node {
public:
    static OP_Node *myConstructor(OP_Network *,const char *,OP_Operator *);
    static PRM_Template myTemplateList[];
    static int generateCallback(void *,int,fpreal,const PRM_Template *);
    const char *inputLabel(OP_InputIdx) const override;
protected:
    SOP_Kimodo(OP_Network *,const char *,OP_Operator *);
    ~SOP_Kimodo() override;
    OP_ERROR cookMySop(OP_Context &) override;
    bool updateParmsFlags() override;
private:
    kimodo_houdini::KimodoSessionCache mySessionCache;
    bool myGenerateRequested=false;
    int myFrames=0,myJoints=0,myStart=1,myGeneration=0;
    std::vector<float> myRoots,myRotations;
    std::string myLastError;
};
