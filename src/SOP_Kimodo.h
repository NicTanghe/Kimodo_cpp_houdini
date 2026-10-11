#pragma once

#include <SOP/SOP_Node.h>
#include <kimodo_houdini/motion_cache.hpp>

class SOP_Kimodo final : public SOP_Node {
public:
    static OP_Node *myConstructor(OP_Network *net, const char *name, OP_Parameters *params);
    static PRM_Template myTemplateList[];

protected:
    SOP_Kimodo(OP_Network *net, const char *name, OP_Parameters *params);
    ~SOP_Kimodo() override;

    OP_ERROR cookMySop(OP_Context &context) override;

private:
    kimodo_houdini::KimodoSessionCache mySessionCache;
};
