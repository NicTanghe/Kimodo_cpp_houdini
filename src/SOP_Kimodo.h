#pragma once

#include <SOP/SOP_Node.h>
#include <kimodo_houdini/motion_cache.hpp>

class SOP_Kimodo final : public SOP_Node {
public:
    static OP_Node *myConstructor(OP_Network *net, const char *name, OP_Operator *entry);
    static PRM_Template myTemplateList[];

protected:
    SOP_Kimodo(OP_Network *net, const char *name, OP_Operator *entry);
    ~SOP_Kimodo() override;

    OP_ERROR cookMySop(OP_Context &context) override;

private:
    kimodo_houdini::KimodoSessionCache mySessionCache;
};
