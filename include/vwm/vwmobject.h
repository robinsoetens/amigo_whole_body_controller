#ifndef VWMOBJECT_H
#define VWMOBJECT_H

#include <wire_volume/Entity.h>

namespace vwm_tools {

class vwmObject
{
private:
    int shape_revision_;
public:
    vwmObject(vwm::EntityHandle e);
};

} // namespace

#endif // VWMOBJECT_H
