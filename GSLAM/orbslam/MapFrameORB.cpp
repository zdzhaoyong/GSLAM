#include "MapFrameORB.h"

MonoMapFrameORB::MonoMapFrameORB(const FrameID& id_,const Camera& camera,
                                 const Image& imgColor,const Image& imgGray=Image())
    :MonoMapFrame(id_,camera,imgColor,imgGray)
{
}
