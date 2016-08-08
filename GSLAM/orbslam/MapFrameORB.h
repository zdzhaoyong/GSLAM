#ifndef MAPFRAMEORB_H
#define MAPFRAMEORB_H
#include <GSLAM/types/MonoMapFrame.h>

class ORBextractor;
class MonoMapFrameORB:public MonoMapFrame
{
public:
    MonoMapFrameORB(const FrameID& id_,const Camera& camera,
                    const Image& imgColor,const Image& imgGray=Image());


};

#endif // MAPFRAMEORB_H
