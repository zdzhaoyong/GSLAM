#ifndef MONOMAPFRAME_H
#define MONOMAPFRAME_H
#include <GSLAM/core/GSLAM.h>
#include <GSLAM/core/types/GImage.h>
#include <GSLAM/core/types/Camera.h>

namespace GSLAM{

class MonoMapFrame : public MapFrame
{
public:
    MonoMapFrame(const FrameID& id_,const Camera& camera,
                 const Image& imgColor,const Image& imgGray=Image());

    virtual std::string type(){return "MonoMapFrame";}

    // Once defined never change
    Image  _img,_imgGray;
    Camera _camera;
};

}
#endif // MONOMAPFRAME_H
