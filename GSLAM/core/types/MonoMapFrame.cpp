#include "MonoMapFrame.h"

namespace GSLAM{

MonoMapFrame::MonoMapFrame(const FrameID& id_,const Camera& camera,
                           const Image& imgColor,const Image& imgGray)
    :MapFrame(id_),_camera(camera),_img(imgColor),_imgGray(imgGray)
{
    if(_imgGray.empty())
    {
        CVWarper::cvtColorBGR2Gray(_img,_imgGray);
    }
}

}
