#include "VideoFrame.h"

namespace GSLAM{

VideoFrameMono::VideoFrameMono(const GImage& img,const Camera& camera,FrameID id,double time)
    :MapFrame(id,time),_img(img),_camera(camera)
{

}

GImage VideoFrameMono::getImage(int idx)
{
    pi::ReadMutex lock(_mutexPose);
    return _img;// FIXME: .clone()?
}

Camera VideoFrameMono::getCamera(int idx)
{
    pi::ReadMutex lock(_mutexPose);
    return _camera;
}

VideoFrameRGBD::VideoFrameRGBD(const GImage& img,const GImage& depth,const Camera& camera,FrameID id,double timestamp)
    :MapFrame(id,timestamp),_img(img),_depth(depth),_camera(camera)
{

}

GImage VideoFrameRGBD::getImage(int idx)
{
    pi::ReadMutex lock(_mutexPose);
    if(idx==0)
        return _img;// FIXME: .clone()?
    else return _depth;
}

Camera VideoFrameRGBD::getCamera(int idx)
{
    pi::ReadMutex lock(_mutexPose);
    return _camera;// FIXME: .clone()?
}

VideoFrameStereo::VideoFrameStereo(const GImage& imgLeft,const GImage& imgRight,const Camera& camera,FrameID id,double timestamp)
    :MapFrame(id,timestamp),_imgL(imgLeft),_imgR(imgRight),_camera(camera)
{

}

GImage VideoFrameStereo::getImage(int idx)
{
    pi::ReadMutex lock(_mutexPose);
    if(idx==0)
        return _imgL;// FIXME: .clone()?
    else return _imgR;
}

Camera VideoFrameStereo::getCamera(int idx)
{
    pi::ReadMutex lock(_mutexPose);
    return _camera;// FIXME: .clone()?
}

VideoFrameMultiSensor::VideoFrameMultiSensor(const SPtr<std::vector<SensorFrame> >& inputs,FrameID id,double timestamp)
    :MapFrame(id,timestamp),_inputs(inputs)
{

}

}// end of namespace GSLAM
