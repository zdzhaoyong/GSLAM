#include "VideoFrame.h"

namespace GSLAM{

VideoFrameMono::VideoFrameMono(const GImage& img,const Camera& camera,FrameID id,double time,const Camera& recCamera)
    :MapFrame(id,time),_img(img),_camera(camera),_recCamera(recCamera)
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
    if(idx==0)
        return _camera;
    else if(idx==1)
        return _recCamera;
}

VideoFrameMonoWithExposure::VideoFrameMonoWithExposure(const GImage &img, const Camera &camera, FrameID id, double time,const Camera& recCamera,
                                                       float explosure_time,const GImage& G,const GImage& vignetteMap)
    :VideoFrameMono(img,camera,id,time,recCamera),_explosureTime(explosure_time)
{
}

GImage VideoFrameMonoWithExposure::getImage(int idx)// 0:_img 1:_G 2:_vignetteMap
{
    pi::ReadMutex lock(_mutexPose);
    if(idx==1) return _G;
    else if(idx==2) return _vignetteMap;
    else   return _img;// FIXME: .clone()?
}

void  VideoFrameMonoWithExposure::call(const std::string& command,void* arg){
    if("getExposure"==command)
    {
        *(float*)arg=_explosureTime;
    }
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
