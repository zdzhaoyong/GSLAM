#ifndef VIDEOFRAME_H
#define VIDEOFRAME_H

#include <GSLAM/core/GSLAM.h>

// The VideoFrame classes are used as the input of different slam systems

namespace GSLAM {

class VideoFrameMono : public MapFrame
{
public:
    VideoFrameMono(const GImage& img,const Camera& camera,FrameID id,double time,const Camera& recCamera=Camera());
    virtual std::string type() const{return "VideoFrameMono";}

    virtual GImage getImage(int idx=0);
    virtual Camera getCamera(int idx=0);//0:origin camera, 1:recommand slam camera
protected:
    GImage       _img;
    Camera       _camera,_recCamera;
};

class VideoFrameMonoWithExposure : public VideoFrameMono
{
public:
    VideoFrameMonoWithExposure(const GImage& img, const Camera& camera, FrameID id, double time,const Camera& recCamera=Camera(),
                               float explosure_time=0,const GImage& G=GImage(),const GImage& vignetteMap=GImage());

    virtual std::string type() const{return "VideoFrameMonoWithExposure";}

    virtual void  call(const std::string& command,void* arg=NULL);// "getExposure" float*

    virtual GImage getImage(int idx=0);// 0:_img 1:_G 2:_vignetteMap

    float  _explosureTime;
    GImage _G,_vignetteMap;
};

class VideoFrameRGBD : public MapFrame
{
public:
    VideoFrameRGBD(const GImage& img,const GImage& depth,const Camera& camera,FrameID id,double timestamp);
    virtual std::string type() const{return "VideoFrameRGBD";}

    virtual GImage getImage(int idx=0);
    virtual Camera getCamera(int idx=0);

protected:
    GImage       _img,_depth;
    Camera       _camera;
};

class VideoFrameStereo : public MapFrame
{
public:
    VideoFrameStereo(const GImage& imgLeft,const GImage& imgRight,const Camera& camera,FrameID id,double timestamp);
    virtual std::string type() const{return "VideoFrameStereo";}

    virtual GImage getImage(int idx=0);
    virtual Camera getCamera(int idx=0);

protected:
    GImage       _imgL,_imgR;
    Camera       _camera;
};

struct SensorFrame
{
public:
    enum SensorType{ColorfulCamera,DepthCamera,SAR,Others};
    SensorType _sensorType;
    GImage     _img;
    Camera     _camera;
    SE3        _pose;
};

class VideoFrameMultiSensor : public MapFrame
{
public:
    VideoFrameMultiSensor(const SPtr<std::vector<SensorFrame>>& inputs,FrameID id,double timestamp);

    SPtr<std::vector<SensorFrame>> _inputs;
};


inline VideoFrameMono::VideoFrameMono(const GImage& img,const Camera& camera,FrameID id,double time,const Camera& recCamera)
    :MapFrame(id,time),_img(img),_camera(camera),_recCamera(recCamera)
{

}

inline GImage VideoFrameMono::getImage(int idx)
{
    pi::ReadMutex lock(_mutexPose);
    return _img;// FIXME: .clone()?
}

inline Camera VideoFrameMono::getCamera(int idx)
{
    pi::ReadMutex lock(_mutexPose);
    if(idx==0)
        return _camera;
    else if(idx==1)
        return _recCamera;
}

inline VideoFrameMonoWithExposure::VideoFrameMonoWithExposure(const GImage &img, const Camera &camera, FrameID id, double time,const Camera& recCamera,
                                                       float explosure_time,const GImage& G,const GImage& vignetteMap)
    :VideoFrameMono(img,camera,id,time,recCamera),_explosureTime(explosure_time)
{
}

inline GImage VideoFrameMonoWithExposure::getImage(int idx)// 0:_img 1:_G 2:_vignetteMap
{
    pi::ReadMutex lock(_mutexPose);
    if(idx==1) return _G;
    else if(idx==2) return _vignetteMap;
    else   return _img;// FIXME: .clone()?
}

inline void  VideoFrameMonoWithExposure::call(const std::string& command,void* arg){
    if("getExposure"==command)
    {
        *(float*)arg=_explosureTime;
    }
}

inline VideoFrameRGBD::VideoFrameRGBD(const GImage& img,const GImage& depth,const Camera& camera,FrameID id,double timestamp)
    :MapFrame(id,timestamp),_img(img),_depth(depth),_camera(camera)
{

}

inline GImage VideoFrameRGBD::getImage(int idx)
{
    pi::ReadMutex lock(_mutexPose);
    if(idx==0)
        return _img;// FIXME: .clone()?
    else return _depth;
}

inline Camera VideoFrameRGBD::getCamera(int idx)
{
    pi::ReadMutex lock(_mutexPose);
    return _camera;// FIXME: .clone()?
}

inline VideoFrameStereo::VideoFrameStereo(const GImage& imgLeft,const GImage& imgRight,const Camera& camera,FrameID id,double timestamp)
    :MapFrame(id,timestamp),_imgL(imgLeft),_imgR(imgRight),_camera(camera)
{

}

inline GImage VideoFrameStereo::getImage(int idx)
{
    pi::ReadMutex lock(_mutexPose);
    if(idx==0)
        return _imgL;// FIXME: .clone()?
    else return _imgR;
}

inline Camera VideoFrameStereo::getCamera(int idx)
{
    pi::ReadMutex lock(_mutexPose);
    return _camera;// FIXME: .clone()?
}

inline VideoFrameMultiSensor::VideoFrameMultiSensor(const SPtr<std::vector<SensorFrame> >& inputs,FrameID id,double timestamp)
    :MapFrame(id,timestamp),_inputs(inputs)
{

}

}

#endif // VIDEOFRAME_H
