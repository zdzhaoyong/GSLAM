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

}

#endif // VIDEOFRAME_H
