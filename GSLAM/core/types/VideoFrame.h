#ifndef VIDEOFRAME_H
#define VIDEOFRAME_H

#include <GSLAM/core/GSLAM.h>

// The VideoFrame classes are used as the input of different slam systems

namespace GSLAM {

class VideoFrameMono : public MapFrame
{
public:
    VideoFrameMono(const GImage& img,const Camera& camera,FrameID id,double time);
    virtual std::string type() const{return "VideoFrameMono";}

    virtual GImage getImage(int idx=0);
    virtual Camera getCamera(int idx=0);

protected:
    GImage       _img;
    Camera       _camera;
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
