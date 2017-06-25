#ifndef VIDEOFRAME_H
#define VIDEOFRAME_H

#include "GSLAM.h"

/** The VideoFrame classes are used as the input of different slam systems
Class name : "Frame"+MainSonsor+OtherSensor
MainSonsor : Mono,Stereo,RGBD,Multi,Lidar
OtherSensor: GPS,IMU,Compass
**/
namespace GSLAM {

class FrameMono : public MapFrame
{
public:
    FrameMono(const GImage& img,const Camera& camera,FrameID id,double time,const Camera& recCamera=Camera());
    virtual std::string type() const{return "FrameMono";}

    virtual GImage getImage(int idx=0); // 0:origin image
    virtual Camera getCamera(int idx=0);// 0:origin camera, 1:recommand slam camera
protected:
    GImage       _img;
    Camera       _camera,_recCamera;
};

class FrameRGBD : public MapFrame
{
public:
    FrameRGBD(const GImage& img,const GImage& depth,const Camera& camera,FrameID id,double timestamp);
    virtual std::string type() const{return "FrameRGBD";}

    virtual GImage getImage(int idx=0); // 0:RGB 1:Depth CV16UC1-factor 1000, CV32FC1-factor 1.f
    virtual Camera getCamera(int idx=0);// The RGB and Depth are treated as unified

protected:
    GImage       _img,_depth;
    Camera       _camera;
};

class FrameStereo : public MapFrame
{
public:
    FrameStereo(const GImage& imgLeft,const GImage& imgRight,
                const Camera& cameraLeft,const Camera& cameraRight,
                const SE3&    right2left,FrameID id,double timestamp);
    virtual std::string type() const{return "FrameStereo";}

    virtual GImage getImage(int idx=0);
    virtual Camera getCamera(int idx=0);

protected:
    GImage       _imgL,_imgR;
    Camera       _cameraL,_cameraR;
    SE3          _right2left;
};


inline FrameMono::FrameMono(const GImage& img,const Camera& camera,FrameID id,double time,const Camera& recCamera)
    :MapFrame(id,time),_img(img),_camera(camera),_recCamera(recCamera)
{

}

inline GImage FrameMono::getImage(int idx)
{
    GSLAM::ReadMutex lock(_mutexPose);
    return _img;// FIXME: .clone()?
}

inline Camera FrameMono::getCamera(int idx)
{
    GSLAM::ReadMutex lock(_mutexPose);
    if(idx==0)
        return _camera;
    else if(idx==1)
        return _recCamera;
}

inline FrameRGBD::FrameRGBD(const GImage& img,const GImage& depth,const Camera& camera,FrameID id,double timestamp)
    :MapFrame(id,timestamp),_img(img),_depth(depth),_camera(camera)
{

}

inline GImage FrameRGBD::getImage(int idx)
{
    GSLAM::ReadMutex lock(_mutexPose);
    if(idx==0)
        return _img;// FIXME: .clone()?
    else return _depth;
}

inline Camera FrameRGBD::getCamera(int idx)
{
    GSLAM::ReadMutex lock(_mutexPose);
    return _camera;// FIXME: .clone()?
}

inline FrameStereo::FrameStereo(const GImage& imgLeft,const GImage& imgRight,
                                const Camera& cameraLeft,const Camera& cameraRight,
                                const SE3&    right2left,FrameID id,double timestamp)
    : MapFrame(id,timestamp),_imgL(imgLeft),_imgR(imgRight),
      _cameraL(cameraLeft),_cameraR(cameraRight),_right2left(right2left)
{

}

inline GImage FrameStereo::getImage(int idx)
{
    GSLAM::ReadMutex lock(_mutexPose);
    if(idx==0)
        return _imgL;
    else
        return _imgR;
}

inline Camera FrameStereo::getCamera(int idx)
{
    GSLAM::ReadMutex lock(_mutexPose);
    if(idx==0)
        return _cameraL;
    else return _cameraR;
}

}

#endif // VIDEOFRAME_H
