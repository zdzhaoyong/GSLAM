#ifndef VIDEOFRAME_H
#define VIDEOFRAME_H

#include "GSLAM.h"
#include "GPS.h"

/** The VideoFrame classes are used as the input of different slam systems
Class name : "Frame"+MainSonsor+OtherSensor
MainSonsor : Mono,Stereo,RGBD,Multi,Lidar
OtherSensor: GPS,IMU,Compass
**/
namespace GSLAM {

class FrameMono : public MapFrame
{
public:
    FrameMono(FrameID id,double time,const GImage& img,const Camera& camera,
              int channel=IMAGE_RGBA,const Camera& recCamera=Camera());
    virtual std::string type() const{return "FrameMono";}

    virtual int    imageChannels(int idx) const{return _channel;}
    virtual int    cameraNum()const{return 1;}     // Camera number
    virtual GImage getImage(int idx,int channels); // 0:origin image
    virtual Camera getCamera(int idx=0);           // 0:origin camera, 1:recommand slam camera

    int          _channel;
protected:
    GImage       _img;
    Camera       _camera,_recCamera;
};

class FrameRGBD : public MapFrame
{
public:
    FrameRGBD(const GImage& img,const GImage& depth,const Camera& camera,FrameID id,double timestamp);
    virtual std::string type() const{return "FrameRGBD";}

    virtual int    cameraNum()const{return 1;}                // Camera number
    virtual GImage getImage(int idx,int channels) // 0:RGB 1:Depth CV16UC1-factor 1000, CV32FC1-factor 1.f
    {
        GSLAM::ReadMutex lock(_mutexPose);
        if(channels&&IMAGE_DEPTH) return _depth;
        else return _img;
    }
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

    virtual int    cameraNum()const{return 2;}                // Camera number
    virtual GImage getImage(int idx,int channels);
    virtual Camera getCamera(int idx=0);
    virtual SE3    getCameraPose(int idx) const{
        if(idx==0) return SE3();
        else       return _right2left;
    }

protected:
    GImage       _imgL,_imgR;
    Camera       _cameraL,_cameraR;
    SE3          _right2left;
};

class FrameMonoGPS : public FrameMono
{
public:
    FrameMonoGPS(FrameID id,double time,const GImage& img,const Camera& camera,
                 double longtitude,double latitude,double altitude,
                 double sigmaHorizon,double sigmaVertical)
        :FrameMono(id,time,img,camera),_longtitude(longtitude),_latitude(latitude),_altitude(altitude),
    _sigmaHorizon(sigmaHorizon),_sigmaVertical(sigmaVertical){}

    FrameMonoGPS(FrameID id,double time,const GImage& img,const Camera& camera,
                 double longtitude,double latitude,double altitude,
                 double sigmaHorizon,double sigmaVertical,double height)
        :FrameMono(id,time,img,camera),_longtitude(longtitude),_latitude(latitude),_altitude(altitude),
    _sigmaHorizon(sigmaHorizon),_sigmaVertical(sigmaVertical),_height(height){}

    virtual std::string type() const{return "FrameMonoGPS";}
    virtual int     getGPSNum()const{return 1;}
    virtual SE3     getGPSPose(int idx=0)const{return SE3();}

    virtual bool    getGPSLLA(Point3d& LonLatAlt,int idx=0)const{  // WGS84 [longtitude latitude altitude]
        LonLatAlt=Point3d(_longtitude,_latitude,_altitude);
        return true;
    }

    virtual bool    getGPSLLASigma(Point3d& llaSigma,int idx=0)const{
        llaSigma=Point3d(_sigmaHorizon,_sigmaHorizon,_sigmaVertical);return true;}     // meter

    virtual bool    getGPSECEF(Point3d& xyz,int idx=0)const{  // meter
        xyz=GPS<>::GPS2XYZ(_latitude,_longtitude,_altitude);
        return true;
    }

    virtual bool    getHeight2Ground(Point2d &height, int idx=0) const{
        height=Point2d(_height,_sigmaVertical);
        return true;
    }

    double _longtitude,_latitude,_altitude,_sigmaHorizon,_sigmaVertical,_height;
};

class FrameMonoGPSPYR: public FrameMonoGPS
{
public:
    FrameMonoGPSPYR(FrameID id,double time,const GImage& img,const Camera& camera,
                 double longtitude,double latitude,double altitude,
                 double sigmaHorizon,double sigmaVertical,
                 double pitch,double yaw,double roll,
                 double sigmaPitch,double sigmaYaw,double sigmaRoll)
        :FrameMonoGPS(id,time,img,camera,longtitude,latitude,altitude,sigmaHorizon,sigmaVertical),
          _pitch(pitch),_yaw(yaw),_roll(roll),
          _sigmaPitch(sigmaPitch),_sigmaYaw(sigmaYaw),_sigmaRoll(sigmaRoll){}

    virtual std::string type() const{return "FrameMonoGPSPYR";}
    virtual int     getIMUNum()const{return 1;}

    virtual bool    getPitchYawRoll(Point3d& pyr,int idx=0)const{
        pyr=Point3d(_pitch,_yaw,_roll);
        return true;}     // in rad

    virtual bool    getPYRSigma(Point3d& pyrSigma,int idx=0)const
    {
        pyrSigma=Point3d(_sigmaPitch,_sigmaYaw,_sigmaRoll);
        return true;
    }    // in rad

    double _pitch,_yaw,_roll,_sigmaPitch,_sigmaYaw,_sigmaRoll;
};

class FrameMonoIMU : public FrameMono
{
public:
    FrameMonoIMU(FrameID id,double time,const GImage& img,const Camera& camera,
                    Point3d acc ,Point3d angularV,Point3d mag,
                    Point3d accN,Point3d gyrN)
           :FrameMono(id,time,img,camera),_acc(acc),_angularV(angularV),
             _mag(mag),_accN(accN),_gyrN(gyrN){}

    virtual int     getIMUNum()const{return 1;}
    virtual bool    getAcceleration(Point3d& acc,int idx=0)const{acc=_acc;return true;}        // m/s^2
    virtual bool    getAccelerationNoise(Point3d &accN, int idx) const{accN=_accN;return true;}
    virtual bool    getAngularVelocity(Point3d& angularV,int idx=0)const{angularV=_angularV;return true;}// rad/s
    virtual bool    getAngularVNoise(Point3d &angularVN, int idx) const{angularVN=_gyrN;return true;}
    virtual bool    getMagnetic(Point3d& mag,int idx=0)const{mag=_mag;return true;}            // gauss

    Point3d _acc,_angularV,_mag,_accN,_gyrN;
};

inline FrameMono::FrameMono(FrameID id,double time,const GImage& img,const Camera& camera,int channel,const Camera& recCamera)
    :MapFrame(id,time),_img(img),_camera(camera),_recCamera(recCamera),_channel(channel)
{

}

inline GImage FrameMono::getImage(int idx,int channels)
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

inline GImage FrameStereo::getImage(int idx,int channels)
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
