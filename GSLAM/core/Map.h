// GSLAM - A general SLAM framework and benchmark
// Copyright 2018 PILAB Inc. All rights reserved.
// https://github.com/zdzhaoyong/GSLAM
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
// * Neither the name of Google Inc. nor the names of its contributors may be
//   used to endorse or promote products derived from this software without
//   specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// Author: zd5945@126.com (Yong Zhao)
//
// A Map is constructed by MapPoints and MapFrames, relationship between point
// and frame calls observation, relationship between frames calls connection.
// This header only provides interface but no implementation.

#ifndef GSLAM_MAP_H
#define GSLAM_MAP_H

#include "SIM3.h"
#include "GImage.h"
#include "Camera.h"
#include "Svar.h"

namespace GSLAM {

class GObject;
class MapFrame;
class MapPoint;
class Map;
class SLAM;
class FrameConnection;

typedef size_t                 NodeId;
typedef size_t                 WordId;
typedef float                  WordValue;
typedef std::map<WordId,float> BowVector;
typedef std::map<WordId,std::vector<unsigned int> > FeatureVector;
typedef Point3d    Point3Type;
typedef Point3ub       ColorType;
typedef size_t         PointID;
typedef size_t         FrameID;
typedef std::shared_ptr<GObject>  GObjectPtr;
typedef std::shared_ptr<MapFrame> FramePtr;
typedef std::shared_ptr<MapPoint> PointPtr;
typedef std::shared_ptr<FrameConnection> FrameConnectionPtr;
typedef std::shared_ptr<Map>      MapPtr;
typedef std::shared_ptr<SLAM>     SLAMPtr;
typedef std::vector<FramePtr> FrameArray;
typedef std::vector<PointPtr> PointArray;
typedef std::map<GSLAM::FrameID, FrameConnectionPtr > FrameConnectionMap;
typedef std::vector<std::pair<FrameID,size_t> > MapPointObsVec;
typedef GSLAM::Point3d CameraAnchor;        // for both Pinhole projection and Sphere projection
typedef GSLAM::Point2d IdepthEstimation;    // [idepth,invSqSigma]^T
typedef GSLAM::SLAMPtr (*funcCreateSLAMInstance)();
typedef std::mutex MutexRW;
typedef std::unique_lock<std::mutex> ReadMutex;
typedef ReadMutex WriteMutex;

enum ImageChannelFlags{
    IMAGE_UNDEFINED     =0,
    // For colorful or depth camera
    IMAGE_RGBA          =1<<0,// when channals()==3 means RGB, Alpha means the image mask
    IMAGE_BGRA          =1<<1,
    IMAGE_GRAY          =1<<2,
    IMAGE_DEPTH         =1<<3,// the depth image cam be obtained from lidar or depth camera such as Kinect
    IMAGE_IDEPTH        =1<<4,// this is usually obtained with

    IMAGE_RGBD          =IMAGE_BGRA|IMAGE_DEPTH,

    // For multispectral camera
    IMAGE_GRE           =1<<5,// Green
    IMAGE_NIR           =1<<6,// Near
    IMAGE_RED           =1<<7,// Red
    IMAGE_REG           =1<<8,// Red Edge

    IMAGE_LIDAR         =1<<9,
    IMAGE_SONAR         =1<<10,
    IMAGE_SAR           =1<<11,

    IMAGE_THUMBNAIL     =1<<12// The thumbnail should be in format RGBA
};

class GObject
{
public:
    virtual ~GObject(){}
    virtual std::string type()const{return "GObject";}
    virtual void  call(const std::string& command,void* arg=NULL){}
    virtual void  draw(){}
    virtual bool  toByteArray(std::vector<uchar>& array){return false;}
    virtual bool  fromByteArray(std::vector<uchar>& array){return false;}

};

class GObjectHandle
{
public:
    virtual ~GObjectHandle(){}
    virtual void handle(const GObjectPtr& obj){}
    void handle(GObject* obj){handle(GObjectPtr(obj));}
};

class KeyPoint {
 public:
  KeyPoint()
      : pt(0, 0), size(0), angle(-1), response(0), octave(0), class_id(-1) {}

  KeyPoint(Point2f _pt, float _size, float _angle = -1, float _response = 0,
           int _octave = 0, int _class_id = -1)
      : pt(_pt),
        size(_size),
        angle(_angle),
        response(_response),
        octave(_octave),
        class_id(_class_id) {}

  KeyPoint(float x, float y, float _size, float _angle = -1,
           float _response = 0, int _octave = 0, int _class_id = -1)
      : pt(x, y),
        size(_size),
        angle(_angle),
        response(_response),
        octave(_octave),
        class_id(_class_id) {}

#ifdef __OPENCV_FEATURES_2D_HPP__
  explicit KeyPoint(const cv::KeyPoint& kp)
      : pt(kp.pt.x, kp.pt.y),
        size(kp.size),
        angle(kp.angle),
        response(kp.response),
        octave(kp.octave),
        class_id(kp.class_id) {}

  operator cv::KeyPoint() const {
    return cv::KeyPoint(pt.x, pt.y, size, angle, response, octave, class_id);
  }
#endif
  typedef union Cv32suf {
    int i;
    unsigned u;
    float f;
  } Cv32suf;

  size_t hash() const {
    size_t _Val = 2166136261U, scale = 16777619U;
    Cv32suf u;
    u.f = pt.x;
    _Val = (scale * _Val) ^ u.u;
    u.f = pt.y;
    _Val = (scale * _Val) ^ u.u;
    u.f = size;
    _Val = (scale * _Val) ^ u.u;
    u.f = angle;
    _Val = (scale * _Val) ^ u.u;
    u.f = response;
    _Val = (scale * _Val) ^ u.u;
    _Val = (scale * _Val) ^ ((size_t)octave);
    _Val = (scale * _Val) ^ ((size_t)class_id);
    return _Val;
  }

  Point2f pt;  //!< coordinates of the keypoints
  float size;  //!< diameter of the meaningful keypoint neighborhood
  float
      angle;  //!< computed orientation of the keypoint (-1 if not applicable);
              //!< it's in [0,360) degrees and measured relative to
              //!< image coordinate system, ie in clockwise.
  float response;  //!< the response by which the most strong keypoints have
                   //! been selected. Can be used for the further sorting or
  //! subsampling
  int octave;    //!< octave (pyramid layer) from which the keypoint has been
                 //! extracted
  int class_id;  //!< object class (if the keypoints need to be clustered by an
                 //! object they belong to)
};

/**
 * @brief The MapPoint class
 */
class MapPoint : public GObject
{
public:
    MapPoint(const PointID& id,const Point3Type& position=Point3Type(0,0,0));
    virtual ~MapPoint(){}
    virtual std::string type()const{return "InvalidPoint";}
    const PointID& 	id()const{return _id;}

    Point3Type    getPose()const;
    void          setPose(const Point3Type& pt);

    virtual Point3Type getNormal()const{return Point3Type(0,0,0);}
    virtual bool       setNormal(const Point3Type& nVec){return false;}

    virtual ColorType  getColor()const{return ColorType(255,255,255);}
    virtual bool       setColor(const ColorType& color)const{return false;}

    virtual bool       setDescriptor(const GImage& des){return false;}
    virtual GImage     getDescriptor()const{return GImage();}

    virtual bool       isPoseRelative()const{return false;}     // Default use world coordinate
    virtual FrameID    refKeyframeID()const{return 0;}          // If using relative pose, which keyframe referencing
    virtual FramePtr   refKeyframe()const{return FramePtr();}

    virtual int        observationNum()const{return -1;}
    virtual bool       getObservations(std::map<FrameID,size_t>& obs)const{return false;}
    virtual MapPointObsVec getObservations()const{
        MapPointObsVec r;
        std::map<FrameID,size_t> m;
        getObservations(m);
        r.insert(r.end(),m.begin(),m.end());
        return r;
    }
    virtual bool       addObservation(GSLAM::FrameID frId,size_t featId){return false;}
    virtual bool       eraseObservation(GSLAM::FrameID frId){return false;}
    virtual bool       clearObservation(){return false;}

protected:
    mutable MutexRW     _mutexPt;

private:
    const PointID           _id;
    Point3Type              _pt;
};

// MapFrame <-> MapFrame  : MapFrame connections for Pose Graph
class FrameConnection : public GObject
{
public:
    virtual std::string type()const{return "FrameConnection";}
    virtual int  matchesNum(){return 0;}

    virtual bool getMatches(std::vector<std::pair<int,int> >& matches){return false;}
    std::vector<std::pair<int,int> > getMatches(){std::vector<std::pair<int,int> > r;getMatches(r);return r;}
    virtual bool getChild2Parent(GSLAM::SIM3& sim3){return false;}
    virtual bool getChild2Parent(GSLAM::SE3& se3){return false;}
    virtual bool getInformation(double* info){return false;}

    virtual bool setMatches(std::vector<std::pair<int,int> >& matches){return false;}
    virtual bool setChild2Parent(GSLAM::SIM3& sim3){return false;}
    virtual bool setChild2Parent(GSLAM::SE3& se3){return false;}
    virtual bool setInformation(double* info){return false;}
};

class MapFrame : public GObject
{
public:
    MapFrame(const FrameID& id=0,const double& timestamp=0);
    virtual ~MapFrame(){}
    virtual std::string type()const{return "InvalidFrame";}

    // Basic things, ID, Timestamp, Image, CameraModel, IMU, GPS informations
    const PointID&  id()const{return _id;}
    const double&   timestamp()const{return _timestamp;}

    // Frame transform from local to world, this is essential
    void            setPose(const SE3& pose);
    void            setPose(const SIM3& pose);
    SE3             getPose()const;
    bool            getPose(SIM3& pose)const;
    SIM3            getPoseScale()const;

    // When the frame contains one or more images captured from cameras
    virtual int     cameraNum()const{return 0;}                                      // Camera number
    virtual SE3     getCameraPose(int idx=0) const{return SE3();}                    // The transform from camera to local
    virtual int     imageChannels(int idx=0) const{return IMAGE_BGRA;}               // Default is a colorful camera
    virtual Camera  getCamera(int idx=0){return Camera();}                           // The camera model
    virtual GImage  getImage(int idx=0,int channalMask=IMAGE_UNDEFINED){return GImage();}              // Just return the image if only one channel is available
    virtual bool    setImage(const GImage& img,int idx=0,int channalMask=IMAGE_UNDEFINED){return false;}
    virtual bool    setCamera(const Camera& camera,int idx=0){return false;}

    // When the frame contains IMUs or GPSs
    virtual int     getIMUNum()const{return 0;}
    virtual SE3     getIMUPose(int idx=0)const{return SE3();}
    virtual bool    getAcceleration(Point3d& acc,int idx=0)const{return false;}        // m/s^2
    virtual bool    getAngularVelocity(Point3d& angularV,int idx=0)const{return false;}// rad/s
    virtual bool    getMagnetic(Point3d& mag,int idx=0)const{return false;}            // gauss
    virtual bool    getAccelerationNoise(Point3d& accN,int idx=0)const{return false;}
    virtual bool    getAngularVNoise(Point3d& angularVN,int idx=0)const{return false;}
    virtual bool    getPitchYawRoll(Point3d& pyr,int idx=0)const{return false;}     // in rad
    virtual bool    getPYRSigma(Point3d& pyrSigma,int idx=0)const{return false;}    // in rad

    virtual int     getGPSNum()const{return 0;}
    virtual SE3     getGPSPose(int idx=0)const{return SE3();}
    virtual bool    getGPSLLA(Point3d& LonLatAlt,int idx=0)const{return false;}        // WGS84 [longtitude latitude altitude]
    virtual bool    getGPSLLASigma(Point3d& llaSigma,int idx=0)const{return false;}    // meter
    virtual bool    getGPSECEF(Point3d& xyz,int idx=0)const{return false;}             // meter
    virtual bool    getHeight2Ground(Point2d& height,int idx=0)const{return false;}    // height against ground

    // Tracking things for feature based methods
    virtual int     keyPointNum()const{return 0;}
    virtual bool    setKeyPoints(const std::vector<GSLAM::KeyPoint>& keypoints,
                                 const GSLAM::GImage& descriptors=GSLAM::GImage()){return false;}
    virtual bool    getKeyPoint(int idx, Point2f& pt)const{return false;}
    virtual bool    getKeyPoint(int idx, KeyPoint &pt) const{return false;}
    virtual bool    getKeyPoints(std::vector<Point2f>& keypoints)const{return false;}
    virtual bool    getKeyPoints(std::vector<KeyPoint>& keypoints) const{return false;}
    virtual std::vector<KeyPoint>  getKeyPoints() const{std::vector<KeyPoint> r;getKeyPoints(r);return r;}
    virtual bool    getKeyPointColor(int idx,ColorType& color){return false;}
    virtual bool    getKeyPointIDepthInfo(int idx,Point2d& idepth){return false;}
    virtual PointID getKeyPointObserve(int idx){return 0;}
    virtual GImage  getDescriptor(int idx=-1)const{return GImage();}        // idx<0: return all descriptors
    virtual bool    getBoWVector(BowVector& bowvec)const{return false;}
    virtual BowVector getBoWVector()const{BowVector r;getBoWVector(r);return r;}
    virtual bool    getFeatureVector(FeatureVector& featvec)const{return false;}
    virtual FeatureVector getFeatureVector()const{FeatureVector r;getFeatureVector(r);return r;}
    virtual std::vector<size_t> getFeaturesInArea(const float& x,const float& y,
                                          const float& r,bool precisely=true)const{return std::vector<size_t>();}

    // MapPoint <-> KeyPoint  : MapPoint Observation usually comes along Mappoint::*obs*
    virtual int     observationNum()const{return 0;}
    virtual bool    getObservations(std::map<GSLAM::PointID,size_t>& obs)const{return false;}
    virtual std::map<GSLAM::PointID,size_t>    getObservations()const{std::map<GSLAM::PointID,size_t> r;getObservations(r);return r;}
    virtual bool    addObservation(const GSLAM::PointPtr& pt,size_t featId,bool add2Point=false){return false;}
    virtual bool    eraseObservation(const GSLAM::PointPtr& pt,bool erasePoint=false){return false;}
    virtual bool    clearObservations(){return false;}

    virtual std::shared_ptr<FrameConnection> getParent(GSLAM::FrameID parentId)const{return std::shared_ptr<FrameConnection>();}
    virtual std::shared_ptr<FrameConnection> getChild(GSLAM::FrameID childId)const{return std::shared_ptr<FrameConnection>();}
    virtual bool    getParents(FrameConnectionMap& parents)const{return false;}
    virtual bool    getChildren(FrameConnectionMap& children)const{return false;}
    FrameConnectionMap getParents()const{FrameConnectionMap r;getParents(r);return r;}
    FrameConnectionMap getChildren()const{FrameConnectionMap r;getChildren(r);return r;}
    virtual bool    addParent(GSLAM::FrameID parentId, const std::shared_ptr<FrameConnection>& parent){return false;}
    virtual bool    addChildren(GSLAM::FrameID childId, const std::shared_ptr<FrameConnection>& child){return false;}
    virtual bool    eraseParent(GSLAM::FrameID parentId){return false;}
    virtual bool    eraseChild(GSLAM::FrameID  childId){return false;}
    virtual bool    clearParents(){return false;}
    virtual bool    clearChildren(){return false;}

    // Extra utils
    virtual double  getMedianDepth(){return getPoseScale().get_scale();}

    static std::string channelTypeString(const int channels);
    std::string	       channelString(int idx=0) const{return channelTypeString(imageChannels(idx));}

public:
    const FrameID           _id;
    double                  _timestamp;

protected:
    mutable MutexRW         _mutexPose;

private:
    SIM3                    _c2w;//worldPt=c2w*cameraPt;
};

struct LoopCandidate
{
    LoopCandidate(const FrameID& frameId_,const double& score_)
        :frameId(frameId_),score(score_){}

    FrameID  frameId;
    double   score;
    friend bool operator<(const LoopCandidate& l,const LoopCandidate& r)
    {
        return l.score<r.score;
    }
};
typedef std::vector<LoopCandidate> LoopCandidates;

// The LoopDetector is used to detect loops with Poses or Descriptors information
class LoopDetector: public GObject
{
public:
    virtual std::string type()const{return "LoopDetector";}
    virtual bool insertMapFrame(const FramePtr& frame){return false;}
    virtual bool eraseMapFrame(const FrameID& frame){return false;}
    virtual bool obtainCandidates(const FramePtr& frame,LoopCandidates& candidates){return false;}
    virtual LoopCandidates obtainCandidates(const FramePtr& frame){
        LoopCandidates c;
        obtainCandidates(frame,c);
        return c;
    }
};
typedef std::shared_ptr<LoopDetector> LoopDetectorPtr;

class Map : public GObject
{
public:
    Map();
    virtual ~Map(){}
    virtual std::string type()const{return "InvalidMap";}

    /// MapFrame & MapPoint interface
    virtual bool insertMapPoint(const PointPtr& point){return false;}
    virtual bool insertMapFrame(const FramePtr& frame){return false;}
    virtual bool eraseMapPoint(const PointID& pointId){return false;}
    virtual bool eraseMapFrame(const FrameID& frameId){return false;}
    virtual void clear(){}

    virtual std::size_t frameNum()const{return 0;}
    virtual std::size_t pointNum()const{return 0;}

    virtual FramePtr getFrame(const FrameID& id)const{return FramePtr();}
    virtual PointPtr getPoint(const PointID& id)const{return PointPtr();}
    virtual bool     getFrames(FrameArray& frames)const{return false;}
    virtual bool     getPoints(PointArray& points)const{return false;}
    virtual FrameArray  getFrames()const{FrameArray r;getFrames(r);return r;}
    virtual PointArray  getPoints()const{PointArray r;getPoints(r);return r;}

    virtual bool     setLoopDetector(const LoopDetectorPtr& loopdetector){return false;}
    virtual LoopDetectorPtr getLoopDetector()const{return LoopDetectorPtr();}
    virtual bool     obtainCandidates(const FramePtr& frame,LoopCandidates& candidates){return false;}
    virtual LoopCandidates obtainCandidates(const FramePtr& frame){
        LoopCandidates c;
        obtainCandidates(frame,c);
        return c;
    }

    /// Save or load the map from/to the file
    virtual bool save(std::string path)const{return false;}
    virtual bool load(std::string path){return false;}

    /// 0 is reserved for INVALID
    PointID getPid(){return _ptId++;}//obtain an unique point id
    FrameID getFid(){return _frId++;}//obtain an unique frame id

protected:
    PointID _ptId;
    FrameID _frId;
};

typedef std::shared_ptr<Map> MapPtr;


inline MapPoint::MapPoint(const PointID& id,const Point3Type& position)
    :_id(id),_pt(position)
{
}

inline Point3Type   MapPoint::getPose()const
{
    ReadMutex lock(_mutexPt);
    return _pt;
}

inline void MapPoint::setPose(const Point3Type& pt)
{
    WriteMutex lock(_mutexPt);
    _pt=pt;
}

inline MapFrame::MapFrame(const FrameID& id,const double& timestamp)
    :_id(id),_timestamp(timestamp)
{
}

inline SE3 MapFrame::getPose()const
{
    ReadMutex lock(_mutexPose);
    return _c2w.get_se3();
}

inline SIM3 MapFrame::getPoseScale()const
{
    ReadMutex lock(_mutexPose);
    return _c2w;
}

inline bool MapFrame::getPose(SIM3& pose)const
{
    pose=getPoseScale();
    return true;
}

inline void MapFrame::setPose(const SE3& pose)
{
    WriteMutex lock(_mutexPose);
    _c2w.get_se3()=pose;
}


inline void MapFrame::setPose(const SIM3& pose)
{
    WriteMutex lock(_mutexPose);
    _c2w=pose;
}

inline std::string MapFrame::channelTypeString(const int channels)
{
    static std::string _type[32]={"RGBA","BGRA","GRAY","DEPTH","IDEPTH","GRE","NIR","RED","REG","LIDAR","SONAR","SAR","THUMBNAIL"};
    std::string type="";
    if(channels == 0) {
        return "IMAGE_UNDEFINED";
    }
    else
    {
        for(uint i=0; i<sizeof(channels)*8;i++)
        {
            int j = channels & (0x00000001<<i);
            if(j !=0)
            {
                type+=(type.empty()?"":"|")+_type[i];
            }
        }
        return type;
    }
}


inline Map::Map():_ptId(1),_frId(1)
{

}


} //end of namespace GSLAM

#endif
