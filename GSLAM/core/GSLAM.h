/**
  This file defines data types shared by all SLAM systems.
  A SLAM system contains one active map for current tracking and some in-active maps after losted or loaded from history.
  A Map are generally constructed with the follow things:
  1. MapFrame : the keyframes (Mono,Stereo,RGBD .etc)
  2. MapPoint : the keypoints (not used in direct methods)
  3. BOWs     : for relocalization and loop closure
  4. Other data structures
 */

#ifndef GSLAM_H
#define GSLAM_H

#include <vector>
#include "types/SE3.h"
#include "types/GImage.h"
#include "types/Camera.h"

#define GSLAM_VERSION "1.1.1"
#define GSLAM_VERSION_MAJOR 1
#define GSLAM_VERSION_MINOR 1
#define GSLAM_VERSION_PATCH 1

namespace GSLAM {

class Map;

class GObject
{
public:
    virtual ~GObject(){}
    virtual std::string type()const{return "GObject";}
    virtual void  call(const std::string& command,void* arg=NULL){}
    virtual void  draw(){}

};

class GObjectHandle
{
public:
    virtual ~GObjectHandle(){}
    virtual void handle(const SPtr<GObject>& obj){}
};

/**
 * @brief The MapPoint class
 */
class MapPoint : public GObject
{
public:
    MapPoint(const PointID& id,const Point3Type& position=Point3Type(0,0,0));
    virtual ~MapPoint(){}
    virtual std::string type(){return "InvalidPoint";}
    const PointID id(){return _id;}
    Point3Type    getPose();
    void          setPose(const Point3Type& pt);

    virtual Point3Type getNormal(){return Point3Type(0,0,0);}
    virtual ColorType  getColor(){return ColorType(255,255,255);}

    virtual int        observationNum(){return -1;}
    virtual bool       getObservations(std::map<FrameID,size_t>& obs){return false;}

protected:
    mutable pi::MutexRW     _mutexPt;

private:
    const PointID           _id;
    Point3Type              _pt;
};


class MapFrame : public GObject
{
public:
    MapFrame(const FrameID& id=0,const double& timestamp=0);
    virtual ~MapFrame(){}
    virtual std::string type()const{return "InvalidFrame";}

    const PointID id()const{return _id;}
    SE3           getPose()const;
    void          setPose(const SE3& pose);

    virtual GImage getImage(int idx=0){return GImage();}
    virtual Camera getCamera(int idx=0){return Camera();}

    virtual bool   getObservations(std::map<PointID,size_t>& obs)const{return false;}
    virtual bool   getConnects(std::map<FrameID,int>& connects)const{return false;}

public:
    const FrameID           _id;
    double                  _timestamp;

protected:
    mutable pi::MutexRW     _mutexPose;

private:
    SE3                     _c2w;//worldPt=c2w*cameraPt;
};

typedef SPtr<MapFrame> FramePtr;
typedef SPtr<MapPoint> PointPtr;
typedef std::vector<FramePtr> FrameArray;
typedef std::vector<PointPtr> PointArray;

class Map : public GObject
{
public:
    Map();
    virtual ~Map(){}
    virtual std::string type(){return "InvalidMap";}

    /// MapFrame & MapPoint interface
    virtual bool insertMapPoint(const PointPtr& point){return false;}
    virtual bool insertMapFrame(const FramePtr& frame){return false;}
    virtual bool eraseMapPoint(const PointID& pointId){return false;}
    virtual bool eraseMapFrame(const FrameID& frameId){return false;}

    virtual std::size_t frameNum()const{return 0;}
    virtual std::size_t pointNum()const{return 0;}

    virtual FramePtr getFrame(const FrameID& id){return FramePtr();}
    virtual PointPtr getPoint(const PointID& id){return PointPtr();}
    virtual bool     getFrames(FrameArray& frames){return false;}
    virtual bool     getPoints(PointArray& points){return false;}

    /// Save or load the map from/to the file
    virtual bool save(std::string path)const{return false;}
    virtual bool load(std::string path){return false;}

    /// 0 is reserved for INVALID
    PointID getPid(){return _ptId++;}//obtain an unique point id
    FrameID getFid(){return _frId++;}//obtain an unique frame id

private:
    PointID _ptId;
    FrameID _frId;
};

typedef SPtr<Map> MapPtr;

class SLAM : public GObject
{
public:
    SLAM(){}
    virtual ~SLAM(){}
    virtual std::string type()const{return "InvalidSLAM";}
    virtual bool valid()const{return false;}

    bool    setMap(const MapPtr& map);
    MapPtr  getMap()const;

    virtual bool    track(FramePtr& frame){return false;}
    virtual bool    setCallback(GObjectHandle* cbk){return false;}

protected:
    MapPtr              _curMap;
    mutable pi::MutexRW _mutexMap;
};

inline MapPoint::MapPoint(const PointID& id,const Point3Type& position)
    :_id(id),_pt(position)
{
}

inline Point3Type   MapPoint::getPose()
{
    pi::ReadMutex lock(_mutexPt);
    return _pt;
}

inline void MapPoint::setPose(const Point3Type& pt)
{
    pi::WriteMutex lock(_mutexPt);
    _pt=pt;
}

inline MapFrame::MapFrame(const FrameID& id,const double& timestamp)
    :_id(id),_timestamp(timestamp)
{
}

inline SE3 MapFrame::getPose()const
{
    pi::ReadMutex lock(_mutexPose);
    return _c2w;
}

inline void MapFrame::setPose(const SE3& pose)
{
    pi::WriteMutex lock(_mutexPose);
    _c2w=pose;
}

inline Map::Map():_ptId(1),_frId(1)
{

}

inline bool SLAM::setMap(const MapPtr& map)
{
    _curMap=map;
}

inline MapPtr SLAM::getMap()const
{
    pi::ReadMutex lock(_mutexMap);
    return _curMap;
}


} //end of namespace GSLAM

#endif
