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
#include <base/Types/SPtr.h>
#include <base/Types/SIM3.h>
#include <base/Thread/Thread.h>

namespace GSLAM {

typedef pi::UInt8   uchar;
typedef uchar       byte;
typedef std::size_t PointID;
typedef std::size_t FrameID;
typedef pi::Point2d Point2D;
typedef pi::Point2i Point2i;
typedef pi::Point3d Point3Type;
typedef pi::Point3ub ColorType;
typedef pi::SE3d    SE3;

class GObject
{
public:
    virtual ~GObject(){}
    virtual std::string type(){return "GObject";}
    virtual void  call(const std::string& command,void* arg=NULL){}
    virtual void  draw(){}

};

/**
 * @brief The MapPoint class
 */
class MapPoint : public GObject
{
public:
    MapPoint(const PointID& id_,const Point3Type& position=Point3Type(0,0,0));
    virtual ~MapPoint(){}
    virtual std::string type(){return "InvalidPoint";}
    const PointID id(){return _id;}
    Point3Type    getPose();
    void          setPose(const Point3Type& pt_);

protected:
    pi::MutexRW   mutexPt;

private:
    const PointID _id;
    Point3Type    pt;
};


class MapFrame : public GObject
{
public:
    MapFrame(const FrameID& id_);
    virtual ~MapFrame(){}
    virtual std::string type(){return "InvalidFrame";}

    const PointID id(){return _id;}
    SE3           getPose();
    void          setPose(const SE3& pose);

protected:
    pi::MutexRW    mutexPose;

private:
    SE3             c2w;//worldPt=c2w*cameraPt;
    const FrameID   _id;
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

    virtual std::size_t frameNum(){return 0;}
    virtual std::size_t pointNum(){return 0;}

    virtual FramePtr getFrame(const FrameID& id){return FramePtr();}
    virtual PointPtr getPoint(const PointID& id){return PointPtr();}
    virtual bool     getFrames(FrameArray& frames){return false;}
    virtual bool     getPoints(PointArray& points){return false;}

    /// Save or load the map from/to the file
    virtual bool save(std::string path){return false;}
    virtual bool load(std::string path){return false;}

    /// 0 is reserved for INVALID
    std::size_t getPid(){return ptId++;}//obtain an unique point id
    std::size_t getFid(){return frId++;}//obtain an unique frame id

private:
    PointID ptId;
    FrameID frId;
};

typedef SPtr<Map> MapPtr;

class SLAM : public GObject
{
public:
    SLAM();
    virtual ~SLAM(){}
    virtual std::string type(){return "InvalidSLAM";}
    virtual bool valid(){return false;}

    bool    setMap(const MapPtr& map);
    MapPtr  getMap();

    bool    track(FramePtr frame){return false;}

protected:
    MapPtr      curMap;
    pi::MutexRW mutexMap;
};

} //end of namespace GSLAM

#endif
