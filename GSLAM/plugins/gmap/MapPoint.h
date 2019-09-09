#ifndef MAPPOINT_H
#define MAPPOINT_H
#include <GSLAM/core/GSLAM.h>

class MapPoint: public GSLAM::MapPoint
{
public:
    MapPoint(const GSLAM::PointID&    id,const GSLAM::Point3Type& position,
             const GSLAM::Point3Type& norm,const GSLAM::Point3ub& color,
             const GSLAM::FrameID     refKF);

    ~MapPoint();
    virtual std::string type(){return "DIYMapPoint";}

    virtual bool              setNormal(const GSLAM::Point3Type& nVec){GSLAM::WriteMutex lock(_mutexPt);_nVec=nVec;return true;}
    virtual GSLAM::Point3Type getNormal()const{GSLAM::ReadMutex lock(_mutexPt);return _nVec;}
    virtual GSLAM::ColorType  getColor()const{return _color;}
    virtual GSLAM::FrameID    refKeyframeID()const{return _refKF;}

    virtual int        observationNum()const{return _obs.size();}
    virtual bool       getObservations(std::map<GSLAM::FrameID,size_t>& obs)const;

    virtual bool       setDescriptor(const GSLAM::GImage& des){GSLAM::WriteMutex lock(_mutexPt);_descriptor=des;return true;}
    virtual GSLAM::GImage    getDescriptor()const{GSLAM::ReadMutex lock(_mutexPt);return _descriptor;}

protected:// Do NOT add|erase the observe from a mappoint manually!
    virtual bool       addObservation(GSLAM::FrameID frId,size_t featId);
    virtual bool       eraseObservation(GSLAM::FrameID frId);
    virtual bool       clearObservation();

private:
    GSLAM::Point3Type  _nVec;
    GSLAM::Point3ub       _color;
    GSLAM::FrameID     _refKF;
    int                _flags;

    std::map<GSLAM::FrameID,size_t>      _obs;
    GSLAM::GImage                              _descriptor;

    friend class MapFrame;
    friend class Mapper;
};

#endif // MAPPOINT_H
