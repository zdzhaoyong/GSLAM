#include "GSLAM.h"

namespace GSLAM{

MapPoint::MapPoint(const PointID& id,const Point3Type& position)
    :_id(id),_pt(position)
{
}

Point3Type   MapPoint::getPose()
{
    pi::ReadMutex lock(_mutexPt);
    return _pt;
}

void MapPoint::setPose(const Point3Type& pt)
{
    pi::WriteMutex lock(_mutexPt);
    _pt=pt;
}

MapFrame::MapFrame(const FrameID& id,const double& timestamp)
    :_id(id),_timestamp(timestamp)
{
}

SE3 MapFrame::getPose()
{
    pi::ReadMutex lock(_mutexPose);
    return _c2w;
}

void MapFrame::setPose(const SE3& pose)
{
    pi::WriteMutex lock(_mutexPose);
    _c2w=pose;
}

Map::Map():_ptId(1),_frId(1)
{

}

SLAM::SLAM()
{
}

bool SLAM::setMap(const MapPtr& map)
{
    _curMap=map;
}

MapPtr SLAM::getMap()
{
    pi::ReadMutex lock(_mutexMap);
    return _curMap;
}

}
