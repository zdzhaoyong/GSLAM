#include "GSLAM.h"

namespace GSLAM{

MapPoint::MapPoint(const PointID& id_,const Point3Type& position)
    :_id(id_),pt(position)
{
}

Point3Type   MapPoint::getPose()
{
    pi::ReadMutex lock(mutexPt);
    return pt;
}

void MapPoint::setPose(const Point3Type& pt_)
{
    pi::WriteMutex lock(mutexPt);
    pt=pt_;
}

MapFrame::MapFrame(const FrameID& id_)
    :_id(id_)
{
}

SE3 MapFrame::getPose()
{
    pi::ReadMutex lock(mutexPose);
    return c2w;
}

void MapFrame::setPose(const SE3& pose)
{
    pi::WriteMutex lock(mutexPose);
    c2w=pose;
}

Map::Map():ptId(1),frId(1)
{

}

SLAM::SLAM()
{
}

bool SLAM::setMap(const MapPtr& map)
{
    curMap=map;
}

MapPtr SLAM::getMap()
{
    pi::ReadMutex lock(mutexMap);
    return curMap;
}

}
