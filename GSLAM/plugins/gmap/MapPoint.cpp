#include "MapPoint.h"

using namespace std;

MapPoint::MapPoint(const GSLAM::PointID&    id,const GSLAM::Point3Type& position,
                   const GSLAM::Point3Type& norm,const GSLAM::Point3ub& color,
                   const GSLAM::FrameID     refKF)
    :GSLAM::MapPoint(id,position),_nVec(norm),_color(color),_refKF(refKF)
{

}

MapPoint::~MapPoint()
{
}

bool MapPoint::getObservations(std::map<GSLAM::FrameID,size_t>& obs)const
{
    GSLAM::ReadMutex lock(_mutexPt);
    obs=_obs;
    return true;
}

bool MapPoint::addObservation(GSLAM::FrameID frId,size_t featId)
{
    GSLAM::WriteMutex lock(_mutexPt);
    std::map<GSLAM::FrameID,size_t>::iterator it=_obs.find(frId);
    if(it!=_obs.end())
    {
        // already inserted!
        DLOG(WARNING)<<"Failed to add observation. Already inserted!";
        return false;
    }
    _obs.insert(std::make_pair(frId,featId));
    return true;
}

bool MapPoint::eraseObservation(GSLAM::FrameID frId)
{
    GSLAM::WriteMutex lock(_mutexPt);
    std::map<GSLAM::FrameID,size_t>::iterator it=_obs.find(frId);
    if(it==_obs.end()) return false;//not exist!

    _obs.erase(it);
    return true;
}

bool MapPoint::clearObservation()
{
    GSLAM::WriteMutex lock(_mutexPt);
    _obs.clear();
	return true;
}
