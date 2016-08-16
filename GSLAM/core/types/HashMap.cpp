#include "HashMap.h"

namespace GSLAM{

HashMap::HashMap()
{
}


bool HashMap::insertMapPoint(const PointPtr& point)
{
    pi::WriteMutex lock(_mutexPoints);
    PointIt it=_points.find(point->id());
    if(it!=_points.end()) return false;
    _points.insert(make_pair(point->id(),point));
    return true;
}

bool HashMap::insertMapFrame(const FramePtr& frame)
{
    pi::WriteMutex lock(_mutexFrames);
    FrameIt it=_frames.find(frame->id());
    if(it!=_frames.end()) return false;
    _frames.insert(make_pair(frame->id(),frame));
    return true;
}

bool HashMap::eraseMapPoint(const PointID& pointId)
{
    pi::WriteMutex lock(_mutexPoints);
    return _points.erase(pointId);
}

bool HashMap::eraseMapFrame(const FrameID& frameId)
{
    pi::WriteMutex lock(_mutexFrames);
    return _frames.erase(frameId);
}

std::size_t HashMap::frameNum()const
{
    pi::ReadMutex lock(_mutexFrames);
    return _frames.size();
}

std::size_t HashMap::pointNum()const
{
    pi::ReadMutex lock(_mutexPoints);
    return _points.size();
}

FramePtr HashMap::getFrame(const FrameID& id)
{
    pi::ReadMutex lock(_mutexFrames);
    return _frames[id];
}

PointPtr HashMap::getPoint(const PointID& id)
{
    pi::ReadMutex lock(_mutexPoints);
    return _points[id];
}

bool  HashMap::getFrames(FrameArray& frames)
{
    pi::ReadMutex lock(_mutexFrames);
    frames.clear();
    frames.reserve(_frames.size());
    for(const std::pair<FrameID,FramePtr>& fr:_frames) frames.push_back(fr.second);
    return true;
}

bool  HashMap::getPoints(PointArray& points)
{
    pi::ReadMutex lock(_mutexPoints);
    points.clear();
    points.reserve(_points.size());
    for(const std::pair<PointID,PointPtr>& pt:_points) points.push_back(pt.second);
    return true;
}


/// Save or load the map from/to the file
bool HashMap::save(std::string path)const
{
    return false;
}

bool HashMap::load(std::string path)
{
    return false;
}


}
