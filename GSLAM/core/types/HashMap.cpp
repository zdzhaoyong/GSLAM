#include "HashMap.h"

namespace GSLAM{

HashMap::HashMap()
{
}


bool HashMap::insertMapPoint(const PointPtr& point)
{
    pi::WriteMutex lock(_mutexPoints);
    PointIt it=_points.find(point);
    if(it!=_points.end()) return false;
    _points.insert(make_pair(point->id(),point));
    return true;
}

bool HashMap::insertMapFrame(const FramePtr& frame)
{
    pi::WriteMutex lock(_mutexFrames);
    PointIt it=_frames.find(frame);
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
    return _frames.
}

std::size_t HashMap::frameNum()const
{

}

std::size_t HashMap::pointNum()const
{

}

FramePtr HashMap::getFrame(const FrameID& id)
{

}

PointPtr HashMap::getPoint(const PointID& id)
{

}

bool     getFrames(FrameArray& frames)
{

}

bool     getPoints(PointArray& points)
{

}


/// Save or load the map from/to the file
bool save(std::string path)const
{

}

bool load(std::string path)
{

}


}
