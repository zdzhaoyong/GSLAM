#ifndef HASHMAP_H
#define HASHMAP_H
#include "GSLAM.h"
#include <unordered_map>

namespace GSLAM
{

typedef std::unordered_map<PointID,PointPtr> PointMap;
typedef std::unordered_map<FrameID,FramePtr> FrameMap;
typedef PointMap::iterator                   PointIt;
typedef FrameMap::iterator                   FrameIt;
typedef PointMap::const_iterator             ConstPointIt;
typedef FrameMap::const_iterator             ConstFrameIt;

class HashMap : public Map
{
public:
    HashMap(){}

    virtual ~HashMap(){}

    virtual std::string type()const{return "HashMap";}

    /// MapFrame & MapPoint interface
    virtual bool insertMapPoint(const PointPtr& point);
    virtual bool insertMapFrame(const FramePtr& frame);
    virtual bool eraseMapPoint(const PointID& pointId);
    virtual bool eraseMapFrame(const FrameID& frameId);
    virtual void clear();

    virtual std::size_t frameNum()const;
    virtual std::size_t pointNum()const;

    virtual FramePtr getFrame(const FrameID& id)const;
    virtual PointPtr getPoint(const PointID& id)const;
    virtual bool     getFrames(FrameArray& frames)const;
    virtual bool     getPoints(PointArray& points)const;

    /// Save or load the map from/to the file
    virtual bool save(std::string path)const;
    virtual bool load(std::string path);

private:
    PointMap _points;
    FrameMap _frames;
    mutable MutexRW  _mutexPoints,_mutexFrames;
};

inline bool HashMap::insertMapPoint(const PointPtr& point)
{
    WriteMutex lock(_mutexPoints);
    PointIt it=_points.find(point->id());
    if(it!=_points.end()) return false;
    _points.insert(make_pair(point->id(),point));
    return true;
}

inline bool HashMap::insertMapFrame(const FramePtr& frame)
{
    WriteMutex lock(_mutexFrames);
    FrameIt it=_frames.find(frame->id());
    if(it!=_frames.end()) return false;
    _frames.insert(make_pair(frame->id(),frame));
    return true;
}

inline bool HashMap::eraseMapPoint(const PointID& pointId)
{
    WriteMutex lock(_mutexPoints);
    return _points.erase(pointId)!=0;
}

inline bool HashMap::eraseMapFrame(const FrameID& frameId)
{
    WriteMutex lock(_mutexFrames);
    return _frames.erase(frameId)!=0;
}

inline void  HashMap::clear()
{
    WriteMutex lock1(_mutexFrames);
    WriteMutex lock2(_mutexPoints);
    _points.clear();
    _frames.clear();
}

inline std::size_t HashMap::frameNum()const
{
    ReadMutex lock(_mutexFrames);
    return _frames.size();
}

inline std::size_t HashMap::pointNum()const
{
    ReadMutex lock(_mutexPoints);
    return _points.size();
}

inline FramePtr HashMap::getFrame(const FrameID& id)const
{
    ReadMutex lock(_mutexFrames);
    ConstFrameIt it=_frames.find(id);
    if(it!=_frames.end())
        return it->second;
    else
        return FramePtr();
}

inline PointPtr HashMap::getPoint(const PointID& id)const
{
    ReadMutex lock(_mutexPoints);
    ConstPointIt it=_points.find(id);
    if(it!=_points.end())
        return it->second;
    else
        return PointPtr();
}

inline bool  HashMap::getFrames(FrameArray& frames)const
{
    ReadMutex lock(_mutexFrames);
    frames.clear();
    frames.reserve(_frames.size());
    for(const std::pair<FrameID,FramePtr>& fr:_frames) frames.push_back(fr.second);
    return true;
}

inline bool  HashMap::getPoints(PointArray& points)const
{
    ReadMutex lock(_mutexPoints);
    points.clear();
    points.reserve(_points.size());
    for(const std::pair<PointID,PointPtr>& pt:_points) points.push_back(pt.second);
    return true;
}


/// Save or load the map from/to the file
inline bool HashMap::save(std::string path)const
{
    return false;
}

inline bool HashMap::load(std::string path)
{
    return false;
}
}
#endif // HASHMAP_H
