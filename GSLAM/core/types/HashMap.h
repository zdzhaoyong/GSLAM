#ifndef HASHMAP_H
#define HASHMAP_H
#include <GSLAM/core/GSLAM.h>
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
    HashMap();

    virtual ~HashMap(){}

    virtual std::string type(){return "HashMap";}

    /// MapFrame & MapPoint interface
    virtual bool insertMapPoint(const PointPtr& point);
    virtual bool insertMapFrame(const FramePtr& frame);
    virtual bool eraseMapPoint(const PointID& pointId);
    virtual bool eraseMapFrame(const FrameID& frameId);

    virtual std::size_t frameNum()const;
    virtual std::size_t pointNum()const;

    virtual FramePtr getFrame(const FrameID& id);
    virtual PointPtr getPoint(const PointID& id);
    virtual bool     getFrames(FrameArray& frames);
    virtual bool     getPoints(PointArray& points);

    /// Save or load the map from/to the file
    virtual bool save(std::string path)const;
    virtual bool load(std::string path);

    /// 0 is reserved for INVALID
    PointID getPid(){return _ptId++;}//obtain an unique point id
    FrameID getFid(){return _frId++;}//obtain an unique frame id

     PointMap _points;

};

}
#endif // HASHMAP_H
