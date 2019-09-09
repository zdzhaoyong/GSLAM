#include <fstream>
#include <unordered_map>

#ifdef HAS_OPENCV
#include <opencv2/highgui/highgui.hpp>
#endif

#include "MapFrame.h"
#include "MapPoint.h"

using namespace std;
using GSLAM::PointPtr;
using GSLAM::PointID;
using GSLAM::FrameID;
using GSLAM::FramePtr;
using GSLAM::PointArray;
using GSLAM::FrameArray;
using GSLAM::ReadMutex;
using GSLAM::WriteMutex;
using GSLAM::LoopDetectorPtr;
using GSLAM::MutexRW;
using GSLAM::LoopCandidates;

typedef std::unordered_map<PointID,PointPtr> PointMap;
typedef std::unordered_map<FrameID,FramePtr> FrameMap;
typedef PointMap::iterator                   PointIt;
typedef FrameMap::iterator                   FrameIt;
typedef PointMap::const_iterator             ConstPointIt;
typedef FrameMap::const_iterator             ConstFrameIt;

class MapHash : public GSLAM::Map
{
public:
    MapHash();
    ~MapHash();
    virtual std::string type()const{return "MapHash";}

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

    virtual bool     setLoopDetector(const LoopDetectorPtr& loopdetector){
        _loopDetector=loopdetector;
        return true;
    }

    virtual LoopDetectorPtr getLoopDetector()const{return _loopDetector;}
    virtual bool     obtainCandidates(const FramePtr& frame,LoopCandidates& candidates){
        if(!_loopDetector)
            return false;
        return _loopDetector->obtainCandidates(frame,candidates);
    }

    /// Save or load the map from/to the file
    virtual bool save(std::string path)const;
    virtual bool load(std::string path);

    virtual void  draw();

private:
    bool saveTrajectory(std::string filename="trajectory.txt")const;
    bool savePointCloud(std::string filename)const;
    bool saveMap2DFusion(std::string filepath)const;
    bool saveMapFusion(std::string file2save="mapfusion.txt")const;
    bool saveDense(std::string folder2save)const;

    class MapFrameConnection : public GSLAM::FrameConnection
    {
    public:
        MapFrameConnection(int matchesNum=0){}
        virtual std::string type()const{return "MapFrameConnection";}
        virtual int  matchesNum(){return _matchesNum;}

        int _matchesNum;
    };

    PointMap _points;
    FrameMap _frames;
    mutable MutexRW  _mutexPoints,_mutexFrames;
    LoopDetectorPtr  _loopDetector;
};


