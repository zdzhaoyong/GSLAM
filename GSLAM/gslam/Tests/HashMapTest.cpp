#include "gtest.h"
#include "../../core/HashMap.h"
#include "../../core/Svar.h"
#include "../../core/Random.h"
#include  "../../core/Timer.h"

using namespace GSLAM;

class HashMapTest : public testing::Test
{
public:
    HashMapTest():
        _maxID(svar.GetInt("HashMapTest.MaxID",1000)),
        _shouldStop(false),
        _map(new GSLAM::HashMap())
    {}

    void  eraseThread()
    {
        while(!_shouldStop)
        {
            GSLAM::FrameID fid=Random::RandomInt(1,_maxID);
            if(_map->getFrame(fid))
                _map->eraseMapFrame(fid);

            GSLAM::FrameID pid=Random::RandomInt(1,_maxID);
            if(_map->getPoint(pid))
                _map->eraseMapPoint(pid);
            Rate::sleep(0.001);
        }
    }

    void  addThread()
    {
        while(!_shouldStop)
        {
            GSLAM::FrameID fid=Random::RandomInt(1,_maxID);
            if(!_map->getFrame(fid))
            {
                GSLAM::FramePtr fr(new GSLAM::MapFrame(fid));
                _map->insertMapFrame(fr);
            }

            GSLAM::FrameID pid=Random::RandomInt(1,_maxID);
            if(!_map->getPoint(pid))
            {
                GSLAM::PointPtr pt(new GSLAM::MapPoint(pid));
                _map->insertMapPoint(pt);
            }
            Rate::sleep(0.001);
        }
    }

    int            _maxID;
    bool           _shouldStop;
    GSLAM::MapPtr  _map;
};

TEST_F(HashMapTest,MultiThreadReadWrite)
{
    std::thread addT(&HashMapTest::addThread,this);
    std::thread eraseT(&HashMapTest::eraseThread,this);
    Rate::sleep(svar.GetDouble("HashMapTest.MutiThreadSeconds",0.001));
    _shouldStop=true;
    addT.join();
    eraseT.join();
}
