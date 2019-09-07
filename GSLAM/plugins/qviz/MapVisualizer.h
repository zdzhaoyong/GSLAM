#include <GSLAM/core/GSLAM.h>
#include <GSLAM/core/Messenger.h>

namespace GSLAM{
typedef unsigned int uint;
class MapVisualizer : public GSLAM::GObject
{
public:
    MapVisualizer(MapPtr slam_map,std::string slam_name,GObjectHandle* _handle);
    MapVisualizer(std::string slam_name,GObjectHandle* _handle);

    void setMap(const MapPtr& map){_map=map;update();}

    virtual void draw();
    void drawRect(GSLAM::SIM3 pose,GSLAM::ColorType color);

    void update();
    void updateCurframe(const GSLAM::FramePtr& curFrame);

private:
    static bool compareFr(GSLAM::FramePtr a,GSLAM::FramePtr b)
    {
        return a->id()<b->id();
    }

    MapPtr                  _map;
    std::string             _name;
    GObjectHandle*          _handle;

    GSLAM::MutexRW          _mutex;
    std::vector<Point3f>    _vetexTraj,_gpsTraj;
    std::vector<Point3d>    _vetexConnection,_gpsError;
    std::vector<Point3f>    _pointCloudVertex;
    std::vector<Point3ub>   _pointCloudColors;
    std::vector<GSLAM::SIM3> _keyframes,_gpsFrames;
    GSLAM::FramePtr          _curFrame;
    std::vector<Point3d>    _curConnection;
    GSLAM::Camera           _camera;

    uint                    _vetexTrajBuffer,_gpsTrajBuffer;
    uint                    _vetexConnectionBuffer,_gpsErrorBuffer,_curConnectionBuffer;
    uint                    _pointCloudVertexBuffer;
    uint                    _pointCloudColorsBuffer;
    bool                    _mapUpdated,_curFrameUpdated,_firstUpdate;
    GSLAM::Point3d          _scenceCenter,_scenceOrigin;
    float                   _scenceRadius;
    GSLAM::SE3              _viewPoint;
};


}
