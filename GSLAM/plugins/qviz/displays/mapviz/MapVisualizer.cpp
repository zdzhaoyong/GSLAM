#if QT_VERSION>=0x050000
#include <QtOpenGL/QGLFunctions>
#include <QOpenGLFunctions_3_3_Core>
#else
#include <GL/glew.h>
#endif

#include <QTabWidget>
#include <QGridLayout>

#include <GSLAM/core/GSLAM.h>
#include <GSLAM/core/Display.h>

using namespace GSLAM;
using namespace std;

#if QT_VERSION>=0x050000
class MapVisualizer : public QOpenGLFunctions
        #else
class MapVisualizer : public GSLAM::GObject
        #endif
{
public:
    static Svar displayPlugin(){
        SvarFunction create=[]()->Svar{
            std::shared_ptr<MapVisualizer> vis(new MapVisualizer());
            Svar display=vis->_config;
            display["__holder__"]=vis;
            display["__icon__"]=":/icon/mapviz.png";
            display["__name__"]="Map Visualizer";
            return display;
        };
        Svar plugin;
        plugin["__icon__"]=":/icon/mapviz.png";
        plugin["__name__"]="Map Visualizer";
        plugin["__type__"]=SvarClass::instance<MapPtr>();
        plugin["__init__"]=create;
        return plugin;
    }

    MapVisualizer():_vetexTrajBuffer(0),_mapUpdated(false),_curFrameUpdated(false){
        _curFrame.get_scale()=-1;
        _pubScenceOrigin=messenger.advertise<GSLAM::Point3d>("mapviz/gl_origin");
        _pubScenceRadius=messenger.advertise<double>("qviz/gl_radius");
        _pubScenceCenter=messenger.advertise<Point3d>("qviz/gl_center");

        _config["subDraw"]=messenger.subscribe("qviz/gl_draw",[this](Svar status){
              this->draw();
        });

        _pubUpdateGL=messenger.advertise<bool>("qviz/gl_update");

        _config.arg("map_topic",Topic(SvarClass::instance<MapPtr>()),"the map to subscribe");
        _config.arg("curframe_topic",Topic(SvarClass::instance<FramePtr>()),"the current frame");
        _config.arg("enable",true,"draw this map or not");
        _config.arg("trajectory",true,"draw the trajectory");
        _config.arg("trajectory_width",2.5,"the trajectory width");
        _config.arg("pointcloud",true,"draw the pointcloud or not");
        _config.arg("pointcloud_size",2.5,"the pointcloud size");
        _config.arg("gps_trajectory",true,"draw gps trajectory or not");
        _config.arg("connections",true,"draw connections or not");
        _config.arg("frames",true,"draw frames or not");
        _config.arg("current_frame",true,"draw current frame or not");

        _config["__cbk__map_topic"]=SvarFunction([this](){
            Topic topic=_config.get("map_topic",Topic());
            this->_config["subMap"]=messenger.subscribe(topic.name(),0,&MapVisualizer::update,this);
        });
        _config["__cbk__curframe_topic"]=SvarFunction([this](){
            Topic topic=_config.get("curframe_topic",Topic());
            this->_config["_subCurFrame"]=messenger.subscribe(topic.name(),[this](FramePtr fr){
//                    this->updateCurrentFrame(fr);
            });
        });
        _config["__cbk__"]=Svar::lambda([this](){
            this->_pubUpdateGL.publish(true);
        });
    }

    virtual void draw()
    {
        if(!_config.get("enable",true)) return;

        GSLAM::ReadMutex lock(_mutex);
        glPushMatrix();
//        glTranslated(_scenceOrigin.x,_scenceOrigin.y,_scenceOrigin.z);

        double trajectoryWidth=_config.get("trajectory_width",2.5);
        double connectionWidth=_config.get("connection_width",1.5);
        double pointCloudSize =_config.get("pointcloud_size",2.5);

        GSLAM::Point3ub trajectoryColor     =_config.get("trajectory_color",Point3ub(255,255,0));
        GSLAM::Point3ub gpsTrajectoryColor  =_config.get("gps_trajectory_color",Point3ub(255,0,0));
        GSLAM::Point3ub connectionColor     =_config.get("connection_color",Point3ub(0,255,255));
        GSLAM::Point3ub frameColor          =_config.get("frame_color",Point3ub(0,0,255));
        GSLAM::Point3ub curFrameColor       =_config.get("current_frame_color",Point3ub(255,0,0));

        if(!_vetexTrajBuffer)
        {
            glewInit();
            glGenBuffers(7, &_vetexTrajBuffer);
        }

        if(_mapUpdated)
        {
            if(_vetexTraj.size())
            {
                glBindBuffer(GL_ARRAY_BUFFER,_vetexTrajBuffer);
                glBufferData(GL_ARRAY_BUFFER,_vetexTraj.size()*sizeof(Point3f),_vetexTraj.data(), GL_STATIC_DRAW);
            }

            if(_gpsTraj.size())
            {
                glBindBuffer(GL_ARRAY_BUFFER,_gpsTrajBuffer);
                glBufferData(GL_ARRAY_BUFFER,_gpsTraj.size()*sizeof(Point3f),_gpsTraj.data(), GL_STATIC_DRAW);
            }

            if(_vetexConnection.size()){
                glBindBuffer(GL_ARRAY_BUFFER,_vetexConnectionBuffer);
                glBufferData(GL_ARRAY_BUFFER,_vetexConnection.size()*sizeof(Point3d),_vetexConnection.data(), GL_STATIC_DRAW);
            }

            if(_gpsError.size()){
                glBindBuffer(GL_ARRAY_BUFFER,_gpsErrorBuffer);
                glBufferData(GL_ARRAY_BUFFER,_gpsError.size()*sizeof(Point3d),_gpsError.data(), GL_STATIC_DRAW);
            }

            if(_pointCloudVertex.size()){
                glBindBuffer(GL_ARRAY_BUFFER,_pointCloudVertexBuffer);
                glBufferData(GL_ARRAY_BUFFER,_pointCloudVertex.size()*sizeof(Point3f),_pointCloudVertex.data(), GL_STATIC_DRAW);
            }

            if(_pointCloudColors.size()){
                glBindBuffer(GL_ARRAY_BUFFER,_pointCloudColorsBuffer);
                glBufferData(GL_ARRAY_BUFFER,_pointCloudColors.size()*sizeof(Point3ub),_pointCloudColors.data(), GL_STATIC_DRAW);
            }

            _mapUpdated=false;
        }

        if(_curFrameUpdated)
        {
            if(_curConnection.size())
            {
                glBindBuffer(GL_ARRAY_BUFFER,_curConnectionBuffer);
                glBufferData(GL_ARRAY_BUFFER,_curConnection.size()*sizeof(Point3d),_curConnection.data(), GL_STATIC_DRAW);
            }
            _curFrameUpdated=false;
        }

        if(_config.get("trajectory",true))
        {
            glDisable(GL_LIGHTING);
            glLineWidth(trajectoryWidth);
            glColor3f(trajectoryColor.x,trajectoryColor.y,trajectoryColor.z);
            glBindBuffer(GL_ARRAY_BUFFER,_vetexTrajBuffer);
            glVertexPointer(3, GL_FLOAT, 0, 0);
            glEnableClientState(GL_VERTEX_ARRAY);
            glDrawArrays(GL_LINE_STRIP,0,_vetexTraj.size());
            glDisableClientState(GL_VERTEX_ARRAY);
        }

        if(_config.get("gps_trajectory",true))
        {
            glDisable(GL_LIGHTING);
            glBindBuffer(GL_ARRAY_BUFFER,_gpsTrajBuffer);
            glVertexPointer(3, GL_FLOAT, 0, 0);
            glLineWidth(trajectoryWidth);
            glColor3ub(gpsTrajectoryColor.x,gpsTrajectoryColor.y,gpsTrajectoryColor.z);
            glEnableClientState(GL_VERTEX_ARRAY);
            glDrawArrays(GL_LINE_STRIP,0,_gpsTraj.size());
            glDisableClientState(GL_VERTEX_ARRAY);
        }

        if(_config.get("connections",true))
        {
            glBindBuffer(GL_ARRAY_BUFFER,_vetexConnectionBuffer);
            glVertexPointer(3, GL_DOUBLE, 0, 0);
            glLineWidth(connectionWidth);
            glColor3ub(connectionColor.x,connectionColor.y,connectionColor.z);
            glEnableClientState(GL_VERTEX_ARRAY);
            glDrawArrays(GL_LINES,0,_vetexConnection.size());
            glDisableClientState(GL_VERTEX_ARRAY);
        }

        if(_config.get("pointcloud",true))
        {
            glBindBuffer(GL_ARRAY_BUFFER,_pointCloudVertexBuffer);
            glVertexPointer(3, GL_FLOAT, 0, 0);
            glBindBuffer(GL_ARRAY_BUFFER,_pointCloudColorsBuffer);
            glColorPointer(3, GL_UNSIGNED_BYTE, 0, 0);
            glEnableClientState(GL_VERTEX_ARRAY);
            glEnableClientState(GL_COLOR_ARRAY);
            glPointSize(pointCloudSize);
            glDrawArrays(GL_POINTS,0,_pointCloudVertex.size());
            glDisableClientState(GL_COLOR_ARRAY);
            glDisableClientState(GL_VERTEX_ARRAY);
        }

        if(_config.get("frames",true))
        {
            for(auto sim3:_keyframes)
            {
                drawRect(sim3,frameColor);
            }
        }

        if(_config.get("current_frame",true))
        {
            drawRect(_curFrame,curFrameColor);

            glBindBuffer(GL_ARRAY_BUFFER,_curConnectionBuffer);
            glVertexPointer(3, GL_DOUBLE, 0, 0);
            glLineWidth(connectionWidth);
            glEnableClientState(GL_VERTEX_ARRAY);
            glDrawArrays(GL_LINES,0,_curConnection.size());
            glDisableClientState(GL_VERTEX_ARRAY);
        }
        glPopMatrix();
    }

    void glVertex(const GSLAM::Point3f& p){glVertex3f(p.x,p.y,p.z);}
    void glVertex(const GSLAM::Point3d& p){glVertex3d(p.x,p.y,p.z);}
    void drawRect(GSLAM::SIM3 pose,GSLAM::ColorType color)
    {
        if(pose.get_scale()<=0) return;
        if(!_camera.isValid()) _camera=GSLAM::Camera({640.,480.,500.,500.,320.,240.});
        {
            Point3d t=pose.get_translation();
            Point3d tl=_camera.UnProject(Point2d(0,0));
            Point3d tr=_camera.UnProject(Point2d(_camera.width(),0));
            Point3d bl=_camera.UnProject(Point2d(0,_camera.height()));
            Point3d br=_camera.UnProject(Point2d(_camera.width(),_camera.height()));

            GSLAM::Point3Type  W_tl=pose*(Point3d(tl.x,tl.y,1));
            GSLAM::Point3Type  W_tr=pose*(Point3d(tr.x,tr.y,1));
            GSLAM::Point3Type  W_bl=pose*(Point3d(bl.x,bl.y,1));
            GSLAM::Point3Type  W_br=pose*(Point3d(br.x,br.y,1));

            glBegin(GL_LINES);
            glLineWidth(2.5);
            glColor3ub(color.x,color.y,color.z);
            glVertex(t);        glVertex(W_tl);
            glVertex(t);        glVertex(W_tr);
            glVertex(t);        glVertex(W_bl);
            glVertex(t);        glVertex(W_br);
            glVertex(W_tl);     glVertex(W_tr);
            glVertex(W_tr);     glVertex(W_br);
            glVertex(W_br);     glVertex(W_bl);
            glVertex(W_bl);     glVertex(W_tl);
            glEnd();
        }
    }

    void update(const GSLAM::MapPtr& _map)
    {
        // if _map is empty, then clear all data
        if(!_map)
        {
            // FIXME: need add new draw item for clear
            _keyframes.clear();
            _vetexTraj.clear();
            _gpsTraj.clear();
            _gpsError.clear();
            _vetexConnection.clear();
            _pointCloudVertex.clear();
            _pointCloudColors.clear();
            _curConnection.clear();

            return;
        }
        this->_map=_map;

        GSLAM::FrameArray mapFrames;
        GSLAM::PointArray mapPoints;
        if(!_map->getFrames(mapFrames)) return;
        if(!_map->getPoints(mapPoints)) return;

        std::sort(mapFrames.begin(),mapFrames.end(),[](GSLAM::FramePtr a,GSLAM::FramePtr b)
        {
            return a->id()<b->id();
        });

        std::vector<GSLAM::SIM3>    keyframes;
        std::vector<GSLAM::Point3f> vetexTraj,gpsTraj;
        std::vector<GSLAM::Point3d> vetexConnection,gpsError;
        GSLAM::Point3d              boxMin(1e10,1e10,1e10),boxMax(-1e10,-1e10,-1e10);
        bool                        centerSeted=false;
        GSLAM::Point3d              center;


        int                  maxKeyFrameId = 0;


        for(GSLAM::FramePtr& fr:mapFrames)
        {
            if (fr->id()>maxKeyFrameId) maxKeyFrameId = fr->id();


            if(!centerSeted)
            {
                center=fr->getPose().get_translation();
                centerSeted=true;
            }
            Point3d t=fr->getPose().get_translation()-center;
            keyframes.push_back(GSLAM::SIM3(fr->getPose().get_rotation(),
                                            t,
                                            fr->getMedianDepth()*0.1));

            vetexTraj.push_back(t);
            boxMin.x=std::min(boxMin.x,t.x);boxMax.x=std::max(boxMax.x,t.x);
            boxMin.y=std::min(boxMin.y,t.y);boxMax.y=std::max(boxMax.y,t.y);
            boxMin.z=std::min(boxMin.z,t.z);boxMax.z=std::max(boxMax.z,t.z);

            Point3d ecef;
            if(fr->getGPSECEF(ecef))
            {
                ecef=ecef-center;
                gpsTraj.push_back(ecef);
                gpsError.push_back(t);
                gpsError.push_back(ecef);
            }

            std::map<GSLAM::FrameID,FrameConnectionPtr> children;
            if(!fr->getParents(children)) continue;
            for(std::pair<GSLAM::FrameID,FrameConnectionPtr> child:children)
            {
                GSLAM::FramePtr ch=_map->getFrame(child.first);
                if(!ch) continue;
                vetexConnection.push_back(t);
                vetexConnection.push_back(ch->getPoseScale().get_translation()-center);
            }
        }

        std::vector<GSLAM::Point3f>   pointCloudVertex;
        std::vector<GSLAM::Point3ub>  pointCloudColors;
        if(mapPoints.size())
        {
            pointCloudVertex.reserve(mapPoints.size());
            pointCloudColors.reserve(mapPoints.size());
            for(GSLAM::PointPtr& pt:mapPoints)
            {
                pointCloudVertex.push_back(pt->getPose()-center);
                auto color=pt->getColor();
                pointCloudColors.push_back(color);
            }
        }
        else
        {
            GSLAM::Point2d idepth;
            GSLAM::Point2f pt;
            GSLAM::ColorType color;
            for(GSLAM::FramePtr& fr:mapFrames)
            {
                GSLAM::Camera cam=fr->getCamera();
                GSLAM::SIM3   sim3=fr->getPoseScale();
                for(int i=0,iend=fr->keyPointNum();i<iend;i++)
                {
                    if(!fr->getKeyPointIDepthInfo(i,idepth)) break;
                    if(!fr->getKeyPoint(i,pt)) break;
                    if(!fr->getKeyPointColor(i,color)) break;
                    if(idepth.y>1000)
                    {
                        pointCloudVertex.push_back(sim3*(cam.UnProject(pt)/idepth.x)-center);
                        pointCloudColors.push_back(color);//osg::Vec4(color.x/255.,color.y/255.,color.z/255.,1));
                    }
                }
            }
        }

        {
            GSLAM::WriteMutex lock(_mutex);
            _keyframes=keyframes;
            _vetexTraj=vetexTraj;
            _gpsTraj=gpsTraj;
            _gpsError=gpsError;
            _vetexConnection=vetexConnection;
            _pointCloudVertex=pointCloudVertex;
            _pointCloudColors=pointCloudColors;
            _mapUpdated=true;
            _scenceCenter=(boxMax+boxMin)/2.;
            _scenceRadius=(boxMax-boxMin).norm()/2;
            if(_curFrame.get_scale()>0)
                _viewPoint.get_rotation()=_curFrame.get_rotation();
            else if(_keyframes.size())
                _viewPoint.get_rotation()=_keyframes.back().get_rotation();
            double r[9];_viewPoint.get_rotation().getMatrix(r);
            _viewPoint.get_translation()=_scenceCenter-_scenceRadius*GSLAM::Point3d(r[2],r[5],r[8]);
            _scenceOrigin=center;
            _pubScenceOrigin.publish(_scenceOrigin);
        }
        _pubScenceRadius.publish(_scenceRadius);
        _pubScenceCenter.publish(_scenceCenter);
        _pubUpdateGL.publish(true);
        //        LOG(INFO)<<"updated map."<<_keyframes.size()<<",points:"<<_pointCloudVertex.size()<<",radius:"<<_scenceRadius;
    }

    void updateCurrentFrame(const GSLAM::FramePtr& curFrame,GSLAM::MapPtr map=MapPtr())
    {
        if(!map) map=_map;
        std::map<GSLAM::FrameID,FrameConnectionPtr > parents;
        if(!curFrame->getParents(parents)) return;
        _curFrame=GSLAM::SIM3(curFrame->getPose(),curFrame->getMedianDepth()*0.1);
        _curFrame.get_translation()=_curFrame.get_translation()-_scenceOrigin;
        Point3d t=_curFrame.get_translation();
        std::vector<Point3d> curConnection;
        curConnection.reserve(parents.size()*2);
        if(map)
        for(auto parent:parents)
        {
            GSLAM::FramePtr fr=map->getFrame(parent.first);
            if(!fr) continue;
            curConnection.push_back(t);
            curConnection.push_back(fr->getPose().get_translation()-_scenceOrigin);
        }
        {
            GSLAM::WriteMutex lock(_mutex);
            _curConnection=curConnection;
            _curFrameUpdated=true;
        }
    }

    Svar                    _config;

    bool                    _firstFrame;
    GSLAM::MutexRW          _mutex;
    std::vector<GSLAM::Point3f>    _vetexTraj,_gpsTraj;
    std::vector<GSLAM::Point3d>    _vetexConnection,_gpsError;
    std::vector<GSLAM::Point3f>    _pointCloudVertex;
    std::vector<GSLAM::Point3ub>   _pointCloudColors;
    std::vector<GSLAM::SIM3> _keyframes;
    GSLAM::SIM3             _curFrame;
    std::vector<GSLAM::Point3d>    _curConnection;
    GSLAM::Camera           _camera;

    uint                    _vetexTrajBuffer,_gpsTrajBuffer;
    uint                    _vetexConnectionBuffer,_gpsErrorBuffer,_curConnectionBuffer;
    uint                    _pointCloudVertexBuffer;
    uint                    _pointCloudColorsBuffer;
    bool                    _mapUpdated,_curFrameUpdated;
    GSLAM::Point3d          _scenceCenter,_scenceOrigin;
    double                  _scenceRadius;
    GSLAM::SE3              _viewPoint;

    GSLAM::MapPtr           _map;
    GSLAM::Publisher        _pubScenceOrigin,_pubUpdateGL,_pubScenceRadius,_pubScenceCenter;
};

REGISTER_SVAR_MODULE(){
    GSLAM_REGISTER_GLOG_SINKS;
    GSLAM_REGISTER_MESSENGER;
    svar["gslam"]["displays"]["map"]=MapVisualizer::displayPlugin();
}
EXPORT_SVAR_INSTANCE
