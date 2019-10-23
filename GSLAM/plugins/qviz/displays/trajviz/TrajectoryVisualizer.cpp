#include <QTabWidget>
#include <QGridLayout>

#if QT_VERSION>=0x050000
#include <QtOpenGL/QGLFunctions>
#include <QOpenGLFunctions_3_3_Core>
#else
#include <GL/glew.h>
#endif


#include <GSLAM/core/GSLAM.h>
#include <GSLAM/core/Display.h>

using namespace GSLAM;
using namespace std;

#if QT_VERSION>=0x050000
class TrajectoryVisualizer : public QOpenGLFunctions
        #else
class TrajectoryVisualizer : public GSLAM::GObject
        #endif
{
public:
    static Svar displayPlugin(){
        SvarFunction create=[]()->Svar{
            std::shared_ptr<TrajectoryVisualizer> vis(new TrajectoryVisualizer());
            Svar display=vis->_config;
            display["__holder__"]=vis;
            display["__icon__"]=":/icon/trajviz.png";
            display["__name__"]="Trajectory Visualizer";
            return display;
        };
        Svar plugin;
        plugin["__icon__"]=":/icon/trajviz.png";
        plugin["__name__"]="Trajectory Visualizer";
        plugin["__type__"]=SvarClass::instance<FramePtr>();
        plugin["__init__"]=create;
        return plugin;
    }

    TrajectoryVisualizer():_vetexTrajBuffer(0),_curFrameUpdated(false),_isGLInitialized(false){
        _curFrame.get_scale()=-1;
        _pubScenceRadius=messenger.advertise<double>("qviz/gl_radius");
        _pubScenceCenter=messenger.advertise<Point3d>("qviz/gl_center");

        _config["subDraw"]=messenger.subscribe("qviz/gl_draw",[this](Svar status){
              this->draw();
        });

        _pubUpdateGL=messenger.advertise<bool>("qviz/gl_update");

        _config.arg("frame_topic",Topic(SvarClass::instance<FramePtr>()),"the frame topic");
        _config.arg("enable",true,"draw this map or not");
        _config.arg("trajectory_width",2.5,"the trajectory width");
        _config.arg("current_rect",true,"draw current frame or not");

        _config["__cbk__frame_topic"]=SvarFunction([this](){
            Topic topic=_config.get("frame_topic",Topic());
            this->_config["_subCurFrame"]=messenger.subscribe(topic.name(),[this](FramePtr fr){
               this->updateCurrentFrame(fr);
            });
        });
    }

    virtual void draw()
    {
        if(!_config.get("enable",true)) return;

        if(!_isGLInitialized){
#if QT_VERSION>=0x050000
            initializeOpenGLFunctions();
#else
            glewInit();
#endif
            _isGLInitialized=true;
        }


        GSLAM::ReadMutex lock(_mutex);
        glPushMatrix();
//        glTranslated(_scenceOrigin.x,_scenceOrigin.y,_scenceOrigin.z);

        double trajectoryWidth=_config.get("trajectory_width",2.5);

        GSLAM::Point3ub trajectoryColor     =_config.get("trajectory_color",Point3ub(255,255,0));
        GSLAM::Point3ub curFrameColor       =_config.get("current_frame_color",Point3ub(255,0,0));

        if(!_vetexTrajBuffer)
        {
            glGenBuffers(1, &_vetexTrajBuffer);
        }

        if(_curFrameUpdated)
        {
            if(_vetexTraj.size())
            {
                glBindBuffer(GL_ARRAY_BUFFER,_vetexTrajBuffer);
                glBufferData(GL_ARRAY_BUFFER,_vetexTraj.size()*sizeof(Point3f),_vetexTraj.data(), GL_STATIC_DRAW);
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

        if(_config.get("current_frame",true))
        {
            drawRect(_curFrame,curFrameColor);
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

    void updateCurrentFrame(GSLAM::FramePtr curFrame)
    {
        if(!curFrame) return;
        GSLAM::WriteMutex lock(_mutex);
        _curFrame=GSLAM::SIM3(curFrame->getPose(),curFrame->getMedianDepth()*0.1);
        _vetexTraj.push_back(_curFrame.get_translation());
        _curFrameUpdated=true;
    }

    Svar                    _config;

    GSLAM::MutexRW          _mutex;
    std::vector<GSLAM::Point3f>    _vetexTraj;
    GSLAM::SIM3              _curFrame;
    std::vector<GSLAM::Point3d>    _curConnection;
    GSLAM::Camera           _camera;

    uint                    _vetexTrajBuffer;
    bool                    _curFrameUpdated,_isGLInitialized;
    GSLAM::Point3d          _scenceCenter,_scenceOrigin;

    GSLAM::Publisher        _pubScenceOrigin,_pubUpdateGL,_pubScenceRadius,_pubScenceCenter;
};

REGISTER_SVAR_MODULE(trajviz){
    GSLAM_REGISTER_GLOG_SINKS;
    GSLAM_REGISTER_MESSENGER;
    svar["gslam"]["displays"]["traj"]=TrajectoryVisualizer::displayPlugin();
}
EXPORT_SVAR_INSTANCE
