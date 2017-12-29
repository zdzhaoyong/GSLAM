#if defined(HAS_QT)&&defined(HAS_QGLVIEWER)
#include <GL/glew.h>
#include "SLAMVisualizer.h"
#include "../../core/Event.h"

namespace GSLAM{
bool compareFr(GSLAM::FramePtr a,GSLAM::FramePtr b)
{
    return a->id()<b->id();   //升序排列，如果改为return a>b，则为降序
}

class MapVisualizer : public GSLAM::GObject
{
public:
    MapVisualizer():_vetexTrajBuffer(0),_mapUpdated(false),_curFrameUpdated(false){}

    virtual void draw()
    {
        GSLAM::ReadMutex lock(_mutex);

        double& trajectoryWidth=svar.GetDouble("MainWindow.TrajectoryWidth",2.5);
        double& connectionWidth=svar.GetDouble("MainWindow.ConnectionWidth",1.);
        double& pointCloudSize =svar.GetDouble("MainWindow.PointCloudSize",2.5);

        GSLAM::Point3ub trajectoryColor=svar.get_var("MainWindow.TrajectoryColor",Point3ub(255,255,0));
        GSLAM::Point3ub gpsTrajectoryColor=svar.get_var("MainWindow.GPSTrajectoryColor",Point3ub(255,0,0));
        GSLAM::Point3ub connectionColor=svar.get_var("MainWindow.ConnectionColor",Point3ub(0,255,255));
        GSLAM::Point3ub frameColor  =svar.get_var("MainWindow.FrameColor",Point3ub(0,0,255));
        GSLAM::Point3ub curFrameColor  =svar.get_var("MainWindow.CurrentFrameColor",Point3ub(255,0,0));

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

        if(svar.GetInt("MainWindow.DrawTrajectory",1))
        {
            glDisable(GL_LIGHTING);
            glBindBuffer(GL_ARRAY_BUFFER,_vetexTrajBuffer);
            glVertexPointer(3, GL_FLOAT, 0, 0);
            glLineWidth(trajectoryWidth);
            glColor3f(trajectoryColor.x,trajectoryColor.y,trajectoryColor.z);
            glEnableClientState(GL_VERTEX_ARRAY);
            glDrawArrays(GL_LINE_STRIP,0,_vetexTraj.size());
            glDisableClientState(GL_VERTEX_ARRAY);
        }

        if(svar.GetInt("MainWindow.DrawGPSTraj",1))
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

        if(svar.GetInt("MainWindow.DrawConnection",1))
        {
            glBindBuffer(GL_ARRAY_BUFFER,_vetexConnectionBuffer);
            glVertexPointer(3, GL_DOUBLE, 0, 0);
            glLineWidth(connectionWidth);
            glColor3ub(connectionColor.x,connectionColor.y,connectionColor.z);
            glEnableClientState(GL_VERTEX_ARRAY);
            glDrawArrays(GL_LINES,0,_vetexConnection.size());
            glDisableClientState(GL_VERTEX_ARRAY);
        }


        if(svar.GetInt("MainWindow.DrawPointCloud",1))
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

        if(svar.GetInt("MainWindow.DrawFrames",1))
        {
            for(auto sim3:_keyframes)
            {
                drawRect(sim3,frameColor);
            }
        }

        if(svar.GetInt("MainWindow.DrawCurrentFrame",1))
        {
            drawRect(_curFrame,curFrameColor);

            glBindBuffer(GL_ARRAY_BUFFER,_curConnectionBuffer);
            glVertexPointer(3, GL_DOUBLE, 0, 0);
            glLineWidth(connectionWidth);
            glEnableClientState(GL_VERTEX_ARRAY);
            glDrawArrays(GL_LINES,0,_curConnection.size());
            glDisableClientState(GL_VERTEX_ARRAY);
        }

    }

    void glVertex(const GSLAM::Point3f& p){glVertex3f(p.x,p.y,p.z);}
    void glVertex(const GSLAM::Point3d& p){glVertex3d(p.x,p.y,p.z);}
    void drawRect(GSLAM::SIM3 pose,GSLAM::ColorType color)
    {
        if(!_camera.isValid()) _camera=GSLAM::Camera({640.,480.,500.,500.,320.,240.});
        {
            Point3d t=pose.get_translation();
            pi::Point3d tl=_camera.UnProject(pi::Point2d(0,0));
            pi::Point3d tr=_camera.UnProject(pi::Point2d(_camera.width(),0));
            pi::Point3d bl=_camera.UnProject(pi::Point2d(0,_camera.height()));
            pi::Point3d br=_camera.UnProject(pi::Point2d(_camera.width(),_camera.height()));

            GSLAM::Point3Type  W_tl=pose*(pi::Point3d(tl.x,tl.y,1));
            GSLAM::Point3Type  W_tr=pose*(pi::Point3d(tr.x,tr.y,1));
            GSLAM::Point3Type  W_bl=pose*(pi::Point3d(bl.x,bl.y,1));
            GSLAM::Point3Type  W_br=pose*(pi::Point3d(br.x,br.y,1));

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

    void update(GSLAM::MapPtr _map)
    {
        if(!_map) return ;

        GSLAM::FrameArray mapFrames;
        GSLAM::PointArray mapPoints;
        if(!_map->getFrames(mapFrames)) return;
        if(!_map->getPoints(mapPoints)) return;

        std::sort(mapFrames.begin(),mapFrames.end(),compareFr);

        std::vector<GSLAM::SIM3> keyframes;
        std::vector<Point3f> vetexTraj,gpsTraj;
        std::vector<Point3d> vetexConnection,gpsError;
        for(GSLAM::FramePtr& fr:mapFrames)
        {
            keyframes.push_back(GSLAM::SIM3(fr->getPose(),fr->getMedianDepth()*0.1));
            Point3d t=fr->getPose().get_translation();
            vetexTraj.push_back(t);

            Point3d ecef;
            if(fr->getGPSECEF(ecef))
            {
                gpsTraj.push_back(ecef);
                gpsError.push_back(t);
                gpsError.push_back(ecef);
            }

            std::map<GSLAM::FrameID,SPtr<GSLAM::FrameConnection> > children;
            if(!fr->getParents(children)) continue;
            for(std::pair<GSLAM::FrameID,SPtr<GSLAM::FrameConnection> > child:children)
            {
                GSLAM::FramePtr ch=_map->getFrame(child.first);
                if(!ch) continue;
                vetexConnection.push_back(t);
                vetexConnection.push_back(ch->getPoseScale().get_translation());
            }
        }

        std::vector<Point3f>   pointCloudVertex;
        std::vector<Point3ub>  pointCloudColors;
        if(mapPoints.size())
        {
            pointCloudVertex.reserve(mapPoints.size());
            pointCloudColors.reserve(mapPoints.size());
            for(GSLAM::PointPtr& pt:mapPoints)
            {
                pointCloudVertex.push_back(pt->getPose());
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
                        pointCloudVertex.push_back(sim3*(cam.UnProject(pt)/idepth.x));
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
        }
    }

    void update(GSLAM::MapPtr map,const GSLAM::FramePtr& curFrame)
    {
        std::map<GSLAM::FrameID,SPtr<GSLAM::FrameConnection> > parents;
        if(!curFrame->getParents(parents)) return;
        _curFrame=GSLAM::SIM3(curFrame->getPose(),curFrame->getMedianDepth()*0.1);
        Point3d t=_curFrame.get_translation();
        std::vector<Point3d> curConnection;
        curConnection.reserve(parents.size()*2);
        for(auto parent:parents)
        {
            GSLAM::FramePtr fr=map->getFrame(parent.first);
            if(!fr) continue;
            curConnection.push_back(t);
            curConnection.push_back(fr->getPose().get_translation());
        }
        {
            GSLAM::WriteMutex lock(_mutex);
            _curConnection=curConnection;
            _curFrameUpdated=true;
        }
    }

    GSLAM::MutexRW          _mutex;
    std::vector<Point3f>    _vetexTraj,_gpsTraj;
    std::vector<Point3d>    _vetexConnection,_gpsError;
    std::vector<Point3f>    _pointCloudVertex;
    std::vector<Point3ub>   _pointCloudColors;
    std::vector<GSLAM::SIM3> _keyframes;
    GSLAM::SIM3             _curFrame;
    std::vector<Point3d>    _curConnection;
    GSLAM::Camera           _camera;

    uint                    _vetexTrajBuffer,_gpsTrajBuffer;
    uint                    _vetexConnectionBuffer,_gpsErrorBuffer,_curConnectionBuffer;
    uint                    _pointCloudVertexBuffer;
    uint                    _pointCloudColorsBuffer;
    bool                    _mapUpdated,_curFrameUpdated;
};

class SLAMVisualizerImpl
{
public:
    SLAMVisualizerImpl(std::string pluginPath):_slamplugin(pluginPath),_firstFrame(true){}
    ~SLAMVisualizerImpl(){
        _slam.reset();
    }

    SLAMPtr         _slam;
    std::string     _slamplugin;
    MapVisualizer   _vis;
    bool            _firstFrame;
};

SLAMVisualizer::SLAMVisualizer(QWidget* parent,QString pluginPath)
    :QGLViewer(parent),impl(new SLAMVisualizerImpl(pluginPath.toStdString()))
{
}

bool open(QString pluginPath){

}

SLAMPtr SLAMVisualizer::slam(){
    if(!impl->_slam)
    {
        impl->_slam=SLAM::create(impl->_slamplugin);

        if(impl->_slam)
        {
            impl->_slam->call("SetSvar",&svar);
            impl->_slam->setCallback(this);
        }
    }
    return impl->_slam;
}

void SLAMVisualizer::draw(){
    if(!impl->_slam) return;
    if(impl->_slam->isDrawable()) impl->_slam->draw();
    else impl->_vis.draw();
}

void SLAMVisualizer::handle(const SPtr<GObject>& obj){
    if(!obj) return;
    if(FramePtr e=std::dynamic_pointer_cast<MapFrame>(obj))
    {
        e->setImage(GImage());
        if(!impl->_slam) return;
        impl->_vis.update(impl->_slam->getMap());
        if(impl->_firstFrame)
        {
            impl->_firstFrame=false;
            SE3 pose=e->getPose();
            const pi::Point3f& t=pose.get_translation();
            const pi::SO3f& r=pose.get_rotation();
            camera()->setPosition(qglviewer::Vec(t.x,t.y,t.z));
            camera()->setOrientation(qglviewer::Quaternion(r.w,r.z,-r.y,-r.x));
        }
        update();
    }
    else if(auto e=std::dynamic_pointer_cast<CurrentFrameEvent>(obj))
    {
        if(!impl->_slam) return;
        impl->_vis.update(impl->_slam->getMap(),e->_frame);
        update();
    }
    else if(obj->type()=="ScenceCenterEvent")
    {
        ScenceCenterEvent* sce=((ScenceCenterEvent*)obj.get());
        Point3d center=sce->_center;
        setSceneCenter(qglviewer::Vec(center.x,center.y,center.z));
        sce->setHandled();
        update();
    }
    else if(obj->type()=="ScenceRadiusEvent")
    {
        ScenceRadiusEvent* radiusE=(ScenceRadiusEvent*)obj.get();
        setSceneRadius(radiusE->_radius);
        update();
    }
}

}
#endif
