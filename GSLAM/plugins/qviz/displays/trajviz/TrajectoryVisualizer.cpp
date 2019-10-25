#include <GSLAM/core/GSLAM.h>

using namespace GSLAM;
using namespace std;

class TrajectoryVisualizer
{
public:
    static Svar displayPlugin(){
        SvarFunction create=[]()->Svar{
            std::shared_ptr<TrajectoryVisualizer> vis(new TrajectoryVisualizer());
            Svar display;
            display["__holder__"]=vis;
            display["__icon__"]=":/icon/trajviz.png";
            display["__name__"]="Trajectory Visualizer";
            display.arg("frame_topic",Topic(SvarClass::instance<FramePtr>()),"the frame topic");

            display["__cbk__frame_topic"]=SvarFunction([vis](Topic topic){
                vis->_subCurFrame=messenger.subscribe(topic.name(),[vis](FramePtr fr){
                   vis->updateCurrentFrame(fr);
                });
            });
            return display;
        };
        Svar plugin;
        plugin["__icon__"]=":/icon/trajviz.png";
        plugin["__name__"]="Trajectory Visualizer";
        plugin["__type__"]=SvarClass::instance<FramePtr>();
        plugin["__init__"]=create;
        return plugin;
    }

    TrajectoryVisualizer(){


    }

    void updateCurrentFrame(GSLAM::FramePtr curFrame)
    {
        if(!curFrame) return;
        std::string topic_name=_subCurFrame.getTopic();
        NodeGLPtr nodeTraj(new NodeGL(topic_name+"/trajectory"));
        NodeGLPtr nodeCurrent(new NodeGL(topic_name+"/current"));
        nodeTraj->displayMode=NodeGL::LINE_STRIP;
        nodeCurrent->displayMode=NodeGL::LINES;
        {
            GSLAM::WriteMutex lock(_mutex);
            SIM3 pose=GSLAM::SIM3(curFrame->getPose(),curFrame->getMedianDepth()*0.1);
            Camera _camera=curFrame->getCamera();
            _vetexTraj.push_back(pose.get_translation());

            nodeTraj->vertices=_vetexTraj;
            nodeTraj->colors.push_back(Point3ub(255,255,0));

            Point3d t=pose.get_translation();
            Point3d tl=_camera.UnProject(Point2d(0,0));
            Point3d tr=_camera.UnProject(Point2d(_camera.width(),0));
            Point3d bl=_camera.UnProject(Point2d(0,_camera.height()));
            Point3d br=_camera.UnProject(Point2d(_camera.width(),_camera.height()));

            GSLAM::Point3Type  W_tl=pose*(Point3d(tl.x,tl.y,1));
            GSLAM::Point3Type  W_tr=pose*(Point3d(tr.x,tr.y,1));
            GSLAM::Point3Type  W_bl=pose*(Point3d(bl.x,bl.y,1));
            GSLAM::Point3Type  W_br=pose*(Point3d(br.x,br.y,1));

            nodeCurrent->vertices=std::vector<Point3f>({t,W_tl,t,W_tr,t,W_bl,t,W_br,W_tl,W_tr,W_tr,W_br,W_br,W_bl,W_bl,W_tl});
            nodeCurrent->colors.push_back(Point3ub(255,0,0));
        }
        messenger.publish("qviz/gl_node",nodeTraj);
        messenger.publish("qviz/gl_node",nodeCurrent);
    }

    Subscriber              _subCurFrame;

    GSLAM::MutexRW          _mutex;
    std::vector<GSLAM::Point3f>    _vetexTraj;
};

REGISTER_SVAR_MODULE(trajviz){
    GSLAM_REGISTER_GLOG_SINKS;
    GSLAM_REGISTER_MESSENGER;
    svar["gslam"]["displays"]["traj"]=TrajectoryVisualizer::displayPlugin();
}
EXPORT_SVAR_INSTANCE
