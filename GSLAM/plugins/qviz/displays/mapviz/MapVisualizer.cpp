#include <GSLAM/core/GSLAM.h>

using namespace GSLAM;
using namespace std;

class MapVisualizer
{
public:
    static Svar displayPlugin(){
        SvarFunction create=[]()->Svar{
            std::shared_ptr<MapVisualizer> vis(new MapVisualizer());
            Svar display;
            display["map"]=vis->_config;
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
        plugin["__stay__"]=true;
        return plugin;
    }

    MapVisualizer(){

        _pubNode=messenger.advertise<NodeGLPtr>("qviz/gl_node");
        _config.arg("map_topic",Topic(SvarClass::instance<MapPtr>()),"the map to subscribe");
        _config["__cbk__map_topic"]=SvarFunction([this](){
            Topic topic=_config.get("map_topic",Topic());
            this->_config["subMap"]=messenger.subscribe(topic.name(),[this](MapPtr mp){this->update(mp);});
        });
        _config["__cbk__curframe_topic"]=SvarFunction([this](){
            Topic topic=_config.get("curframe_topic",Topic());
            this->_config["_subCurFrame"]=messenger.subscribe(topic.name(),[this](FramePtr fr){
//                    this->updateCurrentFrame(fr);
            });
        });
    }

    void update(const GSLAM::MapPtr& _map)
    {
        std::string topicName=_config.get("map_topic",Topic()).name();
        NodeGLPtr keyframes(new NodeGL(topicName+"/keyframes"));
        NodeGLPtr slamTraj(new NodeGL(topicName+"/slam_traj"));
        NodeGLPtr gpsTrajNode(new NodeGL(topicName+"/gps_traj"));
        NodeGLPtr pointcloud(new NodeGL(topicName+"/pointcloud"));
        NodeGLPtr connections(new NodeGL(topicName+"/connections"));
        // if _map is empty, then clear all data
        if(!_map)
        {
            _pubNode.publish(keyframes);
            _pubNode.publish(slamTraj);
            _pubNode.publish(gpsTrajNode);
            _pubNode.publish(pointcloud);
            _pubNode.publish(connections);
            return;
        }

        GSLAM::FrameArray mapFrames;
        GSLAM::PointArray mapPoints;
        if(!_map->getFrames(mapFrames)) ;
        if(!_map->getPoints(mapPoints)) ;

        std::sort(mapFrames.begin(),mapFrames.end(),[](GSLAM::FramePtr a,GSLAM::FramePtr b)
        {
            return a->id()<b->id();
        });

        std::vector<GSLAM::Point3f> vetexTraj,gpsTraj;
        std::vector<GSLAM::Point3f> vetexConnection,gpsError;
        bool                        centerSeted=false;
        GSLAM::Point3d              center;


        int                  maxKeyFrameId = 0;


        for(GSLAM::FramePtr& fr:mapFrames)
        {
            if (fr->id()>maxKeyFrameId)
                maxKeyFrameId = fr->id();

            if(!centerSeted)
            {
                center=fr->getPose().get_translation();
                centerSeted=true;
            }

            Point3d t=fr->getPose().get_translation()-center;

            vetexTraj.push_back(t);

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


        for(GSLAM::FramePtr& curFrame:mapFrames){
            SIM3 pose=GSLAM::SIM3(curFrame->getPose(),curFrame->getMedianDepth()*0.1);
            Camera _camera=curFrame->getCamera();

            Point3d t=pose.get_translation();
            Point3d tl=_camera.UnProject(Point2d(0,0));
            Point3d tr=_camera.UnProject(Point2d(_camera.width(),0));
            Point3d bl=_camera.UnProject(Point2d(0,_camera.height()));
            Point3d br=_camera.UnProject(Point2d(_camera.width(),_camera.height()));

            GSLAM::Point3Type  W_tl=pose*(Point3d(tl.x,tl.y,1));
            GSLAM::Point3Type  W_tr=pose*(Point3d(tr.x,tr.y,1));
            GSLAM::Point3Type  W_bl=pose*(Point3d(bl.x,bl.y,1));
            GSLAM::Point3Type  W_br=pose*(Point3d(br.x,br.y,1));

            std::vector<Point3f> vertices({t,W_tl,t,W_tr,t,W_bl,t,W_br,W_tl,W_tr,W_tr,W_br,W_br,W_bl,W_bl,W_tl});
            keyframes->vertices.insert(keyframes->vertices.end(),vertices.begin(),vertices.end());
        }
        keyframes->colors.push_back(Point3ub(0,0,255));
        keyframes->displayMode=NodeGL::LINES;

        slamTraj->displayMode=NodeGL::LINE_STRIP;
        slamTraj->vertices=vetexTraj;
        slamTraj->colors.push_back(Point3ub(255,255,0));
        slamTraj->transform=SE3(SO3(),center);

        gpsTrajNode->displayMode=NodeGL::LINE_STRIP;
        gpsTrajNode->vertices=gpsTraj;
        gpsTrajNode->colors.push_back(Point3ub(255,0,0));
        gpsTrajNode->transform=SE3(SO3(),center);

        pointcloud->displayMode=NodeGL::POINTS;
        pointcloud->vertices=pointCloudVertex;
        pointcloud->colors=pointCloudColors;
        pointcloud->transform=SE3(SO3(),center);

        connections->displayMode=NodeGL::LINES;
        connections->vertices=vetexConnection;
        connections->colors.push_back(Point3ub(0,255,255));
        connections->transform=SE3(SO3(),center);


        _pubNode.publish(keyframes);
        _pubNode.publish(slamTraj);
        _pubNode.publish(gpsTrajNode);
        _pubNode.publish(pointcloud);
        _pubNode.publish(connections);
    }

    Svar                    _config;

    GSLAM::MapPtr           _map;
    GSLAM::Publisher        _pubNode;
};

REGISTER_SVAR_MODULE(mapviz){
    GSLAM_REGISTER_GLOG_SINKS;
    GSLAM_REGISTER_MESSENGER;
    svar["gslam"]["displays"]["map"]=MapVisualizer::displayPlugin();
}
EXPORT_SVAR_INSTANCE
