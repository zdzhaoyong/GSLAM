#include "MapHash.h"

inline bool MapHash::insertMapPoint(const PointPtr& point)
{
    WriteMutex lock(_mutexPoints);
    PointIt it=_points.find(point->id());
    if(it!=_points.end()) return false;
    _points.insert(make_pair(point->id(),point));
    return true;
}

inline bool MapHash::insertMapFrame(const FramePtr& frame)
{
    WriteMutex lock(_mutexFrames);
    FrameIt it=_frames.find(frame->id());
    if(it!=_frames.end()) return false;
    _frames.insert(make_pair(frame->id(),frame));

    if(_loopDetector) _loopDetector->insertMapFrame(frame);
    return true;
}

inline bool MapHash::eraseMapPoint(const PointID& pointId)
{
    WriteMutex lock(_mutexPoints);
    return _points.erase(pointId);
}

inline bool MapHash::eraseMapFrame(const FrameID& frameId)
{
    WriteMutex lock(_mutexFrames);
    if(_loopDetector) _loopDetector->eraseMapFrame(frameId);
    return _frames.erase(frameId);
}

inline void  MapHash::clear()
{
    WriteMutex lock1(_mutexFrames);
    WriteMutex lock2(_mutexPoints);
    _points.clear();
    _frames.clear();
}

inline std::size_t MapHash::frameNum()const
{
    ReadMutex lock(_mutexFrames);
    return _frames.size();
}

inline std::size_t MapHash::pointNum()const
{
    ReadMutex lock(_mutexPoints);
    return _points.size();
}

inline FramePtr MapHash::getFrame(const FrameID& id)const
{
    ReadMutex lock(_mutexFrames);
    ConstFrameIt it=_frames.find(id);
    if(it!=_frames.end())
        return it->second;
    else
        return FramePtr();
}

inline PointPtr MapHash::getPoint(const PointID& id)const
{
    ReadMutex lock(_mutexPoints);
    ConstPointIt it=_points.find(id);
    if(it!=_points.end())
        return it->second;
    else
        return PointPtr();
}

inline bool  MapHash::getFrames(FrameArray& frames)const
{
    ReadMutex lock(_mutexFrames);
    frames.clear();
    frames.reserve(_frames.size());
    for(const std::pair<FrameID,FramePtr>& fr:_frames) frames.push_back(fr.second);
    return true;
}

inline bool  MapHash::getPoints(PointArray& points)const
{
    ReadMutex lock(_mutexPoints);
    points.clear();
    points.reserve(_points.size());
    for(const std::pair<PointID,PointPtr>& pt:_points) points.push_back(pt.second);
    return true;
}


MapHash::MapHash()
{

}

MapHash::~MapHash()
{
    if(frameNum())
        cerr<<type()<<" with "<<frameNum()<<" MapFrames and "<<pointNum()<<" MapPoints released.\n";
}

void  MapHash::draw()
{
#ifdef HAS_OPENGL
    glDisable(GL_LIGHTING);

    GSLAM::PointArray points;
    if(getPoints(points))
    {
        glPointSize(2.5);
        glBegin(GL_POINTS);
        for(GSLAM::PointPtr pt:points)
        {
//            if(pt->observationNum()<3) continue;
            const GSLAM::Point3Type& nvec=pt->getNormal();
            glNormal3d(nvec.x,nvec.y,nvec.z);
            glColor(pt->getColor());
            glVertex(pt->getPose());
        }
        glEnd();
    }

    GSLAM::FrameArray frames;
    GSLAM::Camera     camera;
    bool gpsFitted=svar.GetInt("GPS.Fitted");
    if(getFrames(frames))
    {
        double r[9];
        for(auto fr:frames)
        {
            double     depth=fr->getMedianDepth()/10;
            GSLAM::SE3 pose=fr->getPose();
            GSLAM::Point3Type t=pose.get_translation();
            pose.get_rotation().getMatrixUnsafe(r);
//            glMultMatrix(pose);
            if(!camera.isValid())
                camera=fr->getCamera(0);

            if(false)
            {
                double axis_length=depth;
                glBegin(GL_LINES);
                glLineWidth(2.5);
                glColor3ub(255,0,0);
                glVertex3d(t[0],t[1],t[2]);
                glVertex3d(t[0]+axis_length*r[0],t[1]+axis_length*r[1],t[2]+axis_length*r[2]);
                glColor3ub(0,255,0);
                glVertex3d(t[0],t[1],t[2]);
                glVertex3d(t[0]+axis_length*r[3],t[1]+axis_length*r[4],t[2]+axis_length*r[5]);
                glColor3ub(0,0,255);
                glVertex3d(t[0],t[1],t[2]);
                glVertex3d(t[0]+axis_length*r[6],t[1]+axis_length*r[7],t[2]+axis_length*r[8]);
                glEnd();
            }

            // Draw camera rect
            {
                GSLAM::Point3d tl=camera.UnProject(GSLAM::Point2d(0,0));
                GSLAM::Point3d tr=camera.UnProject(GSLAM::Point2d(camera.width(),0));
                GSLAM::Point3d bl=camera.UnProject(GSLAM::Point2d(0,camera.height()));
                GSLAM::Point3d br=camera.UnProject(GSLAM::Point2d(camera.width(),camera.height()));
                //GSLAM::Point2d ct=cam_out->UnProject(GSLAM::Point2d(cam_out->Cx(),cam_out->Cy()));

                GSLAM::Point3Type  W_tl=pose*(GSLAM::Point3d(tl.x,tl.y,1)*depth);
                GSLAM::Point3Type  W_tr=pose*(GSLAM::Point3d(tr.x,tr.y,1)*depth);
                GSLAM::Point3Type  W_bl=pose*(GSLAM::Point3d(bl.x,bl.y,1)*depth);
                GSLAM::Point3Type  W_br=pose*(GSLAM::Point3d(br.x,br.y,1)*depth);

        //        Point3Type  W_ct=pose*(GSLAM::Point3d(ct.x,ct.y,1)*depth);
                glBegin(GL_LINES);
                glLineWidth(2.5);
                glColor3f(0, 1, 1);
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

            // draw GPS error
            if(gpsFitted)
            {
                GSLAM::Point3d gpsPosition;
                if(fr->getGPSECEF(gpsPosition))
                {
                    glColor3f(1, 0.2, 0.2);
                    glLineWidth(2.5);
                    glBegin(GL_LINES);
                    glVertex(t);
                    glVertex(gpsPosition);
                    glEnd();
                }
            }

        }
    }
#endif
}

class OutStream{
public:
    OutStream(ostream& out):_out(out){}

    template <typename T>
    OutStream& operator <<(const T& obj){
        T tmp=obj;
        _out.write((char*)&tmp,sizeof(tmp));
        return *this;
    }

    template <typename T>
    OutStream& operator <<(const vector<T>& obj){
        *this<<(size_t)obj.size();
        for(const T& t:obj) *this<<t;
        return *this;
    }

    OutStream& operator <<(const GSLAM::GImage& obj){
        *this<<obj.cols<<obj.rows<<obj.flags;
        _out.write((const char*)obj.data,obj.total()*obj.elemSize());
        return *this;
    }

    OutStream& operator <<(const string& obj){
        *this<<obj.size();
        _out.write(obj.data(),obj.size());
        return *this;
    }

    ostream& _out;
};
class InStream{
public:
    InStream(istream& in):_in(in){}

    template <typename T>
    InStream& operator >>(T& obj){
        _in.read((char*)&obj,sizeof(obj));
        return *this;
    }

    template <typename T>
    InStream& operator >>(vector<T>& obj){
        size_t sz;
        *this>>sz;
        obj.resize(sz);
        for(int i=0;i<sz;i++) *this>>obj[i];
        return *this;
    }

    InStream& operator >>(GSLAM::GImage& obj){
        int cols,rows,flags;
        *this>>cols>>rows>>flags;
        obj=GSLAM::GImage::create(rows,cols,flags);
        _in.read((char*)obj.data,obj.total()*obj.elemSize());
        return *this;
    }

    InStream& operator >>(string& obj){
        size_t sz;
        *this>>sz;
        obj.resize(sz);
        _in.read((char*)obj.data(),obj.size());
        return *this;
    }

    istream& _in;
};

bool MapHash::save(std::string path)const
{
    if(path.empty()) return false;
    if(path.find_last_of('.')==std::string::npos) return saveMap2DFusion(path);
    if(path.find(".ply")!=std::string::npos) return savePointCloud(path);
    if(path.find(".mf")!=std::string::npos) return saveMapFusion(path);
    if(path.find(".txt")!=std::string::npos) return saveTrajectory(path);

    ofstream ofs;
    ofs.open(path,std::ios::out|std::ios::binary);
    if(!ofs.is_open()) return false;
    OutStream file(ofs);

    ReadMutex lockPoints(_mutexPoints);
    ReadMutex lockFrames(_mutexFrames);
    ofs<<"Hash"<<endl;
    ofs<<"binary"<<endl;
    file<<_frames.size()<<_points.size();
    for(std::pair<PointID,PointPtr> it:_points){
        PointPtr& pt=it.second;
        file<<pt->id()
           <<pt->getPose()
          <<pt->getNormal()
         <<pt->getColor()
        <<pt->refKeyframeID()
        <<GSLAM::GImage();//pt->getDescriptor();
    }

    for(std::pair<FrameID,FramePtr> it:_frames){
        std::shared_ptr<MapFrame> fr=std::dynamic_pointer_cast<MapFrame>(it.second);
        std::string imgFile;
        fr->call("GetImagePath",&imgFile);
        std::vector<double> gpsData;
        fr->call("GetGPS",&gpsData);
        file<<fr->id()
           <<fr->timestamp()
          <<fr->getPoseScale()
         <<GSLAM::GImage()// fr->getImage()
        <<imgFile
        <<fr->imageChannels()
        <<fr->getCamera().getParameters()
        <<gpsData
        <<GSLAM::GImage();// fr->getDescriptor(-1);

        // keypoints and colors
        std::vector<GSLAM::KeyPoint>  keypoints;
        std::vector<GSLAM::ColorType> colors;
        keypoints.reserve(fr->keyPointNum());
        colors.reserve(fr->keyPointNum());
        for(uint i=0;i<fr->keyPointNum();i++){
            KeyPointData& kp=fr->keyPoint(i);
            keypoints.push_back(kp._pt);
            colors.push_back(kp._color);
        }
        file<<keypoints<<colors;

        // observations
        std::map<GSLAM::PointID,size_t> observations;
        fr->getObservations(observations);
        std::vector<std::pair<GSLAM::PointID,size_t> > obsVec;
        obsVec.assign(observations.begin(),observations.end());
        file<<obsVec;

        std::map<GSLAM::FrameID,std::shared_ptr<GSLAM::FrameConnection> > children,parents;
        fr->getChildren(children);
        fr->getParents(parents);
        std::vector<std::pair<GSLAM::FrameID,int> > parentsVec,childrenVec;
        for(std::pair<GSLAM::FrameID,std::shared_ptr<GSLAM::FrameConnection> > c:children){
            childrenVec.push_back(std::make_pair(c.first,int(c.second?c.second->matchesNum():-1)));
        }
        for(std::pair<GSLAM::FrameID,std::shared_ptr<GSLAM::FrameConnection> > c:parents){
            parentsVec.push_back(std::make_pair(c.first,int(c.second?c.second->matchesNum():-1)));
        }
        file<<childrenVec<<parentsVec;
    }

    LOG(INFO)<<"Map saved to "<<path;

    return true;
}


bool MapHash::load(std::string path)
{
    if(path.empty()) return false;
    ifstream ifs;
    ifs.open(path,std::ios::in|std::ios::binary);
    if(!ifs.is_open()) return false;
    InStream file(ifs);

    string maptype,format;
    getline(ifs,maptype);
    getline(ifs,format);
    if(maptype!="Hash"||format!="binary") return false;

    size_t frameNum,pointNum;
    file>>frameNum>>pointNum;
    for(int i=0;i<pointNum;i++){
        PointID id;
        GSLAM::Point3d position,norm;
        GSLAM::ColorType color;
        GSLAM::FrameID refId;
        GSLAM::GImage  des;
        file>>id>>position>>norm>>color>>refId>>des;

        PointPtr pt(new MapPoint(id,position,norm,color,refId));
        pt->setDescriptor(des);
        insertMapPoint(pt);

#if (GSLAM_VERSION_MAJOR<<16|GSLAM_VERSION_MINOR<<8|GSLAM_VERSION_PATCH) >= 0x020402
        if(id>_ptId) _ptId=id;
#endif
    }

    for(int i=0;i<frameNum;i++){
        FrameID id;
        double  timestamp;
        GSLAM::SIM3 pose;
        GSLAM::GImage img,des;
        int     imageChannel;
        std::string imgFile;
        std::vector<double> gpsData,camParas;
        std::vector<GSLAM::KeyPoint>  keypoints;
        std::vector<GSLAM::ColorType> colors;
        std::vector<std::pair<GSLAM::PointID,size_t> > obsVec;
        std::vector<std::pair<GSLAM::FrameID,int> > parents,children;
        file>>id>>timestamp>>pose>>img>>imgFile>>imageChannel
                >>camParas>>gpsData>>des>>keypoints>>colors>>obsVec
                >>children>>parents;
#if (GSLAM_VERSION_MAJOR<<16|GSLAM_VERSION_MINOR<<8|GSLAM_VERSION_PATCH) >= 0x020402
        if(id>_frId) _frId=id;
#endif

        std::shared_ptr<MapFrame> fr(new MapFrame(id,timestamp,img,imgFile,GSLAM::Camera(camParas),gpsData,imageChannel));
        GSLAM::Point3d ecef;
        if(fr->getGPSECEF(ecef)){
            LOG(INFO)<<gpsData.size()<<":"<<ecef;
        }
        fr->setKeyPoints(keypoints,des);
        fr->setPose(pose);
        assert(colors.size()==keypoints.size());
        for(int i=0;i<colors.size();i++){
            fr->keyPoint(i)._color=colors[i];
        }

        for(std::pair<GSLAM::PointID,size_t>& obs:obsVec){
            PointPtr pt=getPoint(obs.first);
            if(!pt) {
                LOG(ERROR)<<"No mappoint "<<obs.first;
                continue;
            }
            if(!fr->addObservation(pt,obs.second,true))
            {
                LOG(ERROR)<<"Failed to add observation";
            }
        }

        for(std::pair<GSLAM::FrameID,int> c:children){
            fr->addChildren(c.first,std::shared_ptr<GSLAM::FrameConnection>(new MapFrameConnection(c.second)));
        }
        for(std::pair<GSLAM::FrameID,int> c:parents){
            fr->addParent(c.first,std::shared_ptr<GSLAM::FrameConnection>(new MapFrameConnection(c.second)));
        }

        insertMapFrame(fr);
    }

    LOG(INFO)<<"Map loaded from "<<path;

    return true;
}

bool MapHash::savePointCloud(std::string filename)const
{
    std::fstream file;
    file.open(filename.c_str(),std::ios::out|std::ios::binary);
    if(!file.is_open()){
        fprintf(stderr,"\nERROR: Could not open File %s for writing!",(filename).c_str());
        return false;
    }
    file.precision(15);
    typedef GSLAM::Point3f Vertex3f;
    typedef GSLAM::Point3ub Color3b;

    std::vector<GSLAM::Point3f>  vertices;
    std::vector<unsigned int> faces;
    std::vector<GSLAM::Point3f>  normals;
    std::vector<GSLAM::Point3ub> colors;
    std::vector<unsigned int> edges;

    GSLAM::PointArray mappoints;
    if(!getPoints(mappoints)) return false;
    for(GSLAM::PointPtr& pt:mappoints)
    {
        vertices.push_back(pt->getPose());
        normals.push_back(pt->getNormal());
        colors.push_back(pt->getColor());
    }

    int _verticesPerFace=3;
    bool binary=false;

    file << "ply";
    if(binary)file << "\nformat binary_little_endian 1.0";
    else file << "\nformat ascii 1.0";
    file << "\nelement vertex " << vertices.size();
    file << "\nproperty float32 x\nproperty float32 y\nproperty float32 z";
    if(normals.size())
        file << "\nproperty float32 nx\nproperty float32 ny\nproperty float32 nz";
    if(colors.size())
        file << "\nproperty uchar red\nproperty uchar green\nproperty uchar blue";
    if(faces.size()){
        file << "\nelement face " << faces.size()/_verticesPerFace;
        file << "\nproperty list uint8 int32 vertex_indices";
    }
    if(edges.size()){
        file << "\nElement edge " << edges.size()/2;
        file << "\nproperty int vertex1\nproperty int vertex2";
    }
    file << "\nend_header";
    if(binary) file << "\n";

    for(unsigned int i=0;i<vertices.size();i++){
        if(binary){
            file.write((char*)(&(vertices[i])),sizeof(Vertex3f));
        }
        else file << "\n" << vertices[i].x << " " << vertices[i].y << " " << vertices[i].z;

        if(normals.size())
        {
            if(binary){
                file.write((char*)(&(normals[i])),sizeof(Vertex3f));
            }
            else file << " " << normals[i].x << " " << normals[i].y << " " << normals[i].z;
        }
        if(colors.size()){
            if(binary){
                file.write((char*)(&(colors[i])),sizeof(Color3b));
            }
            else file << " " << (int)(colors[i].x) << " " << (int)(colors[i].y) << " " << (int)(colors[i].z);
        }
    }
    for(unsigned int i=0;i<faces.size();i+=_verticesPerFace){
        if(binary){
            file.write((char*)(&_verticesPerFace),sizeof(u_char));
        }
        else file << "\n" << (int)_verticesPerFace;
        for(unsigned int j=0;j<_verticesPerFace;j++)
            if(binary){
                unsigned int idx = faces[i+j];
                file.write((char*)(&idx),sizeof(unsigned int));
            }
            else file << " " << (faces[i+j]);
    }
    for(unsigned int i=0;i<edges.size();i+=2){
        if(binary){
            unsigned int idx = edges[i];
            file.write((char*)(&idx),sizeof(unsigned int));
            idx = edges[i+1]; file.write((char*)(&idx),sizeof(unsigned int));
        }
        else file << "\n " << edges[i] << " " << edges[i+1];
    }

    file.close();
    return true;
}

inline bool compareFr(GSLAM::FramePtr a,GSLAM::FramePtr b)
{
      return a->id()<b->id();   //升序排列，如果改为return a>b，则为降序

}

bool comparePt(GSLAM::PointPtr a,GSLAM::PointPtr b)
{
      return a->id()<b->id();   //升序排列，如果改为return a>b，则为降序

}

bool MapHash::saveMap2DFusion(std::string foler2save)const
{
//    // save things for Map2DFusion
//    using namespace std;

//    GSLAM::FrameArray frames;
//    if(!getFrames(frames)) return false;
//    if(frames.empty()) return false;

//    std::vector<GSLAM::Point3d> points;
//    GSLAM::SE3d plane;
//    double thresholdZ=frames.front()->getMedianDepth()*0.1;
//    GSLAM::Camera camera=frames.front()->getCamera();
//    GSLAM::Undistorter undis;
//    if(camera.CameraType()!="PinHole")
//    {
//        GSLAM::Camera pinholeCam=camera.estimatePinHoleCamera();
//        undis=GSLAM::Undistorter(camera,pinholeCam);
//        camera=pinholeCam;
//    }

//    GSLAM::SE3      ecef2local;
//    GSLAM::Point3d  lla;
//    bool       hasGPS=false;
//    GSLAM::FramePtr firstFrame=frames.front();
//    while(firstFrame->getGPSNum()){
//        if(!firstFrame->getGPSLLA(lla)) break;
//        hasGPS=true;
//        // Local to ECEF
//        GSLAM::SE3 local2ECEF;
//        local2ECEF.get_translation()=GSLAM::GPS<>::GPS2XYZ(lla.y,lla.x,lla.z);
//        double D2R=3.1415925/180.;
//        double lon=lla.x*D2R;
//        double lat=lla.y*D2R;
//        GSLAM::Point3d up(cos(lon)*cos(lat), sin(lon)*cos(lat), sin(lat));
//        GSLAM::Point3d east(-sin(lon), cos(lon), 0);
//        GSLAM::Point3d north=up.cross(east);
//        double R[9]={east.x, north.x, up.x,
//                     east.y, north.y, up.y,
//                     east.z, north.z, up.z};
//        local2ECEF.get_rotation().fromMatrix(R);
//        ecef2local=local2ECEF.inverse();
//        break;
//    }

//    std::shared_ptr<GSLAM::Estimator> estimator=GSLAM::Estimator::create();

//    GSLAM::PointArray mappoints;
//    getPoints(mappoints);
//    for(GSLAM::PointPtr pt:mappoints) points.push_back(ecef2local*pt->getPose());
//    if(!estimator->findPlane(plane,points,0,thresholdZ)) return false;

////    string foler2save=svar.GetString("Map2DFusionFolder","map2dfusion");
//    scommand.Call("system","mkdir "+foler2save+"/rgb -p");
//    std::ofstream configFile((foler2save+"/config.cfg").c_str());
//    configFile.precision(10);
//    configFile<<"Plane="<<plane<<endl
//             <<"Camera.CameraType="<<camera.CameraType()<<endl
//            <<"Camera.Paraments="<<VecParament<double>(camera.getParameters()).toString()<<endl
//    <<"TrajectoryFile=$(Svar.ParsingPath)/trajectory.txt"<<endl;
//    if(hasGPS)
//        configFile<<"GPS.Origin="<<lla;
//    std::ofstream trajFile((foler2save+"/trajectory.txt").c_str());
//    trajFile.precision(10);
//    std::sort(frames.begin(),frames.end(),compareFr);
//    for(GSLAM::FramePtr& fr:frames)
//    {
//        if(svar.GetInt("Map2DFusion.GPSTrans")){
//            GSLAM::Point3d  lla;
//            if(!fr->getGPSLLA(lla)) break;
//            auto pose=fr->getPose();
//            pose.get_translation()=GSLAM::GPS<>::GPS2XYZ(lla.y,lla.x,lla.z);
//            fr->setPose(pose);
//        }
//        double time=fr->timestamp();
//        if(time<=1e-9) time=fr->id();
//        string timestamp=GSLAM::Svar::dtos(time,6);
//        trajFile<<timestamp<<" "<<ecef2local*fr->getPose()<<endl;
//        string imageDest=foler2save+"/rgb/"+timestamp+".jpg";
//#ifdef HAS_OPENCV
//        GSLAM::GImage image=fr->getImage();
//        if(image.empty())
//        {
//            string imagePath;
//            fr->call("GetImagePath",&imagePath);
//            image=cv::imread(imagePath);
//        }
//        if(!image.empty())
//        {
//            if(undis.valid()){
//                GSLAM::GImage result;
//                undis.undistortFast(image,result);
//                cv::imwrite(imageDest,(cv::Mat)result);
//            }
//            else
//                cv::imwrite(imageDest,(cv::Mat)image);
//        }
//        else LOG(ERROR)<<"Failed to load image.";
//#else
//        if(access(imageDest.c_str(),0)==0) continue;
//        if(!undis.valid())
//        {
//            string imagePath;
//            fr->call("GetImagePath",&imagePath);
//            if(imagePath.empty()) continue;
//            scommand.Call("system","cp "+imagePath+" "+imageDest);
//        }
//        else LOG(ERROR)<<"Failed to undistort image.";
//#endif
//    }
//    return true;
}

bool MapHash::saveTrajectory(std::string file2save)const
{
    cout<<"Saving trajectory to "<<file2save<<endl;
    ofstream ofs(file2save.c_str());
    if(!ofs.is_open()) return false;

    GSLAM::FrameArray frames;
    getFrames(frames);
    if(frames.empty()) return false;
    sort(frames.begin(),frames.end(),compareFr);

    ofs.precision(6);
    for(GSLAM::FramePtr& fr:frames){
        ofs<<setiosflags(ios::fixed)<<setprecision(15)<<fr->timestamp()<<" "<<fr->getPose()<<endl;
    }
    return true;
}

bool MapHash::saveMapFusion(std::string file2save)const
{
    cout<<"Saving MapFusion File to "<<file2save<<endl;
    ofstream ofs(file2save.c_str());
    if(!ofs.is_open()) return false;

    GSLAM::FrameArray frames;
    getFrames(frames);
    sort(frames.begin(),frames.end(),compareFr);

    for(GSLAM::FramePtr& fr:frames)
    {
        GSLAM::SE3   frPose=fr->getPose();
        GSLAM::SE3   w2c=frPose.inverse();
        string imagePath;
        fr->call("GetImagePath",&imagePath);
        if(imagePath.empty()) continue;
        GSLAM::Camera camIn=fr->getCamera();
        std::vector<double> camInParas=camIn.getParameters();
        std::map<GSLAM::PointID,size_t> observes;
        fr->getObservations(observes);
        std::vector<std::pair<GSLAM::Point2d,GSLAM::Point2d>> kpIdepthInfo;
        kpIdepthInfo.reserve(observes.size());
        for(std::pair<GSLAM::PointID,size_t> obs:observes)
        {
            GSLAM::PointPtr pt=getPoint(obs.first);
            if(!pt) continue;
            GSLAM::Point2f featPt;
            if(!fr->getKeyPoint(obs.second,featPt)) continue;
            GSLAM::Point3d ptPlane=camIn.UnProject(featPt.x,featPt.y);
            GSLAM::Point3d pc=w2c*pt->getPose();
            if(pc.z<=0.01) continue;
            kpIdepthInfo.push_back(make_pair(GSLAM::Point2d(ptPlane.x,ptPlane.y),
                                             GSLAM::Point2d(1./pc.z,-1)));

        }
        auto t=frPose.get_translation();
        auto r=frPose.get_rotation();
        ofs.precision(12);
        ofs<<imagePath
          <<", "<<t.x<<", "<<t.y<<", "<<t.z<<", "<<r.x<<", "<<r.y<<", "<<r.z<<", "<<r.w
          <<", "<<camInParas.size();
        ofs.precision(6);
        for(double para:camInParas) ofs<<", "<<para;
        ofs<<", "<<kpIdepthInfo.size();
        for(std::pair<GSLAM::Point2d,GSLAM::Point2d>& kp:kpIdepthInfo)
        {
            ofs<<", "<<kp.first.x<<", "<<kp.first.y<<", "<<kp.second.x<<", "<<kp.second.y;
        }
        ofs<<endl;
    }


    return true;
}

bool MapHash::saveDense(std::string folder2save)const
{
//    cout<<"Saving dense to "<<folder2save<<endl;
//    if(!scommand.Call("system mkdir"," -p "+folder2save));
//    system(("mkdir -p "+folder2save).c_str());

//    save(folder2save+"/sparse.ply");

//    GSLAM::PointArray points;
//    GSLAM::FrameArray frames;
//    getPoints(points);
//    getFrames(frames);
//    sort(points.begin(),points.end(),comparePt);
//    sort(frames.begin(),frames.end(),compareFr);

//    std::map<GSLAM::PointID,uint> pointIDMap;
//    std::map<GSLAM::FrameID,uint> frameIDMap;

//    double cx=0,cy=0;

//    ofstream ofs(folder2save+"/bundle.rd.out");
//    ofstream imageListFile(folder2save+"/images.txt");
//    ofstream camFile(folder2save+"/camera.txt");

//    if(!ofs.is_open()) return false;
//    ofs<<"# Bundle file v0.3\n";
//    ofs<<frameNum()<<" "<<pointNum()<<endl;
//    for(auto& fr:frames)
//    {
//        GSLAM::Camera cam=fr->getCamera();
//        VecParament<double> paras=cam.getParameters();
//        double f,k1,k2;
//        if(paras.size()==6)//width,height,fx,fy,cx,cy
//        {
//            f=(paras[2]+paras[3])*0.5;
//            k1=0;k2=0;
//            if(!cx)
//            {
//                cx=paras[4];cy=paras[5];
//                camFile<<paras;
//            }
//        }
//        else if(paras.size()==11)//width,height,fx,fy,cx,cy,k1,k2,p1,p2,k3
//        {
//            f=(paras[2]+paras[3])*0.5;
//            k1=paras[6];
//            k2=paras[7];
//            if(!cx)
//            {
//                cx=paras[4];cy=paras[5];
//                camFile<<paras;
//            }
//        }
//        else {return false;}
//        GSLAM::SE3 pose=fr->getPose();
//        GSLAM::Point3Type t=pose.get_translation();
//        double r[9];
//        pose.get_rotation().getMatrix(r);

//        ofs<<f<<" "<<k1<<" "<<k2<<endl;
//        for(int i=0;i<9;i++)
//            ofs<<r[i]<<(i%3==2?"\n":" ");
//        ofs<<t.x<<" "<<t.y<<" "<<t.z<<"\n";
//        frameIDMap[fr->id()]=frameIDMap.size();
//        string imagePath;
//        fr->call("GetImagePath",&imagePath);
//        imageListFile<<imagePath<<endl;
//    }

//    for(auto& pt:points)
//    {
//        if(!pt) continue;
//        ofs<<pt->getPose()<<"\n"
//          <<(GSLAM::Point3i)pt->getColor()<<endl;
//        std::map<GSLAM::FrameID,size_t> observations;
//        pt->getObservations(observations);

//        vector<std::pair<GSLAM::Point2i,GSLAM::Point2d> > observes;

//        for(auto& obs:observations)
//        {
//            GSLAM::FramePtr fr=getFrame(obs.first);

//            if(!fr.get()) continue;
//            int frameID=frameIDMap[fr->id()];
//            std::pair<int,GSLAM::Point2d> kp(obs.second,GSLAM::Point2d());
//            fr->call("GetKeyPoint",&kp);

//            observes.push_back(make_pair(GSLAM::Point2i(frameID,obs.second),
//                                         kp.second-GSLAM::Point2d(cx,cy)));

//        }

//        ofs<<observes.size();
//        for(auto& obs:observes)
//            ofs<<" "<<obs.first<<" "<<obs.second;
//        ofs<<endl;
//    }

    return true;
}
