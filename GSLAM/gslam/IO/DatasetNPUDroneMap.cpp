#include <GSLAM/core/Dataset.h>
#include <GSLAM/core/GPS.h>
#include <GSLAM/core/VecParament.h>
#include <GSLAM/core/VideoFrame.h>
#include "IO.h"

using namespace std;

namespace GSLAM {

/**
 * 1. Download dataset from : http://zhaoyong.adv-ci.com/downloads/npu-dronemap-dataset/
 * 2. Play dataset with gslam Dataset=<dir>/DroneMap/gopro-npu/gopro-npu-unified/mono.npudronemap
 *    or Dataset=<dir>/DroneMap/gopro-npu/gopro-npu-kfs/mono.npudronemap
 */
class RTMapperFrame : public GSLAM::MapFrame
{
public:
    RTMapperFrame(const GSLAM::FrameID& id=0,const double& timestamp=0)
        :GSLAM::MapFrame(id,timestamp){
        GSLAM::WriteMutex lock(_mutexPose);
    }

    virtual std::string type()const{return "RTMapperFrame";}

    virtual int     cameraNum()const{return 1;}                                      // Camera number
    virtual GSLAM::SE3     getCameraPose(int idx=0) const{return GSLAM::SE3();}                    // The transform from camera to local
    virtual int     imageChannels(int idx=0) const{return GSLAM::IMAGE_BGRA;}               // Default is a colorful camera
    virtual GSLAM::Camera  getCamera(int idx=0){return _camera;}                            // The camera model

    virtual bool           setImage(const GSLAM::GImage &img, int idx, int channalMask)
    {
        GSLAM::WriteMutex lock(_mutexPose);
        if(idx==0)
        {
            _image=img;
        }
        else _thumbnail=img;
        return true;
    }

    virtual GSLAM::GImage  getImage(int idx=0,int channalMask=GSLAM::IMAGE_UNDEFINED){
        GSLAM::WriteMutex lock(_mutexPose);
        if(idx==0)
        {
            return _image;
        }
        else return _thumbnail;
    }   // Just return the image if only one channel is available

    // When the frame contains IMUs or GPSs
    virtual int     getIMUNum()const{return (_gpshpyr.size()==11||_gpshpyr.size()==12||_gpshpyr.size()==14)?1:0;}
    virtual bool    getAcceleration(GSLAM::Point3d& acc,int idx=0)const{return false;}        // m/s^2
    virtual bool    getAngularVelocity(GSLAM::Point3d& angularV,int idx=0)const{return false;}// rad/s
    virtual bool    getMagnetic(GSLAM::Point3d& mag,int idx=0)const{return false;}            // gauss
    virtual bool    getAccelerationNoise(GSLAM::Point3d& accN,int idx=0)const{return false;}
    virtual bool    getAngularVNoise(GSLAM::Point3d& angularVN,int idx=0)const{return false;}
    virtual bool    getPitchYawRoll(GSLAM::Point3d& pyr,int idx=0)const{
        if(_gpshpyr.size()==11&&_gpshpyr[8]<20) {pyr=GSLAM::Point3d(_gpshpyr[5],_gpshpyr[6],_gpshpyr[7]);return true;}
        else if(_gpshpyr.size()==14&&_gpshpyr[11]) {pyr=GSLAM::Point3d(_gpshpyr[8],_gpshpyr[9],_gpshpyr[10]);return true;}
        else if(_gpshpyr.size()==12&&_gpshpyr[9]<20) {pyr=GSLAM::Point3d(_gpshpyr[6],_gpshpyr[7],_gpshpyr[8]);return true;}
        return false;
    }     // in rad
    virtual bool    getPYRSigma(GSLAM::Point3d& pyrSigma,int idx=0)const{
        if(_gpshpyr.size()==11&&_gpshpyr[8]<20) {pyrSigma=GSLAM::Point3d(_gpshpyr[8],_gpshpyr[9],_gpshpyr[10]);return true;}
        else if(_gpshpyr.size()==14&&_gpshpyr[11]) {pyrSigma=GSLAM::Point3d(_gpshpyr[11],_gpshpyr[12],_gpshpyr[13]);return true;}
        else if(_gpshpyr.size()==12&&_gpshpyr[9]<20) {pyrSigma=GSLAM::Point3d(_gpshpyr[9],_gpshpyr[10],_gpshpyr[11]);return true;}
        return false;
    }    // in rad

    virtual int     getGPSNum()const{return (_gpshpyr.size()>=6&&_gpshpyr[3]<10)?1:0;}
    virtual bool    getGPSLLA(GSLAM::Point3d& LonLatAlt,int idx=0)const{
        if(getGPSNum()==0) return false;
        LonLatAlt=GSLAM::Point3d(_gpshpyr[0],_gpshpyr[1],_gpshpyr[2]);
        return _gpshpyr[3]<100;
    }        // WGS84 [longtitude latitude altitude]
    virtual bool    getGPSLLASigma(GSLAM::Point3d& llaSigma,int idx=0)const{
        if(_gpshpyr.size()>=6||_gpshpyr.size()==8||_gpshpyr.size()==12||_gpshpyr.size()==14)
        {llaSigma=GSLAM::Point3d(_gpshpyr[3],_gpshpyr[4],_gpshpyr[5]);return true;}
        else if(_gpshpyr.size()==7) {llaSigma=GSLAM::Point3d(_gpshpyr[3],_gpshpyr[3],_gpshpyr[4]);return true;}
        return false;
    }    // meter
    virtual bool    getGPSECEF(GSLAM::Point3d& xyz,int idx=0)const{
        GSLAM::Point3d lla;
        if(!getGPSLLA(lla)) return false;
        xyz=GSLAM::GPS<>::GPS2XYZ(lla.y,lla.x,lla.z);
        return true;
    }             // meter
    virtual bool    getHeight2Ground(GSLAM::Point2d& height,int idx=0)const{
        if(_gpshpyr.size()==14||_gpshpyr.size()==8){height=GSLAM::Point2d(_gpshpyr[6],_gpshpyr[7]);return _gpshpyr[7]<100;}
        return false;
    }    // height against ground

    virtual void call(const std::string &command, void *arg)
    {
        if("GetImagePath"==command)
        {
            if(arg) *(std::string*)arg=_imagePath;
        }
        else if("ReleaseImage"==command)
        {
            _image=GSLAM::GImage();
        }
        else if("GetGPS"==command)
        {
            if((!arg)||_gpshpyr.empty()) return;
            std::vector<double>* dest=(std::vector<double>*)arg;
            *dest=_gpshpyr;
        }
    }

    std::string imagePath(){return _imagePath;}
    GSLAM::GImage& thumbnail(){return _thumbnail;}

    std::string         _imagePath;       // where image come from or where to cache
    std::string         _cameraName;
    GSLAM::GImage       _image,_thumbnail;// should be RGBA
    GSLAM::Camera       _camera;
    std::vector<double> _gpshpyr;
    // 6 : long,lat,alt,sigmaX,sigmaY,sigmaZ
    // 8 : long,lat,alt,sigmaX,sigmaY,sigmaZ,Height,sigmaH
    // 11: long,lat,alt,sigmaH,sigmaV,pitch,yaw,roll,sigmaP,sigmaR,sigmaP
    // 12: long,lat,alt,sigmaX,sigmaY,sigmaZ,pitch,yaw,roll,sigmaP,sigmaR,sigmaP
    // 14: long,lat,alt,sigmaX,sigmaY,sigmaZ,Height,sigmaH,pitch,yaw,roll,sigmaP,sigmaR,sigmaP
};

class GPSFrame: public GSLAM::MapFrame
{
public:
    GPSFrame(FrameID id,double time,
                 double longtitude,double latitude,double altitude,
                 double sigmaHorizon,double sigmaVertical,double height)
        :GSLAM::MapFrame(id,time),_longtitude(longtitude),_latitude(latitude),_altitude(altitude),
          _sigmaHorizon(sigmaHorizon),_sigmaVertical(sigmaVertical),_height(height){}

    virtual std::string type() const{return "FrameGPS";}
    virtual int     getGPSNum()const{return 1;}
    virtual SE3     getGPSPose(int idx=0)const{return SE3();}

    virtual bool    getGPSLLA(Point3d& LonLatAlt,int idx=0)const{  // WGS84 [longtitude latitude altitude]
        LonLatAlt=Point3d(_longtitude,_latitude,_altitude);
        return true;
    }

    virtual bool    getGPSLLASigma(Point3d& llaSigma,int idx=0)const{
        llaSigma=Point3d(_sigmaHorizon,_sigmaHorizon,_sigmaVertical);return true;}     // meter

    virtual bool    getGPSECEF(Point3d& xyz,int idx=0)const{  // meter
        xyz=GPS<>::GPS2XYZ(_latitude,_longtitude,_altitude);
        return true;
    }

    virtual bool    getHeight2Ground(Point2d &height, int idx=0) const{
        height=Point2d(_height,_sigmaVertical);
        return true;
    }

    double _longtitude,_latitude,_altitude,_sigmaHorizon,_sigmaVertical,_height;
};

class DroneMapKFDataset : public GSLAM::Dataset
{
    struct DroneMapFrame
    {
        double      timestamp;
        std::string imagePath;
        pi::SE3d    pose;
    };
public:
    DroneMapKFDataset():curIdx(0){}
    virtual std::string type() const{return "DroneMapKFDataset";}
    virtual bool        isOpened(){return frames.size();}

    virtual bool        open(const std::string& dataset)
    {
        string path=Svar::getFolderPath(dataset);
        Svar var;
        var.ParseFile(path+"/config.cfg");
        plane=var.get_var<pi::SE3d>("Plane",plane);
        origin=var.get_var("GPS.Origin",origin);
        // Local to ECEF
        local2ECEF.get_translation()=GSLAM::GPS<>::GPS2XYZ(Point3d(origin.y,origin.x,origin.z));
        double D2R=3.1415925/180.;
        double lon=origin.x*D2R;
        double lat=origin.y*D2R;
        GSLAM::Point3d up(cos(lon)*cos(lat), sin(lon)*cos(lat), sin(lat));
        GSLAM::Point3d east(-sin(lon), cos(lon), 0);
        GSLAM::Point3d north=up.cross(east);
        double R[9]={east.x, north.x, up.x,
                     east.y, north.y, up.y,
                     east.z, north.z, up.z};
        local2ECEF.get_rotation().fromMatrix(R);

        camParameters=var.get_var<VecParament<double> >("Camera.Paraments",VecParament<double>()).data;
        camera=GSLAM::Camera(camParameters);
        if(!camera.isValid()) return false;

        ifstream ifs(path+"/trajectory.txt");
        if(!ifs.is_open()) {
            LOG(ERROR)<<"Can't open tracjectory file.";
            return false;
        }
        string line;
        DroneMapFrame frame;
        while(getline(ifs,line))
        {
            stringstream sst(line);
            string& imgfile=frame.imagePath;
            sst>>imgfile;

            stringstream st(imgfile);
            st>>frame.timestamp;
            imgfile=path+"/rgb/"+imgfile+".jpg";

            sst>>frame.pose;
            frames.push_back(frame);
        }
        return frames.size();
    }

    virtual FramePtr    grabFrame()
    {
        int frameId=curIdx++;
        if(frameId>=frames.size()) return FramePtr();
        DroneMapFrame& df=frames[frameId];
        SPtr<RTMapperFrame> fr(new RTMapperFrame(frameId,df.timestamp));

        fr->_image=imread(df.imagePath);
        fr->_camera=camera;
        GSLAM::SE3 pose=local2ECEF*df.pose;
        GSLAM::Point3d lla=GSLAM::GPS<>::XYZ2GPS(pose.get_translation());
        fr->_gpshpyr=std::vector<double>({lla.y,lla.x,lla.z,5.,5.,10.});

        return fr;
    }
    string          datasetPath;
    GSLAM::SE3      plane,local2ECEF;
    Point3d         origin;
    vector<double>  camParameters;
    GSLAM::Camera   camera;
    vector<DroneMapFrame> frames;
    int             curIdx;
};

class DatasetDroneMapUnified : public GSLAM::Dataset
{
public:
    DatasetDroneMapUnified():_frameId(1){}
    std::string type() const {return "DroneMapDataset";}

    virtual bool open(const string &dataset)
    {
        string folderPath=Svar::getFolderPath(dataset);
        Svar var;
        if(!var.ParseFile(dataset)) var.ParseFile(folderPath+"/config.cfg");

        _seqTop=var.GetString("DatasetPath",folderPath);

        _name  =Svar::getBaseName(dataset);
        _skip  =var.GetInt("Video.Skip",0);
        _video.open(var.GetString("Video.File",_seqTop+"/frames.txt"));
        if(!_video.is_open()) return false;

        if(_name.find("mono")==std::string::npos)
            _gps.open(_seqTop+"/gps.txt");
        _camera=camFromName(var.GetString("Video.CameraInName","Video.CameraInName"),var);

        prepareGPSFrame();
        prepareImageFrame();

        return true;
    }

    inline GSLAM::Camera camFromName(string name,Svar& var)
    {
        VecParament<double> paras;
        paras=var.get_var(name+".Paraments",paras);
        return GSLAM::Camera(paras.data);
    }

    bool isOpened(){return _video.is_open()&&_camera.isValid();}

    bool prepareGPSFrame(){
        if(!_gps.is_open()) return false;
        string line;
        _nextGPS.data.clear();
        if(!getline(_gps,line)) {return false;}
        stringstream sst(line);
        sst>>_nextGPS;
        return _nextGPS.size()==5;
    }

    bool prepareImageFrame(){
        if(!_video.is_open()) return false;
        string line;_nextImage.data.clear();
        for(int i=_skip;i>=0;i--)
            if(!getline(_video,line)) { return false;}
        stringstream sst(line);
        sst>>_nextImage;
        return _nextGPS.size()==2;
    }

    GSLAM::FramePtr grabFrame()
    {
        if(!isOpened()) return GSLAM::FramePtr();
        double gpsTime=1e30,imageTime=1e30;
        if(_nextGPS.size()) gpsTime=std::stod(_nextGPS[0]);
        if(_nextImage.size()) imageTime=std::stod(_nextImage[0]);
        else return GSLAM::FramePtr();

        if(gpsTime<imageTime)
        {
            // GPSFrame
            GSLAM::FramePtr frame(new GPSFrame(_frameId++,gpsTime,
                                                std::stod(_nextGPS[1]),std::stod(_nextGPS[2]),std::stod(_nextGPS[3]),
                    5,10,std::stod(_nextGPS[3])));
            prepareGPSFrame();
            return frame;
        }

        string imgFile=_seqTop+"/"+_nextImage[1];
        GImage img=imread(imgFile);
        SPtr<GSLAM::MapFrame> frame(new GSLAM::FrameMono(_frameId++,imageTime,img,_camera,IMAGE_BGRA));
        prepareImageFrame();
        return frame;
    }

    string           _seqTop;
    string           _type;
    ifstream         _video,_gps;
    int              _skip;
    GSLAM::FrameID   _frameId;
    GSLAM::Camera    _camera;
    VecParament<string> _nextImage,_nextGPS;
};

class DatasetNPUDroneMap : public Dataset
{
public:
    virtual bool open(const string &dataset){
        std::string datasetFolder=Svar::getFolderPath(dataset);
        ifstream ifs(datasetFolder+"/trajectory.txt");
        if(ifs.is_open())
        {
            _impl=DatasetPtr(new DroneMapKFDataset());
            return _impl->open(dataset);
        }
        ifs.open(datasetFolder+"/frames.txt");
        if(ifs.is_open())
        {
            _impl=DatasetPtr(new DatasetDroneMapUnified());
            return _impl->open(dataset);
        }
    }
};


REGISTER_DATASET(DatasetNPUDroneMap,npudronemap);

}
