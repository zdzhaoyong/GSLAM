#include "../../core/Dataset.h"
#include "../../core/VecParament.h"
#include "../../core/VideoFrame.h"
#include <opencv2/highgui/highgui.hpp>


using namespace std;
using namespace GSLAM;

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

inline GSLAM::Camera camFromName(string name,Svar& var)
{
    VecParament<double> paras;
    paras=var.get_var(name+".Paraments",paras);
    return GSLAM::Camera(paras.data);
}

class DroneMapDataset : public GSLAM::Dataset
{
public:
    DroneMapDataset():_frameId(0){}
    std::string type() const {return "cfg";}

    virtual bool open(const string &dataset)
    {
        Svar var;
        if(var.ParseFile(dataset))
            return open(var,dataset);
        else
            return false;
    }

    bool open(Svar& var,const std::string& name)
    {
        if(var.exist("Video.File")&&var.GetString("Video.Type","")=="GSLAM")
            _type="GSLAM";

        if(_type.empty()) return false;


        _seqTop=Svar::getFolderPath(name);
        _name  =Svar::getBaseName(name);
        _skip  =var.GetInt("Video.Skip",0);
        _video.open(var.GetString("Video.File",""));
        _gps.open(_seqTop+"/gps.txt");
        if(!_video.is_open()) return false;
        _camera=camFromName(var.GetString("Video.CameraInName","Video.CameraInName"),var);

        prepareGPSFrame();
        prepareImageFrame();

        return true;
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
#ifdef HAS_OPENCV
        cv::Mat img=cv::imread(imgFile);
        SPtr<GSLAM::MapFrame> frame(new GSLAM::FrameMono(_frameId++,imageTime,img,_camera,IMAGE_BGRA));
        prepareImageFrame();
        return frame;
#else
        QImage qimage(imgFile.c_str(),"Format_RGB888");
        GImage img;

        if(qimage.format()==QImage::Format_RGB32)
        {
            img=GImage(qimage.height(),qimage.width(),
                       GImageType<uchar,4>::Type,qimage.bits(),true);
        }
        else if(qimage.format()==QImage::Format_RGB888){
            img=GImage(qimage.height(),qimage.width(),
                       GImageType<uchar,3>::Type,qimage.bits(),true);
        }
        else if(qimage.format()==QImage::Format_Indexed8)
        {
            img=GImage(qimage.height(),qimage.width(),
                       GImageType<uchar,1>::Type,qimage.bits(),true);
        }
        SPtr<GSLAM::MapFrame> frame(new GSLAM::FrameMono(_frameId++,imageTime,img,_camera));

        prepareImageFrame();
        return frame;
#endif
        return GSLAM::FramePtr();
    }

    string           _seqTop;
    string           _type;
    ifstream         _video,_gps;
    int              _skip;
    GSLAM::FrameID   _frameId;
    GSLAM::Camera    _camera;
    VecParament<string> _nextImage,_nextGPS;
};

REGISTER_DATASET(DroneMapDataset,cfg)
