#ifdef HAS_OPENCV
#include "../../core/Dataset.h"
#include "../../core/VideoFrame.h"
#include "../../core/Svar.h"
#include "../../core/VecParament.h"
#include "../../core/Timer.h"


#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace std;
using namespace GSLAM;

inline GSLAM::Camera camFromName(string name,Svar& var)
{
    VecParament<double> paras;
    paras=var.get_var(name+".Paraments",paras);
    return GSLAM::Camera(paras.data);
}

class ImageFrameWithGPSYRP : public GSLAM::FrameMonoGPS
{
public:
    ImageFrameWithGPSYRP(FrameID id,double time,const GImage& img,const Camera& camera,
                         double longtitude,double latitude,double altitude,
                         double sigmaHorizon,double sigmaVertical,
                         double pitch,double yaw,double roll,
                         double sigmaPitch,double sigmaYaw,double sigmaRoll,
                         string imgPath="")
        :GSLAM::FrameMonoGPS(id,time,img,camera,longtitude,latitude,altitude,sigmaHorizon,sigmaVertical,
                             pitch,yaw,roll,sigmaPitch,sigmaYaw,sigmaRoll),_imgPath(imgPath)
    {
    }

    virtual std::string type() const{return "ImageFrameWithGPSYRP";}

    virtual void call(const string &command, void *arg)
    {
        if("GetGPS"==command)
        {
            if((!arg)) return;
            std::vector<double>* dest=(std::vector<double>*)arg;
            *dest={_longtitude,_latitude,_altitude,_sigmaHorizon,_sigmaVertical,
                   _pitch,_yaw,_roll,_sigmaPitch,_sigmaYaw,_sigmaRoll};
        }
        else if("GetImagePath"==command)
        {
            if(arg) *(std::string*)arg=_imgPath;
        }
    }
public:
    std::string         _imgPath;
};

class DatasetRTMapper : public Dataset
{
public:
    DatasetRTMapper(const std::string& name="")
        :_frameId(0),
         _name(name)
    {
    }

    std::string type() const {return "rtm";}

    virtual bool open(const string &dataset)
    {
        Svar var;
        if(var.ParseFile(dataset))
            return open(var,"VideoReader");
        else
            return open(svar,dataset);
    }

    bool open(Svar& var,const std::string& name)
    {
        if(_ifs.is_open())_ifs.close();
        _ifs.open(var.GetString(name+".VideoFile",""));
        if(!_ifs.is_open()) return false;
        _seqTop=Svar::getFolderPath(var.GetString(name+".VideoFile",""));
        _camera=camFromName(var.GetString(name+".Camera",name+".Camera"),var);
        if(!_camera.isValid()) return false;
        _name=name;
        _frameId=1;
        return true;
    }

    bool isOpened(){return _ifs.is_open()&&_camera.isValid();}

    GSLAM::FramePtr grabFrame()
    {
        string line,lineCopy;
        if(!getline(_ifs,line)) return GSLAM::FramePtr();
        lineCopy=line;
        string imgFile;
        double timestamp;
        vector<double> para;
        para.resize(11);
        int dotPosition=line.find(',');
        imgFile=line.substr(0,dotPosition);

        line=line.substr(dotPosition+1);
        sscanf(line.c_str(),"%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf",
               &timestamp,&para[0],&para[1],&para[2],&para[3],&para[4],
                &para[5],&para[6],&para[7],&para[8],&para[9],&para[10]);
        if(para[2]<100) {
//            cout<<"ErrorHeight:"<<para[2];
//            cout<<lineCopy<<endl;
        }

        cv::Mat img=cv::imread(imgFile);
        if(img.empty())
        {
            imgFile=_seqTop+"/"+imgFile;
            img=cv::imread(imgFile);
        }
//        if(img.type()==CV_8UC3) cv::cvtColor(img,img,CV_BGR2RGB);
        ImageFrameWithGPSYRP* frame=new ImageFrameWithGPSYRP(_frameId++,timestamp,img,_camera,para[0],para[1],para[2],para[3],
                para[4],para[5],para[6],para[7],para[8],para[9],para[10],imgFile);
        frame->_channel=IMAGE_BGRA;
        return SPtr<GSLAM::MapFrame>(frame);
    }

    string           _seqTop;
    GSLAM::FrameID   _frameId;
    GSLAM::Camera    _camera;
    string           _name;
    ifstream         _ifs;
};

#if 1
// Buildin
REGISTER_DATASET(DatasetRTMapper)
#else
// Use as a plugin
extern "C"
{
SPtr<Dataset> createDataset()
{
    return SPtr<Dataset>(new DatasetRTMapper());
}
}
#endif

#endif
