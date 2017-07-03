
#include "../../core/Dataset.h"
#include "../../core/VideoFrame.h"
#include "../../core/Svar.h"
#include "../../core/VecParament.h"
#include "../../core/Timer.h"
#undef HAS_OPENCV

#ifdef HAS_OPENCV
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#elif defined(HAS_QT)
#include <QImage>
#endif
using namespace std;
using namespace GSLAM;

inline GSLAM::Camera camFromName(string name,Svar& var)
{
    VecParament<double> paras;
    paras=var.get_var(name+".Paraments",paras);
    return GSLAM::Camera(paras.data);
}

class ImageFrameWithGPSPYR : public GSLAM::FrameMonoGPSPYR
{
public:
    ImageFrameWithGPSPYR(FrameID id,double time,const GImage& img,const Camera& camera,
                         double longtitude,double latitude,double altitude,
                         double sigmaHorizon,double sigmaVertical,
                         double pitch,double yaw,double roll,
                         double sigmaPitch,double sigmaYaw,double sigmaRoll,
                         string imgPath="")
        :GSLAM::FrameMonoGPSPYR(id,time,img,camera,longtitude,latitude,altitude,sigmaHorizon,sigmaVertical,
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
#ifdef HAS_OPENCV
        cv::Mat img=cv::imread(imgFile);
        if(img.empty())
        {
            imgFile=_seqTop+"/"+imgFile;
            img=cv::imread(imgFile);
        }
        if(img.type()==CV_8UC3) cv::cvtColor(img,img,CV_BGR2RGB);
//        frame->_channel=IMAGE_BGRA;
#else
        QImage qimage(imgFile.c_str(),"Format_RGB888");
        GImage img;

        if(qimage.isNull())
        {
            imgFile=_seqTop+"/"+imgFile;
            qimage.load(imgFile.c_str(),"Format_RGB888");
        }

        if(qimage.format()==QImage::Format_RGB32)
        {
            img=GImage(qimage.width(),qimage.height(),
                       GImageType<uchar,4>::Type,qimage.bits(),true);
        }
        else if(qimage.format()==QImage::Format_RGB888){
            img=GImage(qimage.width(),qimage.height(),
                       GImageType<uchar,3>::Type,qimage.bits(),true);
        }
        else if(qimage.format()==QImage::Format_Indexed8)
        {
            img=GImage(qimage.width(),qimage.height(),
                       GImageType<uchar,1>::Type,qimage.bits(),true);
        }
#endif
        ImageFrameWithGPSPYR* frame=new ImageFrameWithGPSPYR(_frameId++,timestamp,img,_camera,para[0],para[1],para[2],para[3],
                para[4],para[5],para[6],para[7],para[8],para[9],para[10],imgFile);
        return SPtr<GSLAM::MapFrame>(frame);
    }

    string           _seqTop;
    GSLAM::FrameID   _frameId;
    GSLAM::Camera    _camera;
    string           _name;
    ifstream         _ifs;
};

REGISTER_DATASET(DatasetRTMapper,rtm)

