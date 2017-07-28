#include "../../../core/Dataset.h"
#include "../../../core/VideoFrame.h"
#include "../../../core/VecParament.h"
#ifdef HAS_OPENCV
#include <opencv2/highgui/highgui.hpp>
using namespace std;
using namespace GSLAM;

class DatasetKITTI : public GSLAM::Dataset
{
public:
    DatasetKITTI(){}
    virtual std::string type() const{return "DatasetKITTI";}

    virtual bool        isOpened(){return camera.isValid();}

    virtual GSLAM::FramePtr    grabFrame(){
        if(datasetType=="Monocular") return grabMono();
        if(datasetType=="Stereo") return grabStereo();
        return GSLAM::FramePtr();
    }

    virtual bool  open(const std::string& dataset)
    {
        GSLAM::Svar var;
        if(!var.ParseFile(dataset)) return false;
        datasetFolder=var.GetString("SequenceFolder","");
        datasetType  =var.GetString("VideoType","");
        cameraIdx    =var.GetInt("CameraIdx",0);
        skip         =var.GetInt("VideoSkip",0);

        ifstream  times(datasetFolder+"/times.txt");
        if(!times.is_open()) return false;

        string line;
        while(getline(times,line))
        {
            timestamps.push_back(stod(line));
        }
        LOG(INFO)<<"Loaded "<<timestamps.size()<<" timestamps.";

        VecParament<double> camParas;
        camParas=var.get_var("Camera.Paraments",camParas);
        if(camParas.size()<6)
        {
            LOG(ERROR)<<"Camera.Paraments not setted!";
            return false;
        }
        Point3d t=var.get_var("Camera1.Translation",Point3d(0,0,0));
        right2left.get_translation()=t;
        camera=GSLAM::Camera(camParas.data);
        curIdx=0;
        return true;
    }

    GSLAM::FramePtr grabMono()
    {
        string imgPath=datasetFolder+"/image_"+to_string(cameraIdx)+"/";
        char imgFile[16];
        sprintf(imgFile,"%06d.png",curIdx);
        cv::Mat img=cv::imread(imgPath+imgFile);
        if(img.empty()) return GSLAM::FramePtr();
        GSLAM::FramePtr result(new GSLAM::FrameMono(curIdx,timestamps[curIdx],img,camera,
                                                    (img.channels()==1?GSLAM::IMAGE_GRAY:GSLAM::IMAGE_BGRA)));
        curIdx++;
        return result;
    }

    GSLAM::FramePtr grabStereo()
    {
        char imgFile[16];
        sprintf(imgFile,"%06d.png",curIdx);
        cv::Mat imgL=cv::imread(datasetFolder+"/image_0/"+imgFile);
        cv::Mat imgR=cv::imread(datasetFolder+"/image_1/"+imgFile);
        if(imgL.empty()||imgR.empty()) return GSLAM::FramePtr();
        GSLAM::FramePtr result(new GSLAM::FrameStereo(imgL,imgR,camera,camera,right2left,curIdx,timestamps[curIdx]));
        curIdx++;
        return result;
    }

    string datasetFolder;
    string datasetType;
    string dataset;
    int    cameraIdx,skip,curIdx;
    std::vector<double> timestamps;
    GSLAM::Camera   camera;
    GSLAM::SE3      right2left;
};


REGISTER_DATASET(DatasetKITTI,kitti);
#endif
