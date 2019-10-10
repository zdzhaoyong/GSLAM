#ifdef HAS_OPENCV
#include "GSLAM/core/Dataset.h"
#include "GSLAM/core/VideoFrame.h"
#include "GSLAM/core/Svar.h"
#include "GSLAM/core/VecParament.h"
#include "GSLAM/core/Timer.h"


#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace GSLAM;

inline GSLAM::Camera camFromName(string name,Svar& var)
{
    VecParament<double> paras;
    paras=var.get(name+".Paraments",paras);
    return GSLAM::Camera(paras.data);
}

class DatasetOpenCVMono : public GSLAM::Dataset
{
public:
    DatasetOpenCVMono(){
//        cerr<<"Created Dataset "<<type();
    }
    virtual std::string type() const{return "DatasetOpenCVMono";}

    virtual bool open(const std::string& dataset){
        ifstream ifs(dataset.c_str());
        if(ifs.is_open())// is a file
        {
            ifs.close();
            Svar var;
            var.parseFile(dataset.c_str());
            return open(var,"Dataset");
        }
        return open(svar,dataset);
    }

    bool open(Svar& var,const std::string& name)
    {
        if(!video.open(var.GetString(name+".VideoFile",""))) return false;
        camera=camFromName(name+".Camera",var);
        if(!camera.isValid()) return false;
        skip=var.GetInt(name+".Skip",0);
        _name=name;
        frameId=1;
        return true;
    }

    virtual bool isOpened(){return video.isOpened()&&camera.isValid();}

    virtual FramePtr grabFrame(){
        for(int i=0;i<skip;i++) video.grab();
        //        double timestamp=video.get(CV_CAP_PROP_POS_MSEC)*0.001;
        double timestamp=GSLAM::TicToc::timestamp();
        video>>img;
        GSLAM::GImage gimg(img.rows,img.cols,img.type(),img.data,true);
        return std::shared_ptr<GSLAM::FrameMono>(new GSLAM::FrameMono(frameId++,timestamp,gimg,camera,IMAGE_BGRA));
    }

    GSLAM::FrameID   frameId;
    cv::VideoCapture video;
    GSLAM::Camera    camera;
    cv::Mat          img;
    int              skip;
};

GSLAM_REGISTER_DATASET(DatasetOpenCVMono,cvmono)


#endif // HAS_OPENCV

