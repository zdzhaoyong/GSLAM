#ifdef HAS_OPENCV
#include "../../core/Dataset.h"
#include "../../core/VideoFrame.h"
#include "../../core/Svar.h"
#include "../../core/VecParament.h"
#include "../../core/Timer.h"


#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace GSLAM;

inline GSLAM::Camera camFromName(string name,Svar& var)
{
    VecParament<double> paras;
    paras=var.get_var(name+".Paraments",paras);
    return GSLAM::Camera(paras.data);
}

class DatasetOpenCVMono : public GSLAM::Dataset
{
public:
    DatasetOpenCVMono(){
//        cerr<<"Created Dataset "<<type();
    }
    virtual std::string type() const{return "cvmono";}

    virtual bool open(const std::string& dataset){
        ifstream ifs(dataset.c_str());
        if(ifs.is_open())// is a file
        {
            Svar var;
            var.ParseStream(ifs);
            return open(var,"Dataset");
        }
        return open(svar,dataset);
    }

    bool open(Svar& var,const std::string& name)
    {
        if(!video.open(var.s[name+".VideoFile"])) return false;
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
        GSLAM::GImage gimg(img.cols,img.rows,img.type(),img.data,true);
        return SPtr<GSLAM::FrameMono>(new GSLAM::FrameMono(gimg,camera,frameId++,timestamp,IMAGE_BGRA));
    }

    GSLAM::FrameID   frameId;
    cv::VideoCapture video;
    GSLAM::Camera    camera;
    cv::Mat          img;
    int              skip;
};

#if 1
// Buildin
REGISTER_DATASET(DatasetOpenCVMono)
#else
// Use as a plugin
extern "C"
{
SPtr<Dataset> createDataset()
{
    return SPtr<Dataset>(new DatasetOpenCVMono());
}
}
#endif


#endif // HAS_OPENCV

