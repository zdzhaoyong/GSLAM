#include <GSLAM/core/Dataset.h>
#include <GSLAM/core/VecParament.h>
#include <GSLAM/core/VideoFrame.h>

#ifdef HAS_OPENCV

#include <opencv2/highgui/highgui.hpp>

/**
 * 1. Download dataset from : http://zhaoyong.adv-ci.com/downloads/npu-dronemap-dataset/
 * 2. Play dataset with gslam Dataset=<dir>/LibraryFloors/.npurgbd
 */
class DatasetNPURGBD : public GSLAM::Dataset{
public:
    DatasetNPURGBD():imgSkip(0),curID(1){}
    virtual std::string type() const{return "DatasetNPURGBD";}
    virtual bool        isOpened(){return camera.isValid()&&ifs.is_open();}

    virtual bool        open(const std::string& dataset)
    {
        GSLAM::Svar var;
        var.ParseFile(dataset);

        std::string cameraName=var.GetString("Camera","");
        if(cameraName.empty()) return false;

        std::string cameraPara=var.GetString(cameraName+".Paraments","");
        VecParament<double> vecPara;
        vecPara.fromString(cameraPara);
        camera=GSLAM::Camera(vecPara.data);

        if(!camera.isValid())
        {
            LOG(ERROR)<<"Camera not valid:"<<camera.info();
            return false;
        }
        ifs.open(var.GetString("VideoFile","").c_str());
        if(!ifs.is_open())
        {
            LOG(ERROR)<<"Can't open file "<<var.GetString("VideoFile","");
            return false;
        }
        video_top=var.getFolderPath(dataset);
        return true;
    }


    virtual GSLAM::FramePtr grabFrame(){
        double t1,t2,t3;
        double x,y,z,qx,qy,qz,qw;
        std::string rgb_file,d_file;
        for(int i=0; i<imgSkip+1; i++) {
            ifs>>t1>>x>>y>>z>>qx>>qy>>qz>>qw>>t2;
            ifs>>d_file>>t3>>rgb_file;
        }
        rgb_file=video_top+"/"+rgb_file;
        d_file  =video_top+"/"+d_file;

        GSLAM::GImage img=cv::imread(rgb_file);
        GSLAM::GImage depth=cv::imread(d_file,cv::IMREAD_UNCHANGED);
//        if(img.empty()) return GSLAM::FramePtr();

        return GSLAM::FramePtr(new GSLAM::FrameRGBD(img,depth,camera,curID++,t3));
    }

    GSLAM::FrameID curID;
    std::string video_top;
    GSLAM::Camera camera;
    std::ifstream ifs;
    int           imgSkip;
};


REGISTER_DATASET(DatasetNPURGBD,npurgbd);
#endif
