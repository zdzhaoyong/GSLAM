#ifdef HAS_OPENCV

#include <GSLAM/core/Dataset.h>
#include <GSLAM/core/VecParament.h>
#include <GSLAM/core/VideoFrame.h>

#include <opencv2/highgui/highgui.hpp>

/**
 * 1. Download dataset from : https://vision.in.tum.de/data/datasets/rgbd-dataset
 * 2. Play dataset with gslam Dataset=<dir>/rgbd_dataset_freiburg1_360/rgbd.tumrgbd
 */

class DatasetTUMRGBD : public GSLAM::Dataset{
public:
    DatasetTUMRGBD():imgSkip(0),curID(1){}
    virtual std::string type() const{return "DatasetNPURGBD";}
    virtual bool        isOpened(){return camera.isValid()&&ifs.is_open();}

    virtual bool        open(const std::string& dataset)
    {
        var.ParseFile(dataset);

        video_top=var.GetString("DatasetFolder",var.getFolderPath(dataset));

        if(var.exist("Camera"))
        {

            std::string cameraName=var.GetString("Camera","");
            if(cameraName.empty()) return false;

            std::string cameraPara=var.GetString(cameraName+".Paraments","");
            VecParament<double> vecPara;
            vecPara.fromString(cameraPara);
            camera=GSLAM::Camera(vecPara.data);
        }

        if(!camera.isValid()&&!detectCamera(var.getBaseName(video_top)))
        {
            LOG(ERROR)<<"Camera not valid:"<<camera.info();
            return false;
        }

        ifs.open(var.GetString("VideoFile",video_top+"/associate.txt").c_str());
        if(!ifs.is_open())
        {
            LOG(ERROR)<<"Can't open file "<<var.GetString("VideoFile","");
            return false;
        }
        return true;
    }


    virtual GSLAM::FramePtr grabFrame(){
        double t1,t2,t3;
        double x,y,z,qx,qy,qz,qw;
        GSLAM::SE3 pose;
        std::string rgb_file,d_file;
        for(int i=0; i<imgSkip+1; i++) {
            ifs>>t1>>pose>>t2;
            ifs>>d_file>>t3>>rgb_file;
        }
        rgb_file=video_top+"/"+rgb_file;
        d_file  =video_top+"/"+d_file;

        GSLAM::GImage img=cv::imread(rgb_file);
        GSLAM::GImage depth=cv::imread(d_file,cv::IMREAD_UNCHANGED);
        if(img.empty()) return GSLAM::FramePtr();

        GSLAM::FramePtr frame(new GSLAM::FrameRGBD(img,depth,camera,curID++,t3));
        frame->setPose(pose);
        return frame;
    }

    bool detectCamera(std::string baseName){
        if(var.GetInt("UseRosCamera",1))
        {
            camera=GSLAM::Camera({640,480,525.0,525.0,319.5,239.5});
            return true;
        }

        auto idx=baseName.find("freiburg");
        if(idx==std::string::npos)
            camera=GSLAM::Camera({640,480,525.0,525.0,319.5,239.5});

        char c=baseName.at(idx+8);
        switch (c) {
        case '1':
            camera=GSLAM::Camera({640,480,517.3,516.5,318.6,255.3,0.2624,-0.9531,-0.0054,0.0026,1.1633});
            break;
        case '2':
            camera=GSLAM::Camera({640,480,520.9,521.0,325.1,249.7,0.2312,-0.7849,-0.0033,-0.0001,0.9172});
            break;
        case '3':
            camera=GSLAM::Camera({640,480,535.4,539.2,320.1,247.6});
            break;
        default:
            camera=GSLAM::Camera({640,480,525.0,525.0,319.5,239.5});
            break;
        }
        return true;
    }

    GSLAM::Svar     var;
    GSLAM::FrameID curID;
    std::string video_top;
    GSLAM::Camera camera;
    std::ifstream ifs;
    int           imgSkip;
};

REGISTER_DATASET(DatasetTUMRGBD,tumrgbd);

#endif
