#ifdef HAS_OPENCV
#include <GSLAM/core/Dataset.h>
#include <GSLAM/core/VideoFrame.h>
#include <opencv2/highgui/highgui.hpp>

/**
 * 1. Download dataset from : https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets
 * 2. Play dataset with gslam Dataset=<dir>/MH_01_easy/mav0/mono.euroc
 */

namespace GSLAM {

class FrameIMUEuroc : public MapFrame
{
public:
    FrameIMUEuroc(FrameID id,double time,
             Point3d acc ,Point3d angularV,
             Point3d accN,Point3d gyrN,SE3 imu2body=SE3())
           : MapFrame(id,time),
             _acc(acc),_angularV(angularV),
             _accN(accN),_gyrN(gyrN),_i2b(imu2body){}

    virtual std::string type() const{return "FrameIMU";}
    virtual int     getIMUNum()const{return 1;}
    virtual bool    getAcceleration(Point3d& acc,int idx=0)const{acc=_acc;return true;}        // m/s^2
    virtual bool    getAccelerationNoise(Point3d &accN, int idx) const{accN=_accN;return true;}
    virtual bool    getAngularVelocity(Point3d& angularV,int idx=0)const{angularV=_angularV;return true;}// rad/s
    virtual bool    getAngularVNoise(Point3d &angularVN, int idx) const{angularVN=_gyrN;return true;}

    Point3d _acc,_angularV,_accN,_gyrN;
    SE3     _i2b;
};

class FrameMonoEuroc : public MapFrame
{
public:
    FrameMonoEuroc(FrameID id,double time,
                   const GImage& img,const Camera& camera,
                   const SE3& camera2body)
        : MapFrame(id,time),_imgLeft(img),_camLeft(camera),_c2bLeft(camera2body){}

    virtual std::string type() const{return "FrameMonoEuroc";}

    virtual int    cameraNum()const{return 1;}     // Camera number
    virtual GImage getImage(int idx,int channels){return _imgLeft;} // 0:origin image
    virtual Camera getCamera(int idx=0){return _camLeft;}
    virtual SE3    getCameraPose(int idx) const{return _c2bLeft;}
    virtual int    imageChannels(int idx=0) const{return IMAGE_GRAY;}

protected:
    GImage       _imgLeft;
    Camera       _camLeft;
    SE3          _c2bLeft;
};

class FrameStereoEuroc : public FrameMonoEuroc{
public:
    FrameStereoEuroc(FrameID id,double timestamp,
                     const GImage& imgLeft,const GImage& imgRight,
                     const Camera& cameraLeft,const Camera& cameraRight,
                     const SE3&    poseLeft,const SE3& poseRight)
        : FrameMonoEuroc(id,timestamp,imgLeft,cameraLeft,poseLeft),
          _imgRight(imgRight),_camRight(cameraRight),_c2bRight(poseRight){}


    virtual std::string type() const{return "FrameStereoEuroc";}

    virtual int    cameraNum()const{return 2;}     // Camera number
    virtual GImage getImage(int idx,int channels){return idx==0?_imgLeft:_imgRight;} // 0:origin image
    virtual Camera getCamera(int idx=0){return idx==0?_camLeft:_camRight;}
    virtual SE3    getCameraPose(int idx) const{return idx==0?_c2bLeft:_c2bRight;}

    GImage       _imgRight;
    Camera       _camRight;
    SE3          _c2bRight;
};

class DatasetEuroc : public Dataset{
public:
    DatasetEuroc():curID(1){}
    virtual std::string type() const{return "DatasetEuroc";}
    virtual bool        isOpened(){return cam0.isValid()&&ifs0.is_open();}

    virtual bool        open(const std::string& dataset)
    {
        svar.ParseFile(dataset);
        dirtop=Svar::getFolderPath(dataset);
        std::string basename=Svar::getBaseName(dataset);
        // load camera data
        checkHeader(dirtop+"/cam0/sensor.yaml");
        checkHeader(dirtop+"/cam1/sensor.yaml");

        cv::FileStorage cam0Sensor(dirtop+"/cam0/sensor.yaml", cv::FileStorage::READ);
        cv::FileStorage cam1Sensor(dirtop+"/cam1/sensor.yaml", cv::FileStorage::READ);

        if(!cam0Sensor.isOpened()&&!cam1Sensor.isOpened())
        {
            LOG(ERROR)<<"Can't load camera sensor data.";
            return false;
        }

        cam0 =loadCamera(cam0Sensor);
        if(basename!="mono"&&basename!="Monocular")
            cam1 =loadCamera(cam1Sensor);

        cam0P=loadPose(cam0Sensor);
        cam1P=loadPose(cam1Sensor);

        ifs0.open(dirtop+"/cam0/data.csv");
        ifs1.open(dirtop+"/cam1/data.csv");
        std::string line;
        if(ifs0.is_open()) getline(ifs0,line);
        else return false;

        if(ifs1.is_open()) getline(ifs1,line);

        checkHeader(dirtop+"/imu0/sensor.yaml");
        cv::FileStorage imu0Sensor(dirtop+"/imu0/sensor.yaml", cv::FileStorage::READ);
        if(!imu0Sensor.isOpened()) {
            LOG(ERROR)<<"Can't load imu sensor data.";
            return false;
        }

        imu0P=loadPose(imu0Sensor);
        loadIMUNoise(imu0Sensor,gyrNoise,accNoise);

        ifsIMU.open(dirtop+"/imu0/data.csv");

//        LOG(INFO)<<"CAM0: "<<cam0.info()<<","<<cam0P
//                <<"\nCAM1: "<<cam1.info()<<","<<cam1P
//               <<"\nIMU0: "<<imu0P<<",Noise: gyr["<<gyrNoise<<"],acc["<<accNoise<<"]";

        if(ifsIMU.is_open()) getline(ifsIMU,line);
        else return false;

        nextImageFrame=grabImageFrame();
        nextIMUFrame  =grabIMUFrame();

        return nextImageFrame&&nextIMUFrame;
    }

    Camera loadCamera(cv::FileStorage& fs){
        cv::FileNode wh=fs["resolution"];
        cv::FileNode k=fs["intrinsics"];
        cv::FileNode d=fs["distortion_coefficients"];

        if(wh.isNone()||k.isNone()) return Camera();

        return Camera({wh[0],wh[1],k[0],k[1],k[2],k[3],
                              d[0],d[1],d[2],d[3],0.});
    }

    bool   checkHeader(std::string file){
        using namespace std;
        std::ifstream ifs(file.c_str());
        if(!ifs.is_open()) return false;
        std::vector<std::string> lines;
        std::string line;
        if(getline(ifs,line)&&line=="%YAML:1.0") return true;

        while(getline(ifs,line)) lines.push_back(line);
        ifs.close();

        std::ofstream ofs(file.c_str());
        ofs<<"%YAML:1.0\n";
        for(auto line:lines) ofs<<line<<endl;
        return true;
    }

    SE3    loadPose(cv::FileStorage& fs){
        cv::FileNode T_BS=fs["T_BS"];
        if(T_BS.isNone()) return SE3();

        cv::FileNode m=T_BS["data"];
        if(m.isNone()) return SE3();
        double M[12]={m[0],m[1],m[2],m[3],
                      m[4],m[5],m[6],m[7],
                      m[8],m[9],m[10],m[11]};
        SE3 pose;
        pose.fromMatrix(M);
        return pose;
    }

    bool loadIMUNoise(cv::FileStorage& fs,Point3d& gyrNoise,Point3d& accNoise){
        double  gyroscope_noise_density,gyroscope_random_walk,
                accelerometer_noise_density,accelerometer_random_walk;
        fs["gyroscope_noise_density"]>>gyroscope_noise_density;
        fs["gyroscope_random_walk"]>>gyroscope_random_walk;
        fs["accelerometer_noise_density"]>>accelerometer_noise_density;
        fs["accelerometer_random_walk"]>>accelerometer_random_walk;
        gyrNoise=Point3d(gyroscope_noise_density,gyroscope_random_walk,0);
        accNoise=Point3d(accelerometer_noise_density,accelerometer_random_walk,0);
        return true;
    }

    GSLAM::FramePtr grabIMUFrame(){
        std::string line;
        double      time,rx,ry,rz,ax,ay,az;
        if(!std::getline(ifsIMU,line)) return nullptr;
        sscanf(line.c_str(),"%lf,%lf,%lf,%lf,%lf,%lf,%lf",
               &time,&rx,&ry,&rz,&ax,&ay,&az);

        return GSLAM::FramePtr(new FrameIMUEuroc(curID++,time*1e-9,Point3d(ax,ay,az),Point3d(rx,ry,rz),accNoise,gyrNoise,imu0P));
    }

    GSLAM::FramePtr grabImageFrame(){
        std::string line;
        if(!std::getline(ifs0,line)) return GSLAM::FramePtr();

        int idx=line.find_first_of(',');
        if(idx==std::string::npos) return GSLAM::FramePtr();
        double nextImageTime=std::stod(line.substr(0,idx))*1e-9;
        std::string nextImage=line.substr(0,idx)+".png";

        GSLAM::GImage img0=cv::imread(dirtop+"/cam0/data/"+nextImage);
        if(img0.empty())
            LOG(ERROR)<<"Failed to open image "<<nextImage;

        if(cam1.isValid()){
            GSLAM::GImage img1=cv::imread(dirtop+"/cam1/data/"+nextImage);
            return FramePtr(new FrameStereoEuroc(curID++,nextImageTime,img0,img1,cam0,cam1,cam0P,cam1P));
        }

        return FramePtr(new FrameMonoEuroc(curID++,nextImageTime,img0,cam0,cam0P));
    }

    virtual GSLAM::FramePtr grabFrame(){
        if(!nextImageFrame||!nextIMUFrame) return FramePtr();
        GSLAM::FramePtr result;
        if(nextImageFrame->timestamp()<nextIMUFrame->timestamp()){
            result=nextImageFrame;
            nextImageFrame=grabImageFrame();
//            LOG(INFO)<<"Frame:"<<result->timestamp();
        }
        else {
            result=nextIMUFrame;
            nextIMUFrame=grabIMUFrame();
        }
        return result;
    }

    std::string    dirtop;
    GSLAM::Camera  cam0,cam1;
    GSLAM::SE3     cam0P,cam1P,imu0P;// Treat IMU as body
    Point3d        gyrNoise,accNoise;
    std::ifstream  ifs0,ifs1,ifsIMU;

    GSLAM::FrameID curID;
    FramePtr       nextImageFrame,nextIMUFrame;
};

REGISTER_DATASET(DatasetEuroc,euroc);
}
#endif
