#include "GSLAM/core/Dataset.h"
#include "GSLAM/core/VideoFrame.h"
#include "GSLAM/core/VecParament.h"
#include "IO.h"

using namespace std;
using namespace GSLAM;

/**
 * 1. Download dataset from : http://www.cvlibs.net/datasets/kitti/
 * 2. Play dataset with gslam Dataset=<dir>/KITTI/odomentry/color/00/stereo.kitti
 */
class DatasetKITTI : public GSLAM::Dataset
{
public:
    enum DatasetType{
        MONOCULAR_MODE,STEREO_MODE,MULTI_MODE
    };

    DatasetKITTI(){}
    virtual std::string type() const{return "DatasetKITTI";}

    virtual bool        isOpened(){return timestamps.size();}

    virtual GSLAM::FramePtr    grabFrame(){
        if(datasetType==MONOCULAR_MODE) return grabMono();
        return grabStereo();
        return GSLAM::FramePtr();
    }

    virtual bool  open(const std::string& dataset)
    {
        GSLAM::Svar var;
        var.ParseFile(dataset);
        datasetFolder=var.GetString("SequenceFolder",Svar::getFolderPath(dataset));
        string typeStr  =var.GetString("VideoType",Svar::getBaseName(dataset));
        cameraIdx    =var.GetInt("CameraIdx",0);
        skip         =var.GetInt("VideoSkip",0);

        if(!loadCameras()) {
            return false;
        }
        if(!updateCameraMask()) return false;
        if(!updateDatasetType(typeStr)) return false;

        ifstream  times(datasetFolder+"/times.txt");
        if(!times.is_open()) return false;

        string line;
        while(getline(times,line))
        {
            timestamps.push_back(stod(line));
        }
        LOG(INFO)<<"Loaded "<<timestamps.size()<<" timestamps.";
        curIdx=0;
        loadGroundPose(var.GetString("GroundFile",datasetFolder+"/pose.txt"));
        return true;
    }

    GSLAM::FramePtr grabMono()
    {
        string imgPath=datasetFolder+"/image_"+to_string(cameraIdx)+"/";
        char imgFile[16];
        sprintf(imgFile,"%06d.png",curIdx);
        GImage img=imread(imgPath+imgFile);
        if(img.empty()) return GSLAM::FramePtr();
        GSLAM::FramePtr result(new GSLAM::FrameMono(curIdx,timestamps[curIdx],img,camera[cameraIdx],
                                                    (img.channels()==1?GSLAM::IMAGE_GRAY:GSLAM::IMAGE_BGRA)));
        if(groundPose.size()==timestamps.size())
        {
            result->setPose(groundPose[curIdx]);
        }
        curIdx++;

        return result;
    }

    GSLAM::FramePtr grabStereo()
    {
        char imgFile[16];
        sprintf(imgFile,"/%06d.png",curIdx);

        std::vector<GImage> images;
        std::vector<Camera> cameras;
        std::vector<SE3>    camPoses;
        for(int i=0;i<4;i++){
            if(!(cameraMask&1<<i)) continue;
            GImage img=imread(datasetFolder+"/image_"+to_string(i)+imgFile);
            if(img.empty()) return GSLAM::FramePtr();
            images.push_back(img);
            cameras.push_back(camera[i]);
            camPoses.push_back(camPos[i]);
            if(images.size()==2&&datasetType==STEREO_MODE) break;
        }
        if(images.size()<2) return GSLAM::FramePtr();
        GSLAM::FramePtr result(new GSLAM::FrameStereo(images[0],images[1],
                cameras[0],cameras[1],camPoses[0].inverse()*camPoses[1],
                curIdx,timestamps[curIdx]));
        if(groundPose.size()==timestamps.size())
        {
            result->setPose(groundPose[curIdx]);
        }
        curIdx++;
        return result;
    }

    bool loadCameras(){
        ifstream ifs(datasetFolder+"/calib.txt");
        if(!ifs.is_open()) return false;

        string line;
        for(int i=0;i<4&&getline(ifs,line);i++){
            VecParament<double> p(line.substr(4));
            if(p.size()!=12) return false;
            double fx=p[0],fy=p[5],cx=p[2],cy=p[6],x=-p[3],y=-p[7],z=-p[11];

            camera[i]=GSLAM::Camera({1241,376,fx,fy,cx,cy});
            camPos[i]=SE3(SO3(),Point3d(x/fx,y/fy,z));
        }
        return true;
    }

    bool updateCameraMask(){
        cameraMask=0;
        cameraNumber=0;
        for(int i=0;i<4;i++){
            if(imread(datasetFolder+"/image_"+std::to_string(i)+"/000000.png").empty()) continue;
            cameraMask|=1<<i;
            cameraNumber++;
        }
        return cameraMask;
    }

    bool updateDatasetType(string typeStr){
        if(typeStr=="Mono"||typeStr=="Monocular"||typeStr=="mono") datasetType=MONOCULAR_MODE;
        else if(typeStr=="Stereo"||typeStr=="stereo") datasetType=STEREO_MODE;
        else datasetType=MULTI_MODE;

        if(cameraNumber==0) return false;
        if(datasetType==MONOCULAR_MODE) {
            if(cameraMask&1<<cameraIdx) return true;
            for(cameraIdx=0;!(cameraMask&1<<cameraIdx);cameraIdx++);
            return true;
        }
//        if(cameraNumber==2&&datasetType==MULTI_MODE)
            datasetType=STEREO_MODE;// don't use multi mode

        return true;
    }

    bool loadGroundPose(string file){
        ifstream ifs(file);
        if(!ifs.is_open()) return false;

        string line;
        while(getline(ifs,line)){
            VecParament<double> vec(line);
            if(vec.size()!=12) return false;
            SE3 se3;
            se3.fromMatrix(vec.data.data());
            groundPose.push_back(se3);
        }

        return groundPose.size()==timestamps.size();
    }

    string datasetFolder;
    DatasetType datasetType;
    int    cameraIdx,skip,curIdx,cameraMask;
    int    cameraNumber=0;

    std::vector<double> timestamps;
    std::vector<SE3>    groundPose;
    GSLAM::Camera       camera[4];
    GSLAM::SE3          camPos[4];
};


REGISTER_DATASET(DatasetKITTI,kitti);
