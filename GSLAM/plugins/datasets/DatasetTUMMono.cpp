#include <iterator>
#include <sstream>
#include <fstream>

#include "GSLAM/core/Dataset.h"
#include "GSLAM/core/VecParament.h"
#include "GSLAM/core/Undistorter.h"
#include "IO.h"
#include "GSLAM/core/JSON.h"

#ifdef HAS_OPENCV
#include <opencv2/highgui/highgui.hpp>
#endif

/**
 * 1. Download dataset from : https://vision.in.tum.de/data/datasets/mono-dataset
 * 2. Play dataset with gslam Dataset=<dir>/all_calib_sequences/calib_narrowGamma_scene1/.tummono
 */

using namespace std;
using namespace GSLAM;

class VideoFrameMonoWithExposure : public GSLAM::MapFrame
{
public:
    VideoFrameMonoWithExposure(const GImage& img, const Camera& camera, FrameID id, double time,
                               float explosure_time=0,const GImage& G=GImage(),const GImage& vignetteMap=GImage())
        : MapFrame(id,time),_img(img),_camera(camera),
          _explosureTime(explosure_time),_G(G),_vignetteMap(vignetteMap){

    }
    virtual std::string type() const{return "VideoFrameMonoWithExposure";}

    virtual int    imageChannels(int idx) const{return IMAGE_GRAY;}
    virtual int    cameraNum()const{return 1;}     // Camera number
    virtual Camera getCamera(int idx=0){return _camera;}

    virtual void  call(const std::string& command,void* arg=NULL)
    {
        if("getExposure"==command)
        {
            *(float*)arg=_explosureTime;
        }
        else if("getG"==command){
            *(GImage*)arg=_G;
        }
        else if("getVignetteMap"){
            *(GImage*)arg=_vignetteMap;
        }
    }

    virtual GImage getImage(int idx,int channels)
    {
        ReadMutex lock(_mutexPose);
        return _img;
    }

    float  _explosureTime;
    GImage _img,_G,_vignetteMap;
    Camera _camera;
};

class DatasetTUMMono : public GSLAM::Dataset
{
public:
    DatasetTUMMono()
        : id(0)
    {        
    }

    virtual std::string type() const {return "DatasetTUMMono";}

    virtual bool open(const string &dataset)
    {
        Svar var;
        var.ParseFile(dataset);

        datasetPath=var.GetString("SequenceFolder",Svar::getFolderPath(dataset));
        skip=var.GetInt("Video.Skip",0);

        // load camera
        if(!loadCamera((datasetPath+"/camera.txt")))
        {
            LOG(ERROR)<<"Failed to read camera.";
            return false;
        }

        if(loadVignette(datasetPath+"/vignette.png")&&loadG(datasetPath+"/pcalib.txt"))
            LOG(INFO)<<"Photometric calibrated!\n";


        times.open((datasetPath+"/times.txt").c_str());
        imgFolder=datasetPath+"/images/";
        return isOpened();
    }

    virtual bool isOpened(){return times.is_open();}

    virtual GSLAM::FramePtr grabFrame()
    {
        string line;
        for(int i=-1;i<skip;i++) getline(times,line);
        if(line.empty()) return GSLAM::FramePtr();

        stringstream sst(line);
        string imgName;
        double timestamp,explosureTime;

        sst>>imgName>>timestamp>>explosureTime;
        GImage img=imread(imgFolder+imgName+ext);
        if(img.empty()&&ext==".jpg") {
            ext=".png";
            img=imread(imgFolder+imgName+ext);
        }
        if(img.empty()||img.cols!=camera.width()||img.rows!=camera.height())
        {
            LOG(ERROR)<<"img size:"<<img.cols<<","<<img.rows;
            return GSLAM::FramePtr();
        }

        if(undis.valid()){
            GImage imgUn;
            undis.undistortFast(img,imgUn);
            return GSLAM::FramePtr(new VideoFrameMonoWithExposure(imgUn,recCamera,++id,timestamp,explosureTime,
                                                                  _G,_vignette));

        }

        return GSLAM::FramePtr(new VideoFrameMonoWithExposure(img,camera,++id,timestamp,explosureTime,
                                                              _G,_vignette));
    }

    bool loadCamera(const string& calibrationFile)
    {
        ifstream f(calibrationFile.c_str());
        if(!f.is_open())
        {
            cerr<<"Can't open file "<<calibrationFile<<endl;
            return false;
        }

        float ic[10];
        int   width,height;

        std::string l1,l2,l3,l4;

        std::getline(f,l1);
        std::getline(f,l2);
        std::getline(f,l3);
        std::getline(f,l4);
        f.close();
        {
            stringstream sst(l2);
            sst>>width>>height;
        }
        if(std::sscanf(l1.c_str(), "%f %f %f %f %f %f %f %f",
                &ic[0], &ic[1], &ic[2], &ic[3],
                &ic[4], &ic[5], &ic[6], &ic[7]) == 8)
        {
            camera=GSLAM::Camera({double(width),double(height),width*ic[0],height*ic[1],
                                  width*ic[2]-0.5,height*ic[3]-0.5,ic[4],ic[5],ic[6],ic[7],0});
        }
        else if(std::sscanf(l1.c_str(), "KannalaBrandt %f %f %f %f %f %f %f %f",
                &ic[0], &ic[1], &ic[2], &ic[3],
                &ic[4], &ic[5], &ic[6], &ic[7]) == 8)
        {
            cerr<<"Don't support KannalaBrandt camera!\n";
            return false;
        } else if(std::sscanf(l1.c_str(), "%f %f %f %f %f",
                &ic[0], &ic[1], &ic[2], &ic[3], &ic[4]) == 5)
        {
            if(ic[4]==0)
            {
                camera=GSLAM::Camera({(double)width,(double)height,width*ic[0],height*ic[1],
                                      width*ic[2]-0.5,height*ic[3]-0.5});
            }
            else
            {
                camera=GSLAM::Camera({(double)width,(double)height,width*ic[0],height*ic[1],
                                      width*ic[2]-0.5,height*ic[3]-0.5,ic[4]});
            }
        }
        else
        {
            return false;
        }
        if(!camera.isValid())
        {
            LOG(ERROR)<<camera.info()<<" is not valid!";
            return false;
        }

        float widthOut,heightOut;
        float outputCalibration[5];
        {
            stringstream sst(l4);
            sst>>widthOut>>heightOut;
        }
        // l3
        if(l3 == "crop")
        {
            recCamera=GSLAM::Camera({widthOut,heightOut,widthOut*ic[0],heightOut*ic[1],
                                     widthOut*ic[2]-0.5,heightOut*ic[3]-0.5});
        }
        else if(l3 == "full"||l3 == "none")
        {
            recCamera=camera;
        }
        else if(std::sscanf(l3.c_str(), "%f %f %f %f %f", &outputCalibration[0], &outputCalibration[1], &outputCalibration[2], &outputCalibration[3], &outputCalibration[4]) == 5)
        {

            recCamera=GSLAM::Camera({widthOut,heightOut,
                                     widthOut*outputCalibration[0],
                                     heightOut*outputCalibration[1],
                                     widthOut*outputCalibration[2]-0.5,
                                     heightOut*outputCalibration[3]-0.5});
        }
        else
        {
            return false;
        }

        undis=Undistorter(camera,recCamera);


        return true;
    }

    bool loadG(const string& gFile)
    {
        std::ifstream f(gFile.c_str());
        if(!f.is_open()) return false;

        int GDepth;
        float G[256*256];
        {
            std::string line;
            std::getline( f, line );
            std::istringstream l1i( line );
            std::vector<float> Gvec = std::vector<float>( std::istream_iterator<float>(l1i), std::istream_iterator<float>() );



            GDepth = Gvec.size();

            if(GDepth < 256)
            {
                printf("PhotometricUndistorter: invalid format! got %d entries in first line, expected at least 256!\n",(int)Gvec.size());
                return false;
            }


            for(int i=0;i<GDepth;i++) G[i] = Gvec[i];

            for(int i=0;i<GDepth-1;i++)
            {
                if(G[i+1] <= G[i])
                {
                    printf("PhotometricUndistorter: G invalid! it has to be strictly increasing, but it isnt!\n");
                    return false;
                }
            }

            float min=G[0];
            float max=G[GDepth-1];
            for(int i=0;i<GDepth;i++) G[i] = 255.0 * (G[i] - min) / (max-min);			// make it to 0..255 => 0..255.

            _G=GSLAM::GImage(1,GDepth,GSLAM::GImageType<float,1>::Type,(uchar*)G,true);
        }
        return true;
    }

    bool loadVignette(const string& vigFile)
    {
#ifdef HAS_OPENCV
        cv::Mat vigMat=cv::imread(vigFile, CV_LOAD_IMAGE_GRAYSCALE);
        if(!vigMat.empty()&&(vigMat.type()==CV_8U||vigMat.type()==CV_16U))
        {
            _vignette=vigMat;
        }
        else return false;
#endif
        return false;
    }


    int id;
    int skip;
    string ext=".jpg";
    string datasetPath,imgFolder;
    Camera camera,recCamera;
    Undistorter   undis;
    GSLAM::GImage _G,_vignette;
    ifstream times;
};

REGISTER_DATASET(DatasetTUMMono,tummono);

