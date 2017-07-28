#include "../../../core/Dataset.h"
#include "../../../core/VideoFrame.h"
#include "../../../core/VecParament.h"
#include <iterator>
#include <sstream>
#include <fstream>
#ifdef HAS_OPENCV
#include <opencv2/highgui/highgui.hpp>
using namespace std;
using namespace GSLAM;

class VideoFrameMonoWithExposure : public GSLAM::FrameMono
{
public:
    VideoFrameMonoWithExposure(const GImage& img, const Camera& camera, FrameID id, double time,const Camera& recCamera=Camera(),
                               float explosure_time=0,const GImage& G=GImage(),const GImage& vignetteMap=GImage())
        : FrameMono(id,time,img,camera,GSLAM::IMAGE_RGBA,recCamera),
          _explosureTime(explosure_time),_G(G),_vignetteMap(vignetteMap){

    }

    virtual std::string type() const{return "VideoFrameMonoWithExposure";}

    virtual void  call(const std::string& command,void* arg=NULL)
    {
        if("getExposure"==command)
        {
            *(float*)arg=_explosureTime;
        }
    }

    virtual GImage getImage(int idx,int channels)
    {
        ReadMutex lock(_mutexPose);
        if(idx==1) return _G;
        else if(idx==2) return _vignetteMap;
        else   return _img;// FIXME: .clone()?
    }
    float  _explosureTime;
    GImage _G,_vignetteMap;
};

class DatasetTUMRGBD : public GSLAM::Dataset
{
public:
    DatasetTUMRGBD(Svar& var){}
    virtual std::string type() const{return "DatasetTUMRGBD";}

    virtual bool        isOpened(){return camera.isValid();}

    virtual GSLAM::FramePtr    grabFrame(){

        return GSLAM::FramePtr();
    }

    GSLAM::FramePtr grabMono()
    {
    }

    GSLAM::FramePtr grabRGBD()
    {
    }

    GSLAM::Camera   camera;
};

class DatasetTUMMono : public GSLAM::Dataset
{
public:
    DatasetTUMMono(Svar& var)
        : id(0),datasetPath(var.GetString("SequenceFolder","")),
          skip(var.GetInt("Video.Skip",5))
    {
        // load camera
        cerr<<"Loading camera...\n";
        if(!loadCamera((datasetPath+"/camera.txt")))
        {
            return;
        }

        if(loadVignette(datasetPath+"/vignette.png")&&loadG(datasetPath+"/pcalib.txt"))
              cerr<<"Photometric calibrated!\n";


        times.open((datasetPath+"/times.txt").c_str());
        imgFolder=datasetPath+"/images/";
    }
    std::string type() const {return "DatasetTUMMono";}

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
        cv::Mat vigMat=cv::imread(vigFile, CV_LOAD_IMAGE_GRAYSCALE);
        if(!vigMat.empty()&&(vigMat.type()==CV_8U||vigMat.type()==CV_16U))
        {
            _vignette=GSLAM::GImage(vigMat.cols,vigMat.rows,vigMat.type(),vigMat.data,true);
        }
        else return false;
        return false;
    }
    bool isOpened(){return times.is_open();}

    GSLAM::FramePtr grabFrame()
    {
        string line;
        for(int i=-1;i<skip;i++) getline(times,line);
        if(line.empty()) return GSLAM::FramePtr();

        stringstream sst(line);
        string imgName;
        double timestamp,explosureTime;

        sst>>imgName>>timestamp>>explosureTime;
        cv::Mat img=cv::imread(imgFolder+imgName+".jpg",CV_LOAD_IMAGE_GRAYSCALE);
        if(img.empty()||img.cols!=camera.width()||img.rows!=camera.height())
        {
            LOG(ERROR)<<"img size:"<<img.cols<<","<<img.rows;
            return GSLAM::FramePtr();
        }

        return GSLAM::FramePtr(new VideoFrameMonoWithExposure(img,camera,id,timestamp,recCamera,explosureTime,
                                                              _G,_vignette));
    }
    int id;
    int skip;
    string datasetPath,imgFolder;
    Camera camera,recCamera;
    GSLAM::GImage _G,_vignette;
    ifstream times;
};

class DatasetTUM : public GSLAM::Dataset
{
public:
    DatasetTUM(){}

    virtual bool open(const string &dataset)
    {
        Svar var;
        if(!var.ParseFile(dataset)) return false;
        string folder=var.GetString("SequenceFolder","");
        if(access( (folder+"/times.txt").c_str(), F_OK )==0)
            _impl=DatasetPtr(new DatasetTUMMono(var));
        else _impl=DatasetPtr(new DatasetTUMRGBD(var));
        return _impl->isOpened();
    }
};

REGISTER_DATASET(DatasetTUM,tum);


#endif
