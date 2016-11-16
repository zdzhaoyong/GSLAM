#include "VideoReader.h"
#include <sstream>
#include <fstream>
#include <iterator>
#include <GSLAM/core/types/VideoFrame.h>
#include <base/Svar/Svar.h>
#include <base/Time/Timestamp.h>
#include <base/Path/Path.h>
#include <cv/Camera/Undistorter.h>
#include <cv/Camera/CameraImpl.h>

#include <opencv2/highgui/highgui.hpp>

using namespace std;

class VideoReaderMonoOpenCV : public VideoReader
{
public:
    VideoReaderMonoOpenCV(const std::string& name)
        : frameId(0),video(svar.GetString(name+".VideoFile","")),
          camera(svar.GetString(name+".Camera","DefaultCamera")),
          skip(svar.GetInt(name+".Skip",5))
    {
        if(!video.isOpened())
        {
            cerr<<"Can't open video file "<<svar.GetString(name+".VideoFile","")<<endl;
            video.open(0);
        }
        if(video.isOpened())
        {
            int width=video.get(CV_CAP_PROP_FRAME_WIDTH);
            int height=video.get(CV_CAP_PROP_FRAME_HEIGHT);
            if(camera.width()!=width||camera.height()!=height)
            {
                cout<<"The video size ["<<width<<","<<height<<"] is not corrosponding to the setted camera:"<<camera.info()<<endl;
                video.release();
            }
        }
    }

    std::string type() const {return "VideoReaderMonoOpenCV";}

    bool isOpened(){return video.isOpened()&&camera.isValid();}

    GSLAM::FramePtr grabFrame()
    {
        for(int i=0;i<skip;i++) video.grab();
//        double timestamp=video.get(CV_CAP_PROP_POS_MSEC)*0.001;
        double timestamp=pi::Timestamp().timestampF();
        video>>img;
        GSLAM::GImage gimg(img.cols,img.rows,img.type(),img.data,true);
        return SPtr<GSLAM::VideoFrameMono>(new GSLAM::VideoFrameMono(gimg,camera,frameId++,timestamp));
    }

    GSLAM::FrameID   frameId;
    cv::VideoCapture video;
    GSLAM::Camera    camera;
    cv::Mat img;
    int&             skip;
};

class VideoReaderMonoOpenCVWithUndistorter : public VideoReader
{
public:
    VideoReaderMonoOpenCVWithUndistorter(const std::string& name)
        : frameId(0),video(svar.GetString(name+".VideoFile","")),
          cameraIn(svar.GetString(name+".CameraIn","DefaultCamera")),
          cameraOut(svar.GetString(name+".CameraOut","DefaultCamera")),
          undis(GSLAM::Camera(),GSLAM::Camera()),
          skip(svar.GetInt(name+".Skip",5))
    {
        if(!video.isOpened())
        {
            cerr<<"Can't open video file "<<svar.GetString(name+".VideoFile","")<<endl;
            video.open(0);
        }
        // check camera valid
        if(cameraIn.isValid()&&cameraOut.isValid())
        {
            undis=pi::Undistorter(cameraIn,cameraOut);
            if(!undis.valid()) {
                cerr<<"Failed to genarate Undistorter.\n";
                video.release();
            }
        }
        else
        {
            cerr<<"The cameras are not valid.\n";
            video.release();
        }

        if(video.isOpened())
        {
            int width=video.get(CV_CAP_PROP_FRAME_WIDTH);
            int height=video.get(CV_CAP_PROP_FRAME_HEIGHT);
            if(cameraIn.width()!=width||cameraIn.height()!=height)
            {
                cout<<"The video size ["<<width<<","<<height<<"] is not corrosponding to the setted camera:"
                   <<cameraIn.info()<<endl;
                video.release();
            }
        }
    }


    std::string type() const {return "VideoReaderMonoOpenCVWithUndistorter";}

    bool isOpened(){return video.isOpened()&&cameraIn.isValid();}

    GSLAM::FramePtr grabFrame()
    {
        for(int i=0;i<skip;i++) video.grab();
//        double timestamp=video.get(CV_CAP_PROP_POS_MSEC)*0.001;
        double timestamp=pi::Timestamp().timestampF();
        video>>img;
        if(img.empty()) return GSLAM::FramePtr();
        undis.undistort(img.clone(),img);

        GSLAM::GImage gimg(img.cols,img.rows,img.type(),img.data,true);
        return SPtr<GSLAM::VideoFrameMono>(new GSLAM::VideoFrameMono(gimg,cameraOut,frameId++,timestamp));
    }

    GSLAM::FrameID   frameId;
    cv::VideoCapture video;
    GSLAM::Camera    cameraIn,cameraOut;
    pi::Undistorter  undis;
    cv::Mat img;
    int&             skip;
};

///
/// \brief The VideoReaderTUMMonoDataset class load the TUM Mono Dataset and publish
/// frames as VideoFrameMono or VideoFrameMonoWithExplosure
///
class VideoReaderTUMMonoDataset: public VideoReader
{
public:
    VideoReaderTUMMonoDataset(const std::string& name)
        : id(0),datasetPath(svar.GetString(name+".VideoFile","")),
          skip(svar.GetInt(name+".Skip",5)),
          doUndistort(svar.GetInt(name+".Undistort",1))
    {
        pi::Path path(datasetPath);
        datasetPath=path.getFolderPath();
        if(!pi::Path::pathExist(datasetPath+"/images"))
        {
            cerr<<"Path "<<datasetPath+"/images"<<" does not exist! Please untar the images.zip first.\n";
            return;
        }

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

    bool loadCamera(const string& calibrationFile)
    {
        ifstream f(calibrationFile.c_str());
        if(!f.is_open())
        {
            cerr<<"Can't open file "<<calibrationFile<<endl;
            return false;
        }
        SPtr<pi::CameraImpl> cameraP,recCameraP;

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
            cameraP=SPtr<pi::CameraImpl>(new pi::CameraOpenCV(width,height,width*ic[0],height*ic[1],
                    width*ic[2]-0.5,height*ic[3]-0.5,ic[4],ic[5],ic[6],ic[7],0));
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
                cameraP=SPtr<pi::CameraImpl>(new pi::CameraPinhole(width,height,width*ic[0],height*ic[1],
                        width*ic[2]-0.5,height*ic[3]-0.5));
            }
            else
            {
                cameraP=SPtr<pi::CameraImpl>(new pi::CameraPTAM(width,height,width*ic[0],height*ic[1],
                        width*ic[2]-0.5,height*ic[3]-0.5,ic[4]));
            }
        }
        else
        {
            return false;
        }
        if(!cameraP.get()) return false;

        float widthOut,heightOut;
        float outputCalibration[5];
        {
            stringstream sst(l4);
            sst>>widthOut>>heightOut;
        }
        // l3
        if(l3 == "crop")
        {
            recCameraP=SPtr<pi::CameraImpl>(new pi::CameraPinhole(widthOut,heightOut,widthOut*ic[0],
                                            heightOut*ic[1],widthOut*ic[2]-0.5,heightOut*ic[3]-0.5));
        }
        else if(l3 == "full"||l3 == "none")
        {
            recCameraP=cameraP;
        }
        else if(std::sscanf(l3.c_str(), "%f %f %f %f %f", &outputCalibration[0], &outputCalibration[1], &outputCalibration[2], &outputCalibration[3], &outputCalibration[4]) == 5)
        {

            recCameraP=SPtr<pi::CameraImpl>(new pi::CameraPinhole(widthOut,heightOut,widthOut*outputCalibration[0],
                                            heightOut*outputCalibration[1],widthOut*outputCalibration[2]-0.5,heightOut*outputCalibration[3]-0.5));
        }
        else
        {
            return false;
        }

        if(!recCameraP.get()) return false;
        camera=pi::Camera(cameraP);
        recCamera=pi::Camera(recCameraP);
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

    std::string type() const {return "VideoReaderTUMMonoDataset";}

    bool isOpened(){return times.is_open();}

    GSLAM::FramePtr grabFrame()
    {
        string line;
        for(int i=0;i<skip;i++) getline(times,line);
        if(line.empty()) return GSLAM::FramePtr();

        stringstream sst(line);
        string imgName;
        double timestamp,explosureTime;

        sst>>imgName>>timestamp>>explosureTime;
        cv::Mat img=cv::imread(imgFolder+imgName+".jpg",CV_LOAD_IMAGE_GRAYSCALE);
        if(img.empty()||img.cols!=camera.width()||img.rows!=camera.height())
        {
            cerr<<"img size:"<<img.cols<<","<<img.rows<<endl;
            cerr.flush();
            return GSLAM::FramePtr();
        }

        if(doUndistort)
        {
            if(!undistorter.get())
            {
                undistorter=SPtr<pi::Undistorter>(new pi::Undistorter(camera,recCamera));
            }
            undistorter->undistort(img.clone(),img);
            return GSLAM::FramePtr(new GSLAM::VideoFrameMonoWithExposure(GSLAM::GImage(img.cols,img.rows,img.type(),img.data,true),
                                                                         recCamera,id,timestamp,GSLAM::Camera(),explosureTime,
                                                                         _G,_vignette));
        }

        return GSLAM::FramePtr(new GSLAM::VideoFrameMonoWithExposure(GSLAM::GImage(img.cols,img.rows,img.type(),img.data,true),
                                                                     camera,id,timestamp,recCamera,explosureTime,
                                                                     _G,_vignette));
    }

    int id;
    string datasetPath,imgFolder;
    pi::Camera camera,recCamera;
    SPtr<pi::Undistorter> undistorter;
    GSLAM::GImage _G,_vignette;
    ifstream times;
    int skip;
    bool doUndistort;
};

VideoReader::VideoReader(const std::string& name)
{
    if(name.empty()) return;
    string type=svar.GetString(name+".Type","");
    if("VideoReaderMonoOpenCV"==type)
        impl=SPtr<VideoReader>(new VideoReaderMonoOpenCV(name));
    if("VideoReaderMonoOpenCVWithUndistorter"==type)
        impl=SPtr<VideoReader>(new VideoReaderMonoOpenCVWithUndistorter(name));
    if("VideoReaderTUMMonoDataset"==type)
        impl=SPtr<VideoReader>(new VideoReaderTUMMonoDataset(name));
}

std::string VideoReader::type() const
{
    if(impl.get()) return impl->type();
    else return "VideoReader";
}

GSLAM::FramePtr VideoReader::grabFrame()
{
    if(impl.get()) return impl->grabFrame();
    else return GSLAM::FramePtr();
}

bool VideoReader::isOpened()
{
    if(impl.get()) return impl->isOpened();
    else return false;
}
