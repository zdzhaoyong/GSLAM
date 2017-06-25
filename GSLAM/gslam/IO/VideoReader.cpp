#include "VideoReader.h"
#include <sstream>
#include <fstream>
#include <iterator>
#include "../../core/VideoFrame.h"
#include "../../core/Svar.h"
#include "../../core/Undistorter.h"

#include <opencv2/highgui/highgui.hpp>
#include <pil/base/Types/VecParament.h>
#include <pil/base/Time/Timestamp.h>
#include <pil/base/Path/Path.h>

using namespace std;


GSLAM::Camera camFromName(string name)
{
    VecParament<double> paras;
    paras=svar.get_var(name+".Paraments",paras);
    return GSLAM::Camera(paras.data);
}


class VideoReaderMonoOpenCV : public VideoReader
{
public:
    VideoReaderMonoOpenCV(const std::string& name)
        : frameId(0),video(svar.GetString(name+".VideoFile","")),
          camera(camFromName(svar.GetString(name+".Camera","DefaultCamera"))),
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
        return SPtr<GSLAM::FrameMono>(new GSLAM::FrameMono(gimg,camera,frameId++,timestamp));
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
          cameraIn(camFromName(svar.GetString(name+".CameraIn","DefaultCamera"))),
          cameraOut(camFromName(svar.GetString(name+".CameraOut","DefaultCamera"))),
          undis(),
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
            undis=GSLAM::Undistorter(cameraIn,cameraOut);
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
        return SPtr<GSLAM::FrameMono>(new GSLAM::FrameMono(gimg,cameraOut,frameId++,timestamp));
    }

    GSLAM::FrameID   frameId;
    cv::VideoCapture video;
    GSLAM::Camera    cameraIn,cameraOut;
    GSLAM::Undistorter  undis;
    cv::Mat img;
    int&             skip;
};


VideoReader::VideoReader(const std::string& name)
{
    if(name.empty()) return;
    string type=svar.GetString(name+".Type","");
    if("VideoReaderMonoOpenCV"==type)
        impl=SPtr<VideoReader>(new VideoReaderMonoOpenCV(name));
    if("VideoReaderMonoOpenCVWithUndistorter"==type)
        impl=SPtr<VideoReader>(new VideoReaderMonoOpenCVWithUndistorter(name));
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
