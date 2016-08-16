#include "VideoReader.h"
#include <GSLAM/core/types/VideoFrame.h>
#include <base/Svar/Svar.h>

#include <opencv2/highgui/highgui.hpp>

using namespace std;

class VideoReaderMonoOpenCV : public VideoReader
{
public:
    VideoReaderMonoOpenCV(const std::string& name)
        : frameId(0),video(svar.GetString(name+".VideoFile","")),
          camera(svar.GetString(name+".Camera","DefaultCamera"))
    {
    }

    std::string type() const {return "VideoReaderMonoOpenCV";}

    bool isOpened(){return video.isOpened()&&camera.isValid();}

    GSLAM::FramePtr grabFrame()
    {
        double timestamp=video.get(CV_CAP_PROP_POS_MSEC)*0.001;
        video>>img;
        GSLAM::GImage gimg(img.cols,img.rows,img.type(),img.data);
        return SPtr<GSLAM::VideoFrameMono>(new GSLAM::VideoFrameMono(gimg,camera,frameId++,timestamp));
    }

    GSLAM::FrameID   frameId;
    cv::VideoCapture video;
    GSLAM::Camera    camera;
    cv::Mat img;
};

VideoReader::VideoReader(const std::string& name)
{
    if(name.empty()) return;
    string type=svar.GetString(name+".Type","");
    if("VideoReaderMonoOpenCV"==type)
        impl=SPtr<VideoReader>(new VideoReaderMonoOpenCV(name));
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
