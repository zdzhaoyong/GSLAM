#pragma once
#include <GSLAM/core/GImage.h>

#ifdef HAS_OPENCV
#include <opencv2/highgui/highgui.hpp>

namespace GSLAM {

inline GImage imread(const std::string& filepath,int flags=1)
{
    cv::Mat img= cv::imread(filepath,flags);
    return img;
}

}

#elif defined(HAS_QT)

#include <QImage>

namespace GSLAM {

inline GImage imread(const std::string& filepath,int flags=1)
{
    GSLAM::GImage result;
    QImage qimage(filepath.c_str());
    if(qimage.format()==QImage::Format_RGB32)
    {
        result=GImage(qimage.height(),qimage.width(),
                      GImageType<uchar,4>::Type,qimage.bits(),true);// BGRA
    }
    else if(qimage.format()==QImage::Format_RGB888){
        qimage=qimage.rgbSwapped();
        result=GImage(qimage.height(),qimage.width(),
                      GImageType<uchar,3>::Type,qimage.bits(),true);// BGR
    }
    else if(qimage.format()==QImage::Format_Indexed8) // GRAY
    {
        result=GImage(qimage.height(),qimage.width(),
                      GImageType<uchar,1>::Type,qimage.bits(),true);
    }
    return result;
}

}

#else

namespace GSLAM {

inline GImage imread(const std::string& filepath,int flags=1)
{
    return GImage();
}

}
#endif
