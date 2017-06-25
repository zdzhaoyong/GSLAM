#include <thread>

#include "../../core/Timer.h"
#include "../../core/GImage.h"
#include "gtest.h"
using namespace std;

#ifdef HAS_OPENCV
#include <opencv2/highgui/highgui.hpp>
void testGImageType(int type,int cols=1000,int rows=2000)
{
    cv::Mat mat(rows,cols,type);
    timer.enter("GImagefromMat");
    GSLAM::GImage  img=mat;
    EXPECT_TRUE(timer.leave("GImagefromMat")<1e-3);// Should no copy

    EXPECT_TRUE(img.cols==mat.cols);
    EXPECT_TRUE(img.rows==mat.rows);
    EXPECT_TRUE(img.empty()==mat.empty());
    EXPECT_TRUE(img.type()==mat.type());
    EXPECT_TRUE(img.channels()==mat.channels());

    EXPECT_TRUE(img.elemSize()==mat.elemSize());
    EXPECT_TRUE(img.elemSize1()==mat.elemSize1());
    EXPECT_TRUE(img.total()==mat.total());

    EXPECT_TRUE(memcmp(img.data,mat.data,img.total()*img.elemSize())==0);

    timer.enter("GImage2Mat");
    cv::Mat mat1=(img);
    EXPECT_TRUE(timer.leave("GImage2Mat")<1e-3);// Should no copy

    GSLAM::GImage img1=img.clone();

    EXPECT_TRUE(img1.cols==mat1.cols);
    EXPECT_TRUE(img1.rows==mat1.rows);
    EXPECT_TRUE(img1.empty()==mat1.empty());
    EXPECT_TRUE(img1.type()==mat1.type());
    EXPECT_TRUE(img1.channels()==mat1.channels());
    EXPECT_TRUE(img1.elemSize()==mat1.elemSize());
    EXPECT_TRUE(img1.total()==mat1.total());
    EXPECT_TRUE(memcmp(img1.data,mat1.data,img1.total()*img1.elemSize())==0);

    img=img1=img1.clone();
}

TEST(GImageTest,CheckGImageType)
{
    EXPECT_TRUE((CV_8UC1==GSLAM::GImageType<uchar,1>::Type));
    EXPECT_TRUE((CV_8SC1==GSLAM::GImageType<char,1>::Type));
    EXPECT_TRUE((CV_16UC1==GSLAM::GImageType<uint16_t,1>::Type));
    EXPECT_TRUE((CV_16SC1==GSLAM::GImageType<int16_t,1>::Type));
    EXPECT_TRUE((CV_32SC1==GSLAM::GImageType<int32_t,1>::Type));
    EXPECT_TRUE((CV_32FC1==GSLAM::GImageType<float,1>::Type));
    EXPECT_TRUE((CV_64FC1==GSLAM::GImageType<double,1>::Type));

    EXPECT_TRUE((CV_8UC2==GSLAM::GImageType<uchar,2>::Type));
    EXPECT_TRUE((CV_8SC2==GSLAM::GImageType<char,2>::Type));
    EXPECT_TRUE((CV_16UC2==GSLAM::GImageType<uint16_t,2>::Type));
    EXPECT_TRUE((CV_16SC2==GSLAM::GImageType<int16_t,2>::Type));
    EXPECT_TRUE((CV_32SC2==GSLAM::GImageType<int32_t,2>::Type));
    EXPECT_TRUE((CV_32FC2==GSLAM::GImageType<float,2>::Type));
    EXPECT_TRUE((CV_64FC2==GSLAM::GImageType<double,2>::Type));
}

TEST(GImageTest,OpenCVNoCopy)
{
    testGImageType(CV_8UC1);
    testGImageType(CV_8SC1);
    testGImageType(CV_16UC1);
    testGImageType(CV_16SC1);
    testGImageType(CV_32SC1);
    testGImageType(CV_32FC1);
    testGImageType(CV_64FC1);

    testGImageType(CV_8UC3);
    testGImageType(CV_8SC3);
    testGImageType(CV_16UC3);
    testGImageType(CV_16SC3);
    testGImageType(CV_32SC3);
    testGImageType(CV_32FC3);
    testGImageType(CV_64FC3);
}

#endif
