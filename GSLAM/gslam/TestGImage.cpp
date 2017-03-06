#include <base/Utils/TestCase.h>
#include <base/Time/Global_Timer.h>
#include <GSLAM/core/types/GImage.h>
#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace pi;

class GImageTest : public TestCase
{
public:
    GImageTest():TestCase("GImageTest"){}

    void testType(int type,int cols=1000,int rows=2000)
    {
//        cout<<"Testing type "<<type<<endl;
        cv::Mat mat(rows,cols,type);
        pi::timer.enter("GImagefromMat");
        GSLAM::GImage  img=(mat);
        pi::timer.leave("GImagefromMat");
        pi_assert(img.cols==mat.cols);
        pi_assert(img.rows==mat.rows);
        pi_assert(img.empty()==mat.empty());
        pi_assert(img.type()==mat.type());
        pi_assert(img.channels()==mat.channels());
//        cout<<"imgElemSize:"<<img.elemSize()<<",mat.elemSize:"<<mat.elemSize()<<endl;
        pi_assert(img.elemSize()==mat.elemSize());
        pi_assert(img.elemSize1()==mat.elemSize1());
//        cout<<"img.total():"<<img.total()<<",mat.total:"<<mat.total()<<endl;
        pi_assert(img.total()==mat.total());

        pi_assert(memcmp(img.data,mat.data,img.total()*img.elemSize())==0);

        pi::timer.enter("GImage2Mat");
        cv::Mat mat1=(img);
        pi::timer.leave("GImage2Mat");

        GSLAM::GImage img1=img.clone();

        pi_assert(img1.cols==mat1.cols);
        pi_assert(img1.rows==mat1.rows);
        pi_assert(img1.empty()==mat1.empty());
        pi_assert(img1.type()==mat1.type());
        pi_assert(img1.channels()==mat1.channels());
        pi_assert(img1.elemSize()==mat1.elemSize());
        pi_assert(img1.total()==mat1.total());
        pi_assert(memcmp(img1.data,mat1.data,img1.total()*img1.elemSize())==0);

        img=img1=img1.clone();
//        cout<<"type:"<<type<<"passed\n";
    }

    virtual void run()
    {
        // check type corrosponding
        pi_assert((CV_8UC1==GSLAM::GImageType<uchar,1>::Type));
        pi_assert((CV_8SC1==GSLAM::GImageType<char,1>::Type));
        pi_assert((CV_16UC1==GSLAM::GImageType<pi::UInt16,1>::Type));
        pi_assert((CV_16SC1==GSLAM::GImageType<pi::Int16,1>::Type));
        pi_assert((CV_32SC1==GSLAM::GImageType<pi::Int32,1>::Type));
        pi_assert((CV_32FC1==GSLAM::GImageType<float,1>::Type));
        pi_assert((CV_64FC1==GSLAM::GImageType<double,1>::Type));

        pi_assert((CV_8UC2==GSLAM::GImageType<uchar,2>::Type));
        pi_assert((CV_8SC2==GSLAM::GImageType<char,2>::Type));
        pi_assert((CV_16UC2==GSLAM::GImageType<pi::UInt16,2>::Type));
        pi_assert((CV_16SC2==GSLAM::GImageType<pi::Int16,2>::Type));
        pi_assert((CV_32SC2==GSLAM::GImageType<pi::Int32,2>::Type));
        pi_assert((CV_32FC2==GSLAM::GImageType<float,2>::Type));
        pi_assert((CV_64FC2==GSLAM::GImageType<double,2>::Type));

        testType(CV_8UC1);
        testType(CV_8SC1);
        testType(CV_16UC1);
        testType(CV_16SC1);
        testType(CV_32SC1);
        testType(CV_32FC1);
        testType(CV_64FC1);

        testType(CV_8UC3);
        testType(CV_8SC3);
        testType(CV_16UC3);
        testType(CV_16SC3);
        testType(CV_32SC3);
        testType(CV_32FC3);
        testType(CV_64FC3);
    }
};

GImageTest gimageTest;
