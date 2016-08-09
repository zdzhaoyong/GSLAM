#ifndef GIMAGE_H
#define GIMAGE_H
#include <GSLAM/core/GSLAM.h>

namespace GSLAM{

enum GElementType{
    GElementType_8U =0,
    GElementType_8S =1,
    GElementType_16U=2,
    GElementType_16S=3,
    GElementType_32S=4,
    GElementType_32F=5,
    GElementType_64F=6,
    GElementType_UserType=7
};

template <typename C>
class GElement
{
public:
    enum{Type=GElementType_UserType};
};

template <>
class GElement<uchar>
{
public:
    enum{Type=GElementType_8U};
};

template <>
class GElement<char>
{
public:
    enum{Type=GElementType_8S};
};

template <>
class GElement<pi::Int16>
{
public:
    enum{Type=GElementType_16S};
};

template <>
class GElement<pi::UInt16>
{
public:
    enum{Type=GElementType_16U};
};

template <>
class GElement<pi::Int32>
{
public:
    enum{Type=GElementType_32S};
};

template <>
class GElement<float>
{
public:
    enum{Type=GElementType_32F};
};

template <>
class GElement<double>
{
public:
    enum{Type=GElementType_64F};
};

template <typename EleType=uchar,int channelSize=1>
struct GImageType
{
    enum{Type=((GElement<EleType>::Type&0x7)+((channelSize-1)<<3))};
};

/**
 * @brief The GImage class is a tiny implementation of image for removing dependency of opencv.
 * Most apis are corrosponding to "cv::Mat".
 */
class GImage
{
public:
    GImage();
    GImage(int width,int height,int type=GImageType<>::Type,uchar* src=NULL);
    GImage(const GImage& ref);
    ~GImage();

    bool empty()const{return !data;}
    int  elemSize()const{return channels()*elemSize1();}
    int  elemSize1()const{return (1<<((type()&0x7)>>1));}

    int  channels()const{return (type()>>3)+1;}
    int  type()const{return flags;}
    int  total()const{return cols*rows;}

    GImage clone();

    template <typename C>
    C& at(int idx){return ((C*)data)[idx];}

    template <typename C>
    C& at(int ix,int iy){return ((C*)data)[iy*cols+ix];}

    int  cols,rows,flags;
    uchar*          data;
    int*            refCount;
};

}
#endif // GIMAGE_H
