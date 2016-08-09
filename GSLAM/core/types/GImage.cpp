#include "GImage.h"
#include "string.h"

namespace GSLAM{


GImage::GImage()
    :cols(0),rows(0),flags(0),data(NULL),refCount(NULL)
{

}

GImage::GImage(int width,int height,int type,uchar* src)
    :cols(width),rows(height),flags(type),data(NULL),refCount(NULL)
{
    int byteNum=total()*elemSize();
    data=new uchar[byteNum];
    if(src)
    {
        memcpy(data,src,byteNum);
    }
    refCount=new int(1);
}

GImage::~GImage()
{
    if(data)
    {
        if((*refCount)==1)
            delete data;
        else (*refCount)--;
    }
}

GImage::GImage(const GImage& ref)
    :cols(ref.cols),rows(ref.rows),flags(ref.flags),
      data(ref.data),refCount(ref.refCount)
{
    if(refCount)
        (*refCount)++;
}

GImage GImage::clone()
{
    return GImage(cols,rows,flags,data);
}

}
