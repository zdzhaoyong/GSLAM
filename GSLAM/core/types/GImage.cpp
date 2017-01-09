#include "GImage.h"
#include "string.h"

namespace GSLAM{


GImage::GImage()
    :cols(0),rows(0),flags(0),data(NULL),refCount(NULL),notRelease(false)
{

}

GImage::GImage(int width,int height,int type,uchar* src,bool copy)
    :cols(width),rows(height),flags(type),data(NULL),refCount(NULL),notRelease(false)
{
    if(src)
    {
        if(copy)
        {
            int byteNum=total()*elemSize();
            data=new uchar[byteNum];
            memcpy(data,src,byteNum);
        }
        else data=src;
    }
    else
    {
        int byteNum=total()*elemSize();
        data=new uchar[byteNum];
    }
    refCount=new int(1);
}

GImage::~GImage()
{
    if(data)
    {
        if((*refCount)==1)
        {
            if(!notRelease)
            {
                delete data;
                data=NULL;
            }
            delete refCount;
            cols=rows=0;
            refCount=NULL;
        }
        else (*refCount)--;
    }
}

uchar* GImage::getDataCopy(bool deepCopy)
{
    if(data&&!notRelease)
    {
        if(deepCopy)
        {
            int byteNum=total()*elemSize();
            uchar* resultData=new uchar[byteNum];
            memcpy(resultData,data,byteNum);
            return resultData;
        }
        else
        {
            notRelease=true;
            return data;
        }
    }
    else return nullptr;
}

GImage::GImage(const GImage& ref)
    :cols(ref.cols),rows(ref.rows),flags(ref.flags),
      data(ref.data),refCount(ref.refCount),notRelease(ref.notRelease)
{
    if(refCount)
        (*refCount)++;
}

GImage GImage::clone()
{
    return GImage(cols,rows,flags,data,true);
}

GImage& GImage::operator=(const GImage& rhs)
{
    this->~GImage();
    cols=rhs.cols;
    rows=rhs.rows;
    flags=rhs.flags;
    data=rhs.data;
    refCount=rhs.refCount;
    notRelease=rhs.notRelease;
    if(refCount) (*refCount)++;
    return *this;
}

}
