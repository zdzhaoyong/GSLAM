#ifndef GIMAGE
#define GIMAGE
#include <stdint.h>
#include <string.h>

#ifdef HAS_OPENCV
#include <opencv2/core/core.hpp>
#endif

namespace GSLAM{

typedef uint8_t uchar;

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
class GElement<uint8_t>
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
class GElement<int16_t>
{
public:
    enum{Type=GElementType_16S};
};

template <>
class GElement<uint16_t>
{
public:
    enum{Type=GElementType_16U};
};

template <>
class GElement<int32_t>
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

template <typename EleType=uint8_t,int channelSize=1>
struct GImageType
{
    enum{Type=((GElement<EleType>::Type&0x7)+((channelSize-1)<<3))};
};

/**
 * @brief The GImage class is a tiny implementation of image for removing dependency of opencv.
 * Most APIs are corrosponding to "cv::Mat".
 */
class GImage
{
public:
    GImage()
        :cols(0),rows(0),flags(0),data(NULL),refCount(NULL)
    {

    }

    GImage(int width,int height,int type=GImageType<>::Type,uchar* src=NULL,bool copy=true)
        :cols(width),rows(height),flags(type),data(NULL),refCount(NULL)
    {
        if(data&&!copy)
        {
            data=src;
            return;
        }

        int byteNum=total()*elemSize();
        data=(uchar*)fastMalloc(byteNum+sizeof(int*));
        refCount=(int*)(data+byteNum);
        *refCount=1;
        if(src)
            memcpy(data,src,byteNum);
    }

    GImage(const GImage& ref)
        : cols(ref.cols),rows(ref.rows),flags(ref.flags),
          data(ref.data),refCount(ref.refCount)
    {
        if(refCount)
            (*refCount)++;
    }

    ~GImage()
    {
        if(data&&refCount)
        {
            if((*refCount)==1)
            {
                cols=rows=0;
                refCount=NULL;
                fastFree(data);
                data=NULL;
            }
            else (*refCount)--;
        }
    }

    GImage& operator=(const GImage& rhs)
    {
        this->~GImage();
        cols=rhs.cols;
        rows=rhs.rows;
        flags=rhs.flags;
        data=rhs.data;
        refCount=rhs.refCount;
        if(refCount) (*refCount)++;
        return *this;
    }

#ifdef HAS_OPENCV
    operator cv::Mat()const
    {
        cv::Mat result(rows,cols,type(),data);
        result.refcount=refCount;
        (*refCount)++;
        return result;
    }

    GImage(const cv::Mat& mat)
        : cols(mat.cols),rows(mat.rows),flags(mat.type()),
          data(mat.data),refCount(mat.refcount)
    {
        if(refCount) (*refCount)++;
    }
#endif

    bool empty()const{return !data;}
    int  elemSize()const{return channels()*elemSize1();}
    int  elemSize1()const{return (1<<((type()&0x7)>>1));}

    int  channels()const{return (type()>>3)+1;}
    int  type()const{return flags;}
    int  total()const{return cols*rows;}

    GImage clone()
    {
        return GImage(cols,rows,flags,data,true);
    }

    template <typename C>
    C& at(int idx){return ((C*)data)[idx];}

    template <typename C>
    C& at(int ix,int iy){return ((C*)data)[iy*cols+ix];}

private:

    template<typename _Tp> static inline _Tp* alignPtr(_Tp* ptr, int n=(int)sizeof(_Tp))
    {
        return (_Tp*)(((size_t)ptr + n-1) & -n);
    }

    void* fastMalloc( size_t size )
    {
            uchar* udata = (uchar*)malloc(size + sizeof(void*) + 16);
            if(!udata)
                    return NULL;
            uchar** adata = alignPtr((uchar**)udata + 1, 16);
            adata[-1] = udata;
            return adata;
    }

    void fastFree(void* ptr)
    {
            if(ptr)
            {
                    uchar* udata = ((uchar**)ptr)[-1];
                    free(udata);
            }
    }
public:
    int  cols,rows,flags;
    uchar*          data;
    mutable int*    refCount;
};

}
#endif // GIMAGE

