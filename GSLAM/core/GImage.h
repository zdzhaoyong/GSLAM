#ifndef GIMAGE
#define GIMAGE
#include <stdint.h>
#include <string.h>

#if defined(HAS_OPENCV) || defined(HAS_OPENCV3)
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

struct UMatData// for OpenCV Version 3
{
    enum { COPY_ON_MAP=1, HOST_COPY_OBSOLETE=2,
        DEVICE_COPY_OBSOLETE=4, TEMP_UMAT=8, TEMP_COPIED_UMAT=24,
        USER_ALLOCATED=32, DEVICE_MEM_MAPPED=64};
    const void* prevAllocator;
    const void* currAllocator;
    int urefcount;
    int refcount;
    uchar* data;
    uchar* origdata;
    size_t size;

    int flags;
    void* handle;
    void* userdata;
    int allocatorFlags_;
    int mapcount;
    UMatData* originalUMatData;
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
        if(!data)
        {
            cols=0;rows=0;return ;
        }
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
                release();
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


#if defined(HAS_OPENCV) || defined(HAS_OPENCV3)
#if CV_VERSION_EPOCH == 2
    inline operator cv::Mat()const
    {
        if(empty()) return cv::Mat();
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
#elif CV_VERSION_MAJOR == 3
    inline operator cv::Mat()const
    {
        if(!data) return cv::Mat();
        cv::Mat result(rows,cols,type(),data);
        if(((uchar*)refCount)==data+total()*elemSize())// OpenCV2 style => OpenCV3 style
        {
            // WARNING: MAKE SURE THERE ARE NO OTHER HOLDERS
            // construct a UMat that ref to data
            cv::UMatData* u=new cv::UMatData;
            u.origdata=u.data=data;
            u.userdata=refCount;
            (*refCount)++;
            u.refCount=2;
            refCount=&u.refCount;
            return result;
        }
        else // OpenCV3 style => OpenCV3 style
        {
            (*refCount)++;
            cv::UMatData* u=(cv::UMatData*)(((uchar*)refCount)-sizeof(int)-sizeof(cv::MatAllocator*)*2);
            result.u=u;
            return result;
        }
        return result;// no copy but not safe either
    }

    GImage(const cv::Mat& mat)
        : cols(mat.cols),rows(mat.rows),flags(mat.type()),
          data(mat.data),refCount(NULL)
    {
        if(mat.u&&!mat.allocator)
            // try to maintain the refcount things, but this need the mat is allocated by default StdAllocator
        {
            refCount=(&mat.u->refcount);
            (*refCount)++;
        }
        else if(0)// copy the data: SAFE but SLOW
        {
            int byteNum=total()*elemSize();
            data=(uchar*)fastMalloc(byteNum+sizeof(int*));
            if(!data)
            {
                cols=0;rows=0;return ;
            }
            refCount=(int*)(data+byteNum);
            *refCount=1;
            if(mat.data)
                memcpy(data,mat.data,byteNum);
        }
    }
#endif

#endif

    void release()
    {
        int totalBytes=total()*elemSize();
        if(((uchar*)refCount)==data+totalBytes) // OpenCV2 style
        {
            cols=rows=0;
            refCount=NULL;
            fastFree(data);
            data=NULL;
        }
        else// OpenCV3 style
        {
            cols=rows=0;
#if false// use opencv's default deallocate
            cv::UMatData* u=(cv::UMatData*)(((uchar*)refCount)-sizeof(int)-sizeof(void*)*2);
            u->refcount--;
            if(u)
                (u->currAllocator ? u->currAllocator : cv::Mat::getDefaultAllocator())->unmap(u);
#else // use buildin deallocate
            GSLAM::UMatData* u=(GSLAM::UMatData*)(((uchar*)refCount)-sizeof(int)-sizeof(void*)*2);
            if((*refCount)==1&&u->userdata==data+totalBytes)// this is allocated by GImage
            {
                (*((int*)u->userdata))--;
            }
            assert(u->size==totalBytes);
            assert(u->currAllocator==NULL);
            u->refcount--;
            deallocate(u);
#endif
            refCount=NULL;
            data=NULL;
        }
    }
private:

    template<typename _Tp> static inline _Tp* alignPtr(_Tp* ptr, int n=(int)sizeof(_Tp))
    {
        return (_Tp*)(((size_t)ptr + n-1) & -n);
    }

    void* fastMalloc( size_t size ) const
    {
        uchar* udata = (uchar*)malloc(size + sizeof(void*) + 16);
        if(!udata)
            return NULL;
        uchar** adata = alignPtr((uchar**)udata + 1, 16);
        adata[-1] = udata;
        return adata;
    }

    void fastFree(void* ptr) const
    {
        if(ptr)
        {
            uchar* udata = ((uchar**)ptr)[-1];
            free(udata);
        }
    }

    void deallocate(GSLAM::UMatData* u) const// for OpenCV Version 3
    {
        if(!u)
            return;

        assert(u->urefcount == 0);
        assert(u->refcount == 0);
        if( !(u->flags & GSLAM::UMatData::USER_ALLOCATED) )
        {
            fastFree(u->origdata);
            u->origdata = 0;
        }
        delete u;
    }

public:
    int  cols,rows,flags;
    uchar*          data;
    mutable int*    refCount;
};

}
#endif // GIMAGE

