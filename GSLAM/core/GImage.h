#ifndef GIMAGE
#define GIMAGE
#include <stdint.h>
#include <string.h>
#include <assert.h>
#include <atomic>

#if defined(HAS_OPENCV) || defined(HAS_OPENCV3)
#include <opencv2/core/core.hpp>
#else
typedef unsigned char uchar;
#endif

#ifndef CV_XADD

/////// exchange-add operation for atomic operations on reference counters ///////
#if defined __INTEL_COMPILER && !(defined WIN32 || defined _WIN32)   // atomic increment on the linux version of the Intel(tm) compiler
  #define CV_XADD(addr,delta) _InterlockedExchangeAdd(const_cast<void*>(reinterpret_cast<volatile void*>(addr)), delta)
#elif defined __GNUC__

  #if defined __clang__ && __clang_major__ >= 3 && !defined __ANDROID__ && !defined __EMSCRIPTEN__
    #ifdef __ATOMIC_SEQ_CST
        #define CV_XADD(addr, delta) __c11_atomic_fetch_add((_Atomic(int)*)(addr), (delta), __ATOMIC_SEQ_CST)
    #else
        #define CV_XADD(addr, delta) __atomic_fetch_add((_Atomic(int)*)(addr), (delta), 5)
    #endif
  #elif __GNUC__*10 + __GNUC_MINOR__ >= 42

    #if !(defined WIN32 || defined _WIN32) && (defined __i486__ || defined __i586__ || \
        defined __i686__ || defined __MMX__ || defined __SSE__  || defined __ppc__) || \
        (defined __GNUC__ && defined _STLPORT_MAJOR) || \
        defined __EMSCRIPTEN__

      #define CV_XADD __sync_fetch_and_add
    #else
      #include <ext/atomicity.h>
      #define CV_XADD __gnu_cxx::__exchange_and_add
    #endif

  #else
    #include <bits/atomicity.h>
    #if __GNUC__*10 + __GNUC_MINOR__ >= 34
      #define CV_XADD __gnu_cxx::__exchange_and_add
    #else
      #define CV_XADD __exchange_and_add
    #endif
  #endif

#else
  static inline int CV_XADD(int* addr, int delta)
  { int tmp = *addr; *addr += delta; return tmp; }
#endif

#endif

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

    GImage(int rows_,int cols_,int type=GImageType<>::Type,uchar* src=NULL,bool copy=false)
        :cols(cols_),rows(rows_),flags(type),data(NULL),refCount(NULL)
    {
        if(src&&!copy)
        {
            data=src;
            return;
        }

        int byteNum=total()*elemSize();
        if(byteNum<=0) return;
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
            CV_XADD(refCount,1);
    }

    ~GImage()
    {
        if(data&&refCount)
        {
            if(CV_XADD(refCount,-1)==1)
            {
                release();
            }
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
        if(refCount) CV_XADD(refCount,1);
        return *this;
    }

    static GImage create(int rows,int cols,int type=GImageType<>::Type,uchar* src=NULL,bool copy=false)
    {
        return GImage(rows,cols,type,src,copy);
    }

    static GImage zeros(int rows,int cols,int type=GImageType<>::Type,uchar* src=NULL,bool copy=false)
    {
        GImage result(rows,cols,type,src,copy);
        memset(result.data,0,result.total()*result.elemSize());
        return result;
    }

    bool empty()const{return !data;}
    int  elemSize()const{return channels()*elemSize1();}
    int  elemSize1()const{return (1<<((type()&0x7)>>1));}

    int  channels()const{return (type()>>3)+1;}
    int  type()const{return flags;}
    int  total()const{return cols*rows;}

    GImage clone()const
    {
        return GImage(rows,cols,flags,data,true);
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
        if(refCount)
        {
            result.refcount=refCount;
            CV_XADD(refCount,1);
        }
        return result;
    }
    GImage(const cv::Mat& mat)
        : cols(mat.cols),rows(mat.rows),flags(mat.type()),
          data(mat.data),refCount(mat.refcount)
    {
        if(refCount) CV_XADD(refCount,1);
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
            u->origdata=u->data=data;
            u->userdata=refCount;
            CV_XADD(refCount,1);
            u->refCount=2;
            refCount=&u->refCount;
            result.u=u;
            return result;
        }
        else // OpenCV3 style => OpenCV3 style
        {
            CV_XADD(refCount,1);
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
            CV_XADD(refCount,1);
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

    template<typename _Tp> _Tp* ptr(int i0=0){return (_Tp*)(data+i0*cols*elemSize());}

    template<typename _Tp> const _Tp* ptr(int i0=0) const{return (_Tp*)(data+i0*cols*elemSize());}

    const GImage row(int idx=0)const{return GImage(1,cols,type(),data+elemSize()*cols*idx);}

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

