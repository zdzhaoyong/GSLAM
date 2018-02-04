#pragma once
#include "GImage.h"
#include "Camera.h"

#ifdef HAS_OPENMP
#include <omp.h>
#endif

//图像的像素直接提取
#define        _I(x,y) p_img[(c)*((int)(y)*(width_in)+(int)(x))]
//亚像素级灰度值
#define        _IF(x,y) (((int)(x+1)-(x))*((int)(y+1)-(y))*_I((image),(int)(x),(int)(y)) + ((int)(x+1)-(x))*((y)-(int)(y))*_I((image),(int)(x),(int)(y+1)) + ((x)-(int)(x))*((int)(y+1)-(y))*_I((image),(int)(x+1),(int)(y)) + ((x)-(int)(x))*((y)-(int)(y))*_I((image),(int)(x+1),(int)(y+1)) )//插值后的像素值(IN表示interpolation),x、y可以为小数


namespace GSLAM{

class UndistorterImpl
{
public:
    UndistorterImpl(Camera in, Camera out)
        : camera_in(in),camera_out(out),remapX(NULL),remapY(NULL),remapFast(NULL),
          remapIdx(NULL),remapCoef(NULL),valid(true)
    {
        prepareReMap();
    }

    ~UndistorterImpl()
    {
        if( remapX != NULL )        delete[] remapX;
        if( remapY != NULL )        delete[] remapY;
        if( remapFast != NULL )     delete[] remapFast;

        if( remapIdx != NULL )      delete[] remapIdx;
        if( remapCoef != NULL)      delete[] remapCoef;
    }

    bool undistort(const GImage& image, GImage& result);
    bool undistortFast(const GImage& image, GImage& result);

    bool prepareReMap();

public:
    Camera camera_in;
    Camera camera_out;

    float*  remapX;
    float*  remapY;
    int*    remapFast;

    int     *remapIdx;
    float   *remapCoef;

    /// Is true if the undistorter object is valid (has been initialized with
    /// a valid configuration)
    bool    valid;
};

class Undistorter
{
public:
    Undistorter(Camera in=Camera(), Camera out=Camera());

    bool undistort(const GImage& image, GImage& result);
    //Undistorting fast, no interpolate (bilinear) is used
    bool undistortFast(const GImage& image, GImage& result);

    Camera cameraIn();
    Camera cameraOut();

    bool prepareReMap();
    bool valid();
private:
    SPtr<UndistorterImpl> impl;
};


template <int Size>
struct Byte
{
    unsigned char data[Size];
};

typedef Byte<3> rgb;

inline bool UndistorterImpl::prepareReMap()
{
    using namespace std;
    // check camera model
    if( !(camera_in.isValid() && camera_out.isValid()) )
    {
        valid = false;
//        cout<<("Undistorter does not get vallid camera.");
        return false;
    }
    // Prepare remap
    cout << "Undistorter:\n";
    cout << "    Camera IN : " << camera_in.info() << endl;
    cout << "    Camera OUT: " << camera_out.info() << endl << endl;
    int size=camera_out.width() * camera_out.height();
    remapX    = new float[size];
    remapY    = new float[size];
    remapFast = new int[size];

    remapIdx  = new int  [size*4];
    remapCoef = new float[size*4];
    Point3d world_pose;Point2d im_pose;
    int w_in=camera_in.width(),h_in=camera_in.height();
    for(int y=0,yend=camera_out.height(); y<yend; y++)
        for(int x=0,xend=camera_out.width(); x<xend; x++)
        {
            int i = y*xend+x;

            world_pose=camera_out.UnProject(Point2d(x,y));
            im_pose = camera_in.Project(world_pose);

            if(im_pose.x<0 || im_pose.y<0 ||
                    im_pose.x>=w_in||im_pose.y>=h_in)
            {
                remapX[i]    = -1;
                remapY[i]    = -1;
                remapFast[i] = -1;

                remapIdx[i*4+0]  = 0;
                remapIdx[i*4+1]  = 0;
                remapIdx[i*4+2]  = 0;
                remapIdx[i*4+3]  = 0;

                remapCoef[i*4+0] = 0;
                remapCoef[i*4+1] = 0;
                remapCoef[i*4+2] = 0;
                remapCoef[i*4+3] = 0;
                continue;
            }

            {
                remapX[i]    = im_pose.x;
                remapY[i]    = im_pose.y;
                remapFast[i] = (int)im_pose.x+w_in*(int)im_pose.y;
            }

            // calculate fast bi-linear interpolation indices & coefficients
            {
                float xx = remapX[i];
                float yy = remapY[i];

                if( xx < 0.0 ) continue;

                // get integer and rational parts
                int xxi = xx;
                int yyi = yy;
                xx -= xxi;
                yy -= yyi;
                float xxyy = xx*yy;

                remapIdx[i*4+0]  = yyi*w_in + xxi;
                remapIdx[i*4+1]  = yyi*w_in + xxi + 1;
                remapIdx[i*4+2]  = (yyi+1)*w_in + xxi;
                remapIdx[i*4+3]  = (yyi+1)*w_in + xxi + 1;

                remapCoef[i*4+0] = 1-xx-yy+xxyy;
                remapCoef[i*4+1] = xx-xxyy;
                remapCoef[i*4+2] = yy-xxyy;
                remapCoef[i*4+3] = xxyy;
            }
        }
    valid=true;
    return true;
}

//Undistorting fast, no interpolate (bilinear) is used
inline bool UndistorterImpl::undistortFast(const GImage& image, GImage& result)
{
    using namespace std;
    if (!valid)
    {
        result = image;
        return false;
    }
    int width_out=camera_out.width();
    int height_out=camera_out.height();

    if (image.rows != camera_in.height() || image.cols != camera_in.width())
    {
        cerr<<("input image size differs from expected input size! Not undistorting.\n");
        result = image;
        return false;
    }

    int wh=width_out*height_out;
    int c=image.channels();

    result=GImage(height_out,width_out,image.type());

    if(c==1)
    {
        Byte<1>* p_out=(Byte<1>*)result.data;
        Byte<1>* p_img=(Byte<1>*)image.data;
//        #pragma omp parallel for
        for(int i=0;i<wh;i++)
        {
            if(remapFast[i]>0)
            {
               p_out[i] = p_img[remapFast[i]];
            }
        }
    }
    else if(c==3)
    {
        Byte<3>* p_out=(Byte<3>*)result.data;
        Byte<3>* p_img=(Byte<3>*)image.data;
//        #pragma omp parallel for
        for(int i=0;i<wh;i++)
        {
            p_out[i] = p_img[remapFast[i]];
        }
    }
    else
    {
        int eleSize=image.elemSize();
        Byte<1>* p_out=(Byte<1>*)result.data;
        Byte<1>* p_img=(Byte<1>*)image.data;
//        #pragma omp parallel for
        for(int i=0;i<wh;i++)
        {
            if(remapX[i]>0)
            {
                memcpy(p_out+eleSize*i,p_img+eleSize*remapFast[i],eleSize);
            }
        }
    }

    return true;
}

//Undistorting bilinear interpolation
inline bool UndistorterImpl::undistort(const GImage& image, GImage& result)
{
    using namespace std;
    if (!valid)
    {
        result = image;
        return false;
    }
    int width_out=camera_out.width();
    int height_out=camera_out.height();

    if (image.rows != camera_in.height() || image.cols != camera_in.width())
    {
        cerr<<("input image size differs from expected input size! Not undistorting.\n");
        result = image;
        return false;
    }

    int wh=width_out*height_out;
    int c=image.channels();

    result=GImage(height_out,width_out,image.type());

    if(c==1)
    {
        uchar* p_out=(uchar*)result.data;
        uchar* p_img=(uchar*)image.data;

        int   *pIdx  = remapIdx;
        float *pCoef = remapCoef;

//        #pragma omp parallel for
        for(int i = 0; i<wh; i++)
        {
            // get interp. values
            float xx = remapX[i];

            if(xx<0)
                p_out[i] = 0;
            else
            {
                p_out[i] = p_img[pIdx[0]]*pCoef[0] +
                           p_img[pIdx[1]]*pCoef[1] +
                           p_img[pIdx[2]]*pCoef[2] +
                           p_img[pIdx[3]]*pCoef[3];
            }

            pIdx  += 4;
            pCoef += 4;
        }
    }
    else
    {
        uchar* p_out = (uchar*)result.data;
        uchar* p_img = (uchar*)image.data;

        int   *pIdx  = remapIdx;
        float *pCoef = remapCoef;

//        #pragma omp parallel for
        for(int i=0; i<wh; i++)
        {
            if(remapX[i]>0)
            {
                for(int j=0; j<c; j++)
                    p_out[i*3+j] = p_img[pIdx[0]*c+j]*pCoef[0] +
                                   p_img[pIdx[1]*c+j]*pCoef[1] +
                                   p_img[pIdx[2]*c+j]*pCoef[2] +
                                   p_img[pIdx[3]*c+j]*pCoef[3];
            }

            pIdx  += 4;
            pCoef += 4;
        }
    }

    return true;
}

inline Undistorter::Undistorter(Camera in, Camera out)
    :impl(new UndistorterImpl(in,out))
{
}

inline bool Undistorter::undistort(const GImage& image, GImage& result)
{
    return impl->undistort(image,result);
}

inline bool Undistorter::undistortFast(const GImage& image, GImage& result)
{
    return impl->undistortFast(image,result);
}

inline Camera Undistorter::cameraIn(){return impl->camera_in;}
inline Camera Undistorter::cameraOut(){return impl->camera_out;}

inline bool Undistorter::prepareReMap()
{
    return impl->prepareReMap();
}

inline bool Undistorter::valid()
{
    return impl->valid;
}

}
