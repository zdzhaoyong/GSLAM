// GSLAM - A general SLAM framework and benchmark
// Copyright 2018 PILAB Inc. All rights reserved.
// https://github.com/zdzhaoyong/GSLAM
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
// * Neither the name of Google Inc. nor the names of its contributors may be
//   used to endorse or promote products derived from this software without
//   specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// Author: zd5945@126.com (Yong Zhao)
//
// Undistorter is used to remap a image between camera models
#ifndef GSLAM_UNDISTORTER_H
#define GSLAM_UNDISTORTER_H

#include <memory>
#include "GImage.h"
#include "Camera.h"

namespace GSLAM{

class UndistorterImpl
{
public:
    template <int Size>
    struct Byte
    {
        unsigned char data[Size];
    };

    typedef Byte<3> rgb;

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

    GImage undistort(const GImage& image){
        GImage result;
        if(!undistort(image,result)) return GImage();
        return result;
    }

    GImage undistortFast(const GImage& image){
        GImage result;
        if(!undistortFast(image,result)) return GImage();
        return result;
    }

    Camera cameraIn();
    Camera cameraOut();

    bool prepareReMap();
    bool valid();
private:
    std::shared_ptr<UndistorterImpl> impl;
};

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
#endif
