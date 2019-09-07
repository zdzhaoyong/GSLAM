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
// Camera provide projection of 2D from(to) 3D coordinates

#ifndef GSLAM_CAMERA_H
#define GSLAM_CAMERA_H

#include <vector>
#include <sstream>
#include <memory>
#include "Point.h"

#ifdef __SSE3__
#include "pmmintrin.h"
#endif

namespace GSLAM {

class CameraImpl//No camera
{
public:
    CameraImpl(int _w=-1,int _h=-1):w(_w),h(_h){}
    ~CameraImpl(){}

    virtual std::string         CameraType()const{return "NoCamera";}
    virtual bool                isValid()const{return false;}
    virtual Point2d             Project(const Point3d& p3d)const{return Point2d(-1,-1);}
    virtual Point3d             UnProject(const Point2d& p2d)const{return Point3d(0,0,0);}
    virtual std::vector<double> getParameters()const{return std::vector<double>();}

    virtual bool                applyScale(double scale=0.5){return false;}
    virtual std::string         info()const;

    int32_t w,h;
    int32_t state,tmp;
};


/// Identity pinhole camera,fx=fy=1&&cx=cy=0
/// (x y)^T=(fx*X/Z+cx,fy*Y/Z+cy)^T
class CameraIdeal:public CameraImpl
{
public:
    CameraIdeal(int width,int height):CameraImpl(width,height){}

    virtual std::string         CameraType()const{return "Ideal";}
    virtual bool                isValid()const{return true;}
    virtual Point2d             Project(const Point3d& p3d)const{double z_inv=1./p3d.z;return Point2d(p3d.x*z_inv,p3d.y*z_inv);}
    virtual Point3d             UnProject(const Point2d& p2d)const{return Point3d(p2d.x,p2d.y,1.);}
    virtual std::vector<double> getParameters()const{return {(double)w,(double)h};}
};

/// Pinhole model
/// (x y)^T=(fx*X/Z+cx,fy*Y/Z+cy)^T
class CameraPinhole:public CameraImpl
{
public:
    CameraPinhole():fx(0),fy(0),cx(0),cy(0),fx_inv(0),fy_inv(0){}
    CameraPinhole(int _w,int _h,double _fx,double _fy,double _cx,double _cy)
        :CameraImpl(_w,_h),fx(_fx),fy(_fy),cx(_cx),cy(_cy),fx_inv(0),fy_inv(0)
    {refreshParaments();}

    virtual std::string         CameraType()const{return "PinHole";}
    virtual bool                isValid()const{return w>0&&h>0&&fx!=0&&fy!=0;}
    virtual Point2d             Project(const Point3d& p3d)const;
    virtual Point3d             UnProject(const Point2d& p2d)const;
    virtual std::vector<double> getParameters()const{return {(double)w,(double)h,fx,fy,cx,cy};}

    virtual bool                applyScale(double scale=0.5);

    double fx,fy,cx,cy,fx_inv,fy_inv;

private:
    int refreshParaments(){if(isValid()){fx_inv=1./fx;fy_inv=1./fy;
            return 0;}else return -1;}
};

/// Camera described in PTAM(alse called ANTA camera), measurement: \f$(x,y)^T\f$, z=1 plane: (X,Y)^T
/// Project:
/// |x|=|cx| + |fx 0| |X| *r'/r
/// |y| |cy|   |0 fy| |Y|
/// r =sqrt(X*X+Y*Y)
/// r'=atan(r * tan2w) / w
/// tan2w=2.0 * tan(w / 2.0)
///
/// UnProject:
/// |X|=(|x|-|cx|) * |1/fx 0|*r/r'
/// |Y|  |y| |cy|    |0 1/fy|
/// r' =sqrt(X*X+Y*Y)
/// r  =(tan(r' * w) / tan2w)
class CameraATAN:public CameraImpl
{
public:
    CameraATAN();
    CameraATAN(int _w,int _h,double _fx,double _fy,double _cx,double _cy,double _d);

    virtual std::string         CameraType()const{return "ATAN";}
    virtual bool                isValid()const{return w>0&&h>0&&fx!=0&&fy!=0;}
    virtual Point2d             Project(const Point3d& p3d)const;
    virtual Point3d             UnProject(const Point2d& p2d)const;
    virtual std::vector<double> getParameters()const{return {(double)w,(double)h,fx,fy,cx,cy,d};}

    virtual bool    applyScale(double scale=0.5);

    double fx,fy,cx,cy,d,fx_inv,fy_inv;
    double tan2w,d_inv,tan2w_inv;
private:
    virtual int  refreshParaments();
    bool& UseDistortion(){return useDistortion;}
    bool   useDistortion;
};
typedef CameraATAN CameraPTAM;
typedef CameraATAN CameraANTA;

/// Camera model used by OpenCV
/// Project:
/// r^2= X^2+Y^2;
/// X1= X*(1+k1*r^2 + k2*r^4+k3*r^6) + 2*p1*XY + p2*(r^2 + 2*X^2);
/// Y1= Y*(1+k1*r^2 + k2*r^4+k3*r^6) + 2*p2*XY + p1*(r^2 + 2*Y^2);
/// (x y)^T=(fx*X1+cx,fy*Y1+cy)^T
///
/// UnProject:
///
class CameraOpenCV:public CameraImpl
{
public:
    CameraOpenCV():fx(0),fy(0),cx(0),cy(0),fx_inv(0),fy_inv(0){}
    CameraOpenCV(int Width,int Height,
                 double Fx,double Fy,double Cx,double Cy,
                 double K1,double K2,double P1,double P2,double K3)
        :CameraImpl(Width,Height),fx(Fx),fy(Fy),cx(Cx),cy(Cy),
    k1(K1),k2(K2),p1(P1),p2(P2),k3(K3){refreshParaments();}

    virtual std::string         CameraType()const{return "OpenCV";}
    virtual bool                isValid()const{return w>0&&h>0&&fx!=0&&fy!=0;}
    virtual Point2d             Project(const Point3d& p3d)const;
    virtual Point3d             UnProject(const Point2d& p2d)const;
    virtual std::vector<double> getParameters()const{return {(double)w,(double)h,fx,fy,cx,cy,k1,k2,p1,p2,k3};}

    virtual bool        applyScale(double scale=0.5);
    double fx,fy,cx,cy,fx_inv,fy_inv,
    k1,k2,p1,p2,k3;
private:
    int refreshParaments();
};

class Camera
{
public:
    Camera(const std::shared_ptr<CameraImpl>& Impl=std::shared_ptr<CameraImpl>(new CameraImpl()));
    Camera(const std::vector<double>& paras);

    std::string CameraType()const{return impl->CameraType();}
    std::string info()const{return impl->info();}
    bool        isValid()const{return impl->isValid();}
    int         width()const{return impl->w;}
    int         height()const{return impl->h;}

    Point2d     Project(const Point3d& p3d)const{return impl->Project(p3d);}
    Point2d     Project(const double x, const double y, const double z)const { return Project(Point3d(x, y, z)); }

    Point3d     UnProject(const Point2d& p2d)const{return impl->UnProject(p2d);}
    Point3d     UnProject(const double x, const double y)const { return UnProject(Point2d(x, y)); }

    std::vector<double> getParameters()const{return impl->getParameters();}

    bool        applyScale(double scale=0.5){return impl->applyScale(scale);}
    Camera      estimatePinHoleCamera(void)const;

private:
    std::shared_ptr<CameraImpl> impl;
};

inline std::string CameraImpl::info()const{
    std::stringstream sst;
    sst<<CameraType()<<":[";
    std::vector<double> paras=getParameters();
    for(size_t i=0;i<paras.size();i++)
        sst<<paras[i]<<((i+1==paras.size())?"]":",");
    return sst.str();
}

inline bool CameraPinhole::applyScale(double scale) { w=(int32_t)(w*scale); h=(int32_t)(h*scale); fx*=scale;fy*=scale;cx*=scale;cy*=scale; refreshParaments(); return true; }

inline Point2d CameraPinhole::Project(const Point3d& p3d)const
{
#ifdef __SSE3__
    if(p3d.z==1.)
    {
        __m128d xy = _mm_setr_pd(p3d.x,p3d.y);
        xy=_mm_add_pd(_mm_setr_pd(cx,cy),_mm_mul_pd(xy,(__m128d){fx,fy}));
        return *(Point2d*)&xy;
    }
    else if(p3d.z>0)
    {
        double z_inv=1./p3d.z;
        return Point2d(fx*z_inv*p3d.x+cx,fy*z_inv*p3d.y+cy);
    }
    else return Point2d(-1,-1);
#else
    if(p3d.z==1.)
    {
        return Point2d(fx*p3d.x+cx,fy*p3d.y+cy);
    }
    else if(p3d.z>0)
    {
        double z_inv=1./p3d.z;
        return Point2d(fx*z_inv*p3d.x+cx,fy*z_inv*p3d.y+cy);
    }
    else return Point2d(-1,-1);
#endif
}

inline Point3d CameraPinhole::UnProject(const Point2d& p2d)const
{
    return Point3d((p2d.x-cx)*fx_inv,(p2d.y-cy)*fy_inv,1.);
}

inline CameraATAN::CameraATAN():fx(0),fy(0),cx(0),cy(0),d(0),fx_inv(0),fy_inv(0){}

inline CameraATAN::CameraATAN(int _w,int _h,double _fx,double _fy,double _cx,double _cy,double _d)
    :CameraImpl(_w,_h),fx(_fx),fy(_fy),cx(_cx),cy(_cy),d(_d),fx_inv(0),fy_inv(0)
{refreshParaments();}

inline int CameraATAN::refreshParaments()
{
    if(!isValid()) {
        std::cout<<"Camera not valid!Info:"<<info()<<std::endl;
        return -1;}
    if(fx<1&&fy<1)
    {
        fx*=w;fy*=h;cx*=w;cy*=h;
    }
    if(fx!=0) fx_inv=1./fx;else return -1;
    if(fy!=0) fy_inv=1./fy;else return -2;
    if(d!= 0.0)
    {
        tan2w = 2.0 * tan(d / 2.0);
        tan2w_inv=1.0/tan2w;
        d_inv = 1.0 / d;
        useDistortion = 1.0;
    }
    else
    {
        d_inv = 0.0;
        tan2w = 0.0;
        useDistortion = false;
    }
    return 0;
}

inline bool CameraATAN::applyScale(double scale)
{ w=(int32_t)(w*scale); h=(int32_t)(h*scale); fx*=scale; fy*=scale; cx*=scale; cy*=scale; refreshParaments(); return true;}

inline Point2d CameraATAN::Project(const Point3d& p3d)const
{
    if(p3d.z<=0) return Point2d(-1,-1);

#ifdef __SSE3__
    if(useDistortion)
    {
        __m128d xy=(__m128d){p3d.x,p3d.y};
        if(p3d.z!=1.)
        {
            xy=_mm_sub_pd(xy,(__m128d){p3d.z,p3d.z});
        }
        __m128d xy2=_mm_mul_pd(xy,xy);

         xy2=_mm_hadd_pd(xy2,xy2);
         xy2=_mm_sqrt_pd(xy2);
        double r=((Point2d*)&xy2)->x;
        if(r < 0.001 || d == 0.0)
            r=1.0;
        else
            r=(d_inv* atan(r * tan2w) / r);
        xy=_mm_mul_pd((__m128d){fx,fy},xy);
        xy=_mm_mul_pd(xy,(__m128d){r,r});
        xy=_mm_add_pd(xy,(__m128d){cx,cy});
        return *(Point2d*)&xy;
    }
    else
    {
        if(p3d.z==1.)
        {
            __m128d xy = _mm_setr_pd(p3d.x,p3d.y);
            xy=_mm_add_pd(_mm_setr_pd(cx,cy),_mm_mul_pd(xy,(__m128d){fx,fy}));
            return *(Point2d*)&xy;
        }
        else if(p3d.z>0)
        {
            double z_inv=1./p3d.z;
            return Point2d(fx*z_inv*p3d.x+cx,fy*z_inv*p3d.y+cy);
        }
    }
#else
    if(useDistortion)
    {
        double X=p3d.x,Y=p3d.y;
        if(p3d.z!=1.)
        {
            double z_inv=1./p3d.z;
            X*=z_inv;Y*=z_inv;
        }
        double r= sqrt(X*X+Y*Y);
        if(r < 0.001 || d == 0.0)
            r= 1.0;
        else
            r=(d_inv* atan(r * tan2w) / r);

        return Point2d(cx + fx * r * X,cy + fy * r * Y);
    }
    else
    {
        if(p3d.z==1.)
        {
            return Point2d(fx*p3d.x+cx,fy*p3d.y+cy);
        }
        else
        {
            double z_inv=1./p3d.z;
            return Point2d(fx*z_inv*p3d.x+cx,fy*z_inv*p3d.y+cy);
        }
    }
#endif
    return Point2d(-1,-1);// let compiler happy
}

inline Point3d CameraATAN::UnProject(const Point2d& p2d)const
{
    if(useDistortion)
    {
        Point3d result((p2d.x-cx)*fx_inv,(p2d.y-cy)*fy_inv,1.);
        double r = sqrt(result.x*result.x+result.y*result.y);

        if(d != 0.0&& r >0.01)
        {
            r = ( tan(r * d)*tan2w_inv )/r;//;
            result.x=result.x*r;
            result.y=result.y*r;
        }
        return result;
    }
    else return Point3d((p2d.x-cx)*fx_inv,(p2d.y-cy)*fy_inv,1.);
}

inline bool CameraOpenCV::applyScale(double scale){ w=(int32_t)(w*scale); h=(int32_t)(h*scale); fx*=scale;fy*=scale;cx*=scale;cy*=scale; refreshParaments(); return true;}

inline int CameraOpenCV::refreshParaments()
{
    if(!isValid()) {
        std::cout<<"Camera not valid!Info:"<<info()<<std::endl;
        return -1;}
    if(fx!=0) fx_inv=1./fx;else return -1;
    if(fy!=0) fy_inv=1./fy;else return -2;
    return 0;
}

inline Point2d CameraOpenCV::Project(const Point3d& p3d)const
{
    if(p3d.z<=0) return Point2d(-1,-1);
    double X=p3d.x,Y=p3d.y;
    if(p3d.z!=1.)
    {
        double z_inv=1./p3d.z;
        X*=z_inv;Y*=z_inv;
    }

    double r2,r4,r6,X1,Y1,X2,Y2,xy2;
    X2=X*X;
    Y2=Y*Y;
    r2= X2+Y2;
    r4=r2*r2;
    r6=r2*r4;
    xy2=X*Y*2.0;
    X1= X*(1.+k1*r2 + k2*r4+k3*r6) + xy2*p1 + p2*(r2 + 2.0*X2);
    Y1= Y*(1.+k1*r2 + k2*r4+k3*r6) + xy2*p2 + p1*(r2 + 2.0*Y2);

    return Point2d(cx + fx * X1,cy + fy * Y1);
}

inline Point3d CameraOpenCV::UnProject(const Point2d& p2d)const
{
    double x = (p2d.x - cx)*fx_inv;
    double y = (p2d.y - cy)*fy_inv;

    // compensate tilt distortion
    double x0 = x ;
    double y0 = y ;
    // compensate distortion iteratively
    for(int j = 0; j < 5; j++ )
    {
        double r2 = x*x + y*y;
        double icdist = (1)/(1 + ((k3*r2 + k2)*r2 + k1)*r2);
        double deltaX = 2*p1*x*y + p2*(r2 + 2*x*x);
        double deltaY = p1*(r2 + 2*y*y) + 2*p2*x*y;
        x = (x0 - deltaX)*icdist;
        y = (y0 - deltaY)*icdist;
    }
    return Point3d(x,y,1);
}


inline Camera::Camera(const std::shared_ptr<CameraImpl>& Impl):impl(Impl)
{
}

inline Camera::Camera(const std::vector<double> &p)
{
    CameraImpl* cam;
    if(p.size()==2) cam=new CameraIdeal((int)p[0],(int)p[1]);
    else if(p.size()==6) cam=new CameraPinhole((int)p[0],(int)p[1],p[2],p[3],p[4],p[5]);
    else if(p.size()==7) cam=new CameraATAN((int)p[0],(int)p[1],p[2],p[3],p[4],p[5],p[6]);
    else if(p.size()==11) cam=new CameraOpenCV((int)p[0],(int)p[1],p[2],p[3],p[4],p[5],p[6],p[7],p[8],p[9],p[10]);
    else cam=new CameraImpl();
    impl=std::shared_ptr<CameraImpl>(cam);
}

inline Camera Camera::estimatePinHoleCamera()const
{
    if( CameraType() == "PinHole" || CameraType() == "Ideal" ) {
        return *this;
    }

    int iw, ih;
    int ix, iy;

    iw = width();
    ih = height();

    double xl_max = -99999, xr_min = 99999, yt_max = -99999, yb_min = 99999;

    ix = 0;
    for(iy=0; iy<ih; iy++) {
        Point3d p = UnProject(ix, iy);
        if( p.x > xl_max ) xl_max = p.x;
    }

    ix = iw-1;
    for(iy=0; iy<ih; iy++) {
        Point3d p = UnProject(ix, iy);
        if( p.x < xr_min ) xr_min = p.x;
    }

    iy = 0;
    for(ix=0; ix<iw; ix++) {
        Point3d p = UnProject(ix, iy);
        if( p.y > yt_max ) yt_max = p.y;
    }

    iy = ih-1;
    for(ix=0; ix<iw; ix++) {
        Point3d p = UnProject(ix, iy);
        if( p.y < yb_min ) yb_min = p.y;
    }

    double xx, yy;
    double _fx, _fy, _cx, _cy;

    xx = fabs(xl_max); if( xx > fabs(xr_min) ) xx = fabs(xr_min);
    yy = fabs(yt_max); if( yy > fabs(yb_min) ) yy = fabs(yb_min);

    _cx = iw/2.0;
    _cy = ih/2.0;
    _fx = iw/2.0 / xx;
    _fy = ih/2.0 / yy;

    if( _fx < _fy ) _fx = _fy;//FIXME : why need this?
    else            _fy = _fx;

    CameraPinhole* camera = new CameraPinhole(iw, ih, _fx, _fy, _cx, _cy);
    std::shared_ptr<CameraImpl> impl_result = std::shared_ptr<CameraImpl>(camera);

    return Camera(impl_result);
}

}
#endif // CAMERA_H
