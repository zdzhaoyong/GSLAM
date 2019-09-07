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

#ifndef GSLAM_SO3_H
#define GSLAM_SO3_H

#include <math.h>
#include <iostream>
#include "Point.h"
#include "Matrix.h"


#ifndef M_PI
# define M_PI		3.14159265358979323846	/* pi */
#endif
#define NEAR_ZERO 1e-10

namespace GSLAM {


/**
 @brief 3D rotation represented by the quaternion.

 \section sSO3_intro Introduction

 For the rotational component, there are several choices for representation, including the matrix, Euler angle, unit quaternion and Lie algebra \f$so(3)\f$.
 For a given transformation, we can use any of these for representation and can convert one to another.:
 - An unit orthogonal 3x3 matrix \f$ \mathbf{R} \f$.
 - The Euler angle representation uses 3 variables such as yaw, roll, pitch.
 - Quaternion \f$q(x,y,z,w)\f$: the most efficient way to perform multiple.
 - Lie algebra \f$[a,b,c]\f$: the common representation to perform manifold optimization.

 This implementation use Quaternion for computation.
 Class SO3 use 4 paraments to present a 3 dimesion rotation matrix,
 since 3D rotation matrices are members of the Special Orthogonal Lie group SO3.

 Every rotation in the 3D euclidean space can be represented by a rotation with one direction.
 Consider a rotation with direction \f$(a,b,c)^T\f$ and angle theta in radians.
 - w -- \f$cos(theta/2)\f$
 - x -- \f$a*sin(theta/2)\f$
 - y -- \f$b*sin(theta/2)\f$
 - z -- \f$c*sin(theta/2)\f$

 this ensures that \f$x^2+y^2+z^2+w^2=1\f$.
 A quaternion \f$q(x,y,z,w)\f$ is used to present a 3d rotation.

 \section Constructors

 Users can construct a SO3 from different data structure:
 @code
 SO3 I;// default, idendity
 SO3 R1(x,y,z,w);// from quaternion
 SO3 R2(Poin3d(rx,ry,rz),angle)// from axis and angle
 SO3 R3=SO3::exp(Point3d(rx,ry,rz));// from lie algebra
 SO3 R4=SO3(Matrix3d(m));// from matrix
 SO3 R4=SO3(m);// from pointer
 SO3 R5=SO3::fromPitchYawRoll(pitch,yaw,roll); // from Euler
 @endcode

 \section sSO3_usage Usages

 The following testing codes demonstrated the basic usages of SO3:

 @code
TEST(Transform,SO3){
    std::default_random_engine e;

    double pitch=std::uniform_real_distribution<double>(-M_PI/2,M_PI/2)(e);
    double yaw  =std::uniform_real_distribution<double>(-M_PI,M_PI)(e);
    double roll =std::uniform_real_distribution<double>(-M_PI/2,M_PI/2)(e);

    SO3 q=SO3::fromPitchYawRoll(pitch,yaw,roll);

    EXPECT_NEAR(pitch,q.getPitch(),1e-5);
    EXPECT_NEAR(yaw,q.getYaw(),1e-5);
    EXPECT_NEAR(roll,q.getRoll(),1e-5);

    // X: forward Y: right Z: down
    SO3 qRoll =SO3::exp(Point3d(roll,0,0));
    SO3 qPitch=SO3::exp(Point3d(0,pitch,0));
    SO3 qYaw  =SO3::exp(Point3d(0,0,yaw));

    SO3 q1=qYaw*qPitch*qRoll;
    EXPECT_EQ(q,q1);

    Matrix3d m=q.getMatrix();
    q1=SO3(m);
    EXPECT_EQ(q,q1);

    Point3d abc=q.log();
    q1=SO3::exp(abc);
    EXPECT_EQ(q,q1);
    EXPECT_EQ(SO3(),q.inverse()*q);

    std::uniform_real_distribution<double> pt_gen(-1000,1000);
    Point3d xyz(pt_gen(e),pt_gen(e),pt_gen(e));
    Point3d p1=q.inverse()*q*xyz;
    EXPECT_NEAR((xyz-p1).norm(),0,1e-6);
}
 @endcode

 */
template <class Precision=double>
class SO3_
{
public:
    /// Default constructor, Idendity Matrix
    SO3_():x(0),y(0),z(0),w(1){}

    /// Construct from Quaternion
    SO3_(const Precision& X,const Precision& Y,const Precision& Z,const Precision& W)
        :x(X),y(Y),z(Z),w(W) { }

    /// Construct from a direction and angle in radius.
    SO3_(const Point3_<Precision>& direction,Precision angle)
    {
        FromAxis(direction,angle);
    }

    /// Construct from a rotation matrix data (ColMajor).
    SO3_(const Precision* M)
    {
        fromMatrix(M);
    }

    /// Construct from a rotation matrix
    SO3_(const Matrix<Precision,3,3>& M){
        fromMatrix(M.data());
    }

    /// Construct from another precision
    template<typename Scalar>
    SO3_(const SO3_<Scalar>& r):SO3_(r.x,r.y,r.z,r.w){
    }

    /// @return The lie algebra representation in \f$[a,b,c]\f$
    Point3_<Precision> log()const
    {
        const Precision squared_w = w*w;
        const Precision n = sqrt(x*x+y*y+z*z);

        Precision A_inv;
        // Atan-based log thanks to
        //
        // C. Hertzberg et al.:
        // "Integrating Generic Sensor Fusion Algorithms with Sound State
        // Representation through Encapsulation of Manifolds"
        // Information Fusion, 2011

        if (n < NEAR_ZERO)
        {
            //If n is too small
            A_inv = 2./w - 2.*(1.0-squared_w)/(w*squared_w);
        }
        else
        {
            if (fabs(w)<NEAR_ZERO)
            {
                //If w is too small
                if (w>0)
                {
                    A_inv = M_PI/n;
                }
                else
                {
                    A_inv = -M_PI/n;
                }
            }
            else
                A_inv = 2*atan(n/w)/n;
        }
        return Point3_<Precision>(x*A_inv,y*A_inv,z*A_inv);
    }

    /// Construct from lie algebra representation
    template<typename Scalar>
    static SO3_<Precision> exp(const Point3_<Scalar>& r)
    {
        const Scalar theta_sq=r.x*r.x+r.y*r.y+r.z*r.z;
        const Scalar theta = sqrt(theta_sq);
        const Scalar half_theta = 0.5*theta;

        const Scalar W = cos(half_theta);
        Scalar sin_half_theta;
        if(theta<NEAR_ZERO)
        {
          Scalar theta_po4 = theta_sq*theta_sq;
          sin_half_theta = 0.5-0.0208333*theta_sq+0.000260417*theta_po4;
        }
        else
        {
          sin_half_theta = sin(half_theta);
          sin_half_theta = sin_half_theta/theta;
        }

        return SO3_<Precision>(sin_half_theta*r.x,
                              sin_half_theta*r.y,
                              sin_half_theta*r.z,W);
    }

    template <typename T>
    static inline T sine(T x) {
        T sin = 0;
        //always wrap input angle to -PI..PI
        while (x < -3.14159265)
            x += 6.28318531;
        while (x > 3.14159265)
            x -= 6.28318531;
        //compute sine
        if (x < 0) {
            sin = 1.27323954 * x + .405284735 * x * x;
            if (sin < 0)
                sin = .225 * (sin * -sin - sin) + sin;
            else
                sin = .225 * (sin * sin - sin) + sin;
        } else {
            sin = 1.27323954 * x - 0.405284735 * x * x;
            if (sin < 0)
                sin = .225 * (sin * -sin - sin) + sin;
            else
                sin = .225 * (sin * sin - sin) + sin;
        }
        return sin;
    }

    template <typename T>
    static inline T cosine(T x) {
        //compute cosine: sin(x + PI/2) = cos(x)
        return sine(x+1.57079632);
    }

    template<typename Scalar>
    static SO3_<Scalar> expFast(const Point3_<Scalar>& l)
    {
        Scalar theta_sq = l.dot(l);
        Scalar theta    = sqrt(theta_sq);
        Scalar half_theta = static_cast<Scalar>(0.5) * theta;

        Scalar imag_factor;
        Scalar real_factor;

        if (theta < static_cast<Scalar>(1e-10)) {
          Scalar theta_po4 = theta_sq * theta_sq;
          imag_factor = static_cast<Scalar>(0.5) -
                        static_cast<Scalar>(1.0 / 48.0) * theta_sq +
                        static_cast<Scalar>(1.0 / 3840.0) * theta_po4;
          real_factor = static_cast<Scalar>(1) -
                        static_cast<Scalar>(0.5) * theta_sq +
                        static_cast<Scalar>(1.0 / 384.0) * theta_po4;
        } else {
          Scalar sin_half_theta = sine(half_theta);
          imag_factor = sin_half_theta / theta;
          real_factor = cosine(half_theta);
        }

        return SO3_<Scalar>( imag_factor * l.x, imag_factor * l.y,
            imag_factor * l.z,real_factor);
    }

    /// This is an unsafe operation.
    /// Please make sure that your pointer is both valid and has an appropriate size
    /// Wrong usage:
    /// Precision* p;
    /// getMatrix(p);
    ///
    /// Correct:
    /// Precision p[9];
    /// getMatrix(p);
    ///
    /// M is curved as follow
    /// |m0 m1 m2|
    /// |m3 m4 m5|
    /// |m6 m7 m8|
    inline void fromMatrix(const Precision* m)
    {
        auto SIGN=[](const Precision& v){return v>0?1.:-1.;};
        Precision& q0=w;
        Precision& q1=x;
        Precision& q2=y;
        Precision& q3=z;
        const Precision &r11=m[0],&r12=m[1],&r13=m[2],
                &r21=m[3],&r22=m[4],&r23=m[5],
                &r31=m[6],&r32=m[7],&r33=m[8];
        q0 = ( r11 + r22 + r33 + 1.0f) / 4.0f;
        q1 = ( r11 - r22 - r33 + 1.0f) / 4.0f;
        q2 = (-r11 + r22 - r33 + 1.0f) / 4.0f;
        q3 = (-r11 - r22 + r33 + 1.0f) / 4.0f;
        if(q0 < 0.0f) q0 = 0.0f;
        if(q1 < 0.0f) q1 = 0.0f;
        if(q2 < 0.0f) q2 = 0.0f;
        if(q3 < 0.0f) q3 = 0.0f;
        q0 = sqrt(q0);
        q1 = sqrt(q1);
        q2 = sqrt(q2);
        q3 = sqrt(q3);
        if(q0 >= q1 && q0 >= q2 && q0 >= q3) {
            q0 *= +1.0f;
            q1 *= SIGN(r32 - r23);
            q2 *= SIGN(r13 - r31);
            q3 *= SIGN(r21 - r12);
        } else if(q1 >= q0 && q1 >= q2 && q1 >= q3) {
            q0 *= SIGN(r32 - r23);
            q1 *= +1.0f;
            q2 *= SIGN(r21 + r12);
            q3 *= SIGN(r13 + r31);
        } else if(q2 >= q0 && q2 >= q1 && q2 >= q3) {
            q0 *= SIGN(r13 - r31);
            q1 *= SIGN(r21 + r12);
            q2 *= +1.0f;
            q3 *= SIGN(r32 + r23);
        } else if(q3 >= q0 && q3 >= q1 && q3 >= q2) {
            q0 *= SIGN(r21 - r12);
            q1 *= SIGN(r31 + r13);
            q2 *= SIGN(r32 + r23);
            q3 *= +1.0f;
        } else {
            std::cerr<<"Unable to construct SO3 from this Matrix.";
        }
        Precision r = sqrt(q0*q0+q1*q1+q2*q2+q3*q3);
        q0 /= r;
        q1 /= r;
        q2 /= r;
        q3 /= r;
    }

    Matrix<Precision,3,3> getMatrix()const{
        Matrix<Precision,3,3> m;
        getMatrix(m.data());
        return m;
    }

    /// return the matrix M
    inline void getMatrix(Precision* m)const
    {
        Precision x2 = x * x;
        Precision y2 = y * y;
        Precision z2 = z * z;
        Precision xy = x * y;
        Precision xz = x * z;
        Precision yz = y * z;
        Precision wx = w * x;
        Precision wy = w * y;
        Precision wz = w * z;
        m[0]=1.0-2.0*(y2+z2);    m[1]=2.0 * ( xy- wz);  m[2]= 2.0 * (xz + wy);
        m[3]=2.0 * (xy + wz);    m[4]=1.0-2.0*(x2+z2);  m[5]= 2.0 * (yz - wx);
        m[6]=2.0 * (xz - wy);    m[7]=2.0 * ( yz+ wx);  m[8]= 1.0-2.0*(x2+y2);
    }

#ifdef HAS_TOON
    /// Matrix things
    SO3(const TooN::Matrix<3,3,Precision>& m)
    {
        fromMatrix(m);
    }

    bool fromMatrix(const TooN::Matrix<3,3,Precision>& m)
    {
        w=0.5*sqrt(m[0][0]+m[1][1]+m[2][2]+1);
        Precision oneover4w=0.25/w;
        x=(m[2][1]-m[1][2])*oneover4w;
        y=(m[0][2]-m[2][0])*oneover4w;
        z=(m[1][0]-m[0][1])*oneover4w;
        if(w<0){x*=-1;y*=-1;z*=-1;w*=-1;}
        return true;
    }

    //return the matrix
    TooN::Matrix<3,3,Precision> getMatrix()const
    {
        Precision x2 = x * x;
        Precision y2 = y * y;
        Precision z2 = z * z;
        Precision xy = x * y;
        Precision xz = x * z;
        Precision yz = y * z;
        Precision wx = w * x;
        Precision wy = w * y;
        Precision wz = w * z;
        return TooN::Data(1.0-2.0*(y2+z2), 2.0 * ( xy- wz), 2.0 * (xz + wy),
                          2.0 * (xy + wz), 1.0-2.0*(x2+z2), 2.0 * (yz - wx),
                          2.0 * (xz - wy), 2.0 * ( yz+ wx), 1.0-2.0*(x2+y2));
    }

    operator TooN::SO3<Precision>()
    {
//        TooN::SO3<Precision> so3_toon;
        return getMatrix();
    }

#endif

#ifdef SOPHUS_SO3_HPP
    SO3(const Sophus::SO3Group<Precision>& so3)
        : x(so3.unit_quaternion().coeffs()[0]),
          y(so3.unit_quaternion().coeffs()[1]),
          z(so3.unit_quaternion().coeffs()[2]),
          w(so3.unit_quaternion().coeffs()[3])
    {}

    operator Sophus::SO3Group<Precision>()
    {
        return *(Sophus::SO3Group<Precision>*)&x;
    }
#endif

    static SO3_ fromPitchYawRollAngle(const Precision& pitch,const Precision& yaw,const Precision& roll)
    {
        Precision a2r=(3.1415926/180.0);
        return fromPitchYawRoll(pitch*a2r,yaw*a2r,roll*a2r);
    }

    static SO3_ fromPitchYawRoll(const Point3d& pyr){
        return fromPitchYawRoll(pyr.x,pyr.y,pyr.z);
    }

    /// Convert from Euler Angles,
    /// Please use "Radian" rather than degree to present angle
    static SO3_ fromPitchYawRoll(const Precision& pitch,const Precision& yaw,const Precision& roll)
    {
        // Basically we create 3 Quaternions, one for pitch, one for yaw, one for roll
        // and multiply those together.
        // the calculation below does the same, just shorter
        Precision piover360=0.5;//3.1415926/360.0;
        Precision p = pitch * piover360;
        Precision y = yaw * piover360;
        Precision r = roll * piover360;


        Precision sinp = sin(p);
        Precision siny = sin(y);
        Precision sinr = sin(r);
        Precision cosp = cos(p);
        Precision cosy = cos(y);
        Precision cosr = cos(r);


        Precision rx = sinr * cosp * cosy - cosr * sinp * siny;
        Precision ry = cosr * sinp * cosy + sinr * cosp * siny;
        Precision rz = cosr * cosp * siny - sinr * sinp * cosy;
        Precision rw = cosr * cosp * cosy + sinr * sinp * siny;

        SO3_ ret(rx,ry,rz,rw);
        ret.normalise();
        return ret;
    }

    Precision getRoll()const//Radian about axis X
    {
        return atan2(2.0*(w*x+y*z),1.0-2.0*(x*x+y*y));
    }

    Precision getPitch()const//Radian about axis Y
    {
        return asin(2.0*(w*y-z*x));
    }

    Precision getYaw()const//Radian about axis Z
    {
        return atan2(2.0*(w*z+x*y),1.0-2.0*(z*z+y*y));
    }

    ///
    SO3_ operator* (const SO3_& rq) const
    {
        // the constructor takes its arguments as (x, y, z, w)
        return SO3_( w * rq.x + x * rq.w + y * rq.z - z * rq.y,
                    w * rq.y + y * rq.w + z * rq.x - x * rq.z,
                    w * rq.z + z * rq.w + x * rq.y - y * rq.x,
                    w * rq.w - x * rq.x - y * rq.y - z * rq.z);
    }

    // Multiplying a quaternion q with a vector v applies the q-rotation to v
    Point3_<Precision> operator* (const Point3_<Precision>& p) const
    {
        // Note that this algorithm comes from the optimization by hand
        // of the conversion to a Matrix followed by a Matrix/Vector product.
        // It appears to be much faster than the common algorithm found
        // in the literature (30 versus 39 flops). It also requires two
        // Vector3 as temporaries.
        Point3_<Precision> uv = Point3_<Precision>(x,y,z).cross(p);
        uv = uv + uv;
        return p + w * uv + Point3_<Precision>(x,y,z).cross(uv);
    }

    bool operator ==(const SO3_& rq)const{
        return ((*this).log()-rq.log()).norm()<1e-6;
    }

    // Convert from Axis Angle
    static SO3_ FromAxis(const Point3_<Precision>& p,Precision angle)
    {
        Precision det=sqrt(p.x*p.x+p.y*p.y+p.z*p.z);
        if(det<0.00001)
        {
            return SO3_();
        }
        angle *= 0.5;
        Precision p2v=sin(angle)/det;
        return SO3_(p.x*p2v,p.y*p2v,p.z*p2v,cos(angle));
    }

    void normalise()
    {
        // Don't normalize if we don't have to
        Precision mag2 = w * w + x * x + y * y + z * z;
        if (  mag2!=0.f && (fabs(mag2 - 1.0f) > 0.001))
        {
            Precision mag = 1./sqrt(mag2);
            w *= mag;
            x *= mag;
            y *= mag;
            z *= mag;
        }
    }

    // We need to get the inverse of a quaternion to properly apply a quaternion-rotation to a vector
    // The conjugate of a quaternion is the same as the inverse, as long as the quaternion is unit-length
    SO3_ inverse() const
    {
        return SO3_(-x, -y, -z, w);
    }

    void getValue(Precision& X,Precision& Y,Precision& Z,Precision& W) const
    {
        X=x;Y=y;Z=z;W=w;
    }

    operator std::string()const
    {
        std::stringstream sst;sst<<*this;return sst.str();
    }

    Precision getX()const{return x;}
    Precision getY()const{return y;}
    Precision getZ()const{return z;}
    Precision getW()const{return w;}

    void setX(Precision X){x=X;}
    void setY(Precision Y){y=Y;}
    void setZ(Precision Z){z=Z;}
    void setW(Precision W){w=W;}
    SO3_  mul (const SO3_& rq) const{return (*this)*rq;}
    Point3_<Precision> trans(const Point3_<Precision>& p) const{return (*this)*p;}

    std::string toString()const{std::stringstream sst;sst<<*this;return sst.str();}

public:
    Precision x,y,z,w;
};

typedef SO3_<double> SO3d;
typedef SO3_<float> SO3f;
typedef SO3d SO3;

/// Write an SO3 to a stream
/// @relates SO3
template <typename Precision>
inline std::ostream& operator << (std::ostream& os,const SO3_<Precision>& so3)
{
#if 0//HAS_TOON
    os<<so3.getMatrix();
    return os;
#else
    Precision X,Y,Z,W;
    so3.getValue(X,Y,Z,W);
    os<<X<<" "<<Y<<" "<<Z<<" "<<W;
    return os;
#endif
}

/// Write an SO3 from a stream
/// @relates SO3
template <typename Precision>
inline std::istream& operator >> (std::istream& is,SO3_<Precision>& so3)
{
    is>>so3.x>>so3.y>>so3.z>>so3.w;
    return is;
}


} //end of namespace

#endif // GSLAM_CORE_SO3_H
