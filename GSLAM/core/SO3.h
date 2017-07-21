#ifndef GSLAM_SO3_H
#define GSLAM_SO3_H

#include <iostream>
#include "Point.h"
#include "Array.h"

#ifdef HAS_TOON
#include <TooN/se3.h>
#else
#include <math.h>
#endif

#ifndef M_PI
# define M_PI		3.14159265358979323846	/* pi */
#endif

namespace pi {

#define NEAR_ZERO 1e-10

/**
 Class SO3 is actually represented by the quaternion,
 which use 4 paraments to present a 3 dimesion rotation matrix,
 since 3D rotation matrices are members of the Special Orthogonal Lie group SO3.

 Every rotation in the 3D euclidean space can be represented by a rotation with one direction.
 Consider a rotation with direction (a,b,c)^T and angle theta in radians.
 w -- cos(theta/2)
 x -- a*sin(theta/2)
 y -- b*sin(theta/2)
 z -- c*sin(theta/2)
 this ensures that x^2+y^2+z^2+w^2=1.
 A quaternion q(x,y,z,w) is used to present a 3d rotation.

 it can be constructed by a matrix:(tested TooN,Eigen)
 @code
 Eigen::Matrix3d M;
 SO3<double> so3;
 // You should ensure that both the SO3 and matrix has the same precision(double or float)
 so3.fromMatrix(M);
 @endcode

 This group can be parameterised by three numbers (a vector in the space of Lie Algebra).
 Here uses r=(rx,ry,rz)^T to present a rotation,
 where its direction is the axis of rotation and its length is the angle of rotation in radians.
 It is also used by TooN (also know as PTAM) and Sophus.
 @code
 SO3d so3=SO3d::exp(Point3d(1,1,1)); //construct from 3 parament
 cout<<"Three paraments to present a rotation:"<<so3.ln()<<endl;
 @endcode

 */
template <class Precision=double>
class SO3
{
public:
    SO3():x(0),y(0),z(0),w(1){}

    SO3(const Precision& X,const Precision& Y,const Precision& Z,const Precision& W)
        :x(X),y(Y),z(Z),w(W) { }

    /// Construct from a direction and angle in radius.
    SO3(const Point3_<Precision>& direction,Precision angle)
    {
        FromAxis(direction,angle);
    }

    /// Construct from a rotation matrix.
    SO3(const Precision* M)
    {
        fromMatrix(M);
    }

    /// Coversion from different precision
    template<typename Scalar>
    operator SO3<Scalar>()const
    {
        return SO3<Scalar>(x,y,z,w);
    }

    Point3_<Precision> ln()const
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

    template<typename Scalar>
    static SO3<Precision> exp(const Point3_<Scalar>& r)
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

        return SO3<Precision>(sin_half_theta*r.x,
                              sin_half_theta*r.y,
                              sin_half_theta*r.z,W);
    }
    inline Precision SIGN(const Precision& v){return v>0?1.:-1.;}

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
    inline bool fromMatrix(const Precision* m)
    {
        if(false)
        {
            Precision tr=m[0]+m[4]+m[8];
            w=0.5*sqrt(tr+1);
            Precision oneover4w=0.25/w;
            x=(m[7]-m[5])*oneover4w;
            y=(m[2]-m[6])*oneover4w;
            z=(m[3]-m[1])*oneover4w;
            if(w<0){x*=-1;y*=-1;z*=-1;w*=-1;}
            return true;
        }
        else
        {
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
                printf("coding error\n");
                return false;
            }
            Precision r = sqrt(q0*q0+q1*q1+q2*q2+q3*q3);
            q0 /= r;
            q1 /= r;
            q2 /= r;
            q3 /= r;
        }
        return true;
    }

    template <typename T>
    inline bool fromMatrixUnsafe(T& matrix)
    {
        if(sizeof(matrix)==9*sizeof(Precision))
            return fromMatrix((Precision*)&matrix);
        else
        {
            std::cerr<<"SO3::fromMatrixUnsafe: Wrong precision detected!";
            return false;
        }
    }

    /// return the matrix M
    inline bool getMatrix(Precision* m)const
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
        return true;
    }

    template <typename T>
    inline bool getMatrixUnsafe(T& matrix)
    {
        static_assert((sizeof(matrix))==9*sizeof(Precision),"The rotation should be 9*Precision.");
        return getMatrix((Precision*)&matrix);
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

    void FromEulerAngle(const Precision& pitch,const Precision& yaw,const Precision& roll)
    {
        Precision a2r=(3.1415926/180.0);
        FromEuler(pitch*a2r,yaw*a2r,roll*a2r);
    }

    /// Convert from Euler Angles,
    /// Please use "Radian" rather than degree to present angle
    void FromEuler(const Precision& pitch,const Precision& yaw,const Precision& roll)
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


        this->x = sinr * cosp * cosy - cosr * sinp * siny;
        this->y = cosr * sinp * cosy + sinr * cosp * siny;
        this->z = cosr * cosp * siny - sinr * sinp * cosy;
        this->w = cosr * cosp * cosy + sinr * sinp * siny;


        normalise();
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
    SO3 operator* (const SO3& rq) const
    {
        // the constructor takes its arguments as (x, y, z, w)
        return SO3( w * rq.x + x * rq.w + y * rq.z - z * rq.y,
                    w * rq.y + y * rq.w + z * rq.x - x * rq.z,
                    w * rq.z + z * rq.w + x * rq.y - y * rq.x,
                    w * rq.w - x * rq.x - y * rq.y - z * rq.z);
    }

    // Multiplying a quaternion q with a vector v applies the q-rotation to v
    Point3_<Precision> operator* (const Point3_<Precision>& p) const
    {
        SO3 so3_p(p.x,p.y,p.z,0);
        so3_p=(*this)*so3_p*inv();
        return Point3_<Precision>(so3_p.x,so3_p.y,so3_p.z);
    }

    // Convert from Axis Angle
    static SO3 FromAxis(const Point3_<Precision>& p,Precision angle)
    {
        Precision det=sqrt(p.x*p.x+p.y*p.y+p.z*p.z);
        if(det<0.00001)
        {
            return SO3();
        }
        angle *= 0.5;
        Precision p2v=sin(angle)/det;
        return SO3(p.x*p2v,p.y*p2v,p.z*p2v,cos(angle));
    }

    void normalise()
    {
        // Don't normalize if we don't have to
        Precision mag2 = w * w + x * x + y * y + z * z;
        if (  mag2!=0.f && (fabs(mag2 - 1.0f) > 0.001))
        {
            Precision mag = sqrt(mag2);
            w /= mag;
            x /= mag;
            y /= mag;
            z /= mag;
        }
    }

    // We need to get the inverse of a quaternion to properly apply a quaternion-rotation to a vector
    // The conjugate of a quaternion is the same as the inverse, as long as the quaternion is unit-length
    SO3 inv() const
    {
        return SO3(-x, -y, -z, w);
    }

    void getValue(Precision& X,Precision& Y,Precision& Z,Precision& W) const
    {
        X=x;Y=y;Z=z;W=w;
    }

    operator std::string()const
    {
        std::string os=to_str(x)+" "+to_str(y)+" "+to_str(z)+" "+to_str(w);
        return os;
    }

    operator std::ostream()const
    {
        std::ostream os;
        os<<x<<" "<<y<<" "<<z<<" "<<w;
        return os;
    }

public:
    Precision x,y,z,w;
};

typedef SO3<double> SO3d;
typedef SO3<float> SO3f;

/// Write an SO3 to a stream
/// @relates SO3
template <typename Precision>
inline std::ostream& operator << (std::ostream& os,const SO3<Precision>& so3)
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
inline std::istream& operator >> (std::istream& is,SO3<Precision>& so3)
{
    is>>so3.x>>so3.y>>so3.z>>so3.w;
    return is;
}


} //end of namespace

#endif // ZY_SO3_H
